/* extract_lines.c -- see extract_lines.h. A port of extraction.py, not a design.
 *
 * The one structural liberty taken, and it is not a liberty about behaviour:
 * EVERY GROUP OF BEAM INDICES IN THIS ALGORITHM IS A CONTIGUOUS RANGE. The
 * adaptive breakpoints produce contiguous clusters, and the IEPF only ever
 * splits a range into g[:k+1] and g[k:], which are both contiguous and share
 * the index k. So a group is two integers here where the Python carries an
 * index array, and the union of a merge cluster's groups -- which the Python
 * takes with np.unique on a concatenation -- is a handful of disjoint ranges
 * after sorting and coalescing. That is what makes the refit a pair of passes
 * over ranges with no index array and no allocation, and it is also what makes
 * the duplicate shared split point disappear exactly as np.unique removes it.
 */
#include "corridor_perception_c/extract_lines.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define EL_MAX_GROUPS EL_MAX_BEAMS
#define IEPF_SMOOTH 5
#define MIN_FIT_POINTS 3

/* ---------------------------------------------------------------- helpers */

static double wrap_pi(double a) {
  /* Python's % returns a result with the sign of the DIVISOR; C's fmod keeps
   * the sign of the dividend. Without the correction every negative angle
   * comes out 2 pi away from the reference. */
  double m = fmod(a + M_PI, 2.0 * M_PI);
  if (m < 0.0) m += 2.0 * M_PI;
  return m - M_PI;
}

typedef struct {
  double rho, alpha;
  double cov00, cov01, cov11;
  double p0x, p0y, p1x, p1y;
  int32_t n_points;
  double s2; /* mean square sigma of the support, for the covariance */
} el_fit;

typedef struct {
  int32_t lo, hi; /* [lo, hi) beam indices */
} el_range;

/* The static workspace. Sized at compile time, touched by one caller. */
static struct {
  double x[EL_MAX_BEAMS];
  double y[EL_MAX_BEAMS];
  double ang[EL_MAX_BEAMS];
  double rng[EL_MAX_BEAMS]; /* the float32 input widened once */
  double sig[EL_MAX_BEAMS];
  uint8_t valid[EL_MAX_BEAMS];
  uint8_t brk[EL_MAX_BEAMS];
  /* IEPF */
  double dev[EL_MAX_BEAMS];
  double smooth[EL_MAX_BEAMS];
  el_range stack[EL_MAX_BEAMS];
  /* groups and their fits */
  el_range group[EL_MAX_GROUPS];
  el_fit fit[EL_MAX_GROUPS];
  /* merge */
  double midt[EL_MAX_GROUPS]; /* mid . tangent, the lever arm */
  double reach[EL_MAX_GROUPS];
  int32_t akey[EL_MAX_GROUPS];
  int32_t order[EL_MAX_GROUPS];  /* indices sorted by (akey, rho) */
  int32_t parent[EL_MAX_GROUPS];
  int32_t cand[3 * EL_MAX_GROUPS];
  int32_t root_of[EL_MAX_GROUPS]; /* root -> cluster slot, -1 when unseen */
  el_range members[EL_MAX_GROUPS];
  int32_t cluster_head[EL_MAX_GROUPS];
  int32_t cluster_next[EL_MAX_GROUPS];
  int32_t cluster_tail[EL_MAX_GROUPS];
  el_range coalesced[EL_MAX_GROUPS];
} W;

/* bucket offsets over the alpha buckets, rebuilt per scan */
#define EL_MAX_BUCKETS 4096
static int32_t B_start[EL_MAX_BUCKETS + 1];

void el_default_params(el_params *p) {
  if (!p) return;
  p->sigma_0 = 0.015;
  p->sigma_r = 0.002;
  p->lambda_abd = 10.0 * M_PI / 180.0;
  p->abd_sigma_mult = 3.0;
  p->split_dist = 0.045;
  p->merge_alpha = 6.0 * M_PI / 180.0;
  p->merge_rho = 0.05;
  p->merge_gap = 1.50;
  p->min_points = 10;
  p->min_length = 0.50;
  p->max_point_gap = 0.60;
  p->quality_gates = 1;
  p->seed_base_points = MIN_FIT_POINTS;
  p->seed_max_points = 6;
  p->seed_range_m = 2.0;
  p->seed_range_step_m = 1.5;
  p->max_rms_m = 0.03;
  p->min_fill = 0.6;
  p->occlusion_margin_m = 0.10;
  p->min_incidence = 0.05;
}

const char *el_version(void) { return "1"; }

/* ------------------------------------------------------- the TLS line fit */

/* Moments over a set of disjoint beam ranges, then extraction.py's own
 * closed-form fit. Two passes: the first for the centroid and the scatter,
 * the second -- once alpha is known -- for the support's extent along the
 * tangent, which is what gives the endpoints. */
static void fit_tls(const el_range *rs, int32_t nr, el_fit *f) {
  double sx = 0.0, sy = 0.0, ss = 0.0;
  int32_t n = 0;
  for (int32_t k = 0; k < nr; ++k) {
    for (int32_t i = rs[k].lo; i < rs[k].hi; ++i) {
      sx += W.x[i];
      sy += W.y[i];
      ss += W.sig[i] * W.sig[i];
      ++n;
    }
  }
  f->n_points = n;
  if (n <= 0) {
    memset(f, 0, sizeof(*f));
    return;
  }
  const double cx = sx / (double)n, cy = sy / (double)n;
  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (int32_t k = 0; k < nr; ++k) {
    for (int32_t i = rs[k].lo; i < rs[k].hi; ++i) {
      const double dx = W.x[i] - cx, dy = W.y[i] - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
  }
  f->s2 = ss / (double)n;
  double alpha = 0.5 * atan2(-2.0 * sxy, syy - sxx);
  double rho = cx * cos(alpha) + cy * sin(alpha);
  if (rho < 0.0) {
    rho = -rho;
    alpha = wrap_pi(alpha + M_PI);
  } else {
    alpha = wrap_pi(alpha);
  }
  const double ca = cos(alpha), sa = sin(alpha);
  /* The along-line second moment is the larger eigenvalue of the scatter. */
  const double s_sq = sxx * sa * sa - 2.0 * sxy * sa * ca + syy * ca * ca;
  const double denom = s_sq > 1e-12 ? s_sq : 1e-12;
  const double var_alpha = f->s2 / denom;
  const double var_e = f->s2 / (double)n;
  const double t_c = -sa * cx + ca * cy;
  f->rho = rho;
  f->alpha = alpha;
  f->cov00 = var_e + t_c * t_c * var_alpha;
  f->cov01 = t_c * var_alpha;
  f->cov11 = var_alpha;
  double tmin = INFINITY, tmax = -INFINITY;
  for (int32_t k = 0; k < nr; ++k) {
    for (int32_t i = rs[k].lo; i < rs[k].hi; ++i) {
      const double t = -sa * W.x[i] + ca * W.y[i];
      if (t < tmin) tmin = t;
      if (t > tmax) tmax = t;
    }
  }
  f->p0x = rho * ca + tmin * -sa;
  f->p0y = rho * sa + tmin * ca;
  f->p1x = rho * ca + tmax * -sa;
  f->p1y = rho * sa + tmax * ca;
}

static double fit_length(const el_fit *f) {
  const double dx = f->p1x - f->p0x, dy = f->p1y - f->p0y;
  return sqrt(dx * dx + dy * dy);
}

static double perp_rms(const el_range *rs, int32_t nr, double rho, double alpha) {
  const double ca = cos(alpha), sa = sin(alpha);
  double acc = 0.0;
  int32_t n = 0;
  for (int32_t k = 0; k < nr; ++k) {
    for (int32_t i = rs[k].lo; i < rs[k].hi; ++i) {
      const double res = W.x[i] * ca + W.y[i] * sa - rho;
      acc += res * res;
      ++n;
    }
  }
  return n ? sqrt(acc / (double)n) : INFINITY;
}

/* --------------------------------------------------------- the fill ratio */

static double fill_ratio(const el_fit *f, int32_t n, const el_params *p) {
  const double b0 = atan2(f->p0y, f->p0x);
  const double b1 = atan2(f->p1y, f->p1x);
  const double d = wrap_pi(b1 - b0);
  const double half = 0.5 * fabs(d);
  const double centre = wrap_pi(b0 + 0.5 * d);
  const double ca = cos(f->alpha), sa = sin(f->alpha);
  int32_t n_visible = 0, n_usable = 0, n_span = 0;
  for (int32_t i = 0; i < n; ++i) {
    if (fabs(wrap_pi(W.ang[i] - centre)) > half) continue;
    ++n_span;
    const double inc = ca * cos(W.ang[i]) + sa * sin(W.ang[i]);
    if (inc <= p->min_incidence) continue;
    ++n_usable;
    const double s = f->rho / inc;
    /* An inf range is NOT occluded: nothing is nearer, there is simply no
     * return. A NaN compares false and lands here too, as in numpy. */
    if (!(W.rng[i] < s - p->occlusion_margin_m)) ++n_visible;
  }
  if (n_span == 0 || n_usable == 0) return INFINITY;
  const int32_t den = n_visible > 1 ? n_visible : 1;
  return (double)f->n_points / (double)den;
}

/* ------------------------------------------------------------- the stages */

static int32_t seed_min_count(double r_med, const el_params *p) {
  if (!isfinite(r_med)) return p->seed_base_points;
  double q = (r_med - p->seed_range_m) / p->seed_range_step_m;
  int32_t n = p->seed_base_points + (int32_t)floor(q);
  if (n < p->seed_base_points) n = p->seed_base_points;
  if (n > p->seed_max_points) n = p->seed_max_points;
  return n;
}

/* N(r) for a group, with extraction.py's own short circuit on the size. */
static int32_t seed_floor(const el_range *g, const el_params *p) {
  if (!p->quality_gates) return MIN_FIT_POINTS;
  const int32_t size = g->hi - g->lo;
  if (size >= p->seed_max_points || size < p->seed_base_points)
    return p->seed_base_points;
  /* The median of at most seed_max_points finite values. */
  double v[16];
  int32_t m = 0;
  for (int32_t i = g->lo; i < g->hi && m < 16; ++i)
    if (isfinite(W.rng[i])) v[m++] = W.rng[i];
  if (m == 0) return p->seed_base_points;
  for (int32_t a = 1; a < m; ++a) { /* insertion sort, m <= 6 */
    const double key = v[a];
    int32_t b = a - 1;
    while (b >= 0 && v[b] > key) { v[b + 1] = v[b]; --b; }
    v[b + 1] = key;
  }
  return seed_min_count(v[m / 2], p);
}

/* IEPF, LIFO, with the LEFT half pushed last so it is processed first. */
static int32_t iepf_split(el_range g, const el_params *p, el_range *out,
                          int32_t *n_out, int32_t max_out) {
  int32_t top = 0;
  W.stack[top++] = g;
  while (top > 0) {
    const el_range cur = W.stack[--top];
    const int32_t size = cur.hi - cur.lo;
    if (size < 3) {
      if (size >= 2) {
        if (*n_out >= max_out) return EL_ERR_OVERFLOW;
        out[(*n_out)++] = cur;
      }
      continue;
    }
    int32_t w = IEPF_SMOOTH;
    const int32_t third = size / 3;
    if (w > (third > 1 ? third : 1)) w = (third > 1 ? third : 1);
    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
    for (int32_t i = 0; i < w; ++i) {
      ax += W.x[cur.lo + i];
      ay += W.y[cur.lo + i];
      bx += W.x[cur.hi - 1 - i];
      by += W.y[cur.hi - 1 - i];
    }
    ax /= w; ay /= w; bx /= w; by /= w;
    const double chx = bx - ax, chy = by - ay;
    const double length = sqrt(chx * chx + chy * chy);
    for (int32_t i = 0; i < size; ++i) {
      const double rx = W.x[cur.lo + i] - ax, ry = W.y[cur.lo + i] - ay;
      W.dev[i] = (length < 1e-9) ? sqrt(rx * rx + ry * ry)
                                 : (chx * ry - chy * rx) / length;
    }
    const double *dv = W.dev;
    if (length >= 1e-9 && w > 1) {
      /* np.convolve(dist, full(w, 1/w), mode='same'): a centred boxcar with
       * ZERO padding outside, and always divided by w -- the edges are not
       * renormalised, which is the behaviour the split threshold was tuned
       * against. */
      const int32_t off = (w - 1) / 2;
      for (int32_t i = 0; i < size; ++i) {
        double acc = 0.0;
        for (int32_t k = 0; k < w; ++k) {
          const int32_t j = i + off - k;
          if (j >= 0 && j < size) acc += W.dev[j];
        }
        W.smooth[i] = acc / (double)w;
      }
      dv = W.smooth;
    }
    int32_t kbest = 0;
    double dbest = -1.0;
    for (int32_t i = 0; i < size; ++i) {
      const double v = (length < 1e-9) ? dv[i] : fabs(dv[i]);
      if (v > dbest) { dbest = v; kbest = i; }
    }
    if (dbest > p->split_dist && kbest > 0 && kbest < size - 1) {
      if (top + 2 > EL_MAX_BEAMS) return EL_ERR_OVERFLOW;
      const el_range right = {cur.lo + kbest, cur.hi};
      const el_range left = {cur.lo, cur.lo + kbest + 1};
      W.stack[top++] = right;
      W.stack[top++] = left;
    } else {
      if (*n_out >= max_out) return EL_ERR_OVERFLOW;
      out[(*n_out)++] = cur;
    }
  }
  return 0;
}

static int32_t uf_find(int32_t i) {
  while (W.parent[i] != i) {
    W.parent[i] = W.parent[W.parent[i]]; /* path halving, as in the Python */
    i = W.parent[i];
  }
  return i;
}

static int cmp_order(const void *pa, const void *pb) {
  const int32_t a = *(const int32_t *)pa, b = *(const int32_t *)pb;
  if (W.akey[a] != W.akey[b]) return W.akey[a] < W.akey[b] ? -1 : 1;
  if (W.fit[a].rho < W.fit[b].rho) return -1;
  if (W.fit[a].rho > W.fit[b].rho) return 1;
  return a < b ? -1 : (a > b ? 1 : 0); /* stable, like np.argsort's mergesort */
}

/* ------------------------------------------------------------------- main */

int32_t el_extract(const float *ranges, int32_t n, double angle_min,
                   double angle_increment, const el_params *p, el_segment *out,
                   int32_t max_out, el_stats *stats) {
  if (!ranges || !p || !out) return EL_ERR_NULL;
  if (n < 0 || n > EL_MAX_BEAMS) return EL_ERR_TOO_MANY;
  if (stats) memset(stats, 0, sizeof(*stats));
  if (n == 0) return 0;

  /* ---- points, sigmas, validity ---------------------------------------- */
  for (int32_t i = 0; i < n; ++i) {
    const double r = (double)ranges[i];
    W.rng[i] = r;
    W.ang[i] = angle_min + (double)i * angle_increment;
    const int v = isfinite(r) && r > 0.0;
    W.valid[i] = (uint8_t)v;
    const double rv = v ? r : 0.0;
    W.x[i] = rv * cos(W.ang[i]);
    W.y[i] = rv * sin(W.ang[i]);
    W.sig[i] = p->sigma_0 + p->sigma_r * rv;
  }

  /* ---- adaptive breakpoints -------------------------------------------- */
  W.brk[0] = 1;
  for (int32_t i = 1; i < n; ++i) {
    const double dphi = fabs(W.ang[i] - W.ang[i - 1]);
    const double rp = W.valid[i - 1] ? W.rng[i - 1] : 0.0;
    const double den = sin(p->lambda_abd - dphi);
    double dmax;
    if (den > 0.0) {
      const double cand =
          rp * sin(dphi) / den + p->abd_sigma_mult * (p->sigma_0 + p->sigma_r * rp);
      dmax = cand < p->max_point_gap ? cand : p->max_point_gap;
    } else {
      dmax = p->max_point_gap;
    }
    const double dx = W.x[i] - W.x[i - 1], dy = W.y[i] - W.y[i - 1];
    const double gap = sqrt(dx * dx + dy * dy);
    const int joined = W.valid[i] && W.valid[i - 1] && (gap <= dmax);
    W.brk[i] = (uint8_t)(joined ? 0 : 1);
  }

  /* ---- clusters -> seed gate -> IEPF -> groups -------------------------- */
  int32_t n_groups = 0;
  int32_t s = 0;
  while (s < n) {
    int32_t e = s + 1;
    while (e < n && !W.brk[e]) ++e;
    const el_range cluster = {s, e};
    if ((e - s) >= 2 && W.valid[s]) {
      if ((e - s) < seed_floor(&cluster, p)) {
        if (stats) stats->seed++;
      } else {
        const int32_t before = n_groups;
        const int32_t rc =
            iepf_split(cluster, p, W.group, &n_groups, EL_MAX_GROUPS);
        if (rc < 0) return rc;
        /* The seed gate again, on each split group, exactly as the Python
         * applies it inside the comprehension. */
        int32_t keep = before;
        for (int32_t gi = before; gi < n_groups; ++gi) {
          const int32_t size = W.group[gi].hi - W.group[gi].lo;
          if (size >= seed_floor(&W.group[gi], p)) {
            W.group[keep++] = W.group[gi];
          } else if (stats) {
            stats->seed++;
          }
        }
        n_groups = keep;
      }
    }
    s = e;
  }
  if (n_groups == 0) return 0;

  for (int32_t i = 0; i < n_groups; ++i) fit_tls(&W.group[i], 1, &W.fit[i]);

  /* ---- the global collinear merge -------------------------------------- */
  const int32_t m = n_groups;
  for (int32_t i = 0; i < m; ++i) W.parent[i] = i;
  int32_t n_alpha = (int32_t)floor(2.0 * M_PI / p->merge_alpha + 1e-9);
  if (n_alpha < 1) n_alpha = 1;
  if (n_alpha > EL_MAX_BUCKETS) n_alpha = EL_MAX_BUCKETS;
  for (int32_t i = 0; i < m; ++i) {
    const el_fit *f = &W.fit[i];
    const double ca = cos(f->alpha), sa = sin(f->alpha);
    const double midx = 0.5 * (f->p0x + f->p1x), midy = 0.5 * (f->p0y + f->p1y);
    W.midt[i] = -sa * midx + ca * midy;
    W.reach[i] = 1.05 * (p->merge_rho + fabs(W.midt[i]) * p->merge_alpha);
    int32_t k = (int32_t)floor((f->alpha + M_PI) / p->merge_alpha) % n_alpha;
    if (k < 0) k += n_alpha;
    W.akey[i] = k;
    W.order[i] = i;
  }
  if (m > 1) qsort(W.order, (size_t)m, sizeof(int32_t), cmp_order);
  for (int32_t b = 0; b <= n_alpha; ++b) B_start[b] = m;
  for (int32_t idx = m - 1; idx >= 0; --idx) B_start[W.akey[W.order[idx]]] = idx;
  /* make the offsets monotonic, so an empty bucket is an empty slice */
  for (int32_t b = n_alpha - 1; b >= 0; --b)
    if (B_start[b] > B_start[b + 1]) B_start[b] = B_start[b + 1];

  for (int32_t i = 0; i < m; ++i) {
    const el_fit *fi = &W.fit[i];
    const double cai = cos(fi->alpha), sai = sin(fi->alpha);
    const double lo_rho = fi->rho - W.reach[i], hi_rho = fi->rho + W.reach[i];
    int32_t n_cand = 0;
    for (int32_t da = -1; da <= 1; ++da) {
      int32_t b = (W.akey[i] + da) % n_alpha;
      if (b < 0) b += n_alpha;
      const int32_t bs = B_start[b], be = B_start[b + 1];
      /* lower_bound on rho, twice: np.searchsorted's own 'left' semantics */
      int32_t lo = bs, hi = be;
      while (lo < hi) {
        const int32_t mid = lo + (hi - lo) / 2;
        if (W.fit[W.order[mid]].rho < lo_rho) lo = mid + 1; else hi = mid;
      }
      const int32_t from = lo;
      lo = from; hi = be;
      while (lo < hi) {
        const int32_t mid = lo + (hi - lo) / 2;
        if (W.fit[W.order[mid]].rho < hi_rho) lo = mid + 1; else hi = mid;
      }
      for (int32_t k = from; k < lo; ++k) {
        const int32_t j = W.order[k];
        if (j != i) W.cand[n_cand++] = j;
      }
    }
    /* segment i's own tangent, and its support's extent along it */
    const double tix = -sai, tiy = cai;
    double a0 = fi->p0x * tix + fi->p0y * tiy;
    double a1 = fi->p1x * tix + fi->p1y * tiy;
    if (a0 > a1) { const double t = a0; a0 = a1; a1 = t; }
    for (int32_t c = 0; c < n_cand; ++c) {
      const int32_t j = W.cand[c];
      const el_fit *fj = &W.fit[j];
      if (fabs(wrap_pi(fj->alpha - fi->alpha)) >= p->merge_alpha) continue;
      /* the offset between the two lines AT THE POINT BETWEEN their supports,
       * not at the origin */
      const double mx = 0.5 * (0.5 * (fi->p0x + fi->p1x) + 0.5 * (fj->p0x + fj->p1x));
      const double my = 0.5 * (0.5 * (fi->p0y + fi->p1y) + 0.5 * (fj->p0y + fj->p1y));
      const double sep = (mx * cai + my * sai - fi->rho) -
                         (mx * cos(fj->alpha) + my * sin(fj->alpha) - fj->rho);
      if (fabs(sep) >= p->merge_rho) continue;
      const double bs0 = fj->p0x * tix + fj->p0y * tiy;
      const double bs1 = fj->p1x * tix + fj->p1y * tiy;
      const double bmin = bs0 < bs1 ? bs0 : bs1, bmax = bs0 < bs1 ? bs1 : bs0;
      const double gap = (bmin > a0 ? bmin : a0) - (bmax < a1 ? bmax : a1);
      if (!(gap < p->merge_gap)) continue;
      const int32_t ri = uf_find(i), rj = uf_find(j);
      if (ri != rj) W.parent[rj] = ri;
    }
  }

  /* ---- clusters in first-root-seen order, then the gates --------------- */
  for (int32_t i = 0; i < m; ++i) W.root_of[i] = -1;
  int32_t n_clusters = 0;
  for (int32_t i = 0; i < m; ++i) {
    const int32_t r = uf_find(i);
    int32_t slot = W.root_of[r];
    if (slot < 0) {
      slot = n_clusters++;
      W.root_of[r] = slot;
      W.cluster_head[slot] = i;
      W.cluster_tail[slot] = i;
    } else {
      W.cluster_next[W.cluster_tail[slot]] = i;
      W.cluster_tail[slot] = i;
    }
    W.cluster_next[i] = -1;
  }

  int32_t n_out = 0;
  for (int32_t cl = 0; cl < n_clusters; ++cl) {
    int32_t n_members = 0;
    for (int32_t i = W.cluster_head[cl]; i >= 0; i = W.cluster_next[i])
      W.members[n_members++] = W.group[i];
    const int32_t first = W.cluster_head[cl];
    el_fit fit;
    int32_t n_ranges;
    if (n_members == 1) {
      fit = W.fit[first];
      W.coalesced[0] = W.group[first];
      n_ranges = 1;
    } else {
      /* np.unique(concatenate(...)): sort the ranges and coalesce the ones
       * that touch or overlap, which is where the IEPF's shared split points
       * are deduplicated. */
      for (int32_t a = 1; a < n_members; ++a) {
        const el_range key = W.members[a];
        int32_t b = a - 1;
        while (b >= 0 && W.members[b].lo > key.lo) { W.members[b + 1] = W.members[b]; --b; }
        W.members[b + 1] = key;
      }
      n_ranges = 0;
      for (int32_t a = 0; a < n_members; ++a) {
        if (n_ranges && W.members[a].lo <= W.coalesced[n_ranges - 1].hi) {
          if (W.members[a].hi > W.coalesced[n_ranges - 1].hi)
            W.coalesced[n_ranges - 1].hi = W.members[a].hi;
        } else {
          W.coalesced[n_ranges++] = W.members[a];
        }
      }
      fit_tls(W.coalesced, n_ranges, &fit);
    }
    if (fit.n_points < p->min_points || fit_length(&fit) < p->min_length) {
      if (stats) stats->support++;
      continue;
    }
    double rms = 0.0, fill = INFINITY;
    if (p->quality_gates) {
      rms = perp_rms(W.coalesced, n_ranges, fit.rho, fit.alpha);
      if (rms > p->max_rms_m) {
        if (stats) stats->rms++;
        continue;
      }
      fill = fill_ratio(&fit, n, p);
      if (fill < p->min_fill) {
        if (stats) stats->fill++;
        continue;
      }
    }
    if (n_out >= max_out) return EL_ERR_OVERFLOW;
    el_segment *o = &out[n_out++];
    o->rho = fit.rho;
    o->alpha = fit.alpha;
    o->cov00 = fit.cov00;
    o->cov01 = fit.cov01;
    o->cov11 = fit.cov11;
    o->p0x = fit.p0x;
    o->p0y = fit.p0y;
    o->p1x = fit.p1x;
    o->p1y = fit.p1y;
    o->n_points = fit.n_points;
    o->rms = rms;
    o->fill = fill;
  }
  if (stats) stats->kept = n_out;
  return n_out;
}
