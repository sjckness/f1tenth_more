/* extract_lines.h -- the C port of corridor_perception.extraction.extract_segments.
 *
 * ONE CALL PER SCAN, NO ALLOCATION. The caller owns the output buffer; every
 * intermediate lives in a static workspace sized for EL_MAX_BEAMS beams, so a
 * call cannot fail for want of memory and cannot fragment the heap of a node
 * that runs at the scan rate.
 *
 * THIS FILE IS A PORT, NOT A DESIGN. Every threshold, every tie-break and the
 * ORDER of the output follow extraction.py exactly, including the things that
 * look accidental: the IEPF stack is LIFO with the left half pushed last, the
 * merge visits candidate buckets in the order -1, 0, +1 and unions into the
 * root of the LOWER index, and the clusters come out in the order their roots
 * are first met scanning i upwards. Those choices decide which of several
 * equally valid segment orders comes back, and the equivalence test compares
 * segment by segment. If this file and the Python disagree, this file is
 * wrong.
 *
 * NOT THREAD SAFE, deliberately: one static workspace, one caller, which is
 * what the node is. A second caller needs its own process or a second
 * workspace, and the Python wrapper documents that it holds the GIL.
 */
#ifndef CORRIDOR_PERCEPTION_C_EXTRACT_LINES_H
#define CORRIDOR_PERCEPTION_C_EXTRACT_LINES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EL_MAX_BEAMS 2048
#define EL_MAX_SEGMENTS 256

/* extraction.py's ExtractionParams, in the same units and the same order.
 * Doubles, because the Python computes in doubles and this must agree with it
 * to 1e-4 on quantities that are metres. */
typedef struct {
  double sigma_0;            /* range noise = sigma_0 + sigma_r * range */
  double sigma_r;
  double lambda_abd;         /* worst incidence angle believed [rad] */
  double abd_sigma_mult;
  double split_dist;         /* IEPF perpendicular split threshold [m] */
  double merge_alpha;        /* collinear merge angle tolerance [rad] */
  double merge_rho;          /* ... and its offset tolerance [m] */
  double merge_gap;          /* max end-to-end gap to consider merging [m] */
  int32_t min_points;        /* support filter */
  double min_length;         /* [m] */
  double max_point_gap;      /* [m] */
  int32_t quality_gates;     /* 0/1 */
  int32_t seed_base_points;  /* N(r) floor */
  int32_t seed_max_points;   /* N(r) ceiling */
  double seed_range_m;       /* where N(r) starts to grow */
  double seed_range_step_m;  /* one more return per this much range */
  double max_rms_m;          /* perpendicular residual ceiling */
  double min_fill;           /* of the span's unoccluded beams */
  double occlusion_margin_m; /* nearer than this IS an occluder */
  double min_incidence;      /* |n . v| below this has no s_k */
} el_params;

/* One reported segment. The frame is the sensor's, as in extraction.py. */
typedef struct {
  double rho;
  double alpha;
  double cov00, cov01, cov11; /* (rho, alpha) covariance, origin-anchored */
  double p0x, p0y;            /* the support's endpoints */
  double p1x, p1y;
  int32_t n_points;
  double rms;  /* perpendicular residual rms, for reporting and the gate */
  double fill; /* fill ratio, same */
} el_segment;

/* What each stage dropped, mirroring extract_segments' `stats` dict. */
typedef struct {
  int32_t seed;
  int32_t rms;
  int32_t fill;
  int32_t support;
  int32_t kept;
} el_stats;

/* Fill `p` with extraction.py's own ExtractionParams defaults. */
void el_default_params(el_params *p);

/* Segments of one scan.
 *
 * ranges: n float32 ranges THAT HAVE BEEN THROUGH clean_ranges() -- a
 *   no-return is +inf here, never the driver's 65.533 m sentinel. The caller
 *   is responsible for that; this function cannot tell.
 * angles are angle_min + i * angle_increment, computed here in the same order
 *   as numpy does it so the two agree bit for bit.
 * out holds at most max_out segments; the return value is how many were
 *   written, or a negative el_error on bad input.
 * stats may be NULL.
 */
int32_t el_extract(const float *ranges, int32_t n, double angle_min,
                   double angle_increment, const el_params *p,
                   el_segment *out, int32_t max_out, el_stats *stats);

enum el_error {
  EL_ERR_NULL = -1,      /* ranges, p or out is NULL */
  EL_ERR_TOO_MANY = -2,  /* n > EL_MAX_BEAMS or n < 0 */
  EL_ERR_OVERFLOW = -3   /* the scan produced more intermediates than the
                          * workspace holds; the caller should fall back */
};

/* "1" -- so a wrapper can prove it loaded the library it thinks it did. */
const char *el_version(void);

#ifdef __cplusplus
}
#endif
#endif /* CORRIDOR_PERCEPTION_C_EXTRACT_LINES_H */
