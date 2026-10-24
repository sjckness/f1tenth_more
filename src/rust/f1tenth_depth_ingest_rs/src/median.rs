//! `float(np.median(v))` of a float32 vector, bit for bit (numpy 1.26).
//!
//! numpy (lib/function_base.py `_median`) partitions at the middle index (two
//! for an even count, plus the last element for its NaN check) and takes
//! `mean` of the middle slice in the input dtype:
//! - odd n: the middle order statistic, unchanged (`x / 1`);
//! - even n: `float32(float64(a + b) / 2)` with `a + b` summed in float32
//!   (`np.add.reduce` of two float32 values; the float32 / intp division
//!   promotes to float64 and `ret.dtype.type(...)` rounds back);
//! - any NaN: NaN (`_median_nancheck` returns the partition's last element,
//!   which is a NaN of the input).
//!
//! An order statistic is a value, not a position, so any selection algorithm
//! gives numpy's answer, with one exception: `-0.0` and `0.0` compare equal
//! in numpy, so with both present the sign of a zero result depends on
//! numpy's partition. detection_3d_node's medians only ever see values that
//! passed `isfinite & > 0`, where no such tie exists
//! (output/rust/parity_depth_ingest.md).

/// The median of `v` with numpy's semantics (see the module docs); `v` is
/// reordered. `None` for an empty slice (numpy warns and returns NaN; the
/// node never asks: it checks the count first).
pub fn median_f32(v: &mut [f32]) -> Option<f32> {
    let n = v.len();
    if n == 0 {
        return None;
    }
    if let Some(nan) = v.iter().copied().find(|x| x.is_nan()) {
        return Some(nan);
    }
    let mid = n / 2;
    let cmp = |a: &f32, b: &f32| a.total_cmp(b);
    v.select_nth_unstable_by(mid, cmp);
    let upper = v[mid];
    if n % 2 == 1 {
        return Some(upper);
    }
    // The (mid - 1)-th order statistic: the largest of the lower part.
    let lower = v[..mid]
        .iter()
        .copied()
        .max_by(|a, b| a.total_cmp(b))
        .unwrap_or(upper);
    let sum = lower + upper;
    Some((f64::from(sum) / 2.0) as f32)
}

/// [`median_f32`] of a copy, for callers that keep their order.
pub fn median_of(values: &[f32]) -> Option<f32> {
    let mut v = values.to_vec();
    median_f32(&mut v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sorted_median(values: &[f32]) -> f32 {
        let mut s = values.to_vec();
        s.sort_by(|a, b| a.total_cmp(b));
        let n = s.len();
        if n % 2 == 1 {
            s[n / 2]
        } else {
            ((f64::from(s[n / 2 - 1] + s[n / 2])) / 2.0) as f32
        }
    }

    #[test]
    fn odd_and_even_counts() {
        assert_eq!(median_of(&[3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median_of(&[4.0, 1.0, 3.0, 2.0]), Some(2.5));
        assert_eq!(median_of(&[1.0]), Some(1.0));
        assert_eq!(median_of(&[]), None);
    }

    #[test]
    fn the_even_mean_is_summed_in_float32() {
        // Halving is exact, so rounding the float32 sum and rounding the
        // exact mean agree everywhere except where the float32 sum
        // overflows, which numpy's does too.
        assert_eq!(median_of(&[f32::MAX, f32::MAX]), Some(f32::INFINITY));
        let a = 1.0_f32;
        let b = f32::from_bits(a.to_bits() + 1);
        assert_eq!(median_of(&[b, a]), Some(1.0));
    }

    #[test]
    fn nan_anywhere_is_nan_and_infinities_order() {
        assert!(median_of(&[1.0, f32::NAN, 2.0]).is_some_and(f32::is_nan));
        assert_eq!(
            median_of(&[f32::INFINITY, 1.0, f32::NEG_INFINITY]),
            Some(1.0)
        );
        assert!(median_of(&[f32::INFINITY, f32::NEG_INFINITY]).is_some_and(f32::is_nan));
        assert_eq!(
            median_of(&[f32::INFINITY, f32::INFINITY, 1.0, 2.0]),
            Some(f32::INFINITY)
        );
    }

    #[test]
    fn agrees_with_a_full_sort_on_many_shapes() {
        // A small LCG: deterministic, no dependency.
        let mut state = 0x2545_f491_4f6c_dd1d_u64;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (state >> 33) as u32
        };
        for n in 1..300 {
            let values: Vec<f32> = (0..n)
                .map(|_| {
                    let r = next();
                    // Many duplicates, a few distinct magnitudes.
                    0.2 + (r % 97) as f32 * 0.037
                })
                .collect();
            assert_eq!(
                median_of(&values).map(f32::to_bits),
                Some(sorted_median(&values).to_bits()),
                "n = {n}"
            );
        }
    }
}
