//! CPython's float arithmetic and text forms that hard_stop.py's outputs go
//! through, so the Rust node's /safety/hard_stop JSON and log lines are the
//! Python's byte for byte (tools/rust_parity/hard_stop checks it, including a
//! sweep of these functions against CPython itself):
//! - [`degrees`]: `math.degrees(x)` = `x * (180.0 / pi)` (mathmodule.c radToDeg);
//! - [`py_round`]: `round(x, n)`: the correctly rounded decimal string with n
//!   digits after the point (round-half-even on the exact binary value), read
//!   back as a double (floatobject.c double_round); non-finite x unchanged;
//! - [`py_repr`]: `repr(float)`: the shortest round-trip digits, fixed
//!   notation for a decimal exponent in [-4, 16), `e+XX` / `e-XX` otherwise;
//! - [`json_float`]: `json.dumps`'s float: repr, or NaN / Infinity / -Infinity;
//! - [`py_fixed`]: `format(x, '.Nf')` / `'+.Nf'` (f-strings in log lines).
//!
//! Rust's `{:.N}` is exact and correctly rounded (ties to even) and Rust's
//! float parsing is correctly rounded, like David Gay's dtoa that CPython
//! uses. Rust's shortest form (`{:e}`) differs in one case: when the exact
//! value lies exactly halfway between two shortest candidates (e.g.
//! 0.65464019775390625, 17 digits, between ...062 and ...063), Rust takes the
//! upper one, CPython (dtoa mode 0, round-half-even) the even one. [`shortest`]
//! corrects that (found by the parity sweep, 2026-10-23).

/// `math.degrees(x)`.
pub fn degrees(x: f64) -> f64 {
    x * (180.0 / std::f64::consts::PI)
}

/// `round(x, ndigits)` for ndigits >= 0.
pub fn py_round(x: f64, ndigits: usize) -> f64 {
    if !x.is_finite() {
        return x;
    }
    format!("{x:.ndigits$}").parse::<f64>().unwrap_or(x)
}

/// (negative, digits, decpt) of Rust's `{:e}` / `{:.N e}` text.
fn split_exp(s: &str) -> (bool, String, i32) {
    let (neg, s) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let (mantissa, exp) = s.split_once('e').unwrap_or((s, "0"));
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();
    let exp: i32 = exp.parse().unwrap_or(0);
    (neg, digits, exp + 1)
}

/// `digits` (an integer as a decimal string) plus one: ("124", false) from
/// "123", ("100", true) from "99" (the carry adds a digit).
fn increment(digits: &str) -> (String, bool) {
    let mut d: Vec<u8> = digits.bytes().collect();
    for b in d.iter_mut().rev() {
        if *b == b'9' {
            *b = b'0';
        } else {
            *b += 1;
            return (String::from_utf8_lossy(&d).into_owned(), false);
        }
    }
    let mut out = String::from("1");
    out.push_str(&String::from_utf8_lossy(&d));
    (out, true)
}

fn parses_to(neg: bool, digits: &str, decpt: i32, x: f64) -> bool {
    let sign = if neg { "-" } else { "" };
    format!("{sign}0.{digits}e{decpt}")
        .parse::<f64>()
        .is_ok_and(|v| v.to_bits() == x.to_bits())
}

/// The shortest round-trip digits of a finite x, as CPython chooses them:
/// (negative, digits, decpt), x = 0.d1d2... * 10^decpt (dtoa's convention).
/// Zero is ("0", 1).
fn shortest(x: f64) -> (bool, String, i32) {
    let (neg, digits, decpt) = split_exp(&format!("{x:e}"));
    let n = digits.len();
    // An exact tie between the n-digit candidates below and above x: x's
    // exact expansion has n + 1 significant digits, the last one a 5.
    let (_, exact, exact_decpt) = split_exp(&format!("{:.*e}", n + 24, x));
    let tail = exact.as_bytes();
    let is_tie = tail.get(n) == Some(&b'5') && tail.iter().skip(n + 1).all(|&b| b == b'0');
    if !is_tie {
        return (neg, digits, decpt);
    }
    let low = &exact[..n];
    let (high, carried) = increment(low);
    let (high, high_decpt) = if carried {
        (high.trim_end_matches('0').to_owned(), exact_decpt + 1)
    } else {
        (high, exact_decpt)
    };
    let low_is_even = low.as_bytes().last().is_some_and(|&b| (b - b'0') % 2 == 0);
    let (even, even_decpt) = if low_is_even {
        (low.trim_end_matches('0').to_owned(), exact_decpt)
    } else {
        (high, high_decpt)
    };
    if !even.is_empty() && parses_to(neg, &even, even_decpt, x) {
        (neg, even, even_decpt)
    } else {
        (neg, digits, decpt)
    }
}

/// `repr(x)` (`float.__repr__`).
pub fn py_repr(x: f64) -> String {
    if x.is_nan() {
        return "nan".to_owned();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf" } else { "-inf" }.to_owned();
    }
    let (neg, digits, decpt) = shortest(x);
    let sign = if neg { "-" } else { "" };
    let n = digits.len() as i32;
    if decpt <= -4 || decpt > 16 {
        let (first, rest) = digits.split_at(1);
        let exp = decpt - 1;
        let frac = if rest.is_empty() {
            String::new()
        } else {
            format!(".{rest}")
        };
        let esign = if exp < 0 { '-' } else { '+' };
        return format!("{sign}{first}{frac}e{esign}{:02}", exp.unsigned_abs());
    }
    if decpt <= 0 {
        let zeros = "0".repeat(decpt.unsigned_abs() as usize);
        format!("{sign}0.{zeros}{digits}")
    } else if decpt >= n {
        let zeros = "0".repeat((decpt - n) as usize);
        format!("{sign}{digits}{zeros}.0")
    } else {
        let (int, frac) = digits.split_at(decpt as usize);
        format!("{sign}{int}.{frac}")
    }
}

/// A float as `json.dumps` writes it (allow_nan, the default).
pub fn json_float(x: f64) -> String {
    if x.is_nan() {
        "NaN".to_owned()
    } else if x.is_infinite() {
        if x > 0.0 { "Infinity" } else { "-Infinity" }.to_owned()
    } else {
        py_repr(x)
    }
}

/// `format(x, '.{prec}f')`, or `'+.{prec}f'` with `plus`.
pub fn py_fixed(x: f64, prec: usize, plus: bool) -> String {
    let lead = if plus { "+" } else { "" };
    if x.is_nan() {
        // CPython never shows a NaN's sign bit.
        return format!("{lead}nan");
    }
    if x.is_infinite() {
        return if x > 0.0 {
            format!("{lead}inf")
        } else {
            "-inf".to_owned()
        };
    }
    if plus {
        format!("{x:+.prec$}")
    } else {
        format!("{x:.prec$}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repr_matches_cpython() {
        // (value, repr) pairs from CPython 3.12.
        for (x, want) in [
            (0.0, "0.0"),
            (-0.0, "-0.0"),
            (0.5, "0.5"),
            (1.0, "1.0"),
            (0.17, "0.17"),
            (-30.0, "-30.0"),
            (0.1 + 0.2, "0.30000000000000004"),
            (1e16, "1e+16"),
            (1e15, "1000000000000000.0"),
            (123456789012345680.0, "1.2345678901234568e+17"),
            (0.0001, "0.0001"),
            (0.00001, "1e-05"),
            (1.5e-7, "1.5e-07"),
            (1e100, "1e+100"),
            (5e-324, "5e-324"),
            (f64::MAX, "1.7976931348623157e+308"),
            (-123.456, "-123.456"),
            // Exact ties between two shortest candidates: CPython takes the even one.
            (85805.0 / 131072.0, "0.6546401977539062"),
            (207221.0 / 131072.0, "1.5809707641601562"),
            (662393.0 / 65536.0, "10.107315063476562"),
            (f64::NAN, "nan"),
            (f64::NEG_INFINITY, "-inf"),
        ] {
            assert_eq!(py_repr(x), want, "{x:e}");
        }
    }

    #[test]
    fn round_matches_cpython_on_ties_and_signs() {
        // round(0.03125, 4) == 0.0312 (exact tie, half-even); round(-0.0001, 3) == -0.0.
        assert_eq!(py_round(0.03125, 4), 0.0312);
        assert_eq!(py_round(0.125, 2), 0.12);
        assert_eq!(py_round(0.375, 2), 0.38);
        let z = py_round(-0.0001, 3);
        assert!(z == 0.0 && z.is_sign_negative());
        assert_eq!(py_round(0.17000000178813934, 4), 0.17);
        assert!(py_round(f64::NAN, 2).is_nan());
        assert_eq!(py_round(f64::INFINITY, 4), f64::INFINITY);
    }

    #[test]
    fn json_and_fixed_match_cpython() {
        assert_eq!(json_float(f64::NAN), "NaN");
        assert_eq!(json_float(f64::NEG_INFINITY), "-Infinity");
        assert_eq!(json_float(0.5), "0.5");
        assert_eq!(py_fixed(0.18, 3, false), "0.180");
        assert_eq!(py_fixed(-0.0, 3, true), "-0.000");
        assert_eq!(py_fixed(0.0, 3, true), "+0.000");
        assert_eq!(py_fixed(-0.0001, 3, true), "-0.000");
        assert_eq!(py_fixed(f64::NAN, 1, true), "+nan");
        assert_eq!(py_fixed(-f64::NAN, 3, false), "nan");
        assert_eq!(py_fixed(f64::INFINITY, 3, true), "+inf");
        assert_eq!(py_fixed(f64::NEG_INFINITY, 2, false), "-inf");
        assert_eq!(py_fixed(0.125, 2, false), "0.12");
    }

    #[test]
    fn degrees_is_cpython_s() {
        assert_eq!(degrees(std::f64::consts::PI), 180.0);
        assert_eq!(degrees(1.0), 1.0 * (180.0 / std::f64::consts::PI));
    }
}
