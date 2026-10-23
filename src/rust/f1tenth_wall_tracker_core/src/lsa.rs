//! `scipy.optimize.linear_sum_assignment`, scipy 1.11.4, verbatim.
//!
//! scipy/optimize/rectangular_lsap/rectangular_lsap.cpp: Crouse's
//! shortest-augmenting-path variant of the Jonker-Volgenant algorithm. The
//! tie-breaking is part of the result and is reproduced exactly:
//!
//! - a tall matrix (more rows than columns) is solved transposed, and the
//!   pairs are then returned in column order (argsort of col4row);
//! - `remaining` is filled in REVERSE column order (a constant matrix gives
//!   the identity, scipy #11602);
//! - among the columns of equal lowest reduced cost the one whose `row4col`
//!   is free (-1) wins, so the search ends as early as possible;
//! - `r = minVal + cost - u[i] - v[j]`, left to right, in double.
//!
//! Inputs with NaN or -inf are invalid (scipy raises ValueError "matrix
//! contains invalid numeric entries"), an infeasible matrix (a row with no
//! finite path) is [`LsaError::Infeasible`] ("cost matrix is infeasible").

/// Why a matrix has no assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LsaError {
    Invalid,
    Infeasible,
}

#[allow(clippy::too_many_arguments)]
fn augmenting_path(
    nc: usize,
    cost: &[f64],
    u: &[f64],
    v: &[f64],
    path: &mut [isize],
    row4col: &[isize],
    shortest_path_costs: &mut [f64],
    mut i: usize,
    sr: &mut [bool],
    sc: &mut [bool],
    remaining: &mut [usize],
    p_min_val: &mut f64,
) -> isize {
    let mut min_val = 0.0_f64;
    let mut num_remaining = nc;
    for (it, r) in remaining.iter_mut().enumerate().take(nc) {
        *r = nc - it - 1;
    }
    sr.fill(false);
    sc.fill(false);
    shortest_path_costs.fill(f64::INFINITY);

    let mut sink: isize = -1;
    while sink == -1 {
        let mut index: isize = -1;
        let mut lowest = f64::INFINITY;
        sr[i] = true;
        for (it, &j) in remaining.iter().enumerate().take(num_remaining) {
            let r = min_val + cost[i * nc + j] - u[i] - v[j];
            if r < shortest_path_costs[j] {
                path[j] = i as isize;
                shortest_path_costs[j] = r;
            }
            if shortest_path_costs[j] < lowest
                || (shortest_path_costs[j] == lowest && row4col[j] == -1)
            {
                lowest = shortest_path_costs[j];
                index = it as isize;
            }
        }
        min_val = lowest;
        if min_val == f64::INFINITY || index < 0 {
            return -1;
        }
        let index = index as usize;
        let j = remaining[index];
        if row4col[j] == -1 {
            sink = j as isize;
        } else {
            i = row4col[j] as usize;
        }
        sc[j] = true;
        num_remaining -= 1;
        remaining[index] = remaining[num_remaining];
    }
    *p_min_val = min_val;
    sink
}

/// `linear_sum_assignment(cost)` for a row-major `nr x nc` matrix:
/// `(row_ind, col_ind)`, `min(nr, nc)` pairs.
pub fn linear_sum_assignment(
    nr: usize,
    nc: usize,
    cost: &[f64],
) -> Result<(Vec<usize>, Vec<usize>), LsaError> {
    if nr == 0 || nc == 0 {
        return Ok((Vec::new(), Vec::new()));
    }
    let transpose = nc < nr;
    let (nr, nc, cost): (usize, usize, Vec<f64>) = if transpose {
        let mut t = vec![0.0; nr * nc];
        for i in 0..nr {
            for j in 0..nc {
                t[j * nr + i] = cost[i * nc + j];
            }
        }
        (nc, nr, t)
    } else {
        (nr, nc, cost.to_vec())
    };
    if cost.iter().any(|&c| c.is_nan() || c == f64::NEG_INFINITY) {
        return Err(LsaError::Invalid);
    }

    let mut u = vec![0.0_f64; nr];
    let mut v = vec![0.0_f64; nc];
    let mut shortest_path_costs = vec![0.0_f64; nc];
    let mut path = vec![-1_isize; nc];
    let mut col4row = vec![-1_isize; nr];
    let mut row4col = vec![-1_isize; nc];
    let mut sr = vec![false; nr];
    let mut sc = vec![false; nc];
    let mut remaining = vec![0_usize; nc];

    for cur_row in 0..nr {
        let mut min_val = 0.0;
        let sink = augmenting_path(
            nc,
            &cost,
            &u,
            &v,
            &mut path,
            &row4col,
            &mut shortest_path_costs,
            cur_row,
            &mut sr,
            &mut sc,
            &mut remaining,
            &mut min_val,
        );
        if sink < 0 {
            return Err(LsaError::Infeasible);
        }
        u[cur_row] += min_val;
        for i in 0..nr {
            if sr[i] && i != cur_row {
                let c4r = col4row[i] as usize;
                u[i] += min_val - shortest_path_costs[c4r];
            }
        }
        for j in 0..nc {
            if sc[j] {
                v[j] -= min_val - shortest_path_costs[j];
            }
        }
        let mut j = sink;
        loop {
            let i = path[j as usize];
            row4col[j as usize] = i;
            std::mem::swap(&mut col4row[i as usize], &mut j);
            if i as usize == cur_row {
                break;
            }
        }
    }

    if transpose {
        // argsort of col4row (its values are distinct column indices of the
        // transposed matrix, i.e. original rows): std::sort cannot tie.
        let mut index: Vec<usize> = (0..nr).collect();
        index.sort_by_key(|&k| col4row[k]);
        let a: Vec<usize> = index.iter().map(|&k| col4row[k] as usize).collect();
        let b: Vec<usize> = index;
        Ok((a, b))
    } else {
        Ok((
            (0..nr).collect(),
            col4row.iter().map(|&c| c as usize).collect(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_assignment() {
        let cost = [4.0, 1.0, 3.0, 2.0, 0.0, 5.0, 3.0, 2.0, 2.0];
        let (r, c) = linear_sum_assignment(3, 3, &cost).unwrap();
        assert_eq!(r, vec![0, 1, 2]);
        assert_eq!(c, vec![1, 0, 2]);
    }

    #[test]
    fn constant_matrix_gives_the_identity() {
        let (r, c) = linear_sum_assignment(3, 3, &[1.0; 9]).unwrap();
        assert_eq!(r, vec![0, 1, 2]);
        assert_eq!(c, vec![0, 1, 2]);
    }

    #[test]
    fn tall_and_wide() {
        // 3 x 2: solved transposed, the pairs come back in row order.
        let cost = [10.0, 1.0, 1.0, 10.0, 5.0, 5.0];
        let (r, c) = linear_sum_assignment(3, 2, &cost).unwrap();
        assert_eq!((r, c), (vec![0, 1], vec![1, 0]));
        let cost = [5.0, 5.0, 10.0, 1.0, 1.0, 10.0];
        let (r, c) = linear_sum_assignment(3, 2, &cost).unwrap();
        assert_eq!((r, c), (vec![1, 2], vec![1, 0]));
        let wide = [10.0, 1.0, 5.0, 1.0, 10.0, 5.0];
        let (r, c) = linear_sum_assignment(2, 3, &wide).unwrap();
        assert_eq!((r, c), (vec![0, 1], vec![1, 0]));
    }

    #[test]
    fn invalid_and_infeasible() {
        assert_eq!(
            linear_sum_assignment(1, 1, &[f64::NAN]),
            Err(LsaError::Invalid)
        );
        assert_eq!(
            linear_sum_assignment(2, 2, &[f64::INFINITY, f64::INFINITY, 1.0, 2.0]),
            Err(LsaError::Infeasible)
        );
    }
}
