//! The Python `message_filters.ApproximateTimeSynchronizer` (ROS 2 Jazzy,
//! message_filters/__init__.py) that detection_3d_node runs, operation for
//! operation. It is not the C++ ApproximateTime policy.
//!
//! Per input a queue `{stamp_ns: msg}` in insertion order (a message whose
//! stamp is already queued replaces that value in place), trimmed after every
//! insert to `queue_size` by deleting the smallest stamp. Then, for every
//! OTHER input, the queued stamps with `|s - t| <= slop` (Duration `>` skips),
//! sorted by that delta (stable: insertion order on ties); if one input has
//! none, nothing fires. `itertools.product` over those lists (the first input
//! varies slowest), the new stamp inserted at its own index; the first
//! combination with `max - min < slop` (strict) whose stamps are all still
//! queued (the new one may have been trimmed) fires, its entries are deleted
//! from every queue, and the search stops.
//!
//! `slop` becomes `Duration(seconds=slop)`: `int(slop * 1e9)` nanoseconds.

/// A queued message: (stamp in nanoseconds, the message).
type Queued<T> = (i64, T);

pub struct ApproxSync<T> {
    queues: Vec<Vec<Queued<T>>>,
    queue_size: usize,
    slop_ns: i64,
}

/// `int(seconds * S_TO_NS)` for a finite `seconds`.
pub fn duration_ns(seconds: f64) -> i64 {
    // `as` truncates toward zero like int(); finite values only (checked by
    // the caller), saturating far beyond any configurable slop.
    (seconds * 1e9) as i64
}

/// `Time.from_msg(stamp).nanoseconds`.
#[inline]
pub fn stamp_ns(sec: i32, nanosec: u32) -> i64 {
    i64::from(sec) * 1_000_000_000 + i64::from(nanosec)
}

impl<T: Clone> ApproxSync<T> {
    /// `inputs` queues, `queue_size` >= 0, a finite slop [s].
    pub fn new(inputs: usize, queue_size: usize, slop_s: f64) -> Self {
        ApproxSync {
            queues: (0..inputs).map(|_| Vec::new()).collect(),
            queue_size,
            slop_ns: duration_ns(slop_s),
        }
    }

    pub fn inputs(&self) -> usize {
        self.queues.len()
    }

    /// Queued stamps of one input, in insertion order (tests, parity).
    pub fn queued(&self, input: usize) -> Vec<i64> {
        self.queues
            .get(input)
            .map(|q| q.iter().map(|e| e.0).collect())
            .unwrap_or_default()
    }

    /// `add(msg, my_queue, my_queue_index)`: the matched set, one message per
    /// input in input order, when this arrival completes one.
    pub fn add(&mut self, input: usize, stamp: i64, msg: T) -> Option<Vec<T>> {
        let queue_size = self.queue_size;
        {
            let q = self.queues.get_mut(input)?;
            if let Some(e) = q.iter_mut().find(|e| e.0 == stamp) {
                e.1 = msg;
            } else {
                q.push((stamp, msg));
            }
            while q.len() > queue_size {
                let Some(k) = q
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, e)| e.0)
                    .map(|(k, _)| k)
                else {
                    break;
                };
                q.remove(k);
            }
        }

        // Candidate stamps per other input, nearest first.
        let mut lists: Vec<Vec<i64>> = Vec::with_capacity(self.queues.len());
        for (j, q) in self.queues.iter().enumerate() {
            if j == input {
                continue;
            }
            let mut c: Vec<(i64, i64)> = q
                .iter()
                .filter_map(|e| {
                    let delta = (e.0 - stamp).abs();
                    (delta <= self.slop_ns).then_some((e.0, delta))
                })
                .collect();
            if c.is_empty() {
                return None;
            }
            c.sort_by_key(|x| x.1);
            lists.push(c.into_iter().map(|x| x.0).collect());
        }

        // itertools.product, the last list varying fastest.
        let mut idx = vec![0_usize; lists.len()];
        loop {
            let mut vv: Vec<i64> = idx.iter().zip(&lists).map(|(&k, l)| l[k]).collect();
            vv.insert(input, stamp);
            if self.matches(&vv) {
                let mut out = Vec::with_capacity(vv.len());
                for (q, t) in self.queues.iter_mut().zip(&vv) {
                    if let Some(k) = q.iter().position(|e| e.0 == *t) {
                        out.push(q.remove(k).1);
                    }
                }
                return Some(out);
            }
            // Next combination.
            let mut d = lists.len();
            loop {
                if d == 0 {
                    return None;
                }
                d -= 1;
                idx[d] += 1;
                if idx[d] < lists[d].len() {
                    break;
                }
                idx[d] = 0;
            }
        }
    }

    fn matches(&self, vv: &[i64]) -> bool {
        let (Some(&max), Some(&min)) = (vv.iter().max(), vv.iter().min()) else {
            return false;
        };
        max - min < self.slop_ns
            && self
                .queues
                .iter()
                .zip(vv)
                .all(|(q, t)| q.iter().any(|e| e.0 == *t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;

    #[test]
    fn slop_is_python_duration() {
        assert_eq!(duration_ns(0.1), 100_000_000);
        assert_eq!(duration_ns(0.3), 300_000_000);
        assert_eq!(duration_ns(-0.1), -100_000_000);
    }

    #[test]
    fn pairs_the_nearest_and_removes_them() {
        let mut s: ApproxSync<&str> = ApproxSync::new(2, 15, 0.1);
        assert!(s.add(1, 1000 * MS, "d0").is_none());
        assert!(s.add(1, 1033 * MS, "d1").is_none());
        assert!(s.add(1, 1066 * MS, "d2").is_none());
        let got = s.add(0, 1040 * MS, "det");
        assert_eq!(got, Some(vec!["det", "d1"]));
        assert_eq!(s.queued(1), vec![1000 * MS, 1066 * MS]);
        assert!(s.queued(0).is_empty());
    }

    #[test]
    fn the_spread_must_be_strictly_inside_the_slop() {
        let mut s: ApproxSync<u8> = ApproxSync::new(2, 15, 0.1);
        s.add(1, 0, 1);
        // |delta| = slop passes the candidate filter (not >), but max - min
        // = slop is not < slop: nothing fires.
        assert!(s.add(0, 100 * MS, 2).is_none());
        assert!(s.add(0, 99 * MS, 3).is_some());
    }

    #[test]
    fn the_queue_drops_its_smallest_stamp_and_may_drop_the_new_one() {
        let mut s: ApproxSync<u8> = ApproxSync::new(2, 2, 0.025);
        assert!(s.add(1, 50 * MS, 1).is_none());
        assert!(s.add(1, 60 * MS, 2).is_none());
        assert!(s.add(0, 20 * MS, 9).is_none());
        // Older than both and the queue is full: trimmed at once, so the
        // presence check fails although input 0 has a candidate (20).
        assert!(s.add(1, 10 * MS, 3).is_none());
        assert_eq!(s.queued(1), vec![50 * MS, 60 * MS]);
        assert_eq!(s.queued(0), vec![20 * MS]);
    }

    #[test]
    fn a_repeated_stamp_replaces_in_place() {
        let mut s: ApproxSync<u8> = ApproxSync::new(2, 15, 0.1);
        s.add(1, 10 * MS, 1);
        s.add(1, 20 * MS, 2);
        s.add(1, 10 * MS, 3);
        assert_eq!(s.queued(1), vec![10 * MS, 20 * MS]);
        assert_eq!(s.add(0, 12 * MS, 0), Some(vec![0, 3]));
    }

    #[test]
    fn three_inputs_take_the_first_product_combination() {
        let mut s: ApproxSync<u8> = ApproxSync::new(3, 15, 0.1);
        s.add(1, 0, 10);
        s.add(1, 90 * MS, 11);
        s.add(2, 95 * MS, 20);
        // det at 40: depth candidates [0 (40), 90 (50)], mask [95 (55)];
        // (0, 40, 95) spans 95 < 100 and fires first.
        assert_eq!(s.add(0, 40 * MS, 1), Some(vec![1, 10, 20]));
    }
}
