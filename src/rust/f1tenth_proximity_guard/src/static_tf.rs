//! Poses of fixed frames in base_link, from /tf_static alone.
//!
//! The Rust counterpart of f1tenth_behavior/behaviours/static_frames.py
//! (`StaticFramePoses`): a tree of static edges, a lookup that composes the
//! chain between two frames, and (x, y, yaw) of the result with the same yaw
//! formula. Like the Python, a pose is cached the first time it resolves.

use std::collections::HashMap;

use crate::decision::LaserPose;

/// A rigid transform: translation and unit quaternion (x, y, z, w).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub t: [f64; 3],
    pub q: [f64; 4],
}

impl Transform {
    pub const IDENTITY: Transform = Transform {
        t: [0.0; 3],
        q: [0.0, 0.0, 0.0, 1.0],
    };

    fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let [qx, qy, qz, qw] = self.q;
        // v' = v + 2w (q x v) + 2 q x (q x v)
        let c1 = [
            qy * v[2] - qz * v[1],
            qz * v[0] - qx * v[2],
            qx * v[1] - qy * v[0],
        ];
        let c2 = [
            qy * c1[2] - qz * c1[1],
            qz * c1[0] - qx * c1[2],
            qx * c1[1] - qy * c1[0],
        ];
        [
            v[0] + 2.0 * (qw * c1[0] + c2[0]),
            v[1] + 2.0 * (qw * c1[1] + c2[1]),
            v[2] + 2.0 * (qw * c1[2] + c2[2]),
        ]
    }

    /// self * other: apply `other`, then `self`.
    pub fn compose(&self, other: &Transform) -> Transform {
        let [ax, ay, az, aw] = self.q;
        let [bx, by, bz, bw] = other.q;
        let q = [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ];
        let r = self.rotate(other.t);
        Transform {
            t: [self.t[0] + r[0], self.t[1] + r[1], self.t[2] + r[2]],
            q,
        }
    }

    pub fn inverse(&self) -> Transform {
        let [x, y, z, w] = self.q;
        let inv = Transform {
            t: [0.0; 3],
            q: [-x, -y, -z, w],
        };
        let r = inv.rotate(self.t);
        Transform {
            t: [-r[0], -r[1], -r[2]],
            q: inv.q,
        }
    }

    /// (x, y, yaw), yaw by static_frames.py's formula.
    pub fn planar(&self) -> LaserPose {
        let [x, y, z, w] = self.q;
        LaserPose {
            x: self.t[0],
            y: self.t[1],
            yaw: (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
        }
    }
}

/// Static edges child -> (parent, parent_T_child).
#[derive(Default, Debug)]
pub struct StaticTfTree {
    edges: HashMap<String, (String, Transform)>,
    cache: HashMap<String, LaserPose>,
}

/// Longest chain followed before giving up (a cycle in a malformed /tf_static).
const MAX_DEPTH: usize = 64;

fn strip_slash(frame: &str) -> &str {
    frame.strip_prefix('/').unwrap_or(frame)
}

impl StaticTfTree {
    pub fn insert(&mut self, parent: &str, child: &str, tf: Transform) {
        self.edges.insert(
            strip_slash(child).to_owned(),
            (strip_slash(parent).to_owned(), tf),
        );
    }

    /// root_T_frame and the chain of frames from `frame` up to its root.
    fn to_root(&self, frame: &str) -> Option<Vec<(String, Transform)>> {
        let mut chain = vec![(frame.to_owned(), Transform::IDENTITY)];
        let mut acc = Transform::IDENTITY;
        let mut current = frame.to_owned();
        for _ in 0..MAX_DEPTH {
            match self.edges.get(&current) {
                None => return Some(chain),
                Some((parent, tf)) => {
                    acc = tf.compose(&acc);
                    current = parent.clone();
                    chain.push((current.clone(), acc));
                }
            }
        }
        None
    }

    /// target_T_source, or None if the two frames are not connected.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Transform> {
        let (target, source) = (strip_slash(target), strip_slash(source));
        let up_source = self.to_root(source)?;
        let up_target = self.to_root(target)?;
        // The first ancestor of `source` that is also an ancestor of `target`.
        for (ancestor, ancestor_t_source) in &up_source {
            if let Some((_, ancestor_t_target)) = up_target.iter().find(|(f, _)| f == ancestor) {
                return Some(ancestor_t_target.inverse().compose(ancestor_t_source));
            }
        }
        None
    }

    /// (x, y, yaw) of `frame` in `base`, cached once resolved (as the Python).
    pub fn pose(&mut self, base: &str, frame: &str) -> Option<LaserPose> {
        if frame.is_empty() {
            return None;
        }
        if let Some(p) = self.cache.get(frame) {
            return Some(*p);
        }
        let pose = self.lookup(base, frame)?.planar();
        self.cache.insert(frame.to_owned(), pose);
        Some(pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_q(yaw: f64) -> [f64; 4] {
        [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()]
    }

    #[test]
    fn direct_child() {
        let mut tree = StaticTfTree::default();
        tree.insert(
            "base_link",
            "laser",
            Transform {
                t: [0.305, 0.0, 0.19],
                q: [0.0, 0.0, 0.0, 1.0],
            },
        );
        let p = tree.pose("base_link", "laser");
        assert_eq!(
            p,
            Some(LaserPose {
                x: 0.305,
                y: 0.0,
                yaw: 0.0
            })
        );
        assert_eq!(tree.pose("base_link", "nope"), None);
        assert_eq!(tree.pose("base_link", ""), None);
    }

    #[test]
    fn chain_and_inverse() {
        let mut tree = StaticTfTree::default();
        tree.insert(
            "base_link",
            "mount",
            Transform {
                t: [0.2, 0.1, 0.0],
                q: yaw_q(std::f64::consts::FRAC_PI_2),
            },
        );
        tree.insert(
            "mount",
            "/laser",
            Transform {
                t: [0.1, 0.0, 0.0],
                q: yaw_q(0.0),
            },
        );
        // mount rotated +90 deg: its x axis is base_link's y.
        let p = tree
            .pose("base_link", "laser")
            .unwrap_or(LaserPose::FALLBACK);
        assert!((p.x - 0.2).abs() < 1e-12);
        assert!((p.y - 0.2).abs() < 1e-12);
        assert!((p.yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        // And back: base_link in laser.
        let inv = tree
            .lookup("laser", "base_link")
            .map(|t| t.planar())
            .unwrap_or(LaserPose::FALLBACK);
        assert!((inv.yaw + std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn disconnected_and_cyclic() {
        let mut tree = StaticTfTree::default();
        tree.insert("a", "b", Transform::IDENTITY);
        tree.insert("c", "d", Transform::IDENTITY);
        assert!(tree.lookup("a", "d").is_none());
        tree.insert("b", "a", Transform::IDENTITY); // a <-> b cycle
        assert!(tree.lookup("a", "c").is_none());
    }
}
