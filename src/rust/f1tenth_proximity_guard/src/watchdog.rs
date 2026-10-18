//! The scan watchdog's verdict, without ROS.
//!
//! Which clock judges "the last /scan is too old":
//! - **Steady** (the car, `use_sim_time` false): the monotonic clock. A LiDAR or driver that
//!   stops publishing is caught after `scan_timeout`, whatever the system clock does.
//! - **Ros** (simulation, `use_sim_time` true): the node clock, i.e. /clock. When the simulator
//!   pauses or slows down, ROS time stops or slows with it, so a wall-clock gap during which less
//!   than `scan_timeout` of simulated time passed is not a missing scan: the car does not move in
//!   that time either. Such a gap is reported as [`Verdict::SimPause`] (counted, no stop).
//!
//! `no_scan_yet` is judged on the steady clock in both cases: before the first scan there is no
//! scan time to compare against, and a simulator that never sends a scan must still not drive.

use std::time::Duration;

/// The clock that judges staleness.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaleClock {
    Steady,
    Ros,
}

impl StaleClock {
    pub fn as_str(self) -> &'static str {
        match self {
            StaleClock::Steady => "steady",
            StaleClock::Ros => "ros",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// No scan since start (steady clock): stop.
    NoScanYet,
    /// The last scan is older than the timeout on the judging clock: stop.
    Stale,
    /// Sim only: the wall-clock gap exceeds the timeout but simulated time does not. No stop.
    SimPause,
    /// The last scan is recent enough.
    Fresh,
}

/// Ages of the last scan, measured when the watchdog fires.
#[derive(Clone, Copy, Debug)]
pub struct ScanAge {
    /// Steady-clock time since the last scan was received.
    pub steady: Duration,
    /// Node-clock time since then; `None` when the node clock went backwards (a /clock reset),
    /// which is treated as "just received".
    pub ros: Option<Duration>,
}

/// The watchdog verdict. `last` is `None` before the first scan.
pub fn verdict(clock: StaleClock, last: Option<ScanAge>, timeout: Duration) -> Verdict {
    let Some(age) = last else {
        // Before the first scan the guard stops at once (the watchdog's first tick), as before.
        return Verdict::NoScanYet;
    };
    match clock {
        StaleClock::Steady => {
            if age.steady > timeout {
                Verdict::Stale
            } else {
                Verdict::Fresh
            }
        }
        StaleClock::Ros => {
            let ros = age.ros.unwrap_or(Duration::ZERO);
            if ros > timeout {
                Verdict::Stale
            } else if age.steady > timeout {
                Verdict::SimPause
            } else {
                Verdict::Fresh
            }
        }
    }
}

/// Node-clock age from two node-clock readings in ns; `None` if the clock went backwards.
pub fn ros_age(now_ns: i64, last_ns: i64) -> Option<Duration> {
    u64::try_from(now_ns - last_ns)
        .ok()
        .map(Duration::from_nanos)
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: Duration = Duration::from_millis(200);

    fn ms(v: u64) -> Duration {
        Duration::from_millis(v)
    }

    fn age(steady: u64, ros: Option<u64>) -> Option<ScanAge> {
        Some(ScanAge {
            steady: ms(steady),
            ros: ros.map(ms),
        })
    }

    #[test]
    fn car_is_steady_only_and_unchanged() {
        // The ROS age is ignored on the car: only the steady age decides.
        assert_eq!(
            verdict(StaleClock::Steady, age(150, Some(900)), T),
            Verdict::Fresh
        );
        assert_eq!(
            verdict(StaleClock::Steady, age(201, Some(0)), T),
            Verdict::Stale
        );
        assert_eq!(
            verdict(StaleClock::Steady, age(200, None), T),
            Verdict::Fresh
        );
        assert_eq!(verdict(StaleClock::Steady, None, T), Verdict::NoScanYet);
    }

    #[test]
    fn sim_follows_ros_time() {
        // A 1 s wall gap with 50 ms of simulated time: a pause, not a stop.
        assert_eq!(
            verdict(StaleClock::Ros, age(1000, Some(50)), T),
            Verdict::SimPause
        );
        // Simulated time ran on without scans: stale.
        assert_eq!(
            verdict(StaleClock::Ros, age(1000, Some(250)), T),
            Verdict::Stale
        );
        assert_eq!(
            verdict(StaleClock::Ros, age(150, Some(150)), T),
            Verdict::Fresh
        );
        // Clock went backwards (reset): as just received.
        assert_eq!(verdict(StaleClock::Ros, age(100, None), T), Verdict::Fresh);
        assert_eq!(verdict(StaleClock::Ros, None, T), Verdict::NoScanYet);
    }

    #[test]
    fn ros_age_handles_backwards_clock() {
        assert_eq!(ros_age(1_000_000_000, 750_000_000), Some(ms(250)));
        assert_eq!(ros_age(5, 10), None);
    }
}
