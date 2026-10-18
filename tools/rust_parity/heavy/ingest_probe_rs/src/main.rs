//! ingest_probe_rs KIND:TOPIC ... [--touch] [--seconds S]
//! KIND: image | info | cloud | scan. Prints one JSON line of message counts
//! and bytes per topic after S seconds (default 30).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, QoSProfile, SpinOptions};

#[derive(Default)]
struct Count {
    msgs: AtomicU64,
    bytes: AtomicU64,
    sum: AtomicU64,
}

fn touch(data: &[u8]) -> u64 {
    data.iter().map(|&b| u64::from(b)).sum()
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let touch_all = args.iter().any(|a| a == "--touch");
    let seconds = args
        .iter()
        .position(|a| a == "--seconds")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(30.0);
    let context = Context::default_from_env().map_err(|e| e.to_string())?;
    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("ingest_probe_rs")
        .map_err(|e| e.to_string())?;
    let mut subs: Vec<Box<dyn std::any::Any>> = Vec::new();
    let mut counts: Vec<(String, Arc<Count>)> = Vec::new();
    for spec in args.iter().filter(|a| a.contains(':')) {
        let Some((kind, topic)) = spec.split_once(':') else {
            continue;
        };
        let c = Arc::new(Count::default());
        counts.push((topic.to_owned(), Arc::clone(&c)));
        let qos = topic.qos(QoSProfile::sensor_data_default()).keep_last(2);
        let err = |e: rclrs::RclrsError| e.to_string();
        match kind {
            "image" => subs.push(Box::new(
                node.create_subscription::<sensor_msgs::msg::Image, _>(
                    qos,
                    move |m: sensor_msgs::msg::Image| {
                        c.msgs.fetch_add(1, Ordering::Relaxed);
                        c.bytes.fetch_add(m.data.len() as u64, Ordering::Relaxed);
                        if touch_all {
                            c.sum.fetch_add(touch(&m.data), Ordering::Relaxed);
                        }
                    },
                )
                .map_err(err)?,
            )),
            "cloud" => subs.push(Box::new(
                node.create_subscription::<sensor_msgs::msg::PointCloud2, _>(
                    qos,
                    move |m: sensor_msgs::msg::PointCloud2| {
                        c.msgs.fetch_add(1, Ordering::Relaxed);
                        c.bytes.fetch_add(m.data.len() as u64, Ordering::Relaxed);
                        if touch_all {
                            c.sum.fetch_add(touch(&m.data), Ordering::Relaxed);
                        }
                    },
                )
                .map_err(err)?,
            )),
            "info" => subs.push(Box::new(
                node.create_subscription::<sensor_msgs::msg::CameraInfo, _>(
                    qos,
                    move |_m: sensor_msgs::msg::CameraInfo| {
                        c.msgs.fetch_add(1, Ordering::Relaxed);
                    },
                )
                .map_err(err)?,
            )),
            "scan" => subs.push(Box::new(
                node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
                    qos,
                    move |m: sensor_msgs::msg::LaserScan| {
                        c.msgs.fetch_add(1, Ordering::Relaxed);
                        c.bytes
                            .fetch_add(4 * m.ranges.len() as u64, Ordering::Relaxed);
                    },
                )
                .map_err(err)?,
            )),
            other => return Err(format!("unknown kind {other}")),
        }
    }
    let t0 = Instant::now();
    let _ = executor.spin(SpinOptions::default().timeout(Duration::from_secs_f64(seconds)));
    let wall = t0.elapsed().as_secs_f64();
    let body: Vec<String> = counts
        .iter()
        .map(|(t, c)| {
            format!(
                "\"{t}\": {{\"msgs\": {}, \"bytes\": {}}}",
                c.msgs.load(Ordering::Relaxed),
                c.bytes.load(Ordering::Relaxed)
            )
        })
        .collect();
    println!(
        "{{\"wall_s\": {wall:.3}, \"topics\": {{{}}}}}",
        body.join(", ")
    );
    drop(subs);
    Ok(())
}
