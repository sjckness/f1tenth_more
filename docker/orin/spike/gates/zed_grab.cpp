// G3 (ZED): SDK version, device list, and - if a camera is attached - open it
// with the stack's settings (zed2i.yaml HD720 @ 60, zed2_perception.yaml depth
// PERFORMANCE) and grab N frames, retrieving the left image and the depth
// measure each frame as the wrapper does. Read-only: no settings are written.
#include <sl/Camera.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
  const int n = argc > 1 ? std::atoi(argv[1]) : 300;
  std::printf("sdk_version %s\n", sl::Camera::getSDKVersion().c_str());
  auto devs = sl::Camera::getDeviceList();
  std::printf("devices %zu\n", devs.size());
  for (auto& d : devs)
    std::printf("  id %d model %s serial %u state %s path %s\n", d.id,
                sl::toString(d.camera_model).c_str(), d.serial_number,
                sl::toString(d.camera_state).c_str(), d.path.c_str());
  if (devs.empty()) {
    std::printf("RESULT no_camera\n");
    return 3;
  }

  sl::Camera cam;
  sl::InitParameters ip;
  ip.camera_resolution = sl::RESOLUTION::HD720;
  ip.camera_fps = 60;
  ip.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  ip.coordinate_units = sl::UNIT::METER;
  ip.sdk_verbose = 1;
  auto t_open = std::chrono::steady_clock::now();
  sl::ERROR_CODE err = cam.open(ip);
  double open_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open).count();
  std::printf("open %s (%.2f s)\n", sl::toString(err).c_str(), open_s);
  if (err != sl::ERROR_CODE::SUCCESS) {
    std::printf("RESULT open_failed\n");
    return 4;
  }
  auto ci = cam.getCameraInformation();
  std::printf("camera %s serial %u fw %u sensors_fw %u res %dx%d fps %.1f\n",
              sl::toString(ci.camera_model).c_str(), ci.serial_number,
              ci.camera_configuration.firmware_version,
              ci.sensors_configuration.firmware_version,
              (int)ci.camera_configuration.resolution.width,
              (int)ci.camera_configuration.resolution.height, ci.camera_configuration.fps);

  sl::Mat left, depth;
  sl::SensorsData sensors;
  int ok = 0, fail = 0, depth_valid = 0, imu_ok = 0, imu_new = 0;
  unsigned long long last_imu_ts = 0;
  double depth_sum = 0.0, accel_norm_sum = 0.0;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    sl::ERROR_CODE g = cam.grab();
    if (g != sl::ERROR_CODE::SUCCESS) {
      ++fail;
      if (fail <= 5) std::printf("grab %d: %s\n", i, sl::toString(g).c_str());
      continue;
    }
    ++ok;
    cam.retrieveImage(left, sl::VIEW::LEFT);
    cam.retrieveMeasure(depth, sl::MEASURE::DEPTH);
    float d = 0.f;
    depth.getValue(depth.getWidth() / 2, depth.getHeight() / 2, &d);
    if (std::isfinite(d)) { depth_sum += d; ++depth_valid; }
    if (cam.getSensorsData(sensors, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS &&
        sensors.imu.is_available) {
      ++imu_ok;
      auto a = sensors.imu.linear_acceleration;
      accel_norm_sum += std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
      unsigned long long ts = sensors.imu.timestamp.getNanoseconds();
      if (ts != last_imu_ts) { ++imu_new; last_imu_ts = ts; }
      if (imu_ok == 1) {
        auto g = sensors.imu.angular_velocity;
        std::printf("imu first sample: accel %.3f %.3f %.3f m/s2 gyro %.3f %.3f %.3f deg/s\n",
                    a.x, a.y, a.z, g.x, g.y, g.z);
      }
    }
  }
  double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("frames_ok %d frames_failed %d elapsed_s %.2f fps %.2f\n", ok, fail, dt, ok / dt);
  std::printf("left %zux%zu depth %zux%zu center_depth_mean_m %.3f (valid %d)\n",
              left.getWidth(), left.getHeight(), depth.getWidth(), depth.getHeight(),
              depth_valid ? depth_sum / depth_valid : NAN, depth_valid);
  std::printf("imu reads_ok %d of %d, distinct_timestamps %d, accel_norm_mean %.3f m/s2\n",
              imu_ok, ok, imu_new, imu_ok ? accel_norm_sum / imu_ok : NAN);
  cam.close();
  const bool pass = (ok == n) && imu_ok == ok && imu_new > 1;
  std::printf("RESULT %s\n", pass ? "pass" : (ok != n ? "frames_missing" : "imu_missing"));
  return pass ? 0 : 5;
}
