// ZED check: SDK version, device list and, with a camera attached, open it and
// grab N frames, retrieving the left image and the depth measure each frame as
// the wrapper does; per region of interest the median depth over all frames.
//
// Runs on: the Orin host against the host SDK (native side, 4.2.5) and inside
// the container against the image's SDK (5.4.0); built on each side with
// build_zed_grab.sh. From the spike gate G3 (docker/orin/spike/gates on the
// jazzy line), extended with the depth ROIs and a JSON result.
// Plan P8 "ZED SDK 5.4 first start" (open + grab; the SDK downloads its
// resources on the first open) and P9.5 "ZED depth medians within 2 % of
// native": run it on both sides back to back, the car on the stand, the scene
// still, then `parity_gates.py zed-depth native.json container.json`.
//
//   zed_grab [--frames N] [--resolution HD720|HD1080|HD2K|VGA] [--fps F]
//            [--depth-mode PERFORMANCE|QUALITY|ULTRA|NEURAL]
//            [--min-depth M] [--max-depth M] [--json OUT.json]
//
// Defaults: 300 frames, HD720 @ 30 Hz, PERFORMANCE, depth 0.3..10 m (the
// stack's zed2_perception.yaml values). ROIs: a 3 x 3 grid of the image, the
// central half of each cell, every 4th pixel; per frame the median of the
// valid (finite, > 0) depths, then the median over frames.
// Read-only: no camera setting is written. Exit 0 pass, 3 no camera,
// 4 open failed, 5 frames or IMU missing.
#include <sl/Camera.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

double median(std::vector<float>& v) {
  if (v.empty()) return NAN;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  double m = v[mid];
  if (v.size() % 2 == 0) {
    std::nth_element(v.begin(), v.begin() + mid - 1, v.end());
    m = 0.5 * (m + v[mid - 1]);
  }
  return m;
}

bool parse_depth_mode(const std::string& s, sl::DEPTH_MODE* out) {
  if (s == "PERFORMANCE") { *out = sl::DEPTH_MODE::PERFORMANCE; return true; }
  if (s == "QUALITY") { *out = sl::DEPTH_MODE::QUALITY; return true; }
  if (s == "ULTRA") { *out = sl::DEPTH_MODE::ULTRA; return true; }
  if (s == "NEURAL") { *out = sl::DEPTH_MODE::NEURAL; return true; }
  return false;
}

bool parse_resolution(const std::string& s, sl::RESOLUTION* out) {
  if (s == "HD720") { *out = sl::RESOLUTION::HD720; return true; }
  if (s == "HD1080") { *out = sl::RESOLUTION::HD1080; return true; }
  if (s == "HD2K") { *out = sl::RESOLUTION::HD2K; return true; }
  if (s == "VGA") { *out = sl::RESOLUTION::VGA; return true; }
  return false;
}

void usage() {
  std::printf(
      "zed_grab [--frames N] [--resolution HD720|HD1080|HD2K|VGA] [--fps F]\n"
      "         [--depth-mode PERFORMANCE|QUALITY|ULTRA|NEURAL]\n"
      "         [--min-depth M] [--max-depth M] [--json OUT.json]\n"
      "Grabs N frames from the ZED and prints FPS, IMU and the median depth of a\n"
      "3x3 grid of image regions (JSON with --json). Read-only.\n");
}

}  // namespace

int main(int argc, char** argv) {
  int n = 300, fps = 30;
  float min_depth = 0.3f, max_depth = 10.0f;
  std::string res_s = "HD720", mode_s = "PERFORMANCE", json_path;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](void) -> std::string {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "-h" || a == "--help") { usage(); return 0; }
    else if (a == "--frames") n = std::atoi(next().c_str());
    else if (a == "--fps") fps = std::atoi(next().c_str());
    else if (a == "--resolution") res_s = next();
    else if (a == "--depth-mode") mode_s = next();
    else if (a == "--min-depth") min_depth = std::atof(next().c_str());
    else if (a == "--max-depth") max_depth = std::atof(next().c_str());
    else if (a == "--json") json_path = next();
    else if (std::isdigit(static_cast<unsigned char>(a[0]))) n = std::atoi(a.c_str());  // G3 form
    else { usage(); return 2; }
  }
  sl::DEPTH_MODE depth_mode;
  sl::RESOLUTION resolution;
  if (!parse_depth_mode(mode_s, &depth_mode) || !parse_resolution(res_s, &resolution) || n <= 0) {
    usage();
    return 2;
  }

  std::printf("sdk_version %s\n", sl::Camera::getSDKVersion().c_str());
  auto devs = sl::Camera::getDeviceList();
  std::printf("devices %zu\n", devs.size());
  for (auto& d : devs)
    std::printf("  id %d model %s serial %u state %s path %s\n", d.id,
                sl::toString(d.camera_model).c_str(), d.serial_number,
                sl::toString(d.camera_state).c_str(), d.path.c_str());
  FILE* jf = nullptr;
  if (!json_path.empty()) {
    jf = std::fopen(json_path.c_str(), "w");
    if (!jf) { std::perror(json_path.c_str()); return 2; }
  }
  auto finish = [&](const char* result, int rc) {
    std::printf("RESULT %s\n", result);
    if (jf) {
      std::fprintf(jf, "  \"result\": \"%s\"\n}\n", result);
      std::fclose(jf);
    }
    return rc;
  };
  if (jf)
    std::fprintf(jf, "{\n  \"sdk_version\": \"%s\",\n  \"devices\": %zu,\n  \"resolution\": \"%s\",\n"
                     "  \"fps_requested\": %d,\n  \"depth_mode\": \"%s\",\n  \"min_depth_m\": %.3f,\n"
                     "  \"max_depth_m\": %.3f,\n", sl::Camera::getSDKVersion().c_str(), devs.size(),
                 res_s.c_str(), fps, mode_s.c_str(), min_depth, max_depth);
  if (devs.empty()) return finish("no_camera", 3);

  sl::Camera cam;
  sl::InitParameters ip;
  ip.camera_resolution = resolution;
  ip.camera_fps = fps;
  ip.depth_mode = depth_mode;
  ip.coordinate_units = sl::UNIT::METER;
  ip.depth_minimum_distance = min_depth;
  ip.depth_maximum_distance = max_depth;
  ip.sdk_verbose = 1;
  auto t_open = std::chrono::steady_clock::now();
  sl::ERROR_CODE err = cam.open(ip);
  double open_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open).count();
  std::printf("open %s (%.2f s)\n", sl::toString(err).c_str(), open_s);
  if (jf) std::fprintf(jf, "  \"open\": \"%s\",\n  \"open_s\": %.2f,\n", sl::toString(err).c_str(), open_s);
  if (err != sl::ERROR_CODE::SUCCESS) return finish("open_failed", 4);
  auto ci = cam.getCameraInformation();
  std::printf("camera %s serial %u fw %u sensors_fw %u res %dx%d fps %.1f\n",
              sl::toString(ci.camera_model).c_str(), ci.serial_number,
              ci.camera_configuration.firmware_version,
              ci.sensors_configuration.firmware_version,
              (int)ci.camera_configuration.resolution.width,
              (int)ci.camera_configuration.resolution.height, ci.camera_configuration.fps);

  const int R = 3, C = 3;
  std::vector<std::vector<float>> per_roi(R * C);  // per frame medians
  std::vector<int> roi_valid(R * C, 0), roi_total(R * C, 0);
  sl::Mat left, depth;
  sl::SensorsData sensors;
  int ok = 0, fail = 0, imu_ok = 0, imu_new = 0;
  unsigned long long last_imu_ts = 0;
  double accel_norm_sum = 0.0;
  std::vector<float> buf;
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
    const int w = static_cast<int>(depth.getWidth()), h = static_cast<int>(depth.getHeight());
    const size_t step = depth.getStepBytes(sl::MEM::CPU);
    const unsigned char* base = reinterpret_cast<const unsigned char*>(depth.getPtr<sl::float1>(sl::MEM::CPU));
    for (int r = 0; r < R; ++r) {
      for (int c = 0; c < C; ++c) {
        const int x0 = c * w / C + w / (4 * C), x1 = (c + 1) * w / C - w / (4 * C);
        const int y0 = r * h / R + h / (4 * R), y1 = (r + 1) * h / R - h / (4 * R);
        buf.clear();
        int total = 0;
        for (int y = y0; y < y1; y += 4) {
          const float* row = reinterpret_cast<const float*>(base + static_cast<size_t>(y) * step);
          for (int x = x0; x < x1; x += 4) {
            ++total;
            const float d = row[x];
            if (std::isfinite(d) && d > 0.f) buf.push_back(d);
          }
        }
        roi_total[r * C + c] += total;
        roi_valid[r * C + c] += static_cast<int>(buf.size());
        if (!buf.empty()) per_roi[r * C + c].push_back(static_cast<float>(median(buf)));
      }
    }
    if (cam.getSensorsData(sensors, sl::TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS &&
        sensors.imu.is_available) {
      ++imu_ok;
      auto a = sensors.imu.linear_acceleration;
      accel_norm_sum += std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
      unsigned long long ts = sensors.imu.timestamp.getNanoseconds();
      if (ts != last_imu_ts) { ++imu_new; last_imu_ts = ts; }
    }
  }
  double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("frames_ok %d frames_failed %d elapsed_s %.2f fps %.2f\n", ok, fail, dt, ok / dt);
  std::printf("left %zux%zu depth %zux%zu\n", left.getWidth(), left.getHeight(), depth.getWidth(),
              depth.getHeight());
  std::printf("imu reads_ok %d of %d, distinct_timestamps %d, accel_norm_mean %.3f m/s2\n", imu_ok,
              ok, imu_new, imu_ok ? accel_norm_sum / imu_ok : NAN);
  if (jf)
    std::fprintf(jf, "  \"camera_serial\": %u,\n  \"camera_fw\": %u,\n  \"frames_ok\": %d,\n"
                     "  \"frames_failed\": %d,\n  \"fps\": %.2f,\n  \"imu_ok\": %d,\n"
                     "  \"imu_distinct\": %d,\n  \"rois\": [\n", ci.serial_number,
                 ci.camera_configuration.firmware_version, ok, fail, ok / dt, imu_ok, imu_new);
  for (int k = 0; k < R * C; ++k) {
    const double m = median(per_roi[k]);
    const double vf = roi_total[k] ? static_cast<double>(roi_valid[k]) / roi_total[k] : 0.0;
    std::printf("roi r%dc%d median_m %.4f valid_frac %.3f frames %zu\n", k / C, k % C, m, vf,
                per_roi[k].size());
    if (jf)
      std::fprintf(jf, "    {\"name\": \"r%dc%d\", \"median_m\": %s, \"valid_frac\": %.4f, \"frames\": %zu}%s\n",
                   k / C, k % C, std::isfinite(m) ? std::to_string(m).c_str() : "null", vf,
                   per_roi[k].size(), k + 1 < R * C ? "," : "");
  }
  if (jf) std::fprintf(jf, "  ],\n");
  cam.close();
  const bool pass = (ok == n) && imu_ok == ok && imu_new > 1;
  return finish(pass ? "pass" : (ok != n ? "frames_missing" : "imu_missing"), pass ? 0 : 5);
}
