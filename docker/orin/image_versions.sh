#!/bin/bash
# Runs INSIDE the Orin Jazzy image (image_report.sh starts it with the nvidia
# runtime): print every resolved version the manifest pins, in sections that
# diff_manifest.py reads.
set +u
echo "[ros_dpkg]"
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'ros-jazzy-*' | awk '$1=="ii"{print $2, $3}'
echo "[apt]"
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'python3-*' 'libopencv-core*' 'cuda-*-12-9' \
  'libcublas-12-9' 'libcudnn9-cuda-12' 'libnpp-12-9' 'libnvjpeg-12-9' 'ffmpeg' 'libavcodec60' 2>/dev/null | awk '$1=="ii"{print $2, $3}'
echo "[pip]"
pip list --format=freeze 2>/dev/null
echo "[gateway_venv]"
site=$(/opt/f1tenth_gateway/venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])' 2>/dev/null)
[ -n "$site" ] && /opt/f1tenth_gateway/venv/bin/pip list --path "$site" --format=freeze 2>/dev/null \
  | grep -v -i -E '^(pip|setuptools|wheel)=='
echo "[snapshots]"
echo "ubuntu $(cat /etc/f1tenth_ubuntu_snapshot 2>/dev/null || echo unknown)"
echo "ros $(sed -n 's|.*snapshots.ros.org/jazzy/\([^/]*\)/.*|\1|p' /etc/apt/sources.list.d/ros2-snapshot.list 2>/dev/null)"
echo "ubuntu_sources $(grep -h -m1 '^URIs:' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null | awk '{print $2}')"
echo "dpkg_packages $(wc -l < /opt/f1tenth_ws/DPKG_VERSIONS.txt 2>/dev/null) (full list: /opt/f1tenth_ws/DPKG_VERSIONS.txt)"
echo "[rust]"
if command -v rustc > /dev/null; then
  echo "rustc $(rustc --version | awk '{print $2}')"
  echo "cargo $(cargo --version | awk '{print $2}')"
  echo "cargo_ament_build $(cargo install --list 2>/dev/null | awk '/^cargo-ament-build / {print $2}' | tr -d 'v:')"
  pip list --format=freeze 2>/dev/null | grep -i -E '^colcon-(ros-)?cargo==' | sed 's/==/ /' | tr '-' '_'
else
  echo "rustc none (RUST=skip image)"
fi
echo "cargo_lock_sha256 $(sha256sum /opt/f1tenth_ws/src/rust/Cargo.lock 2>/dev/null | cut -d' ' -f1)"
for pb in /opt/f1tenth_ws/install/lib/f1tenth_proximity_guard/proximity_guard_replay \
          /opt/f1tenth_ws/install/lib/f1tenth_costmap_boundary_rs/costmap_boundary_replay \
          /opt/f1tenth_ws/install/lib/f1tenth_hard_stop_rs/hard_stop_replay; do
  [ -x "$pb" ] && echo "build $(basename "$pb") $(echo | "$pb" 2>&1 >/dev/null | head -1)"
done
echo "[platform]"
. /etc/os-release; echo "ubuntu $VERSION_ID"
echo "python $(python3 -c 'import platform; print(platform.python_version())')"
echo "glibc $(ldd --version | head -1 | awk '{print $NF}')"
echo "gcc $(gcc -dumpfullversion)"
echo "cmake $(cmake --version | head -1 | awk '{print $3}')"
echo "ffmpeg $(ffmpeg -hide_banner -version 2>/dev/null | head -1 | awk '{print $3}')"
echo "[gpu]"
cat /usr/local/cuda/version.txt
echo "nvcc $(nvcc --version | tail -1)"
echo "compat_libcuda $(readlink -f /usr/local/cuda/compat/libcuda.so.1)"
echo "cuda_static_dropped $(wc -l < /usr/local/cuda-12.9/L17_dropped_static_libs.txt 2>/dev/null) files (L17)"
python3 - <<'EOF'
import ctypes
for name, lib in (('driver_api_compat', 'libcuda.so.1'),):
    v = ctypes.c_int()
    try:
        ctypes.CDLL(lib).cuDriverGetVersion(ctypes.byref(v))
        print(name, v.value)
    except OSError as e:
        print(name, 'unavailable', e)
import torch
print('torch', torch.__version__, 'cuda', torch.version.cuda, 'cudnn', torch.backends.cudnn.version(),
      'available', torch.cuda.is_available(),
      'device', torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-')
try:
    import tensorrt
    print('tensorrt', tensorrt.__version__)
except Exception as e:  # noqa: BLE001
    print('tensorrt import failed:', e)
EOF
ls /opt/tensorrt/lib/libnvinfer.so.10.*
echo "[zed]"
grep -h -E '#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp | awk '{print $2, $3}'
echo "zed_group $(stat -c '%G(%g) %a' /usr/local/zed)"
echo "[llama]"
cat /opt/llama.cpp/BUILD_INFO
/opt/llama.cpp/bin/llama-server --version 2>&1 | grep -i -E 'version|built' | head -2
echo "[workspace]"
cat /opt/f1tenth_ws/SOURCES
grep -a -E '^Summary:|packages? (failed|had stderr)|^== phase|^RUST=' /opt/f1tenth_ws/colcon_build.log
echo "packages $(find /opt/f1tenth_ws/install/share/ament_index/resource_index/packages -mindepth 1 -maxdepth 1 | wc -l)"
python3 - <<'EOF'
import yaml
d = yaml.safe_load(open('/opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml'))
for k in ('llama_server_path', 'llama_server_cwd', 'yolo_engine_path', 'enable_comms'):
    print('stack_params', k, d[k]['default'] if k in d else '(missing)')
EOF
python3 -c 'import json; d = json.load(open("/opt/f1tenth_ws/CAR_SYNC.json")); print("car_sync native_head", d["native_head"], "captured_at", d.get("captured_at"))'
echo "[groups]"
for g in video render dialout input zed jtop; do echo "$g $(getent group "$g" | cut -d: -f3)"; done
echo "user $(id fabiocar)"
echo "[env]"
env | grep -E '^(RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_DISTRO|LD_LIBRARY_PATH|CUDA_CACHE_PATH|CUDA_CACHE_MAXSIZE|ZED_DIR|PIP_CONSTRAINT|HOME)=' | sort
echo "user $(id)"
