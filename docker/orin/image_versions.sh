#!/bin/bash
# Runs INSIDE the Orin Jazzy image (image_report.sh starts it with the nvidia
# runtime): print every resolved version the manifest pins, in sections that
# diff_manifest.py reads.
set +u
echo "[ros_dpkg]"
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'ros-jazzy-*' | awk '$1=="ii"{print $2, $3}'
echo "[apt]"
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'python3-*' 'libopencv-core*' 'cuda-*-12-9' \
  'libcublas-12-9' 'libcudnn9-cuda-12' 'libnpp-12-9' 'libnvjpeg-12-9' 2>/dev/null | awk '$1=="ii"{print $2, $3}'
echo "[pip]"
pip list --format=freeze 2>/dev/null
echo "[platform]"
. /etc/os-release; echo "ubuntu $VERSION_ID"
echo "python $(python3 -c 'import platform; print(platform.python_version())')"
echo "glibc $(ldd --version | head -1 | awk '{print $NF}')"
echo "gcc $(gcc -dumpfullversion)"
echo "cmake $(cmake --version | head -1 | awk '{print $3}')"
echo "[gpu]"
cat /usr/local/cuda/version.txt
echo "nvcc $(nvcc --version | tail -1)"
echo "compat_libcuda $(readlink -f /usr/local/cuda/compat/libcuda.so.1)"
python3 - <<'EOF'
import ctypes
for name, lib in (('driver_api_compat', 'libcuda.so.1'),):
    v = ctypes.c_int()
    try:
        ctypes.CDLL(lib).cuDriverGetVersion(ctypes.byref(v))
        print(name, v.value)
    except OSError as e:
        print(name, 'unavailable', e)
import torch, tensorrt
print('torch', torch.__version__, 'cuda', torch.version.cuda, 'cudnn', torch.backends.cudnn.version(),
      'available', torch.cuda.is_available(),
      'device', torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-')
print('tensorrt', tensorrt.__version__)
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
grep -a -E '^Summary:|packages? (failed|had stderr)' /opt/f1tenth_ws/colcon_build.log
echo "packages $(ls /opt/f1tenth_ws/install/share/ament_index/resource_index/packages | wc -l)"
python3 - <<'EOF'
import yaml
d = yaml.safe_load(open('/opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml'))
for k in ('llama_server_path', 'llama_server_cwd'):
    print('stack_params', k, d[k]['default'])
EOF
echo "[env]"
env | grep -E '^(RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_DISTRO|LD_LIBRARY_PATH|CUDA_CACHE_PATH|CUDA_CACHE_MAXSIZE|ZED_DIR|PIP_CONSTRAINT|HOME)=' | sort
echo "user $(id)"
