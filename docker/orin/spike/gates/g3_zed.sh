#!/bin/bash
# G3 (ZED): the SDK installed on noble, its libraries resolve with the nvidia
# runtime's L4T mounts, zed_components (86171bc) was built and its library
# loads, and - if the ZED is plugged in - 300 frames are grabbed.
# The calibration is a COPY of the host's /usr/local/zed/settings, mounted at
# /usr/local/zed/settings, so the SDK never writes the host's file.
set -u; set +u  # ROS setup files read unset variables
source /opt/ros/jazzy/setup.bash
source /opt/f1tenth_ws/install/setup.bash
N=${1:-300}

echo "== SDK header version"
grep -h -E '#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION' /usr/local/zed/include/sl/Camera.hpp
echo "== installer summary"
grep -a -i -E 'version|warning|error|cuda|jetpack|container|platform' /usr/local/zed_install.log | sed 's/\x1b\[[0-9;]*m//g' | head -25

echo "== ldd -r SDK libraries (empty = clean)"
for f in /usr/local/zed/lib/*.so; do
  ldd -r "$f" 2>&1 | grep -E 'not found|undefined symbol' | sed "s|^|$f: |"
done
echo "== ldd -r zed_components / zed_debug (empty = clean)"
for f in $(find /opt/f1tenth_ws/install/lib -name 'libzed*.so'); do
  ldd -r "$f" 2>&1 | grep -E 'not found|undefined symbol' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' | sed "s|^|$f: |"
done
echo "== dlopen zed_components (RTLD_NOW)"
python3 - <<'EOF'
import ctypes, glob, os
for f in sorted(glob.glob('/opt/f1tenth_ws/install/lib/libzed_camera_*component*.so')):
    ctypes.CDLL(f, mode=os.RTLD_NOW | os.RTLD_GLOBAL)
    print('loaded', f)
EOF
echo "== rclcpp_components registration"
grep -h -o 'stereolabs::[A-Za-z]*' /opt/f1tenth_ws/install/share/ament_index/resource_index/rclcpp_components/zed_components 2>/dev/null | sort -u

echo "== camera grab ($N frames)"
g++ -O2 -std=c++17 /spike/gates/zed_grab.cpp -o /tmp/zed_grab \
  -I/usr/local/zed/include -I/usr/local/cuda/include \
  -L/usr/local/zed/lib -lsl_zed -L/usr/local/cuda/lib64 -lcudart -Wl,-rpath,/usr/local/zed/lib
/tmp/zed_grab "$N"
echo "zed_grab exit $?"
