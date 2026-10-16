#!/bin/bash
# Build-time checks of the Orin Jazzy image (run in the Dockerfile's ws stage,
# with the host's L4T libs bind-mounted at /usr/lib/aarch64-linux-gnu/nvidia).
# Any failure fails the build.
set -u
fail=0
bad() { echo "BUILD CHECK FAILED: $*" >&2; fail=1; }

echo "== pip check"
pip check || bad "pip check"

echo "== no PyPI CUDA (nvidia-*, cuda-*, triton; nvidia-ml-py is pure-Python NVML bindings)"
cuda_pkgs=$(pip list --format=freeze 2>/dev/null | grep -i -E '^(nvidia-|cuda-|triton)' | grep -v -i '^nvidia-ml-py==' || true)
[ -z "$cuda_pkgs" ] || bad "PyPI CUDA packages installed: $cuda_pkgs"

echo "== apt Python not shadowed by pip (Thor set, F6)"
python3 - <<'EOF' || bad "apt Python shadowed"
import importlib, sys
bad = []
for mod in ('numpy', 'scipy', 'matplotlib', 'jsonschema', 'requests', 'yaml', 'psutil', 'pydot', 'ruamel.yaml'):
    m = importlib.import_module(mod)
    where = getattr(m, '__file__', '') or ''
    ok = where.startswith('/usr/lib/python3/dist-packages/')
    print(f'  {mod:12s} {getattr(m, "__version__", "?"):10s} {where}')
    if not ok:
        bad.append(mod)
sys.exit(f'not from apt: {bad}' if bad else 0)
EOF

echo "== stack Python closure imports"
python3 -c '
import numpy, scipy, osqp, cv2, torch, torchvision, ultralytics, pyarrow, psutil, jsonschema
import matplotlib, pydot, yaml, ruamel.yaml, requests, py_trees, jtop, tensorrt
import rclpy, cv_bridge, rosbag2_py
print("  torch", torch.__version__, "cuda", torch.version.cuda, "| tensorrt", tensorrt.__version__,
      "| ultralytics", ultralytics.__version__, "| cv2", cv2.__version__, "| numpy", numpy.__version__)
' || bad "Python closure import"

echo "== CUDA nvJPEG dev symlink absent (F3)"
[ ! -e /usr/local/cuda/lib64/libnvjpeg.so ] || bad "/usr/local/cuda/lib64/libnvjpeg.so exists (shadows L4T libnvjpeg.so)"

echo "== /usr/local/zed group is the host zed GID (F5)"
[ "$(stat -c %g /usr/local/zed)" = "${ZED_GID}" ] || bad "/usr/local/zed group $(stat -c %g /usr/local/zed) != ${ZED_GID}"

echo "== ldd -r scan (CLAUDE.md): workspace install, ZED SDK, llama.cpp; must print nothing"
scan=$(find /opt/f1tenth_ws/install /usr/local/zed/lib /opt/llama.cpp \( -path '*/lib/*' -o -path '*/bin/*' \) \( -type f -o -type l \) 2>/dev/null |
  while read -r f; do
    file -L "$f" | grep -q ELF || continue
    ldd -r "$f" 2>&1 | grep 'undefined symbol\|not found' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' | sed "s|^|$f: |"
  done)
if [ -n "$scan" ]; then echo "$scan" | head -40; bad "ldd -r scan not empty"; fi

[ $fail = 0 ] && echo "== all build checks passed"
exit $fail
