#!/bin/bash
# G5 (ROS): apt Jazzy in the image equals the Thor's dpkg set, the 31-package
# workspace built, and the CLAUDE.md ldd -r scan of the install space is
# empty. (rosdep coverage and the Discovery Server / SHM probe run separately:
# rosdep needs root for `rosdep init`, the DDS probe needs two containers.)
set -u; set +u  # ROS setup files read unset variables
source /opt/ros/jazzy/setup.bash
source /opt/f1tenth_ws/install/setup.bash

echo "== ros-jazzy-* versions: image vs Thor (/spike/thor/ros_jazzy_dpkg.txt)"
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'ros-jazzy-*' | awk '$1=="ii"{print $2, $3}' | sort > /tmp/image_ros.txt
awk 'NF==2' /spike/thor/ros_jazzy_dpkg.txt | sort > /tmp/thor_ros.txt
python3 - <<'EOF'
img = dict(l.split() for l in open('/tmp/image_ros.txt'))
thor = dict(l.split() for l in open('/tmp/thor_ros.txt'))
both = sorted(set(img) & set(thor))
diff = [(p, thor[p], img[p]) for p in both if img[p] != thor[p]]
print(f"image {len(img)}  thor {len(thor)}  common {len(both)}  version_differs {len(diff)}")
for d in diff:
    print("  DIFFERS", *d)
only_img = sorted(set(img) - set(thor))
only_thor = sorted(set(thor) - set(img))
print(f"image_only {len(only_img)}: {' '.join(p[10:] for p in only_img)}")
print(f"thor_only {len(only_thor)}: {' '.join(p[10:] for p in only_thor)}")
EOF

echo "== system Python versions"
python3 - <<'EOF'
import importlib, platform
print('python', platform.python_version())
for m in ('numpy', 'scipy', 'osqp', 'pyarrow', 'cv2', 'torch', 'torchvision', 'ultralytics', 'yaml', 'psutil', 'jsonschema', 'matplotlib', 'pydot', 'py_trees', 'requests'):
    try:
        mod = importlib.import_module(m)
        print(f'  {m} {getattr(mod, "__version__", "?")}')
    except Exception as e:
        print(f'  {m} MISSING ({type(e).__name__}: {e})')
EOF

echo "== colcon build summary"
grep -a -E '^Summary:|packages? (finished|failed|aborted|had stderr)' /opt/f1tenth_ws/colcon_build.log
echo "packages in install: $(ls /opt/f1tenth_ws/install/share/ament_index/resource_index/packages | wc -l)"

echo "== ldd -r scan of the install space (CLAUDE.md; empty = clean)"
find /opt/f1tenth_ws/install -path '*/lib/*' \( -type f -o -type l \) | while read -r f; do
  file -L "$f" | grep -q ELF || continue
  ldd -r "$f" 2>&1 | grep 'undefined symbol\|not found' | grep -v '__rosidl_\|__msg__\|__srv__\|__action__' \
    | sed "s|^|$f: |"
done
echo "(scan end)"
