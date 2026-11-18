#!/usr/bin/env bash
# export_yolo_engine.sh - build the FP16 TensorRT engine of the YOLO seg model
# on the Jetson AGX Orin, then load-test it against the .pt.
#
# Run it ON THE ORIN (the engine is specific to the GPU and the TensorRT
# version that built it). See --help.
set -euo pipefail

PROG=$(basename "$0")
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

MODEL="yolo26s-seg.pt"
MODELS_DIR="$REPO_ROOT/src/f1tenth_perception/models"
IMGSZ="384,640"
WORKSPACE="4"
PYTHON="python3"
FORCE=0
DRY_RUN=0
LOAD_TEST_RUNS=20
CU12_LIB="${CU12_LIB:-}"    # default: the --python interpreter's user-site nvidia/cu12/lib (below)

usage() {
  cat <<EOF
Usage: $PROG [options]

Export the YOLO segmentation model to an FP16 TensorRT engine and load-test it.
Must run on the Jetson that will use the engine (Orin: TensorRT 10.3.0).

Output (auto-resolved by yolo_detector_node, do not rename):
  <models-dir>/<model-stem>.trt<TRT_VERSION>.fp16.engine
  e.g. src/f1tenth_perception/models/yolo26s-seg.trt10.3.0.fp16.engine
  TRT_VERSION = tensorrt.__version__ of --python.

If the engine already exists (and no --force), the export is skipped and only
the load test runs.

Options:
  --model NAME|PATH  .pt to export: a bare name is looked up in --models-dir
                     (default: $MODEL)
  --models-dir DIR   where the engine is written
                     (default: <repo>/src/f1tenth_perception/models)
  --imgsz H,W        static engine input size (default: $IMGSZ, the rect
                     letterbox of the 640x360 ZED frame); multiples of 32
  --workspace GB     TensorRT builder workspace in GiB (default: $WORKSPACE)
  --python BIN       Python interpreter with torch/ultralytics/tensorrt
                     (default: $PYTHON)
  --force            rebuild even if the engine exists; run even if a
                     yolo_detector_node is running (it holds GPU memory)
  --dry-run          preflight only: print what would be done, change nothing
  -h, --help         this help

Environment:
  CU12_LIB           nvidia cu12 lib dir prepended to LD_LIBRARY_PATH when it
                     exists (default: ~/.local/lib/pythonX.Y/site-packages/nvidia/cu12/lib
                     of the --python interpreter; pip torch in the user site needs it)
  YOLO_AUTOINSTALL   defaults to False here, so Ultralytics never pip-installs
                     missing export deps (onnx, onnxslim, onnxruntime) by itself

Exit codes:
  0  success (dry-run: a real run would proceed)
  1  export or load test failed
  2  usage error
  3  preflight blocked (no TensorRT / CUDA / ultralytics / export deps, a
     running yolo_detector_node without --force, ...). With --dry-run, 3 means
     "a real run would stop here" - e.g. on a PC without TensorRT.
EOF
}

die()  { echo "$PROG: ERROR: $*" >&2; exit 1; }
udie() { echo "$PROG: $*" >&2; echo "Try '$PROG --help'." >&2; exit 2; }
warn() { echo "$PROG: WARN: $*" >&2; }
info() { echo "== $*"; }

need_arg() { [[ $# -ge 2 && -n "$2" ]] || udie "option $1 needs a value"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)       need_arg "$@"; MODEL="$2"; shift 2 ;;
    --model=*)     MODEL="${1#*=}"; shift ;;
    --models-dir)  need_arg "$@"; MODELS_DIR="$2"; shift 2 ;;
    --models-dir=*) MODELS_DIR="${1#*=}"; shift ;;
    --imgsz)       need_arg "$@"; IMGSZ="$2"; shift 2 ;;
    --imgsz=*)     IMGSZ="${1#*=}"; shift ;;
    --workspace)   need_arg "$@"; WORKSPACE="$2"; shift 2 ;;
    --workspace=*) WORKSPACE="${1#*=}"; shift ;;
    --python)      need_arg "$@"; PYTHON="$2"; shift 2 ;;
    --python=*)    PYTHON="${1#*=}"; shift ;;
    --force)       FORCE=1; shift ;;
    --dry-run)     DRY_RUN=1; shift ;;
    -h|--help)     usage; exit 0 ;;
    *)             udie "unknown argument: $1" ;;
  esac
done

# ---------------------------------------------------------------- arguments
[[ "$IMGSZ" =~ ^([0-9]+),([0-9]+)$ ]] || udie "--imgsz must be H,W (got '$IMGSZ')"
IMG_H=${BASH_REMATCH[1]}
IMG_W=${BASH_REMATCH[2]}
(( IMG_H >= 32 && IMG_W >= 32 && IMG_H % 32 == 0 && IMG_W % 32 == 0 )) \
  || udie "--imgsz H,W must be multiples of 32 (got $IMGSZ)"
[[ "$WORKSPACE" =~ ^[0-9]+([.][0-9]+)?$ ]] || udie "--workspace must be a number of GiB (got '$WORKSPACE')"
command -v "$PYTHON" >/dev/null 2>&1 || udie "--python '$PYTHON' not found"

[[ -d "$MODELS_DIR" ]] || udie "models dir not found: $MODELS_DIR"
MODELS_DIR=$(cd "$MODELS_DIR" && pwd)

if [[ "$MODEL" == */* ]]; then
  PT="$MODEL"
else
  PT="$MODELS_DIR/$MODEL"
fi
if [[ "$PT" != *.pt && ! -f "$PT" && -f "$PT.pt" ]]; then
  PT="$PT.pt"
fi
[[ -f "$PT" ]] || udie "model not found: $PT"
[[ "$PT" == *.pt ]] || udie "model must be a .pt file: $PT"
PT=$(cd "$(dirname "$PT")" && pwd)/$(basename "$PT")
STEM=$(basename "$PT" .pt)
[[ "$STEM" != *.pt* ]] || udie "model stem '$STEM' contains '.pt': the engine name would be classified as a .pt by Ultralytics"

# ---------------------------------------------------------- embedded Python
# Python 3.10 syntax (the Orin's /usr/bin/python3).
IFS= read -r -d '' PY_PREFLIGHT <<'PY_PREFLIGHT' || true
import platform
import sys


def out(key, val):
    print(f"{key}={str(val).splitlines()[0] if str(val) else ''}")


out("PYTHON", f"{sys.executable} {platform.python_version()}")
try:
    import torch
    out("TORCH", torch.__version__)
    out("TORCH_CUDA_BUILD", torch.version.cuda)
    ok = torch.cuda.is_available()
    out("CUDA_OK", int(ok))
    if ok:
        out("CUDA_DEVICE", torch.cuda.get_device_name(0))
except Exception as e:  # noqa: BLE001
    out("TORCH_ERR", f"{type(e).__name__}: {e}")
    out("CUDA_OK", 0)
try:
    import ultralytics
    from ultralytics.utils import ASSETS
    out("ULTRALYTICS", ultralytics.__version__)
    out("BUS_JPG", int((ASSETS / "bus.jpg").is_file()))
except Exception as e:  # noqa: BLE001
    out("ULTRALYTICS_ERR", f"{type(e).__name__}: {e}")
try:
    import tensorrt
    out("TRT", tensorrt.__version__)
except Exception as e:  # noqa: BLE001
    out("TRT_ERR", f"{type(e).__name__}: {e}")
for mod in ("onnx", "onnxslim", "onnxruntime"):
    try:
        m = __import__(mod)
        out(mod.upper(), getattr(m, "__version__", "?"))
    except Exception as e:  # noqa: BLE001
        out(mod.upper() + "_ERR", f"{type(e).__name__}: {e}")
PY_PREFLIGHT

IFS= read -r -d '' PY_EXPORT <<'PY_EXPORT' || true
import sys
from pathlib import Path

from ultralytics import YOLO

pt, h, w, ws = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
model = YOLO(pt)
out = model.export(
    format="engine",
    quantize=16,
    imgsz=(h, w),
    device=0,
    dynamic=False,
    batch=1,
    simplify=True,
    workspace=ws,
)
if not out or not Path(out).is_file():
    raise SystemExit(f"FAIL: export returned {out!r}, no engine file")
print(f"EXPORTED={out}")
PY_EXPORT

IFS= read -r -d '' PY_LOADTEST <<'PY_LOADTEST' || true
import sys
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils import ASSETS

engine_path, pt_path = sys.argv[1], sys.argv[2]
imgsz = (int(sys.argv[3]), int(sys.argv[4]))
runs = int(sys.argv[5])
CONF = 0.3
failures = []
warnings = []


def make_frame():
    """640x360 BGR frame: centre 16:9 crop of bus.jpg, resized."""
    src = ASSETS / "bus.jpg"
    img = cv2.imread(str(src), cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"FAIL: cannot read {src}")
    h, w = img.shape[:2]
    tw, th = w, int(round(w * 9 / 16))
    if th > h:
        th, tw = h, int(round(h * 16 / 9))
    y0, x0 = (h - th) // 2, (w - tw) // 2
    crop = img[y0:y0 + th, x0:x0 + tw]
    return np.ascontiguousarray(cv2.resize(crop, (640, 360), interpolation=cv2.INTER_AREA))


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def evaluate(path, label, frame, task=None):
    model = YOLO(path, task=task) if task else YOLO(path)
    kw = dict(conf=CONF, retina_masks=True, device=0, imgsz=imgsz, verbose=False)
    res = model.predict(frame, **kw)[0]  # first call builds the predictor (warm-up)
    sync()
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        model.predict(frame, **kw)
        sync()
        times.append((time.perf_counter() - t0) * 1000.0)
    names = model.names  # after predict: read from the live predictor, no second load
    backend = model.predictor.model
    boxes = res.boxes
    n = len(boxes) if boxes is not None else 0
    cls = [int(c) for c in boxes.cls.tolist()] if n else []
    xyxy = boxes.xyxy.cpu().numpy() if n else np.zeros((0, 4))
    masks = res.masks.data.cpu().numpy() > 0.5 if res.masks is not None else None
    print(f"[{label}] {path}")
    print(f"[{label}] task={model.task} len(names)={len(names)} names[0]={names.get(0)!r} "
          f"backend_fp16={getattr(backend, 'fp16', '?')} backend_imgsz={getattr(backend, 'imgsz', '?')}")
    print(f"[{label}] boxes={n} classes={sorted(cls)} masks_shape="
          f"{None if masks is None else tuple(masks.shape)}")
    print(f"[{label}] latency over {runs} runs: mean={np.mean(times):.2f} ms "
          f"median={np.median(times):.2f} min={np.min(times):.2f} max={np.max(times):.2f}")
    return dict(task=model.task, names=names, n=n, cls=cls, xyxy=xyxy, masks=masks,
                mean_ms=float(np.mean(times)))


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


frame = make_frame()
print(f"frame: {frame.shape} {frame.dtype} (bus.jpg, BGR), conf={CONF}, imgsz={imgsz}, retina_masks=True")
eng = evaluate(engine_path, "engine", frame, task="segment")

if eng["task"] != "segment":
    failures.append(f"engine task is {eng['task']!r}, expected 'segment'")
if eng["names"].get(0) != "person":
    failures.append(f"engine names[0] is {eng['names'].get(0)!r}, expected 'person'")
if eng["n"] and eng["masks"] is None:
    failures.append("engine returned boxes but no masks")
if eng["masks"] is not None and tuple(eng["masks"].shape[1:]) != frame.shape[:2]:
    failures.append(f"engine masks {eng['masks'].shape} do not match the frame {frame.shape[:2]} (retina_masks)")

ref = evaluate(pt_path, "pt", frame)
names = ref["names"]
counts_e, counts_p = {}, {}
for c in eng["cls"]:
    counts_e[names.get(c, c)] = counts_e.get(names.get(c, c), 0) + 1
for c in ref["cls"]:
    counts_p[names.get(c, c)] = counts_p.get(names.get(c, c), 0) + 1
print("per-class counts (engine vs pt):")
for k in sorted(set(counts_e) | set(counts_p), key=str):
    flag = "" if counts_e.get(k, 0) == counts_p.get(k, 0) else "   <-- differs"
    print(f"  {k}: {counts_e.get(k, 0)} vs {counts_p.get(k, 0)}{flag}")
if counts_e != counts_p:
    warnings.append("per-class counts differ (borderline detections near conf=0.3 can flip in FP16)")

# For each pt box, the best same-class engine box.
box_ious, mask_ious = [], []
for i, (c, b) in enumerate(zip(ref["cls"], ref["xyxy"])):
    best, best_j = 0.0, -1
    for j, (ce, be) in enumerate(zip(eng["cls"], eng["xyxy"])):
        if ce == c:
            v = box_iou(b, be)
            if v > best:
                best, best_j = v, j
    box_ious.append(best)
    if best_j >= 0 and ref["masks"] is not None and eng["masks"] is not None:
        mp, me = ref["masks"][i], eng["masks"][best_j]
        union = np.logical_or(mp, me).sum()
        mask_ious.append(float(np.logical_and(mp, me).sum() / union) if union else 0.0)
if ref["n"] and not eng["n"]:
    failures.append(f"engine found no boxes, the pt found {ref['n']}")
if box_ious:
    print(f"best-match box IoU (pt -> engine, same class): mean={np.mean(box_ious):.3f} "
          f"min={np.min(box_ious):.3f} n={len(box_ious)}")
    if np.mean(box_ious) < 0.5:
        failures.append(f"mean best-match box IoU {np.mean(box_ious):.3f} < 0.5: engine output is wrong")
    elif np.mean(box_ious) < 0.9:
        warnings.append(f"mean best-match box IoU {np.mean(box_ious):.3f} < 0.9")
if mask_ious:
    print(f"matched mask IoU: mean={np.mean(mask_ious):.3f} min={np.min(mask_ious):.3f}")
    if np.mean(mask_ious) < 0.5:
        failures.append(f"mean matched mask IoU {np.mean(mask_ious):.3f} < 0.5: masks are wrong")
    elif np.mean(mask_ious) < 0.8:
        warnings.append(f"mean matched mask IoU {np.mean(mask_ious):.3f} < 0.8")
print(f"latency: engine {eng['mean_ms']:.2f} ms vs pt {ref['mean_ms']:.2f} ms "
      f"(x{ref['mean_ms'] / max(eng['mean_ms'], 1e-6):.2f})")
for w in warnings:
    print(f"LOADTEST WARN: {w}")
for f in failures:
    print(f"LOADTEST FAIL: {f}")
print("LOADTEST " + ("FAILED" if failures else "OK"))
sys.exit(1 if failures else 0)
PY_LOADTEST

# ---------------------------------------------------------------- preflight
BLOCKERS=()
block() { BLOCKERS+=("$*"); echo "$PROG: BLOCKED: $*" >&2; }

info "preflight"
echo "repo:        $REPO_ROOT"
echo "model (.pt): $PT"
echo "models dir:  $MODELS_DIR"
echo "imgsz:       ${IMG_H}x${IMG_W} (H x W), workspace ${WORKSPACE} GiB, FP16, batch 1, static"

if [[ -r /etc/nv_tegra_release ]]; then
  echo "jetson:      $(head -n1 /etc/nv_tegra_release)"
else
  warn "no /etc/nv_tegra_release: not a Jetson. An engine built here only works on this GPU + TensorRT."
fi

# The executable as a whole word ('.../lib/f1tenth_perception/yolo_detector_node',
# 'ros2 run ... yolo_detector_node'), not any shell that merely mentions the module.
NODE_PIDS=$(pgrep -f '(^|[/ ])[y]olo_detector_node( |$)' || true)
if [[ -n "$NODE_PIDS" ]]; then
  echo "running yolo_detector_node process(es):"
  # shellcheck disable=SC2086
  ps -o pid=,args= -p $(echo $NODE_PIDS | tr ' ' ',') | cut -c1-200 | sed 's/^/  /' || true
  if (( FORCE )); then
    warn "yolo_detector_node is running (it holds GPU memory); --force given, continuing"
  else
    block "yolo_detector_node is running (pids: $(echo $NODE_PIDS)); stop it or pass --force"
  fi
fi

if [[ -z "$CU12_LIB" ]]; then
  PYVER=$("$PYTHON" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || true)
  [[ -n "$PYVER" ]] && CU12_LIB="$HOME/.local/lib/python$PYVER/site-packages/nvidia/cu12/lib"
fi
if [[ -n "$CU12_LIB" && -d "$CU12_LIB" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":$CU12_LIB:"*) echo "LD_LIBRARY_PATH already has $CU12_LIB" ;;
    *) export LD_LIBRARY_PATH="$CU12_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
       echo "LD_LIBRARY_PATH += $CU12_LIB" ;;
  esac
else
  echo "no ${CU12_LIB:-pip nvidia/cu12 dir} (LD_LIBRARY_PATH unchanged)"
fi
export YOLO_AUTOINSTALL="${YOLO_AUTOINSTALL:-False}"

PF_OUT=$("$PYTHON" -c "$PY_PREFLIGHT") || block "python preflight crashed (see the traceback above)"
declare -A PF=()
while IFS= read -r line; do
  [[ "$line" =~ ^[A-Z0-9_]+= ]] || continue
  PF["${line%%=*}"]="${line#*=}"
  echo "  $line"
done <<<"$PF_OUT"

[[ -n "${PF[TORCH_ERR]:-}" ]] && block "torch does not import: ${PF[TORCH_ERR]} (LD_LIBRARY_PATH needs the nvidia cu12 lib dir?)"
[[ -z "${PF[TORCH_ERR]:-}" && "${PF[CUDA_OK]:-0}" != 1 ]] && block "torch.cuda.is_available() is False"
[[ -n "${PF[ULTRALYTICS_ERR]:-}" ]] && block "ultralytics does not import: ${PF[ULTRALYTICS_ERR]}"
[[ -z "${PF[ULTRALYTICS_ERR]:-}" && "${PF[BUS_JPG]:-0}" != 1 ]] && block "ultralytics/assets/bus.jpg missing (needed by the load test)"
TRT_VER="${PF[TRT]:-}"
if [[ -z "$TRT_VER" ]]; then
  block "TensorRT is not importable from $PYTHON (${PF[TRT_ERR]:-unknown}). Export only works on the Jetson (JetPack TensorRT python bindings); this machine cannot build the engine."
elif [[ ! "$TRT_VER" =~ ^[0-9]+(\.[0-9]+)*$ ]]; then
  block "unexpected tensorrt.__version__ '$TRT_VER'"
fi

TARGET="$MODELS_DIR/$STEM.trt${TRT_VER:-<TRT_VERSION>}.fp16.engine"
DO_EXPORT=1
if [[ -n "$TRT_VER" && -f "$TARGET" && $FORCE -eq 0 ]]; then
  DO_EXPORT=0
fi

if (( DO_EXPORT )); then
  for mod in ONNX ONNXSLIM; do
    [[ -n "${PF[${mod}_ERR]:-}" ]] && block "$(tr 'A-Z' 'a-z' <<<"$mod") not importable (needed by the export with simplify=True): ${PF[${mod}_ERR]}. Install: $PYTHON -m pip install --user 'onnx>=1.12,<2' 'onnxslim>=0.1.82'"
  done
  [[ -n "${PF[ONNXRUNTIME_ERR]:-}" ]] && warn "onnxruntime not importable; Ultralytics lists it for simplify=True but autoinstall is off. If the export fails on it: $PYTHON -m pip install --user onnxruntime"
  [[ -w "$MODELS_DIR" ]] || block "models dir not writable: $MODELS_DIR"
  TMP_FREE_MB=$(df -Pm /tmp | awk 'NR==2{print $4}')
  MD_FREE_MB=$(df -Pm "$MODELS_DIR" | awk 'NR==2{print $4}')
  echo "free disk:   /tmp ${TMP_FREE_MB} MB, models dir ${MD_FREE_MB} MB"
  (( TMP_FREE_MB < 1024 )) && warn "less than 1 GB free in /tmp (work dir: .pt copy + ONNX + engine)"
  (( MD_FREE_MB < 300 )) && warn "less than 300 MB free in the models dir"
fi
MEM_AVAIL_MB=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
echo "RAM avail:   ${MEM_AVAIL_MB} MB (Jetson GPU memory is this RAM)"
(( MEM_AVAIL_MB < 4096 )) && warn "less than 4 GB RAM available; the TensorRT build may fail (try --workspace 2)"

# ---------------------------------------------------------------- plan
info "plan"
if (( DO_EXPORT )); then
  [[ -f "$TARGET" ]] && echo "engine exists, --force: will rebuild and replace $TARGET"
  echo "1. copy $PT into a /tmp work dir, YOLO(pt).export(format='engine', quantize=16, imgsz=($IMG_H, $IMG_W), device=0, dynamic=False, batch=1, simplify=True, workspace=$WORKSPACE)"
  echo "2. copy to $TARGET.tmp, mv to $TARGET"
else
  echo "1. engine exists, export skipped (--force rebuilds): $TARGET"
fi
echo "3. load test: engine vs $PT on a 640x360 frame from bus.jpg (conf 0.3, retina_masks, $LOAD_TEST_RUNS timed runs)"

if (( ${#BLOCKERS[@]} )); then
  echo
  echo "Preflight BLOCKED (${#BLOCKERS[@]}):"
  printf '  - %s\n' "${BLOCKERS[@]}"
  if (( DRY_RUN )); then
    echo "dry-run: a real run would stop here (exit 3)."
  fi
  exit 3
fi
if (( DRY_RUN )); then
  echo
  echo "dry-run: preflight OK, nothing done (exit 0)."
  exit 0
fi

# ---------------------------------------------------------------- export
WORK=""
TMP_TARGET=""
cleanup() {
  local rc=$?
  if [[ -n "$TMP_TARGET" && -e "$TMP_TARGET" ]]; then
    rm -f -- "${TMP_TARGET:?}"
  fi
  if [[ -n "$WORK" ]]; then
    case "${WORK:?}" in
      *..*) warn "not removing suspicious work dir '$WORK'" ;;
      /tmp/export_yolo_engine.?*) rm -rf -- "${WORK:?}" ;;
      *) warn "not removing unexpected work dir '$WORK'" ;;
    esac
  fi
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if (( DO_EXPORT )); then
  WORK=$(mktemp -d -p /tmp export_yolo_engine.XXXXXX)
  [[ "$WORK" == /tmp/export_yolo_engine.?* && -d "$WORK" ]] || die "mktemp gave an unexpected path: '$WORK'"
  cp -- "$PT" "$WORK/$STEM.pt"
  info "export in $WORK (TensorRT $TRT_VER; this takes minutes, the build is silent for long stretches)"
  T0=$(date +%s)
  ( cd "$WORK" && "$PYTHON" -c "$PY_EXPORT" "$WORK/$STEM.pt" "$IMG_H" "$IMG_W" "$WORKSPACE" ) \
    || die "export failed (work dir $WORK is removed on exit)"
  T1=$(date +%s)
  ENGINE_OUT="$WORK/$STEM.engine"
  [[ -s "$ENGINE_OUT" ]] || die "export produced no $ENGINE_OUT"
  echo "export took $(( T1 - T0 )) s"
  if [[ -f "$TARGET" ]]; then
    echo "replacing existing engine (old sha256 $(sha256sum "$TARGET" | cut -d' ' -f1))"
  fi
  TMP_TARGET="$TARGET.tmp"
  cp -- "$ENGINE_OUT" "$TMP_TARGET"
  mv -f -- "$TMP_TARGET" "$TARGET"
  TMP_TARGET=""
fi

info "engine"
SIZE=$(stat -c %s "$TARGET")
echo "path:     $TARGET"
echo "size:     $SIZE bytes ($(( SIZE / 1024 / 1024 )) MiB)"
echo "sha256:   $(sha256sum "$TARGET" | cut -d' ' -f1)"
echo "TensorRT: $TRT_VER"
(( DO_EXPORT )) || echo "(existing engine, not rebuilt; --force rebuilds)"

# ---------------------------------------------------------------- load test
info "load test"
"$PYTHON" -c "$PY_LOADTEST" "$TARGET" "$PT" "$IMG_H" "$IMG_W" "$LOAD_TEST_RUNS" \
  || die "load test failed for $TARGET"
info "done: $TARGET"
