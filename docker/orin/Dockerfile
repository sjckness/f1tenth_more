# syntax=docker/dockerfile:1
#
# f1tenth Jazzy image for the Orin: the car's container (plan P5, branch
# main-jazzy-rust; first built as Phase 5d D1, option C).
#
#   ubuntu:24.04 userspace on the JetPack 6.2.2 / L4T r36.5 host; the Ubuntu
#     archive pinned to snapshot.ubuntu.com at UBUNTU_SNAPSHOT (backlog L18)
#   + CUDA 12.9.1 for Tegra (22.04 debs; NVIDIA ships no 24.04 Tegra build) and
#     cuda-compat-12-9 (12.9 driver API on the r36.5 kernel driver); the -dev
#     packages' static libraries dropped except cudart_static, cudadevrt and
#     culibos (backlog L17)
#   + cuDNN 9.10.2 (24.04 Tegra; torch 2.8.0 is built against it)
#   + TensorRT 10.3.0.26 (L4T tarball, cp312 wheel; engines are built on the car,
#     `docker_orin.sh engine`)
#   + apt Jazzy from snapshots.ros.org/jazzy/2026-09-11, the Thor's exact set
#   + apt Python first (the Thor's numpy/scipy/matplotlib/...), then pip:
#     torch 2.8.0 / torchvision 0.23.0 (jetson-ai-lab jp6/cu129), osqp 1.1.3,
#     pyarrow 25.0.1, ultralytics 8.4.90, opencv-python 4.11.0.86, jetson-stats 4.3.2,
#     onnx 1.22.0 / onnxslim 0.1.94 / onnxruntime 1.23.2 (the engine export)
#   + the PWA gateway's venv /opt/f1tenth_gateway/venv (gateway-requirements.txt:
#     fastapi, uvicorn, faster-whisper on the CPU)
#   + ZED SDK 5.4.0 for L4T 36.5 (Thor parity)
#   + llama.cpp 42532afff, CUDA sm_87, under /opt/llama.cpp
#   + Rust 1.99.1 + colcon-ros-cargo (RUST=build, the default)
#   + the workspace, colcon --merge-install, Release, in two phases: every
#     package but the Rust ones, then the Rust ones (they read the installed
#     message packages)
#   + the container's own files: docker/orin/container/* (main process under
#     tini, healthcheck, gateway config), scripts/export_yolo_engine.sh, CAR_SYNC.json
# Versions are pinned in docker/orin/jazzy_versions.manifest (+ gateway-requirements.txt);
# image_report.sh diffs a built image against it.
#
# BUILD REQUIREMENTS - read before building:
#   * Build only with docker/orin/build.sh. It stages the sources of one commit
#     (stage_sources.sh), checks the binary inputs (inputs.sha256), passes the
#     host's group IDs and the labels, and tags the image
#     f1tenth:jazzy-orin-<commit> (-norust with RUST=skip).
#   * Host networking for every build (CLAUDE.md "Orin Docker"): this dockerd
#     runs bridge:none, a build without --network=host fails at its first RUN.
#   * THE IMAGE CAN ONLY BE BUILT ON A JETPACK 6 / L4T r36.5 HOST (F7). The ZED
#     SDK's libsl_zed.so links L4T libraries (libnvbufsurface,
#     libnvargus_socketclient, ...) that exist only on the host and that the
#     nvidia runtime CSV-mounts at run time. build.sh passes the host's
#     /usr/lib/aarch64-linux-gnu/nvidia as the `l4t` build context, bind-mounted
#     for the workspace build and the build checks only; nothing from it is
#     copied into the image.
#   * Named build contexts: inputs (fetch_inputs.sh downloads), src (staged
#     sources), l4t (above).
#
# The D1a spike (output/phase5d_spike.md on branch jazzy) found the fixes marked
# F1-F7 below. The run side (compose.yaml, scripts/docker_orin.sh, host units)
# is in docker/orin/compose*.yaml and docker/orin/host/.

ARG BASE=ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55
# The Rust packages (src/rust, build type ament_cargo). RUST=build (default):
# the rust-build stage's toolchain, and the ws stage builds them after
# everything else. RUST=skip: the fallback image, no toolchain, the
# ament_cargo packages ignored; every Rust launch switch defaults to py, so
# both images run the same stack.
#   ORIN_RUST=skip docker/orin/build.sh
ARG RUST=build
# Ubuntu archive snapshot (backlog L18): the state of the archive at this UTC
# time, from https://snapshot.ubuntu.com/ubuntu/<id>/. That URL serves arm64
# too (binary-arm64 indexes and pool files, checked 2026-10-24); the
# ports-style URL snapshot.ubuntu.com/ubuntu-ports/ answers 401. The InRelease
# files are the archive's own, verified with ubuntu-archive-keyring as for
# any mirror. Pinned to the first car-image build (2026-10-24).
# Documented fallback, if the snapshot service cannot be reached from the
# Orin: UBUNTU_SNAPSHOT=none (build.sh: ORIN_UBUNTU_SNAPSHOT=none) keeps the
# base image's live ports.ubuntu.com; the image then records every installed
# version in /opt/f1tenth_ws/DPKG_VERSIONS.txt (it does in both cases) and the
# f1tenth.ubuntu_snapshot label says "none".
ARG UBUNTU_SNAPSHOT=20261024T000000Z
ARG ROS_SNAPSHOT=2026-09-11
# The host's user and the group IDs that must be the host's: device nodes come
# from the host's /dev (video, render, dialout, input), the ZED installer makes
# /usr/local/zed root:zed 0770 (F5), jetson-stats' /run/jtop.sock belongs to
# jtop. build.sh passes the build host's values (getent); these defaults are
# the Orin's (2026-10-08). INPUT_GID empty = keep the image's own; build.sh
# always passes it. docker_orin.sh compares the host's GIDs with the image's
# f1tenth.gid.* labels at every `up` and refuses on a mismatch.
ARG USER_NAME=fabiocar
ARG USER_UID=1004
ARG USER_GID=1004
ARG ZED_GID=1005
ARG JTOP_GID=1003
ARG VIDEO_GID=44
ARG DIALOUT_GID=20
ARG RENDER_GID=104
ARG INPUT_GID=

# ---------------------------------------------------------------------------
FROM ${BASE} AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive TZ=Europe/Rome LANG=C.UTF-8 LC_ALL=C.UTF-8
ARG UBUNTU_SNAPSHOT
# The snapshot is served over https only, and the base image has no CA
# certificates yet: the first update and the ca-certificates install skip the
# TLS peer check. Integrity does not depend on it (apt verifies every index
# against the InRelease signature and every .deb against its hash, the same
# as for the http mirrors); every later apt call verifies TLS too.
RUN if [ "${UBUNTU_SNAPSHOT}" = none ]; then \
      echo "UBUNTU_SNAPSHOT=none: live Ubuntu archive (the documented fallback, versions recorded in DPKG_VERSIONS.txt)"; \
    else \
      [[ "${UBUNTU_SNAPSHOT}" =~ ^[0-9]{8}T[0-9]{6}Z$ ]] || { echo "UBUNTU_SNAPSHOT must be YYYYMMDDTHHMMSSZ or none, got '${UBUNTU_SNAPSHOT}'" >&2; exit 1; }; \
      uri="https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT}/"; \
      printf '%s\n' "Types: deb" "URIs: ${uri}" "Suites: noble noble-updates noble-backports" \
        "Components: main universe restricted multiverse" "Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg" "" \
        "Types: deb" "URIs: ${uri}" "Suites: noble-security" \
        "Components: main universe restricted multiverse" "Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg" \
        > /etc/apt/sources.list.d/ubuntu.sources; \
      apt-get -o Acquire::https::Verify-Peer=false -o Acquire::https::Verify-Host=false update \
      && apt-get -o Acquire::https::Verify-Peer=false -o Acquire::https::Verify-Host=false \
           install -y --no-install-recommends ca-certificates \
      && rm -rf /var/lib/apt/lists/*; \
    fi \
 && echo "${UBUNTU_SNAPSHOT}" > /etc/f1tenth_ubuntu_snapshot
# tzdata first, alone (docker/Dockerfile.jetson-thor lesson (a)). bc ... stress-ng:
# the D3 harness's tools (phase5d_analysis.md section 7). ffmpeg: the gateway's
# and the bot's transcription fallback (OGG/Opus -> WAV) and the mission videos.
RUN apt-get update \
 && apt-get install -y --no-install-recommends tzdata \
 && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg lsb-release zstd file binutils sudo udev usbutils less jq \
      build-essential cmake git pkg-config ccache \
      python3 python3-dev python3-pip python3-venv \
      libusb-1.0-0-dev libopenblas0 libgomp1 libturbojpeg libjpeg-turbo8 libpng-dev libv4l-dev libegl1 libx11-6 \
      nlohmann-json3-dev graphviz ffmpeg \
      bc iproute2 procps sqlite3 stress-ng \
 && rm -rf /var/lib/apt/lists/*
# Groups: video and dialout are static in Debian (44, 20); render and input are
# created by udev's postinst above with the image's next free system GID (render
# was 994, finding F2), so they are moved to the host's GIDs, and any file that
# carried the old GID follows. A GID that another image group already uses
# fails the build instead of merging two groups. zed and jtop are created here
# (the ZED installer and jetson-stats would otherwise pick their own GIDs).
ARG USER_NAME
ARG USER_UID
ARG USER_GID
ARG ZED_GID
ARG JTOP_GID
ARG VIDEO_GID
ARG DIALOUT_GID
ARG RENDER_GID
ARG INPUT_GID
RUN set_gid() { \
      local name="$1" want="$2" have taken; \
      have=$(getent group "$name" | cut -d: -f3 || true); \
      if [ -n "$want" ]; then \
        taken=$(getent group "$want" | cut -d: -f1 || true); \
        [ -z "$taken" ] || [ "$taken" = "$name" ] \
          || { echo "group $name: GID $want is taken by group $taken in the image" >&2; return 1; }; \
      fi; \
      if [ -z "$have" ]; then \
        if [ -n "$want" ]; then groupadd -g "$want" "$name"; else groupadd --system "$name"; fi \
          && echo "group $name: created, GID $(getent group "$name" | cut -d: -f3)"; \
        return; \
      fi; \
      if [ -z "$want" ]; then echo "group $name: image GID $have kept (no host GID given)"; return 0; fi; \
      if [ "$have" = "$want" ]; then echo "group $name: GID $have"; return 0; fi; \
      groupmod -g "$want" "$name" && find / -xdev -gid "$have" -exec chgrp -h "$want" {} + \
        && echo "group $name: GID $have -> $want"; \
    } \
 && set_gid video "${VIDEO_GID}" && set_gid dialout "${DIALOUT_GID}" \
 && set_gid render "${RENDER_GID}" && set_gid input "${INPUT_GID}" \
 && groupadd -g "${ZED_GID}" zed \
 && groupadd -g "${JTOP_GID}" jtop \
 && groupadd -g "${USER_GID}" "${USER_NAME}" \
 && useradd -m -d "/home/${USER_NAME}" -s /bin/bash -u "${USER_UID}" -g "${USER_GID}" \
      -G zed,jtop,video,render,dialout,input "${USER_NAME}"

# ---------------------------------------------------------------------------
FROM base AS cuda
# CUDA 12.9.1 for Tegra from the 22.04 local repo. Not cuda-libraries-dev-12-9:
# it pulls libnvjpeg-dev-12-9, whose unversioned libnvjpeg.so shadows L4T's
# Tegra libjpeg of the same name (F3); torchvision only needs libnvjpeg.so.12.
# cuda-compat-12-9 Pre-Depends on nvidia-l4t-core, the host's L4T package, so
# its payload (/usr/local/cuda-12.9/compat/*) is unpacked without dpkg (F1).
# L17: the -dev packages' static libraries (cublas, cufft, cusolver, cusparse,
# npp, nvrtc, ... ~2-3 GB) are removed in the same layer; nothing in the image
# links them. Kept: libcudart_static.a (CMake's and FindCUDA's default CUDA
# runtime: llama.cpp, the ZED wrapper), libcudadevrt.a and libculibos.a (do not
# match the pattern).
RUN --mount=type=bind,from=inputs,source=cuda-tegra-repo-ubuntu2204-12-9-local_12.9.1-1_arm64.deb,target=/tmp/cuda-repo.deb \
    --mount=type=bind,from=inputs,source=cudnn-local-tegra-repo-ubuntu2404-9.10.2_1.0-1_arm64.deb,target=/tmp/cudnn-repo.deb \
    dpkg -i /tmp/cuda-repo.deb /tmp/cudnn-repo.deb \
 && cp /var/cuda-tegra-repo-ubuntu2204-12-9-local/cuda-tegra-*-keyring.gpg \
       /var/cudnn-local-tegra-repo-ubuntu2404-9.10.2/cudnn-local-tegra-*-keyring.gpg /usr/share/keyrings/ \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      cuda-libraries-12-9 cuda-nvcc-12-9 cuda-minimal-build-12-9 cuda-cupti-12-9 cuda-nvtx-12-9 \
      cuda-cudart-dev-12-9 cuda-driver-dev-12-9 cuda-nvrtc-dev-12-9 cuda-profiler-api-12-9 cuda-cccl-12-9 \
      libcublas-dev-12-9 libcufft-dev-12-9 libcufile-dev-12-9 libcurand-dev-12-9 \
      libcusolver-dev-12-9 libcusparse-dev-12-9 libnpp-dev-12-9 libnvjitlink-dev-12-9 \
      libcudnn9-cuda-12=9.10.2.21-1 \
 && dpkg-deb -x /var/cuda-tegra-repo-ubuntu2204-12-9-local/cuda-compat-12-9_*_arm64.deb / \
 && test -e /usr/local/cuda-12.9/compat/libcuda.so.1 \
 && test ! -e /usr/local/cuda-12.9/targets/aarch64-linux/lib/libnvjpeg.so \
 && dpkg -P cuda-tegra-repo-ubuntu2204-12-9-local cudnn-local-tegra-repo-ubuntu2404-9.10.2 \
 && rm -rf /var/lib/apt/lists/* /var/cuda-tegra-repo-* /var/cudnn-local-* \
 && find /usr/local/cuda-12.9 -name 'lib*_static*.a' ! -name 'libcudart_static.a' -printf '%s %p\n' \
      | sort -n > /usr/local/cuda-12.9/L17_dropped_static_libs.txt \
 && echo "L17: dropping $(wc -l < /usr/local/cuda-12.9/L17_dropped_static_libs.txt) static libraries," \
      "$(awk '{s += $1} END {printf "%.2f", s / 1e9}' /usr/local/cuda-12.9/L17_dropped_static_libs.txt) GB" \
 && awk '{print $2}' /usr/local/cuda-12.9/L17_dropped_static_libs.txt | xargs -r rm -f \
 && test -e /usr/local/cuda-12.9/targets/aarch64-linux/lib/libcudart_static.a
RUN echo "CUDA Version $(ls -d /usr/local/cuda-12.* | sed 's/.*cuda-//')" > /usr/local/cuda/version.txt
# TensorRT 10.3.0.26 from the L4T tarball. TRT dlopens
# libnvinfer_builder_resource.so.10.3.0 by file name, but its SONAME is
# "do_not_link_against_nvinfer_builder_resource", so the ld cache never holds
# that name: /opt/tensorrt/lib must be on LD_LIBRARY_PATH (F4).
RUN --mount=type=bind,from=inputs,source=TensorRT-10.3.0.26.l4t.aarch64-gnu.cuda-12.6.tar.gz,target=/tmp/trt.tgz \
    tar -xzf /tmp/trt.tgz -C /opt \
 && ln -s /opt/TensorRT-10.3.0.26 /opt/tensorrt \
 && rm -rf /opt/tensorrt/data /opt/tensorrt/doc /opt/tensorrt/samples /opt/tensorrt/targets/*/lib/*.a \
 && echo /opt/tensorrt/lib > /etc/ld.so.conf.d/tensorrt.conf && ldconfig
# Search order (F3): the 12.9 compat driver API, then the host's L4T libs
# (CSV-mounted by the nvidia runtime), then CUDA, then TensorRT.
ENV CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:/opt/tensorrt/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/compat:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64:/opt/tensorrt/lib

# ---------------------------------------------------------------------------
FROM cuda AS ros
# apt Jazzy pinned to the Thor's sync. The key's fingerprint is checked; the
# snapshot is served over http, apt verifies its signature.
ARG ROS_SNAPSHOT
COPY keys/ros-snapshots.asc /tmp/ros-snapshots.asc
RUN gpg --show-keys --with-colons /tmp/ros-snapshots.asc | grep -q '^fpr:::::::::4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA:' \
 && gpg --dearmor < /tmp/ros-snapshots.asc > /usr/share/keyrings/ros-snapshots-keyring.gpg \
 && rm /tmp/ros-snapshots.asc \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-snapshots-keyring.gpg] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list
# The Thor's ros-jazzy-* list minus GUI/demo/sim, plus the workspace's apt
# Python (rosdep found the six F6 keys missing in the spike; installed BEFORE
# pip so pip sees the Thor's versions and does not fill them from PyPI).
# ros-jazzy-rosidl-generator-rs (added R0, 2026-10-23; the Thor has it): every
# interface package built in the workspace then also gets its Rust message
# crate, which the RUST=build packages need (rclrs 0.8: this snapshot's 0.5.0
# generator). With RUST=skip nothing compiles them.
COPY ros_packages.txt /tmp/ros_packages.txt
RUN apt-get update \
 && apt-get install -y --no-install-recommends $(cat /tmp/ros_packages.txt) \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-numpy python3-scipy python3-matplotlib python3-jsonschema python3-requests \
      python3-ruamel.yaml python3-tk python3-yaml python3-psutil python3-pydot python3-pytest \
 && rosdep init \
 && rm -rf /var/lib/apt/lists/* /tmp/ros_packages.txt

# ---------------------------------------------------------------------------
FROM ros AS py
# PEP 668: system-wide pip on 24.04. Order: apt (above), then pip with the
# constraints, then `pip check` and the no-PyPI-CUDA check, both fatal.
# Long timeout and retries: the Orin downloads over Wi-Fi, and a PyPI read
# timeout killed the first build on the Orin (2026-10-25).
ENV PIP_BREAK_SYSTEM_PACKAGES=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_CONSTRAINT=/etc/pip-constraints.txt PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=10
COPY constraints.txt /etc/pip-constraints.txt
RUN --mount=type=bind,from=inputs,source=torch-2.8.0-cp312-cp312-linux_aarch64.whl,target=/tmp/w/torch-2.8.0-cp312-cp312-linux_aarch64.whl \
    --mount=type=bind,from=inputs,source=torchvision-0.23.0-cp312-cp312-linux_aarch64.whl,target=/tmp/w/torchvision-0.23.0-cp312-cp312-linux_aarch64.whl \
    pip install /tmp/w/torch-2.8.0-cp312-cp312-linux_aarch64.whl /tmp/w/torchvision-0.23.0-cp312-cp312-linux_aarch64.whl \
 && pip install /opt/tensorrt/python/tensorrt-10.3.0-cp312-none-linux_aarch64.whl \
 && pip install osqp==1.1.3 pyarrow==25.0.1 ultralytics==8.4.90 opencv-python==4.11.0.86 jetson-stats==4.3.2 \
 # jetson-stats, installed as root, writes a systemd unit for the jtop server;
 # the container is a jtop CLIENT of the host's /run/jtop.sock, so drop it.
 && rm -f /etc/systemd/system/jtop.service /etc/systemd/system/multi-user.target.wants/jtop.service \
 && pip check \
 && if pip list --format=freeze | grep -i -E '^(nvidia-|cuda-|triton)' | grep -v -i '^nvidia-ml-py=='; then \
      echo 'PyPI CUDA package pulled in - refusing' >&2; exit 1; fi
# The TensorRT engine export (scripts/export_yolo_engine.sh, run on the car by
# `docker_orin.sh engine`): Ultralytics exports through ONNX with simplify=True,
# i.e. onnx + onnxslim (+ onnxruntime). The Orin Humble host's versions
# (output/perf/yolo_recon.md), every dependency pinned too (resolved for
# cp312 aarch64 against this image's system site on 2026-10-24; the rest is
# already here: numpy 1.26.4, sympy, packaging, typing_extensions, mpmath).
# onnxruntime is the CPU build; it is also faster-whisper's (gateway venv).
# typing_extensions 4.16.0: the gateway venv's pydantic and anyio need >= 4.16
# from this system site (apt noble has 4.10; torch's resolution took 4.16.0 in
# D1, unpinned until here).
RUN pip install --only-binary=:all: onnx==1.22.0 onnxslim==0.1.94 onnxruntime==1.23.2 \
      ml_dtypes==0.5.4 protobuf==7.36.2 colorama==0.4.6 coloredlogs==15.0.1 flatbuffers==25.12.19 humanfriendly==10.0 \
      typing_extensions==4.16.0 \
 && pip check \
 && python3 -c 'import onnx, onnxslim, onnxruntime; print("onnx", onnx.__version__, "onnxslim", onnxslim.__version__, "onnxruntime", onnxruntime.__version__)' \
 && if pip list --format=freeze | grep -i -E '^(nvidia-|cuda-|triton)' | grep -v -i '^nvidia-ml-py=='; then \
      echo 'PyPI CUDA package pulled in - refusing' >&2; exit 1; fi
# The PWA gateway's venv (src/f1tenth_gateway, run by the container main). Its
# own packages only: --system-site-packages gives it the apt Python, rclpy's
# dependencies and the onnxruntime above. gateway-requirements.txt pins EVERY
# package the venv installs (top level and transitive), installed with
# --no-deps, so the set is exactly the recorded one; pip check then proves
# nothing is missing. --ignore-installed: a pin the system site already has
# at the same version is installed into the venv all the same (pip would
# skip it otherwise, and build_checks.sh's venv = lock equality would fail
# late). Root-owned and read-only at run time.
COPY gateway-requirements.txt /opt/f1tenth_gateway/requirements.txt
RUN python3 -m venv --system-site-packages /opt/f1tenth_gateway/venv \
 && /opt/f1tenth_gateway/venv/bin/pip install --no-deps --ignore-installed --only-binary=:all: -r /opt/f1tenth_gateway/requirements.txt \
 && /opt/f1tenth_gateway/venv/bin/pip check \
 && site=$(/opt/f1tenth_gateway/venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])') \
 && if /opt/f1tenth_gateway/venv/bin/pip list --path "$site" --format=freeze | grep -i -E '^(nvidia-|cuda-|triton)'; then \
      echo 'PyPI CUDA package in the gateway venv - refusing' >&2; exit 1; fi \
 && /opt/f1tenth_gateway/venv/bin/python -c 'import fastapi, uvicorn, websockets, multipart, qrcode, faster_whisper, ctranslate2, av, tokenizers, huggingface_hub; print("gateway venv: fastapi", fastapi.__version__, "uvicorn", uvicorn.__version__, "faster_whisper", faster_whisper.__version__, "ctranslate2", ctranslate2.__version__, "av", av.__version__)'

# ---------------------------------------------------------------------------
FROM py AS zed
# skip_cuda: CUDA comes from this image. skip_python: no stack code imports
# pyzed (pyzed 5.x would need numpy>=2). skip_drivers: ZED X / GMSL only.
# skip_tools: no firmware/diagnostic tools in the image. The installer never
# runs ldconfig (F5), so it is run here.
ARG ZED_RUN=ZED_SDK_Tegra_L4T36.5_v5.4.0.zstd.run
ARG ZED_GID
RUN --mount=type=bind,from=inputs,source=${ZED_RUN},target=/tmp/zed.run \
    apt-get update \
 && cp /tmp/zed.run /tmp/zed_install.run && chmod +x /tmp/zed_install.run \
 && /tmp/zed_install.run -- silent skip_tools skip_python skip_drivers skip_cuda 2>&1 | tee /usr/local/zed_install.log \
 && rm -f /tmp/zed_install.run && rm -rf /var/lib/apt/lists/* \
 && ldconfig && ldconfig -p | grep libsl_zed.so > /dev/null \
 && test "$(stat -c %g /usr/local/zed)" = "${ZED_GID}"
ENV ZED_DIR=/usr/local/zed

# ---------------------------------------------------------------------------
FROM cuda AS llama
# llama.cpp at the Orin Humble host's commit, rebuilt for sm_87 (the host
# binary has no Orin cubins and PTX-JITs every kernel at first use: ~2 min cold
# start in a fresh container, phase5d_spike.md section 5.4). Same options as the
# host build otherwise (CUDA, shared libs, Release, native CPU flags); tests
# and examples off. Linked against the CUDA driver STUB: on LD_LIBRARY_PATH
# libcuda.so.1 is the compat driver, whose own dependencies (libnvrm_gpu, ...)
# are host L4T libs absent at build time, so linking the executables against
# it fails. The stub carries the same SONAME; at run time the compat driver
# is loaded, and build_checks.sh's ldd -r proves it resolves.
ARG LLAMA_REPO=https://github.com/ggml-org/llama.cpp.git
ARG LLAMA_COMMIT=42532afff43910e619a650c1704525b3acbbec5a
RUN --mount=type=cache,id=f1tenth-orin-ccache-llama,target=/root/.ccache \
    mkdir -p /tmp/cuda-stub && ln -s /usr/local/cuda/lib64/stubs/libcuda.so /tmp/cuda-stub/libcuda.so.1 \
 && export LD_LIBRARY_PATH=/tmp/cuda-stub:/usr/local/cuda/lib64 \
 && git init -q /tmp/llama.cpp && cd /tmp/llama.cpp \
 && git remote add origin "${LLAMA_REPO}" \
 && git fetch -q --depth 1 origin "${LLAMA_COMMIT}" && git checkout -q FETCH_HEAD \
 && test "$(git rev-parse HEAD)" = "${LLAMA_COMMIT}" \
 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_SHARED_LIBS=ON \
      -DLLAMA_BUILD_TESTS=OFF -DLLAMA_BUILD_EXAMPLES=OFF \
      -DCMAKE_INSTALL_PREFIX=/opt/llama.cpp -DCMAKE_INSTALL_RPATH='$ORIGIN/../lib' \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_CUDA_COMPILER_LAUNCHER=ccache \
 && cmake --build build -j"$(nproc)" \
 && cmake --install build \
 && test -x /opt/llama.cpp/bin/llama-server \
 && { echo "commit ${LLAMA_COMMIT}"; echo "repo ${LLAMA_REPO}"; \
      echo "cmake -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release"; \
      echo "nvcc $(nvcc --version | tail -1)"; } > /opt/llama.cpp/BUILD_INFO \
 && rm -rf /tmp/llama.cpp /tmp/cuda-stub

# ---------------------------------------------------------------------------
# Rust toolchain for the ament_cargo packages (RUST=build).
#
# Prepared on linus (main-jazzy-rust R0, 2026-10-23); its first build is the
# car image's (plan P7). What the Rust packages need from this image:
#   * rclrs 0.8 + ros-env 0.3 (plan P3.1): the 2026-09-11 snapshot ships
#     ros-jazzy-rosidl-generator-rs 0.5.0 (ros_packages.txt), whose message
#     crates use rosidl_runtime_rs 0.7. ros-env finds the message packages by
#     scanning the install prefix, so the Rust packages are built after every
#     other package is installed (ws stage, phase B).
#   * f1tenth_mpc_core's solver core (R2) depends on osqp-sys =1.0.1, whose
#     build.rs compiles the vendored OSQP C 1.0.0 with CMake; OSQP's CMake
#     fetches QDLDL v0.1.8 with FetchContent. The image builds it offline
#     (plan P3.3): src/rust/build_env.sh points CMAKE_TOOLCHAIN_FILE_<target>
#     at osqp_offline.cmake (FETCHCONTENT_SOURCE_DIR_QDLDL = the vendored
#     src/rust/third_party/qdldl, FETCHCONTENT_FULLY_DISCONNECTED), and phase B
#     runs with GIT_ALLOW_PROTOCOL=file, so a network clone fails the build.
#   * The crates come from crates.io (registry cache mount; Cargo.lock checked
#     unchanged after the build).
# Pins: rustup-init 1.29.1 (sha256 from static.rust-lang.org's
# rustup-init.sha256 for aarch64-unknown-linux-gnu, fetched 2026-10-23),
# toolchain 1.99.1 = linus's (the crates need >= 1.85), cargo-ament-build
# 0.1.11 --locked, colcon-cargo / colcon-ros-cargo 0.2.0 from pip as on linus
# (colcon-ros-cargo also requires the cargo-ament-build 0.1.11 wheel and toml;
# cargo's copy in /opt/rust/cargo/bin comes first on PATH).
FROM zed AS rust-build
ARG RUSTUP_VERSION=1.29.1
ARG RUSTUP_INIT_SHA256=15f6e4ce9f583b929c996c91562bad6d4454f3281de858b02cdfdef615fac433
ARG RUST_TOOLCHAIN=1.99.1
ARG CARGO_AMENT_BUILD_VERSION=0.1.11
ARG COLCON_CARGO_VERSION=0.2.0
ARG COLCON_ROS_CARGO_VERSION=0.2.0
ENV RUSTUP_HOME=/opt/rust/rustup CARGO_HOME=/opt/rust/cargo PATH=/opt/rust/cargo/bin:${PATH}
RUN --mount=type=cache,id=f1tenth-orin-cargo-registry,target=/opt/rust/cargo/registry \
    curl --proto '=https' --tlsv1.2 -sSfL -o /tmp/rustup-init \
      "https://static.rust-lang.org/rustup/archive/${RUSTUP_VERSION}/aarch64-unknown-linux-gnu/rustup-init" \
 && echo "${RUSTUP_INIT_SHA256}  /tmp/rustup-init" | sha256sum -c - \
 && chmod +x /tmp/rustup-init \
 && /tmp/rustup-init -y --no-modify-path --profile minimal --default-toolchain "${RUST_TOOLCHAIN}" \
 && rm -f /tmp/rustup-init \
 && rustc --version && cargo --version \
 && cargo install --locked cargo-ament-build --version "${CARGO_AMENT_BUILD_VERSION}" \
 && pip install colcon-cargo=="${COLCON_CARGO_VERSION}" colcon-ros-cargo=="${COLCON_ROS_CARGO_VERSION}" \
      cargo-ament-build=="${CARGO_AMENT_BUILD_VERSION}" \
 && pip check \
 && python3 -c 'import colcon_cargo, colcon_ros_cargo' \
 && chmod -R a+rX /opt/rust

# RUST=skip: no toolchain; the ws stage ignores the ament_cargo packages.
FROM zed AS rust-skip

# ---------------------------------------------------------------------------
FROM rust-${RUST} AS ws
ARG USER_NAME
ARG USER_UID
ARG USER_GID
ARG ZED_GID
ARG RUST
COPY --from=llama /opt/llama.cpp /opt/llama.cpp
WORKDIR /opt/f1tenth_ws
COPY --from=src src /opt/f1tenth_ws/src
COPY --from=src scripts/env /opt/f1tenth_ws/scripts/env
COPY set_installed_params.py build_checks.sh entrypoint.sh cargo_lock_check.py /opt/f1tenth_ws/
# Workspace build, phase A: every package except the ament_cargo ones. The
# host's L4T libs are bind-mounted for this step only (F7, header);
# --merge-install, Release, the Thor's skip list. A colcon without
# colcon-ros-cargo (RUST=skip) identifies the Rust packages from package.xml
# ("ros.ament_cargo"; "ament_cargo" with the plugin). No package declares a
# dependency on them (mpc_controller finds f1tenth_mpc_core, and
# f1tenth_perception f1tenth_wall_tracker_core, through ament_index only when
# corridor_impl / mpc_solver_impl / wall_tracker_impl is rust;
# ackermann_mux.launch.py launches f1tenth_hard_stop_rs only when hard_stop_impl
# is rust); their launch switches (guard:=, costmap_boundary_impl:=, impl:=,
# corridor_impl:=, mpc_solver_impl:=, hard_stop_impl:=, wall_tracker_impl:=)
# default to off/py.
RUN --mount=type=bind,from=l4t,target=/usr/lib/aarch64-linux-gnu/nvidia \
    --mount=type=cache,id=f1tenth-orin-ccache-ws,target=/root/.ccache \
    source /opt/ros/jazzy/setup.bash \
 && case "${RUST}" in build|skip) ;; *) echo "RUST must be build or skip, got '${RUST}'" >&2; exit 1 ;; esac \
 && rust_pkgs=$(colcon --log-base /dev/null list -t | awk '$3 ~ /^\((ros\.)?ament_cargo\)$/ {printf "%s ", $1}') \
 && echo "${rust_pkgs}" > /opt/f1tenth_ws/rust_packages.txt \
 && echo "RUST=${RUST}: ament_cargo packages: ${rust_pkgs:-none}" | tee /opt/f1tenth_ws/colcon_build.log \
 && echo "== phase A: every package but the ament_cargo ones" | tee -a /opt/f1tenth_ws/colcon_build.log \
 && colcon build --merge-install \
      --packages-skip f1tenth_sim f1tenth_more husarion_gz_worlds \
      ${rust_pkgs:+--packages-ignore ${rust_pkgs}} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      --event-handlers console_direct- console_cohesion+ summary+ \
      2>&1 | tee -a /opt/f1tenth_ws/colcon_build.log \
 && rm -rf build log
# Phase B (RUST=build): the ament_cargo packages, after phase A installed the
# message packages they read. Offline QDLDL through src/rust/build_env.sh
# (rust-build stage comment, plan P3.3): it must set
# CMAKE_TOOLCHAIN_FILE_<host triple, '-' -> '_'> to an existing
# osqp_offline.cmake; a FetchContent sub-build of QDLDL fails the step. The
# crates registry is a cache mount (never in a layer). Cargo.lock: the
# resolved packages must equal the committed ones (cargo_lock_check.py; the
# [[patch.unused]] list colcon-ros-cargo's [patch] section makes cargo rewrite
# is reported only); the committed file is put back afterwards, so
# /opt/f1tenth_ws/src stays byte-equal to the staged commit.
RUN --mount=type=cache,id=f1tenth-orin-cargo-registry,target=/opt/rust/cargo/registry \
    --mount=type=cache,id=f1tenth-orin-cargo-git,target=/opt/rust/cargo/git \
    if [ "${RUST}" != build ]; then echo "RUST=${RUST}: no phase B"; exit 0; fi \
 && source /opt/ros/jazzy/setup.bash && source /opt/f1tenth_ws/install/setup.bash \
 && rust_pkgs=$(cat /opt/f1tenth_ws/rust_packages.txt) \
 && { [ -n "${rust_pkgs// /}" ] || { echo "RUST=build but colcon lists no ament_cargo package" >&2; exit 1; }; } \
 && { [ -f src/rust/build_env.sh ] || { echo "src/rust/build_env.sh missing: the offline QDLDL hook (plan P3.3)" >&2; exit 1; }; } \
 && { [ -d src/rust/third_party/qdldl ] || { echo "src/rust/third_party/qdldl missing: vendored QDLDL (plan P3.3)" >&2; exit 1; }; } \
 && source src/rust/build_env.sh \
 && triple=$(rustc -vV | sed -n 's/^host: //p') \
 && tc_var="CMAKE_TOOLCHAIN_FILE_${triple//-/_}" \
 && tc="${!tc_var:-}" \
 && { [ "$(basename "${tc:-none}")" = osqp_offline.cmake ] && [ -f "$tc" ] \
      || { echo "build_env.sh did not set ${tc_var} to an existing osqp_offline.cmake (got '${tc}')" >&2; exit 1; }; } \
 && echo "== phase B: ${rust_pkgs}(offline QDLDL: ${tc_var}=${tc})" | tee -a /opt/f1tenth_ws/colcon_build.log \
 && cp src/rust/Cargo.lock /tmp/Cargo.lock.staged \
 && GIT_ALLOW_PROTOCOL=file colcon build --merge-install --packages-select ${rust_pkgs} \
      --event-handlers console_direct- console_cohesion+ summary+ \
      2>&1 | tee -a /opt/f1tenth_ws/colcon_build.log \
 && if find build -type d -name 'qdldl-subbuild' | grep -q .; then \
      echo "QDLDL was fetched (_deps/qdldl-subbuild exists): the offline hook did not apply" >&2; exit 1; fi \
 && python3 /opt/f1tenth_ws/cargo_lock_check.py /tmp/Cargo.lock.staged src/rust/Cargo.lock \
 && cp /tmp/Cargo.lock.staged src/rust/Cargo.lock \
 && rm -rf build log .cargo /tmp/Cargo.lock.staged
# The container's llama-server (the image's sm_87 build) and the TensorRT
# engine the car builds into the f1tenth_car_trt volume (`docker_orin.sh
# engine`): selected in the INSTALLED stack_params.yaml only, the repo
# defaults stay the Humble host's (decision 2026-10-07, backlog L19). Checked
# edit: fails if those keys or their old values changed.
RUN python3 /opt/f1tenth_ws/set_installed_params.py \
      /opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml
# Directories the stack writes at run time, owned by the runtime user. The
# compose file mounts host state over most of them; the image owns them so a
# run without a mount still works and so no mount point is root-owned.
#   * campaign-final nodes write into the source tree they find above install/:
#     mpc_corr opens src/f1tenth_control/corridors_jsons/*.csv in its
#     constructor (MPC_corr.py _resolve_debug_output_path, no try/except), the BT
#     writes src/f1tenth_behavior/mission_reports/*.json, mission_render writes
#     <ws>/mission_videos/, llm_planner_node writes
#     share/f1tenth_behavior/missions/llm_generated/.
#   * /var/cache/f1tenth/{cuda,trt}: CUDA JIT cache and the TensorRT engines
#     (named volumes; a fresh volume inherits the directory's owner).
#   * /usr/local/zed/resources: the ZED SDK's downloaded models (named volume).
#   * HOME subtrees that compose mounts files or directories into: Docker
#     would create a missing mount parent owned by root.
RUN install -d -o "${USER_UID}" -g "${USER_GID}" -m 0755 \
      /opt/f1tenth_ws/src/f1tenth_control/corridors_jsons \
      /opt/f1tenth_ws/src/f1tenth_behavior/mission_reports \
      /opt/f1tenth_ws/mission_videos \
      /opt/f1tenth_ws/install/share/f1tenth_behavior/missions/llm_generated \
      /var/cache/f1tenth /var/cache/f1tenth/cuda /var/cache/f1tenth/trt \
 && install -d -o "${USER_UID}" -g "${ZED_GID}" -m 0775 /usr/local/zed/resources \
 && cd "/home/${USER_NAME}" \
 && install -d -o "${USER_UID}" -g "${USER_GID}" -m 0755 \
      .config .config/f1tenth_gateway .config/f1tenth_car \
      .local .local/state .local/state/f1tenth_ctl .local/state/f1tenth_gateway .local/state/f1tenth_archive_trigger \
      .local/share .local/share/f1tenth_gateway .local/share/f1tenth_gateway/www \
      .cache .cache/huggingface .ros f1tenth_archive projects projects/llm projects/llm/models \
 && chmod 0755 /opt/f1tenth_ws/entrypoint.sh \
 && echo 'source /opt/f1tenth_ws/scripts/env/jazzy.sh' >> /etc/bash.bashrc \
 # f1tenth_ctl, the gateway and the bot's transcriber run from src/ (their
 # bin/ launchers); the runtime user cannot write __pycache__ there, so
 # compile once here (every Telegram command starts an f1tenth_ctl).
 && python3 -m compileall -q /opt/f1tenth_ws/src/f1tenth_ctl /opt/f1tenth_ws/src/f1tenth_gateway \
      /opt/f1tenth_ws/src/f1tenth_telegram \
 && dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' | awk '$1=="ii"{print $2, $3}' \
      > /opt/f1tenth_ws/DPKG_VERSIONS.txt
# Persistent CUDA JIT cache (torch / TRT / any PTX): a named volume at run time.
ENV CUDA_CACHE_PATH=/var/cache/f1tenth/cuda CUDA_CACHE_MAXSIZE=1073741824
# The container's own files (docker/orin/container: main process, healthcheck,
# gateway config) and the engine export script, at their repo paths under
# /opt/f1tenth_ws. After the workspace build, so a change here rebuilds only
# the layers below.
COPY container/ /opt/f1tenth_ws/docker/orin/container/
COPY --from=src scripts/export_yolo_engine.sh /opt/f1tenth_ws/scripts/export_yolo_engine.sh
RUN chmod 0755 /opt/f1tenth_ws/scripts/export_yolo_engine.sh \
 && find /opt/f1tenth_ws/docker/orin/container -name '*.sh' -exec chmod 0755 {} + \
 && find /opt/f1tenth_ws/docker/orin/container -name '*.py' -exec chmod 0644 {} +
# Fatal build checks: pip check, no PyPI CUDA, apt Python not shadowed, imports,
# the gateway venv, onnx, ffmpeg, group IDs, writable-dir ownership, the
# f1tenth_ctl process backend, the ament index, F3/F5/L17 invariants, the Rust
# binaries (RUST=build), and the CLAUDE.md ldd -r scan (needs the L4T libs).
ARG VIDEO_GID
ARG DIALOUT_GID
ARG RENDER_GID
ARG INPUT_GID
ARG JTOP_GID
RUN --mount=type=bind,from=l4t,target=/usr/lib/aarch64-linux-gnu/nvidia \
    source /opt/ros/jazzy/setup.bash && source /opt/f1tenth_ws/install/setup.bash \
 && USER_NAME="${USER_NAME}" USER_UID="${USER_UID}" ZED_GID="${ZED_GID}" JTOP_GID="${JTOP_GID}" \
    VIDEO_GID="${VIDEO_GID}" DIALOUT_GID="${DIALOUT_GID}" RENDER_GID="${RENDER_GID}" INPUT_GID="${INPUT_GID}" \
    RUST="${RUST}" bash /opt/f1tenth_ws/build_checks.sh
# Per-commit metadata last, so a commit that does not change the sources
# reuses every layer above: SOURCES (commit, branch, submodules, tree hash) and
# CAR_SYNC.json (the car's native tree this image reproduces,
# docker/orin/car_sync.json; scripts/orin/drift_check.sh compares the car
# against it at every `docker_orin.sh up`). Its sha256 must equal the label's.
COPY --from=src SOURCES CAR_SYNC.json /opt/f1tenth_ws/
ARG CAR_SYNC_SHA256=unknown
RUN python3 -c 'import json, sys; d = json.load(open(sys.argv[1])); assert d["schema"] == 1 and d["native_head"], d; print("CAR_SYNC native_head", d["native_head"])' \
      /opt/f1tenth_ws/CAR_SYNC.json \
 && if [ "${CAR_SYNC_SHA256}" != unknown ]; then \
      echo "${CAR_SYNC_SHA256}  /opt/f1tenth_ws/CAR_SYNC.json" | sha256sum -c -; fi
ARG GIT_SHA=unknown
ARG GIT_BRANCH=unknown
ARG GIT_DESCRIBE=unknown
ARG SOURCES_TREE=unknown
ARG CAR_SYNC_HEAD=unknown
ARG UBUNTU_SNAPSHOT
ARG ROS_SNAPSHOT
LABEL org.opencontainers.image.title="f1tenth Jazzy car (Orin, JetPack 6 / L4T r36.5)" \
      org.opencontainers.image.revision="${GIT_SHA}" \
      org.opencontainers.image.version="${GIT_DESCRIBE}" \
      org.opencontainers.image.source="f1tenth_more ${GIT_BRANCH}, docker/orin/Dockerfile" \
      f1tenth.branch="${GIT_BRANCH}" \
      f1tenth.rust="${RUST}" \
      f1tenth.sources_tree="${SOURCES_TREE}" \
      f1tenth.car_sync.sha256="${CAR_SYNC_SHA256}" \
      f1tenth.car_sync.native_head="${CAR_SYNC_HEAD}" \
      f1tenth.ros_snapshot="${ROS_SNAPSHOT}" \
      f1tenth.ubuntu_snapshot="${UBUNTU_SNAPSHOT}" \
      f1tenth.tensorrt="10.3.0.26" \
      f1tenth.gid.video="${VIDEO_GID}" \
      f1tenth.gid.render="${RENDER_GID}" \
      f1tenth.gid.dialout="${DIALOUT_GID}" \
      f1tenth.gid.input="${INPUT_GID}" \
      f1tenth.gid.zed="${ZED_GID}" \
      f1tenth.gid.jtop="${JTOP_GID}"
# The nvidia runtime injects the GPU devices and the host's L4T libraries only
# when NVIDIA_VISIBLE_DEVICES is set; NVIDIA's own images set it, ubuntu:24.04
# does not (found by image_report.sh: no libnvrm_gpu, no TensorRT import).
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
# Role car: the Discovery Server's UDP 11811 bound and the gateway's TCP
# 127.0.0.1:8770 answering (W1's healthcheck.sh); any other F1T_CONTAINER_ROLE
# is healthy by definition. docker_orin.sh up waits for "healthy".
HEALTHCHECK --interval=10s --timeout=5s --start-period=90s --retries=3 \
  CMD ["/opt/f1tenth_ws/docker/orin/container/healthcheck.sh"]
# By name, not uid:gid: with a numeric USER Docker does not apply the user's
# /etc/group memberships (zed, jtop, video, ...). compose passes the GIDs anyway.
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}
ENTRYPOINT ["/opt/f1tenth_ws/entrypoint.sh"]
CMD ["bash"]
