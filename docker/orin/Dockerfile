# syntax=docker/dockerfile:1
#
# f1tenth Jazzy image for the Orin (Phase 5d D1, option C).
#
#   ubuntu:24.04 userspace on the JetPack 6.2.2 / L4T r36.5 host
#   + CUDA 12.9.1 for Tegra (22.04 debs; NVIDIA ships no 24.04 Tegra build) and
#     cuda-compat-12-9 (12.9 driver API on the r36.5 kernel driver)
#   + cuDNN 9.10.2 (24.04 Tegra; torch 2.8.0 is built against it)
#   + TensorRT 10.3.0.26 (L4T tarball, cp312 wheel; engines are rebuilt in the image)
#   + apt Jazzy from snapshots.ros.org/jazzy/2026-09-11, the Thor's exact set
#   + apt Python first (the Thor's numpy/scipy/matplotlib/...), then pip:
#     torch 2.8.0 / torchvision 0.23.0 (jetson-ai-lab jp6/cu129), osqp 1.1.3,
#     pyarrow 25.0.1, ultralytics 8.4.90, opencv-python 4.11.0.86, jetson-stats 4.3.2
#   + ZED SDK 5.4.0 for L4T 36.5 (Thor parity)
#   + llama.cpp 42532afff, CUDA sm_87, under /opt/llama.cpp
#   + the workspace (32 packages), colcon --merge-install, Release
# Versions are pinned in docker/orin/jazzy_versions.manifest; the resolved set
# and every deviation are in output/phase5d_image.md.
#
# BUILD REQUIREMENTS - read before building:
#   * Build only with docker/orin/build.sh. It stages the sources of one commit
#     (stage_sources.sh), checks the binary inputs (inputs.sha256) and tags the
#     image f1tenth:jazzy-orin-<commit>.
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
# The D1a spike (docker/orin/spike/, output/phase5d_spike.md) found the fixes
# marked F1-F7 below.

ARG BASE=ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55

# ---------------------------------------------------------------------------
FROM ${BASE} AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive TZ=Europe/Rome LANG=C.UTF-8 LC_ALL=C.UTF-8
# tzdata first, alone (docker/Dockerfile.jetson-thor lesson (a)). The last
# line is the D3 harness's tools (phase5d_analysis.md section 7).
RUN apt-get update \
 && apt-get install -y --no-install-recommends tzdata \
 && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg lsb-release zstd file binutils sudo udev usbutils less jq \
      build-essential cmake git pkg-config ccache \
      python3 python3-dev python3-pip python3-venv \
      libusb-1.0-0-dev libopenblas0 libgomp1 libturbojpeg libjpeg-turbo8 libpng-dev libv4l-dev libegl1 libx11-6 \
      nlohmann-json3-dev graphviz \
      bc iproute2 procps sqlite3 stress-ng \
 && rm -rf /var/lib/apt/lists/*
# The host's user and the groups whose GIDs files in the image must carry.
# zed: the ZED installer makes /usr/local/zed root:zed 0770 and would create
# its own `zed` group with the image's next free GID (F5); jtop: jetson-stats
# does the same for /run/jtop.sock's group. Both must be the host's GIDs.
ARG USER_NAME=fabiocar
ARG USER_UID=1004
ARG USER_GID=1004
ARG ZED_GID=1005
ARG JTOP_GID=1003
RUN groupadd -g ${ZED_GID} zed \
 && groupadd -g ${JTOP_GID} jtop \
 && groupadd -g ${USER_GID} ${USER_NAME} \
 && useradd -m -d /home/${USER_NAME} -s /bin/bash -u ${USER_UID} -g ${USER_GID} -G zed,jtop,video ${USER_NAME}

# ---------------------------------------------------------------------------
FROM base AS cuda
# CUDA 12.9.1 for Tegra from the 22.04 local repo. Not cuda-libraries-dev-12-9:
# it pulls libnvjpeg-dev-12-9, whose unversioned libnvjpeg.so shadows L4T's
# Tegra libjpeg of the same name (F3); torchvision only needs libnvjpeg.so.12.
# cuda-compat-12-9 Pre-Depends on nvidia-l4t-core, the host's L4T package, so
# its payload (/usr/local/cuda-12.9/compat/*) is unpacked without dpkg (F1).
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
 && rm -rf /var/lib/apt/lists/* /var/cuda-tegra-repo-* /var/cudnn-local-*
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
ARG ROS_SNAPSHOT=2026-09-11
COPY keys/ros-snapshots.asc /tmp/ros-snapshots.asc
RUN gpg --show-keys --with-colons /tmp/ros-snapshots.asc | grep -q '^fpr:::::::::4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA:' \
 && gpg --dearmor < /tmp/ros-snapshots.asc > /usr/share/keyrings/ros-snapshots-keyring.gpg \
 && rm /tmp/ros-snapshots.asc \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-snapshots-keyring.gpg] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list
# The Thor's ros-jazzy-* list minus GUI/demo/sim, plus the workspace's apt
# Python (rosdep found the six F6 keys missing in the spike; installed BEFORE
# pip so pip sees the Thor's versions and does not fill them from PyPI).
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
ENV PIP_BREAK_SYSTEM_PACKAGES=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_CONSTRAINT=/etc/pip-constraints.txt
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

# ---------------------------------------------------------------------------
FROM py AS zed
# skip_cuda: CUDA comes from this image. skip_python: no stack code imports
# pyzed (pyzed 5.x would need numpy>=2). skip_drivers: ZED X / GMSL only.
# skip_tools: no firmware/diagnostic tools in the image. The installer never
# runs ldconfig (F5), so it is run here.
ARG ZED_RUN=ZED_SDK_Tegra_L4T36.5_v5.4.0.zstd.run
ARG ZED_GID=1005
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
# and examples off.
ARG LLAMA_REPO=https://github.com/ggml-org/llama.cpp.git
ARG LLAMA_COMMIT=42532afff43910e619a650c1704525b3acbbec5a
RUN --mount=type=cache,id=f1tenth-orin-ccache-llama,target=/root/.ccache \
    git init -q /tmp/llama.cpp && cd /tmp/llama.cpp \
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
 && rm -rf /tmp/llama.cpp

# ---------------------------------------------------------------------------
FROM zed AS ws
ARG USER_NAME=fabiocar
ARG USER_UID=1004
ARG USER_GID=1004
ARG ZED_GID=1005
COPY --from=llama /opt/llama.cpp /opt/llama.cpp
WORKDIR /opt/f1tenth_ws
COPY --from=src src /opt/f1tenth_ws/src
COPY --from=src scripts/env /opt/f1tenth_ws/scripts/env
COPY set_llama_paths.py build_checks.sh entrypoint.sh /opt/f1tenth_ws/
# Workspace build. The host's L4T libs are bind-mounted for this step only
# (F7, header); --merge-install, Release, the Thor's skip list.
RUN --mount=type=bind,from=l4t,target=/usr/lib/aarch64-linux-gnu/nvidia \
    --mount=type=cache,id=f1tenth-orin-ccache-ws,target=/root/.ccache \
    source /opt/ros/jazzy/setup.bash \
 && colcon build --merge-install \
      --packages-skip f1tenth_sim f1tenth_more husarion_gz_worlds \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      --event-handlers console_direct- console_cohesion+ summary+ \
      2>&1 | tee /opt/f1tenth_ws/colcon_build.log \
 && rm -rf build log
# The container's llama-server: the image's sm_87 build, selected through the
# existing llama_server_path / llama_server_cwd parameters in the INSTALLED
# stack_params.yaml (decision 2026-10-07; the repo default stays the Humble
# host's path). Checked edit: fails if those keys or their values changed.
RUN python3 /opt/f1tenth_ws/set_llama_paths.py \
      /opt/f1tenth_ws/install/share/f1tenth_params/config/stack_params.yaml \
      /opt/llama.cpp/bin/llama-server /opt/llama.cpp
# Fatal build checks: pip check, no PyPI CUDA, apt Python not shadowed, imports,
# F3/F5 invariants, and the CLAUDE.md ldd -r scan (needs the L4T libs again).
RUN --mount=type=bind,from=l4t,target=/usr/lib/aarch64-linux-gnu/nvidia \
    source /opt/ros/jazzy/setup.bash && source /opt/f1tenth_ws/install/setup.bash \
 && ZED_GID=${ZED_GID} bash /opt/f1tenth_ws/build_checks.sh
# Persistent CUDA JIT cache (torch / TRT / any PTX): D2 mounts a named volume
# here; a fresh volume inherits this directory's owner.
ENV CUDA_CACHE_PATH=/var/cache/f1tenth/cuda CUDA_CACHE_MAXSIZE=1073741824
RUN mkdir -p /var/cache/f1tenth/cuda && chown -R ${USER_UID}:${USER_GID} /var/cache/f1tenth \
 && chmod 0755 /opt/f1tenth_ws/entrypoint.sh \
 && echo 'source /opt/f1tenth_ws/scripts/env/jazzy.sh' >> /etc/bash.bashrc
# Per-commit metadata last, so a commit that does not change the sources
# reuses every layer above.
COPY --from=src SOURCES /opt/f1tenth_ws/SOURCES
ARG GIT_SHA=unknown
LABEL org.opencontainers.image.title="f1tenth Jazzy (Orin, JetPack 6 / L4T r36.5)" \
      org.opencontainers.image.revision="${GIT_SHA}" \
      org.opencontainers.image.source="f1tenth_more jazzy branch, docker/orin/Dockerfile"
USER ${USER_UID}:${USER_GID}
WORKDIR /home/${USER_NAME}
ENTRYPOINT ["/opt/f1tenth_ws/entrypoint.sh"]
CMD ["bash"]
