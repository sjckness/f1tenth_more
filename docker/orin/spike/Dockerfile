# syntax=docker/dockerfile:1
#
# Phase 5d, D1a spike: option C on the Orin (JetPack 6.2.2 / L4T r36.5 host).
#   ubuntu:24.04 userspace
#   + CUDA 12.9.1 for Tegra (NVIDIA ships it for JP6 only as Ubuntu 22.04 debs;
#     the same approach jetson-containers uses for its r36 / 24.04 images)
#     + cuda-compat-12-9 (runs the 12.9 runtime on the r36.5 driver)
#   + cuDNN 9.8.0 (24.04 Tegra local repo)
#   + TensorRT 10.3.0.26 L4T tarball (cuda-12.6 build, cp312 wheel)
#   + apt Jazzy from snapshots.ros.org/jazzy/2026-09-11 (= the Thor's 386
#     ros-jazzy-* versions, docker/orin/thor_capture/ros_jazzy_dpkg.txt)
#   + torch 2.8.0 / torchvision 0.23.0 cp312 from jetson-ai-lab jp6/cu129
#   + ZED SDK 5.4.1 for L4T 36.5
#   + the workspace at 083c8e8 (31 packages), colcon --merge-install, Release.
#
# THROWAWAY. It exists to answer gates G1-G5 (output/phase5d_spike.md); D1
# writes the real docker/orin/Dockerfile. Large inputs are pre-downloaded and
# passed as named build contexts, so a rebuild never re-downloads 3.8 GB.
#
# Build and run ONLY through docker/orin/spike/spike.sh: the Orin's dockerd
# runs with bridge:none / iptables:false, so every build and container needs
# host networking (CLAUDE.md, "Orin Docker").

ARG BASE=ubuntu:24.04@sha256:534baea6a22c03a63003dbc8dbe78fe34bc0d7e595d9a9dc9834884ff530eb55

# ---------------------------------------------------------------------------
FROM ${BASE} AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive TZ=Europe/Rome LANG=C.UTF-8 LC_ALL=C.UTF-8
# tzdata first, alone (Dockerfile.jetson-thor lesson (a)).
RUN apt-get update \
 && apt-get install -y --no-install-recommends tzdata \
 && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg lsb-release zstd file binutils sudo udev usbutils less jq \
      build-essential cmake git pkg-config ccache \
      python3 python3-dev python3-pip python3-venv \
      libusb-1.0-0-dev libopenblas0 libgomp1 libturbojpeg libjpeg-turbo8 libpng-dev libv4l-dev libegl1 libx11-6 \
      nlohmann-json3-dev bc iproute2 procps sqlite3 \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
FROM base AS cuda
RUN --mount=type=bind,from=dl,source=cuda-tegra-repo-ubuntu2204-12-9-local_12.9.1-1_arm64.deb,target=/tmp/cuda-repo.deb \
    --mount=type=bind,from=dl,source=cudnn-local-tegra-repo-ubuntu2404-9.8.0_1.0-1_arm64.deb,target=/tmp/cudnn-repo.deb \
    dpkg -i /tmp/cuda-repo.deb /tmp/cudnn-repo.deb \
 && cp /var/cuda-tegra-repo-ubuntu2204-12-9-local/cuda-tegra-*-keyring.gpg \
       /var/cudnn-local-tegra-repo-ubuntu2404-9.8.0/cudnn-local-tegra-*-keyring.gpg /usr/share/keyrings/ \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      cuda-libraries-dev-12-9 cuda-minimal-build-12-9 cuda-nvcc-12-9 cuda-cupti-12-9 cuda-nvtx-12-9 \
      libcudnn9-cuda-12 libcudnn9-dev-cuda-12 \
 # cuda-compat-12-9 Pre-Depends on nvidia-l4t-core (the host's L4T package,
 # not installable in a container); its payload is only
 # /usr/local/cuda-12.9/compat/*, so unpack it without registering it.
 && dpkg-deb -x /var/cuda-tegra-repo-ubuntu2204-12-9-local/cuda-compat-12-9_*_arm64.deb / \
 && ls /usr/local/cuda-12.9/compat/libcuda.so.1 \
 && dpkg -P cuda-tegra-repo-ubuntu2204-12-9-local cudnn-local-tegra-repo-ubuntu2404-9.8.0 \
 && rm -rf /var/lib/apt/lists/* /var/cuda-tegra-repo-* /var/cudnn-local-*
# compat first: its libcuda.so.1 is the 12.9 driver API on top of the host's
# r36.5 kernel driver (the host's own libcuda.so.1.1 is CSV-mounted into
# /usr/lib/aarch64-linux-gnu/nvidia by the nvidia runtime).
ENV CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/compat:/usr/local/cuda/lib64
RUN echo "CUDA Version $(ls -d /usr/local/cuda-12.* | sed 's/.*cuda-//')" > /usr/local/cuda/version.txt

RUN --mount=type=bind,from=dl,source=TensorRT-10.3.0.26.l4t.aarch64-gnu.cuda-12.6.tar.gz,target=/tmp/trt.tgz \
    tar -xzf /tmp/trt.tgz -C /opt \
 && ln -s /opt/TensorRT-10.3.0.26 /opt/tensorrt \
 && rm -rf /opt/tensorrt/data /opt/tensorrt/doc /opt/tensorrt/samples /opt/tensorrt/targets/*/lib/*.a \
 && echo /opt/tensorrt/lib > /etc/ld.so.conf.d/tensorrt.conf && ldconfig
ENV PATH=${PATH}:/opt/tensorrt/bin

# ---------------------------------------------------------------------------
FROM cuda AS ros
ARG ROS_SNAPSHOT=2026-09-11
COPY --from=dl ros-snapshots-keyring.gpg /usr/share/keyrings/ros-snapshots-keyring.gpg
RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-snapshots-keyring.gpg] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list
COPY ros_packages.txt /tmp/ros_packages.txt
RUN apt-get update \
 && apt-get install -y --no-install-recommends $(cat /tmp/ros_packages.txt) \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
FROM ros AS py
# PEP 668: system-wide pip on 24.04. The constraints keep numpy<2 (cv_bridge)
# and pin torch/torchvision to the JP6 wheels installed first, so no resolver
# step can swap in a PyPI CUDA torch.
ENV PIP_BREAK_SYSTEM_PACKAGES=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1
COPY constraints.txt /etc/pip-constraints.txt
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt
RUN --mount=type=bind,from=dl,source=torch-2.8.0-cp312-cp312-linux_aarch64.whl,target=/tmp/w/torch-2.8.0-cp312-cp312-linux_aarch64.whl \
    --mount=type=bind,from=dl,source=torchvision-0.23.0-cp312-cp312-linux_aarch64.whl,target=/tmp/w/torchvision-0.23.0-cp312-cp312-linux_aarch64.whl \
    pip install /tmp/w/torch-2.8.0-cp312-cp312-linux_aarch64.whl /tmp/w/torchvision-0.23.0-cp312-cp312-linux_aarch64.whl \
 && pip install /opt/tensorrt/python/tensorrt-10.3.0-cp312-none-linux_aarch64.whl \
 && pip install osqp==1.1.3 pyarrow==25.0.1 ultralytics==8.4.90 opencv-python==4.11.0.86 \
 && if pip list 2>/dev/null | grep -i -E '^nvidia-(cu|cuda|cublas|cudnn|cudss|nccl|cusparse|cufft|curand|cusolver|nvjitlink)'; then \
      echo 'PyPI CUDA package pulled in - refusing' >&2; exit 1; fi

# ---------------------------------------------------------------------------
FROM py AS zed
# skip_cuda: CUDA comes from this image. skip_python: no stack code imports
# pyzed, and pyzed 5.x needs numpy>=2. skip_drivers: ZED X / GMSL only.
# The Thor runs 5.4.0; ZED_RUN selects the installer (5.4.1 = the D0 pick).
ARG ZED_RUN=ZED_SDK_Tegra_L4T36.5_v5.4.1.zstd.run
RUN --mount=type=bind,from=dl,source=${ZED_RUN},target=/tmp/zed.run \
    apt-get update \
 && cp /tmp/zed.run /tmp/zed_install.run && chmod +x /tmp/zed_install.run \
 && /tmp/zed_install.run -- silent skip_tools skip_python skip_drivers skip_cuda 2>&1 | tee /usr/local/zed_install.log \
 && rm -f /tmp/zed_install.run && rm -rf /var/lib/apt/lists/* \
 && test -f /usr/local/zed/lib/libsl_zed.so
ENV ZED_DIR=/usr/local/zed

# ---------------------------------------------------------------------------
FROM zed AS ws
# Name clash: L4T's /usr/lib/aarch64-linux-gnu/nvidia/libnvjpeg.so (the Tegra
# multimedia libjpeg that libnvargus_socketclient.so - and so libsl_zed.so -
# needs) and CUDA's nvJPEG dev symlink /usr/local/cuda/lib64/libnvjpeg.so are
# different libraries with the same unversioned name. With CUDA first on the
# search path zed_debug failed to link (undefined
# jpeg_set_hardware_acceleration_parameters_enc). Drop CUDA's unversioned
# symlink (torchvision uses libnvjpeg.so.12) and search L4T before CUDA.
# D1: do not install libnvjpeg-dev-12-9 at all, set this order in the cuda stage.
RUN rm -f /usr/local/cuda-12.9/targets/aarch64-linux/lib/libnvjpeg.so
ENV LD_LIBRARY_PATH=/usr/local/cuda/compat:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64
WORKDIR /opt/f1tenth_ws
COPY --from=src . /opt/f1tenth_ws/src
# The host's L4T libs (libnvbufsurface, libnvargus_socketclient, ...) that
# libsl_zed.so links are only CSV-mounted at run time; mount the host copy for
# the link step only, it does not end up in the image.
RUN --mount=type=bind,from=l4t,target=/usr/lib/aarch64-linux-gnu/nvidia \
    --mount=type=cache,target=/root/.ccache \
    source /opt/ros/jazzy/setup.bash \
 && colcon build --merge-install \
      --packages-skip f1tenth_sim f1tenth_more husarion_gz_worlds \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      --event-handlers console_direct- console_cohesion+ summary+ \
      2>&1 | tee /opt/f1tenth_ws/colcon_build.log \
 && rm -rf build log
RUN echo 'source /opt/ros/jazzy/setup.bash && source /opt/f1tenth_ws/install/setup.bash' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
FROM ws AS final
# torch 2.8.0 (jp6/cu129) is built against cuDNN 9.10.2 and refuses 9.8.0
# ("cuDNN version incompatibility"). Spike shortcut: upgrade on top, so the
# ROS / Python / workspace layers stay cached. D1: install 9.10.2 in the cuda stage.
RUN --mount=type=bind,from=dl,source=cudnn-local-tegra-repo-ubuntu2404-9.10.2_1.0-1_arm64.deb,target=/tmp/cudnn-repo.deb \
    dpkg -i /tmp/cudnn-repo.deb \
 && cp /var/cudnn-local-tegra-repo-ubuntu2404-9.10.2/cudnn-local-tegra-*-keyring.gpg /usr/share/keyrings/ \
 && apt-get update -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/cudnn-local-tegra-ubuntu2404-9.10.2.list -o Dir::Etc::sourceparts=- \
 && apt-get install -y --no-install-recommends libcudnn9-cuda-12=9.10.2.21-1 libcudnn9-dev-cuda-12=9.10.2.21-1 \
 && dpkg -P cudnn-local-tegra-repo-ubuntu2404-9.10.2 \
 && rm -rf /var/lib/apt/lists/* /var/cudnn-local-* \
 && dpkg-query -W libcudnn9-cuda-12
# TensorRT dlopens libnvinfer_builder_resource.so.10.3.0 by file name, but that
# library's SONAME is "do_not_link_against_nvinfer_builder_resource", so the
# ld.so cache never holds the file name: a tarball install outside the default
# lib dirs needs the dir on LD_LIBRARY_PATH (JetPack's debs use /usr/lib/aarch64-linux-gnu).
ENV LD_LIBRARY_PATH=/usr/local/cuda/compat:/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/cuda/lib64:/opt/tensorrt/lib
# The ZED installer creates group "zed" with the image's next free GID and
# makes /usr/local/zed root:zed 0770, so the container user (host uid 1004,
# host zed GID 1005 via --group-add) could not even read libsl_zed.so: every
# ZED library showed "not found". Give the image's zed group the host's GID.
# D1: groupadd -g <host zed GID> zed BEFORE running the SDK installer.
RUN groupmod -g 1005 zed && chgrp -R zed /usr/local/zed \
 && ldconfig && ldconfig -p | grep libsl_zed.so
