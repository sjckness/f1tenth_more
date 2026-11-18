# Minimal Isaac Sim 5.1 image for scripts/sim/sim_isaac.sh (~/isaac_ws): the base image plus the
# sim's own Python requirements. No node, no CLI tools, unlike ~/isaac_ws/.devcontainer/Dockerfile.
#
#   docker build --network host -t isaac_ws_devcontainer-isaac-sim:latest \
#       -f scripts/sim/isaac_tuning/docker/Dockerfile ~/isaac_ws
#
# The tag is the one sim_isaac.sh and ~/isaac_ws/.devcontainer/docker-compose.yaml expect
# (override with F1T_ISAAC_IMAGE). Base image: 7.6 GB download, ~20 GB on disk.
FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
COPY requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements.txt
