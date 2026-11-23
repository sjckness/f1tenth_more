# Isaac Sim 5.1 image for the F1Tenth HIL sim.
#   docker/build.sh            (from this repo; needs an NGC login for the base image, ~20 GB)
# The base image already carries Isaac's internal Humble and Jazzy ROS 2 bridge libraries; no host
# ROS is used inside the container (Isaac is Python 3.11, the host stacks are 3.10/3.12).
FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# requirements.txt is the sim's own Python dependency list (numpy; PyYAML and PIL ship with Kit).
COPY requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements.txt
