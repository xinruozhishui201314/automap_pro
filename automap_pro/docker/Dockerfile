FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# ── System packages ───────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget lsb-release gnupg2 \
    python3-pip python3-dev python3-setuptools \
    libboost-all-dev libeigen3-dev libflann-dev \
    libpcl-dev libopencv-dev \
    libgeographic-dev \
    libgtest-dev libyaml-cpp-dev \
    nlohmann-json3-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── ROS Noetic ────────────────────────────────────────────
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    python3-rosdep python3-catkin-tools \
 && rosdep init || true && rosdep update \
 && rm -rf /var/lib/apt/lists/*

# ── GTSAM ─────────────────────────────────────────────────
RUN git clone https://github.com/borglab/gtsam.git /opt/gtsam \
 && mkdir -p /opt/gtsam/build \
 && cmake -B /opt/gtsam/build -S /opt/gtsam \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON \
 && make -C /opt/gtsam/build -j$(nproc) install \
 && rm -rf /opt/gtsam/build

# ── Python deps ───────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    numpy scipy matplotlib open3d \
    evo pyquaternion tqdm

# ── Workspace ─────────────────────────────────────────────
RUN mkdir -p /automap_ws/src
WORKDIR /automap_ws

# Source ROS in .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
 && echo "source /automap_ws/devel/setup.bash 2>/dev/null || true" >> ~/.bashrc

COPY . /automap_ws/src/automap_pro

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /automap_ws && catkin build automap_pro -j$(nproc)"

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
