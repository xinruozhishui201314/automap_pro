# Dockerfile for ROS1 to ROS2 bag conversion
# 使用双环境镜像（包含ROS1 Noetic和ROS2 Humble）

FROM osrf/ros:humble-desktop-full

# 安装ROS1相关工具和依赖
RUN apt-get update && apt-get install -y \
    # ROS1 Noetic依赖（不完整安装，只安装必要的包）
    python3-catkin-tools \
    python3-genmsg \
    python3-gencpp \
    python3-genpy \
    python3-rospkg-modules \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # ROS1 bag工具
    python3-rosbag \
    python3-rosbag-storage \
    # Python工具
    python3-pip \
    # 其他工具
    wget \
    curl \
    vim \
    git \
    && rm -rf /var/lib/apt/lists/*

# 安装Python包
RUN pip3 install --no-cache-dir \
    rosbag2-py \
    rosbag2-storage-sqlite3 \
    rosbag2-converter-default-plugins \
    ros1-rosbag-bridge \
    opencv-python-headless

# 创建工作目录
WORKDIR /workspace

# 复制转换脚本
COPY docker/scripts/convert_ros1_to_ros2.py /workspace/
COPY docker/scripts/convert_all_bags.sh /workspace/

# 设置权限
RUN chmod +x /workspace/*.py /workspace/*.sh

# 默认命令
CMD ["/bin/bash"]
