
---

# LOCAL SETUP

---

## Workspace Setup

Create a ROS workspace with the following structure on your host machine:

```
sloam_ws/
 ├── src/
 │    ├── sloam           # Clone from https://github.com/KumarRobotics/sloam
 │    ├── sloam_msgs      # Clone from https://github.com/KumarRobotics/sloam
 │    ├── ouster_decoder  # Clone from https://github.com/KumarRobotics/ouster_decoder
 │    ├── ouster_ros      # Clone from https://github.com/ouster-lidar/ouster-ros
 │    ├── llol            # Clone from https://github.com/versatran01/llol
 │    └── models/         # Create this folder to store segmentation models (ONNX)
```

### Clone the Repositories

```bash
mkdir -p ~/sloam_ws/src && cd ~/sloam_ws/src
git clone https://github.com/KumarRobotics/sloam.git
git clone https://github.com/KumarRobotics/ouster_decoder.git
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
git clone https://github.com/versatran01/llol.git
mkdir models  # Create the models folder
```

### Update llol's CMake Configuration

In the top-level `llol/CMakeLists.txt`, add the following line after the project declaration to ensure that headers are found:

```cmake
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
```

## Dependency Installation

Install the required system packages and ROS dependencies (for ROS Noetic on Ubuntu 20.04):

```bash
sudo apt-get update
sudo apt-get install -y \
    g++ \
    libeigen3-dev \
    git \
    python3-catkin-tools \
    ros-noetic-pcl-ros \
    ros-noetic-rviz \
    build-essential \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    cmake
```

Install tf:

```bash
sudo apt-get install ros-noetic-rqt*
```

Then, update rosdep and install additional ROS dependencies:

```bash
rosdep update
rosdep install --from-paths . --ignore-src -y -r --as-root apt:false
```

---

## Building External Libraries

The GitHub Actions build file for SLOAM installs some dependencies from source. Follow these steps in a temporary directory (e.g., `/tmp`):

### glog

```bash
cd /tmp
git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake -S .. -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17
cmake --build build
sudo cmake --build build --target install
```

### fmt (with PIC enabled)

To avoid linking issues, build fmt with position-independent code:

```bash
cd /tmp
git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git
cd fmt
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE \
      -DCMAKE_CXX_STANDARD=17 \
      -DFMT_TEST=False ..
make -j$(nproc)
sudo make install
```

*Tip:* If a non‑PIC version is already installed in `/usr/local/lib`, consider renaming it to avoid conflicts.
`sudo mv /usr/local/lib/libfmt.a /usr/local/lib/libfmt.a.bak`

### Abseil

```bash
cd /tmp
git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake -DABSL_BUILD_TESTING=OFF \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
sudo cmake --build . --target install
```

Verify that the configuration file is installed at:  
`/usr/lib/x86_64-linux-gnu/cmake/absl/abslConfig.cmake`

### Sophus

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 785fef3
mkdir build && cd build
cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=17 ..
sudo make install
```

---

### Build the Workspace

Clean any previous builds and build with the proper dependency paths:

```bash
cd ~/sloam_ws
rm -rf build devel
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -Dabsl_DIR=/usr/lib/x86_64-linux-gnu/cmake/absl \
  -Dbenchmark_DIR=/usr/lib/x86_64-linux-gnu/cmake/benchmark \
  -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog \
  -DBUILD_BENCHMARK=OFF
```

*Note:* The flag `-DBUILD_BENCHMARK=OFF` disables benchmark executables that may lack a `main()` function.

---

### Source the Workspace

   ```bash
   source ~/sloam_ws/devel/setup.bash
   ```
