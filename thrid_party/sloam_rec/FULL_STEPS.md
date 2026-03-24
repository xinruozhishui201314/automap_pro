
---

# SLOAM & LLOL: Semantic Lidar Odometry, Mapping, and Low-Latency Odometry for Spinning Lidars

<p align="center">
  <img src="./doc/sloam_zamboni.gif" alt="SLOAM Demo" width="800"/>
</p>

**SLOAM** (Semantic Lidar Odometry and Mapping in Forests) is a ROS-based framework designed for forest inventory. It fuses LiDAR data with semantic segmentation to produce a semantic map.  
**LLOL** (Low-Latency Odometry for Spinning Lidars) serves as the odometry backbone, providing a low-latency estimate for LiDAR odometry.

> **Note:** Although we recommend using Docker to run SLOAM, you can also install and run it locally.

---

## Table of Contents

- [Workspace Setup](#workspace-setup)
- [Dependency Installation](#dependency-installation)
- [Building External Libraries](#building-external-libraries)
  - [glog](#glog)
  - [fmt (with PIC)](#fmt-with-pic-enabled)
  - [Abseil](#abseil)
  - [Sophus](#sophus)
- [Cloning and Building the ROS Workspace](#cloning-and-building-the-ros-workspace)
- [Exporting Pretrained Segmentation Models to ONNX](#4-exporting-pretrained-segmentation-models-to-onnx)
- [Running SLOAM](#5-running-sloam)
  - [Docker-Based Run](#docker-based-run)
  - [Local Installation](#local-installation)
- [Running LLOL for Odometry](#6-running-llol-for-odometry)
- [Parameter Tuning & Development](#7-parameter-tuning--development)
- [Citations](#8-citations)

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
### Verify

```bash
sudo find /usr -name abslConfig.cmake 2>/dev/null
sudo find /usr -name spdlogConfig.cmake 2>/dev/null
sudo find /usr -name benchmarkConfig.cmake 2>/dev/null
```
---

## Cloning and Building the ROS Workspace

### Clone the Repositories

```bash
mkdir -p ~/sloam_ws/src && cd ~/sloam_ws/src
git clone https://github.com/KumarRobotics/sloam.git
git clone https://github.com/KumarRobotics/ouster_decoder.git
git clone https://github.com/ouster-lidar/ouster-ros.git
git clone https://github.com/versatran01/llol.git
mkdir models  # Create the models folder
```

### Update llol's CMake Configuration

In the top-level `llol/CMakeLists.txt`, add the following line after the project declaration to ensure that headers are found:

```cmake
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
```

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
```bash
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release   -Dabsl_DIR=/usr/lib/x86_64-linux-gnu/cmake/absl   -Dbenchmark_DIR=/usr/lib/x86_64-linux-gnu/cmake/benchmark   -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog
```

*Note:* The flag `-DBUILD_BENCHMARK=OFF` disables benchmark executables that may lack a `main()` function.

---

## 4. Exporting Pretrained Segmentation Models to ONNX

SLOAM uses a segmentation network (e.g., RangeNet++) for tree segmentation. To export a pretrained model to ONNX, follow these steps (e.g., in a Google Colab notebook):

### a. Install ONNX

```python
!pip install onnx
```

### b. Download and Extract Pretrained Models

Download pretrained model archives (e.g., squeezesegV2) from the provided links and extract them:

```bash
!wget http://www.ipb.uni-bonn.de/html/projects/bonnetal/lidar/semantic/models/squeezesegV2.tar.gz
!tar -xvzf squeezesegV2.tar.gz
```

### c. Clone the Model Repository (if needed)

If required, clone the RangeNet++/LiDAR-Bonnetal repository:

```bash
!git clone https://github.com/PRBonn/lidar-bonnetal.git
```

### d. Export the Model to ONNX

Use a script similar to the following (adjust paths as needed):

```python
import torch, yaml
from tasks.semantic.modules.segmentator import Segmentator

model_path = "/content/squeezesegV2/squeezesegV2"

with open(model_path + "/arch_cfg.yaml", "r") as f:
    ARCH = yaml.safe_load(f)
with open(model_path + "/data_cfg.yaml", "r") as f:
    dataset_cfg = yaml.safe_load(f)

# Determine the number of classes from the learning map
learning_map = dataset_cfg["learning_map"]
n_classes = len(set(learning_map.values()))
print(f"Number of classes: {n_classes}")

# Initialize and evaluate the model
model = Segmentator(ARCH, n_classes, path=model_path)
model.eval()

height = ARCH["dataset"]["sensor"]["img_prop"]["height"]
width = ARCH["dataset"]["sensor"]["img_prop"]["width"]
dummy_input = torch.randn(1, 5, height, width)
dummy_mask = torch.ones(1, height, width)

onnx_path = "/content/squeezesegV2_segmentator.onnx"
torch.onnx.export(
    model,
    (dummy_input, dummy_mask),
    onnx_path,
    export_params=True,
    opset_version=11,
    do_constant_folding=True,
    input_names=["input", "mask"],
    output_names=["output"],
    dynamic_axes={"input": {0: "batch_size"}, "mask": {0: "batch_size"}, "output": {0: "batch_size"}}
)
print(f"Model exported to {onnx_path}")
```

### e. Integrate the ONNX Model

Move the exported ONNX model into the `models` folder of your workspace and update the SLOAM parameter file (`sloam/params/sloam.yaml`) to point to this model.

---

## 5. Running SLOAM

You can run SLOAM using Docker or install locally.

### Docker-Based Run

1. **Build the Docker Image**

   Use the provided script:

   ```bash
   ./docker/build_sloam_image.sh
   ```

2. **Configure the Container Run Script**

   Edit `sloam/docker/run_sloam_container.sh` to set the workspace and bag directories, e.g.:

   ```bash
   SLOAMWS="$HOME/ros/sloam_ws"
   BAGS_DIR="$HOME/bags"
   ```
    Add:
     ```bash
     --env="XAUTHORITY=$XAUTHORITY" \
     --env="XAUTHORITY=/run/user/1000/gdm/Xauthority" \
     --volume="/run/user/1000/gdm/Xauthority:/run/user/1000/gdm/Xauthority:ro" \
     ```
3. **Run the Container**

   ```bash
   ./docker/run_sloam_container.sh
   ```

4. **Inside the Container**

   Verify the shared volume:
   
   ```bash
   cd /opt/sloam_ws && ls src
   ```

   Configure tmux:
   ```bash
    nano ~/.tmux.conf
   ```
   Now save the file: `CTRL+X , Y, ENTER`
   
   Add these lines:
   ```
    # Enable tmux to use the system clipboard
    set-option -g set-clipboard on
    
    # Bind the "y" key in copy mode (using vi keys) to copy the selection to the clipboard
    bind-key -T copy-mode y send-keys -X copy-pipe-and-cancel "xclip -selection clipboard -in"
   ```
   Then
   ```
    tmux source-file ~/.tmux.conf
   ```

    ### tmux shotcuts:
     ```
     Ctrl+b then % --> open another hor pane
     ```
     ```
     Ctrl+b then " --> open another ver pane
     ```
     ```
     Ctrl+b [ % --> start copy mode
     ```
     ```
     Ctrl+b then space --> select text
     ```
     ```
     y --> save to clipboard
     ```
   Launch SLOAM (for simulated data, for example):

   ```bash
   tmux
   source devel/setup.bash
   roslaunch sloam run_sim.launch
   ```

   In another tmux pane, play a bag file:

   ```bash
   cd ../bags
   rosbag play example.bag
   ```

### Local Installation

1. **Source the Workspace**

   ```bash
   source ~/sloam_ws/devel/setup.bash
   ```

2. **Launch SLOAM**

   ```bash
   roslaunch sloam run_sim.launch  # Use run.launch for real sensor data
   ```

---

## 6. Running LLOL for Odometry

LLOL provides the odometry backbone. Follow these steps:

1. **Run the Ouster Driver**

   ```bash
   roslaunch ouster_decoder driver.launch
   ```

2. **Run the Ouster Decoder**

   ```bash
   roslaunch ouster_decoder decoder.launch
   ```

3. **Run LLOL**

   ```bash
   roslaunch llol llol.launch
   ```

   For multithreaded mode with timing output every 5 seconds, run:

   ```bash
   roslaunch llol llol.launch tbb:=1 log:=5
   ```

4. **Replay a Bag File (if applicable)**

   In a separate terminal:

   ```bash
   rosbag play <path_to_bag_file>
   ```

   Use RViz with the configuration provided in `llol/launch/llol.rviz` to visualize odometry output.

---

## 7. Parameter Tuning & Development

- Most SLOAM parameters are configured in the launch files (e.g., `sloam/launch/sloam.launch`, `run.launch`, and `run_sim.launch`).
- Adjust topics, sensor settings, and mapping parameters as needed.
- For development and debugging, we recommend using VSCode with the following extensions:
  - ROS
  - Docker
  - Remote-Containers

A sample VSCode debug configuration:

```json
{
  "version": "0.2.0",
  "configurations": [{
      "name": "Node SLOAM",
      "request": "launch",
      "target": "/opt/sloam_ws/src/sloam/launch/run_sim.launch",
      "type": "ros"
  }]
}
```

---

## 8. Citations

If you use this work in your academic publications, please cite the following:

### SLOAM

```bibtex
@inproceedings{chen2019,
  title={SLOAM: Semantic lidar odometry and mapping for forest inventory},
  author={Chen, Steven W and Nardari, Guilherme V and Lee, Elijah S and Qu, Chao and Liu, Xu and Romero, Roseli Ap Francelin and Kumar, Vijay},
  year={2020}
}
```

```bibtex
@inproceedings{liu2022large,
  title={Large-scale Autonomous Flight with Real-time Semantic SLAM under Dense Forest Canopy},
  author={Liu, Xu and Nardari, Guilherme V and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W and Romero, Roseli AF and Taylor, Camillo J and others},
  year={2022}
}
```

### LLOL

```bibtex
@misc{qu2021llol,
  title={LLOL: Low-Latency Odometry for Spinning Lidars},
  author={Chao Qu and Shreyas S. Shivakumar and Wenxin Liu and Camillo J. Taylor},
  note={https://arxiv.org/abs/2110.01725},
  year={2021}
}
```

---

This README provides a complete guide for setting up, building, and running the SLOAM and LLOL system on Ubuntu 20.04 with ROS Noetic. Adjust file paths and parameters as needed for your environment.

---
