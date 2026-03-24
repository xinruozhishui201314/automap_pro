
---

# DOCKER SETUP

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

### Update ouster_decoder's files
Open /opt/sloam_ws/src/ouster_decoder/CMakeLists.txt and change this line:
```
include(CompilerWarnings)
```
to:
```
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerWarnings.cmake)
```

Edit /opt/sloam_ws/src/ouster_decoder/src/lidar.cpp where px_range() is used.
Replace (line 171):
```
const auto raw_range = pf.px_range(px_buf);
```
With:
```
const auto raw_range = pf.raw_range(px_buf);
```

Edit /opt/sloam_ws/src/ouster_decoder/src/decoder.cpp
Change (line 203):
```
model.info.imu_to_sensor_transform, sensor_frame_, imu_frame_
```
To:
```
model.info.imu_to_sensor_transform, sensor_frame_, imu_frame_, ros::Time::now()
```

### Update sloam's utils.h

Edit the file (/opt/sloam_ws/src/sloam/sloam/include/helpers/utils.h)

Add this at the top:
```
#include <fmt/format.h>
#include <Eigen/Core>

template <>
struct fmt::formatter<Eigen::Transpose<Eigen::Matrix<double, 4, 1>>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
    template <typename FormatContext>
    auto format(const Eigen::Transpose<Eigen::Matrix<double, 4, 1>>& v, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "[{}, {}, {}, {}]", v(0), v(1), v(2), v(3));
    }
};
```

## Container Setup

1. **Build the Docker Image**

   Use the provided script:

   ```bash
   ./docker/build_sloam_image.sh
   ```

2. **Configure the Container Run Script**

   Edit `sloam/docker/run_sloam_container.sh` to set the workspace and bag directories, e.g.:

   ```bash
   SLOAMWS="$HOME/sloam_ws"
   BAGS_DIR="$HOME/bags"
   ```
    Add these lines:
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
   You should see the same structure that you set onyou host machine
   
5. **Configure tmux**
   
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
