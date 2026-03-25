cmake_minimum_required(VERSION 3.10)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading (capped at 2 for stability: avoid SIGSEGV under load / GDB)
# To restore more parallelism: set(MP_PROC_NUM_MAX 4) or remove the min() cap below
set(MP_PROC_NUM_MAX 2)
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  if(PROC_NUM GREATER MP_PROC_NUM_MAX)
    set(PROC_NUM ${MP_PROC_NUM_MAX})
  endif()
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM} (capped at ${MP_PROC_NUM_MAX} for stability)")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  if(PROC_NUM GREATER MP_PROC_NUM_MAX)
    set(PROC_NUM ${MP_PROC_NUM_MAX})
  endif()
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM} (capped at ${MP_PROC_NUM_MAX} for stability)")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find ament and required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(fmt REQUIRED)

# ROS 2 Jazzy 等：SophusConfig 常只导出 Sophus::Sophus，Sophus_INCLUDE_DIRS 可能为空；须链接目标以传递 sophus 头文件路径
if(TARGET Sophus::Sophus)
  set(_FAST_LIVO_SOPHUS Sophus::Sophus)
else()
  set(_FAST_LIVO_SOPHUS ${Sophus_LIBRARIES})
endif()

# Include directories for dependencies
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
  include
)
if(Sophus_INCLUDE_DIRS)
  include_directories(${Sophus_INCLUDE_DIRS})
endif()

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
)

set(COMMON_DEPENDENCIES OpenMP::OpenMP_CXX fmt::fmt)

# link_directories(${COMMON_DEPENDENCIES}
#   ${vikit_common_LIBRARIES}/libvikit_common.so
#   ${vikit_ros_LIBRARIES}/libvikit_ros.so
# )

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
add_library(utils src/utils.cpp)

ament_target_dependencies(vio ${dependencies} )
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

# linking libraries or executables to public dependencies
# vikit_common and vikit_ros are linked via ament_target_dependencies above
target_link_libraries(laser_mapping
    ${COMMON_DEPENDENCIES}
    ${_FAST_LIVO_SOPHUS}
)
target_link_libraries(vio ${COMMON_DEPENDENCIES} ${_FAST_LIVO_SOPHUS})
target_link_libraries(lio utils ${COMMON_DEPENDENCIES} ${_FAST_LIVO_SOPHUS})
target_link_libraries(pre ${COMMON_DEPENDENCIES} ${_FAST_LIVO_SOPHUS})
target_link_libraries(imu_proc ${COMMON_DEPENDENCIES} ${_FAST_LIVO_SOPHUS})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

ament_target_dependencies(fastlivo_mapping ${dependencies})

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${_FAST_LIVO_SOPHUS}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Install the executable
install(TARGETS
  fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  Eigen3
  PCL
  OpenCV 
  Sophus
)

ament_package()