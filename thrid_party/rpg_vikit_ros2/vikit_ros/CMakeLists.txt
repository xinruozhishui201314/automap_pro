# Set project name and version
set(PROJECT_NAME vikit_ros)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 3.5)

# Set build type
set(CMAKE_BUILD_TYPE Release)  # Release, RelWithDebInfo
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
# ROS 2 Humble 的 rclcpp 与 Sophus 均依赖 C++17（std::optional、std::is_same_v 等）
if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()


# Find ament_cmake and required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Sophus: 优先使用工作空间内本地 thrid_party/Sophus（与 vikit_ros 相对路径 ../../Sophus）
set(SOPHUS_LOCAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Sophus")
if(EXISTS "${SOPHUS_LOCAL_DIR}/CMakeLists.txt" AND EXISTS "${SOPHUS_LOCAL_DIR}/sophus/se3.hpp")
  message(STATUS "Using local Sophus: ${SOPHUS_LOCAL_DIR}")
  set(SOPHUS_MASTER_PROJECT OFF)
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Build Sophus tests" FORCE)
  add_subdirectory(${SOPHUS_LOCAL_DIR} ${CMAKE_CURRENT_BINARY_DIR}/sophus_build)
  set(SOPHUS_USE_LOCAL TRUE)
else()
  find_package(Sophus REQUIRED)
  set(SOPHUS_USE_LOCAL FALSE)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}      
  ${tf2_ros_INCLUDE_DIRS}  
  ${visualization_msgs_INCLUDE_DIRS}
)
if(NOT SOPHUS_USE_LOCAL)
  include_directories(${Sophus_INCLUDE_DIRS})
endif()

# Set source files
list(APPEND SOURCEFILES
  src/output_helper.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
if(SOPHUS_USE_LOCAL)
  target_link_libraries(${PROJECT_NAME} Sophus::Sophus)
endif()
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${vikit_common_LIBRARIES}
)
if(NOT SOPHUS_USE_LOCAL)
  target_link_libraries(${PROJECT_NAME} ${Sophus_LIBRARIES})
endif()

# Install library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  vikit_common
  OpenCV
  Eigen3
  Sophus
)

# Declare ament package
ament_package()