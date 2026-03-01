cmake_minimum_required(VERSION 3.16)
project(automap_pro VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")

# ──────────────────────────────────────────────────────────
# Find ament_cmake and rosidl
# ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS
  common filters io visualization segmentation features registration
)
find_package(OpenCV 4.2 REQUIRED)
# Ubuntu/Debian 将 FindGeographicLib.cmake 放在非标准路径，需加入 MODULE_PATH
if(EXISTS "/usr/share/cmake/geographiclib")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()
find_package(GeographicLib REQUIRED)
# 若 Find 脚本未创建 imported target，则用变量创建以便 target_link_libraries 使用
if(NOT TARGET GeographicLib::GeographicLib)
  add_library(GeographicLib::GeographicLib INTERFACE IMPORTED)
  set_target_properties(GeographicLib::GeographicLib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${GeographicLib_LIBRARIES}")
endif()
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
# nlohmann_json：优先本地 thrid_party/nlohmann-json3 源码，否则 FetchContent 拉取（支持 -DNLOHMANN_JSON_LOCAL= 覆盖）
if(DEFINED NLOHMANN_JSON_LOCAL)
  # 用户通过 -DNLOHMANN_JSON_LOCAL=/path/to/nlohmann-json3 指定
  set(_NLOHMANN_CANDIDATE "${NLOHMANN_JSON_LOCAL}")
else()
  # 按常见布局尝试：同 repo 下 thrid_party/nlohmann-json3
  set(_NLOHMANN_CANDIDATE "")
  foreach(_TRY
    "${CMAKE_CURRENT_SOURCE_DIR}/../thrid_party/nlohmann-json3"    # 与 automap_pro 同级（如 src/thrid_party，Docker 挂载）
    "${CMAKE_SOURCE_DIR}/../thrid_party/nlohmann-json3"            # 上级目录的 thrid_party
    "${CMAKE_CURRENT_SOURCE_DIR}/../../thrid_party/nlohmann-json3" # workspace 下的 thrid_party
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../thrid_party/nlohmann-json3" # mapping/src/automap_pro -> mapping/thrid_party
    "${CMAKE_SOURCE_DIR}/../../../thrid_party/nlohmann-json3"
  )
    if(EXISTS "${_TRY}/CMakeLists.txt")
      set(_NLOHMANN_CANDIDATE "${_TRY}")
      break()
    endif()
  endforeach()
endif()
if(_NLOHMANN_CANDIDATE AND EXISTS "${_NLOHMANN_CANDIDATE}/CMakeLists.txt")
  message(STATUS "Using local nlohmann_json: ${_NLOHMANN_CANDIDATE}")
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  add_subdirectory(${_NLOHMANN_CANDIDATE} ${CMAKE_CURRENT_BINARY_DIR}/nlohmann_json)
else()
  message(STATUS "Local nlohmann_json not found, fetching via FetchContent")
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.2
    GIT_SHALLOW    TRUE
  )
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(nlohmann_json)
endif()
find_package(spdlog REQUIRED)

find_package(GTSAM QUIET)
find_package(Ceres QUIET)
if(NOT GTSAM_FOUND)
  message(STATUS "GTSAM not found; optimizer will use Gauss-Newton fallback")
endif()
if(Ceres_FOUND)
  add_definitions(-DUSE_CERES)
endif()

find_package(CUDA QUIET)
if(CUDA_FOUND)
  add_definitions(-DUSE_CUDA)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
endif()
find_package(Torch QUIET)
if(Torch_FOUND)
  add_definitions(-DUSE_TORCH)
  message(STATUS "LibTorch found")
endif()
# TEASER++: 优先使用工作空间源码编译版本，而非系统安装版本
# 工作空间版本通过 CMAKE_PREFIX_PATH 指定（由 run_automap.sh 设置）
# 这确保始终使用最新的源码版本，而非旧版本系统包
find_package(teaserpp QUIET NO_DEFAULT_PATH)
if(teaserpp_FOUND)
  add_definitions(-DUSE_TEASER)
  message(STATUS "TEASER++ found: ${teaserpp_INCLUDE_DIRS}")
else()
  # 如果工作空间版本不存在，尝试查找系统版本（可选）
  message(WARNING "TEASER++ not found in workspace, searching system...")
  find_package(teaserpp QUIET)
  if(teaserpp_FOUND)
    add_definitions(-DUSE_TEASER)
    message(WARNING "TEASER++ found from system (not recommended)")
  else()
    message(STATUS "TEASER++ not found, building without TEASER++ support")
  endif()
endif()
find_package(overlap_transformer_msgs QUIET)
if(overlap_transformer_msgs_FOUND)
  add_definitions(-DUSE_OVERLAP_TRANSFORMER_MSGS)
  message(STATUS "overlap_transformer_msgs found: external OverlapTransformer service enabled")
endif()

# ──────────────────────────────────────────────────────────
# Generate interfaces (msg/srv)
# ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSMeasurementMsg.msg"
  "msg/KeyFrameMsg.msg"
  "msg/LoopConstraintMsg.msg"
  "msg/SubMapEventMsg.msg"
  "srv/SaveMap.srv"
  "srv/TriggerOptimize.srv"
  "srv/TriggerHBA.srv"
  "srv/LoadSession.srv"
  "srv/GetStatus.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
# 同包内使用本包生成的 msg/srv 时，必须链接 typesupport 目标（见 ROS2 Single-Package-Define-And-Use-Interface）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ──────────────────────────────────────────────────────────
# Include directories
# ──────────────────────────────────────────────────────────
set(AM_INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
if(GTSAM_FOUND)
  list(APPEND AM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
endif()
if(Ceres_FOUND)
  list(APPEND AM_INCLUDE_DIRS ${CERES_INCLUDE_DIRS})
endif()
if(Torch_FOUND)
  list(APPEND AM_INCLUDE_DIRS ${TORCH_INCLUDE_DIRS})
endif()
if(teaserpp_FOUND)
  list(APPEND AM_INCLUDE_DIRS ${teaserpp_INCLUDE_DIRS})
endif()

add_definitions(${PCL_DEFINITIONS})
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ──────────────────────────────────────────────────────────
# Library: automap_core
# ──────────────────────────────────────────────────────────
add_library(automap_core
  src/core/config_manager.cpp
  src/core/utils.cpp
)
ament_target_dependencies(automap_core rclcpp)
target_include_directories(automap_core PUBLIC include ${AM_INCLUDE_DIRS})
  target_link_libraries(automap_core
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
)

# ──────────────────────────────────────────────────────────
# Library: automap_robust (新增 - 健壮性框架)
# ──────────────────────────────────────────────────────────
add_library(automap_robust
  src/core/logger.cpp
  src/core/error_handler.cpp
  src/core/validator.cpp
  src/core/performance_monitor.cpp
)
ament_target_dependencies(automap_robust rclcpp)
target_include_directories(automap_robust PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_robust
  automap_core
  spdlog::spdlog
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# ──────────────────────────────────────────────────────────
# Library: automap_sensor
# ──────────────────────────────────────────────────────────
add_library(automap_sensor
  src/sensor/sensor_manager.cpp
  src/sensor/lidar_processor.cpp
  src/sensor/imu_processor.cpp
  src/sensor/camera_processor.cpp
  src/sensor/gps_processor.cpp
  src/sensor/time_sync.cpp
)
ament_target_dependencies(automap_sensor rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge pcl_conversions)
target_include_directories(automap_sensor PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_sensor
  automap_core
  automap_robust
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  GeographicLib::GeographicLib
  Eigen3::Eigen
)

# ──────────────────────────────────────────────────────────
# Library: automap_frontend
# ──────────────────────────────────────────────────────────
add_library(automap_frontend
  src/frontend/fast_livo2_wrapper.cpp
  src/frontend/fast_livo2_adapter.cpp
  src/frontend/keyframe_manager.cpp
  src/frontend/gps_fusion.cpp
)
ament_target_dependencies(automap_frontend rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions)
target_include_directories(automap_frontend PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_frontend automap_core automap_sensor automap_robust ${PCL_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)

# ──────────────────────────────────────────────────────────
# Library: automap_submap
# ──────────────────────────────────────────────────────────
add_library(automap_submap
  src/submap/submap_manager.cpp
  src/submap/ms_mapping_wrapper.cpp
  src/submap/session_manager.cpp
)
ament_target_dependencies(automap_submap rclcpp std_msgs sensor_msgs geometry_msgs)
target_include_directories(automap_submap PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_submap automap_core automap_frontend automap_robust ${PCL_LIBRARIES} nlohmann_json::nlohmann_json Eigen3::Eigen "${cpp_typesupport_target}")

# ──────────────────────────────────────────────────────────
# Library: automap_loop_closure
# ──────────────────────────────────────────────────────────
add_library(automap_loop_closure
  src/loop_closure/loop_detector.cpp
  src/loop_closure/overlap_transformer.cpp
  src/loop_closure/fpfh_extractor.cpp
  src/loop_closure/teaser_matcher.cpp
  src/loop_closure/icp_refiner.cpp
)
ament_target_dependencies(automap_loop_closure rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_conversions)
target_include_directories(automap_loop_closure PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_loop_closure automap_core automap_submap automap_robust ${PCL_LIBRARIES} Eigen3::Eigen Threads::Threads)
# 同包内使用本包 msg（LoopConstraintMsg）需链接 typesupport
add_dependencies(automap_loop_closure ${cpp_typesupport_target})
target_link_libraries(automap_loop_closure ${cpp_typesupport_target})
# TEASER++: 优先链接工作空间源码版本（通过 CMAKE_PREFIX_PATH 指定）
if(teaserpp_FOUND)
  # 如果工作空间版本有提供 target，使用之；否则使用变量
  if(TARGET teaserpp::teaser_registration)
    target_link_libraries(automap_loop_closure teaserpp::teaser_registration)
  elseif(teaserpp_LIBRARIES)
    target_link_libraries(automap_loop_closure ${teaserpp_LIBRARIES})
  else()
    target_link_libraries(automap_loop_closure teaserpp)
  endif()
  message(STATUS "Automap loop closure linking TEASER++: ${teaserpp_INCLUDE_DIRS}")
endif()
if(overlap_transformer_msgs_FOUND)
  ament_target_dependencies(automap_loop_closure overlap_transformer_msgs)
endif()
if(Torch_FOUND)
  target_link_libraries(automap_loop_closure ${TORCH_LIBRARIES})
endif()

# ──────────────────────────────────────────────────────────
# Library: automap_backend
# ──────────────────────────────────────────────────────────
add_library(automap_backend
  src/backend/pose_graph.cpp
  src/backend/factor_types.cpp
  src/backend/optimizer.cpp
  src/backend/hba_wrapper.cpp
  src/backend/hba_bridge.cpp
)
ament_target_dependencies(automap_backend rclcpp std_msgs geometry_msgs nav_msgs)
target_include_directories(automap_backend PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_backend automap_core automap_submap automap_robust Eigen3::Eigen)
if(GTSAM_FOUND)
  target_link_libraries(automap_backend gtsam)
endif()
if(Ceres_FOUND)
  target_link_libraries(automap_backend ${CERES_LIBRARIES})
endif()

# ──────────────────────────────────────────────────────────
# Library: automap_map
# ──────────────────────────────────────────────────────────
add_library(automap_map
  src/map/global_map.cpp
  src/map/map_builder.cpp
  src/map/map_filter.cpp
  src/map/map_exporter.cpp
)
ament_target_dependencies(automap_map rclcpp)
target_include_directories(automap_map PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_map automap_core automap_submap automap_backend automap_robust ${PCL_LIBRARIES} nlohmann_json::nlohmann_json Eigen3::Eigen)

# ──────────────────────────────────────────────────────────
# Library: automap_visualization
# ──────────────────────────────────────────────────────────
add_library(automap_visualization
  src/visualization/rviz_publisher.cpp
)
ament_target_dependencies(automap_visualization rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs)
target_include_directories(automap_visualization PUBLIC include ${AM_INCLUDE_DIRS})
target_link_libraries(automap_visualization automap_core automap_submap automap_backend automap_robust ${PCL_LIBRARIES} Eigen3::Eigen)

# ──────────────────────────────────────────────────────────
# Executables (ROS2 nodes)
# ──────────────────────────────────────────────────────────
macro(add_ros2_node name)
  add_executable(${name} src/nodes/${name}.cpp)
  ament_target_dependencies(${name} rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)
  target_include_directories(${name} PRIVATE include ${AM_INCLUDE_DIRS})
  target_link_libraries(${name}
    automap_core automap_robust automap_sensor automap_frontend automap_submap
    automap_loop_closure automap_backend automap_map automap_visualization
  )
  add_dependencies(${name} ${PROJECT_NAME})
endmacro()

add_ros2_node(automap_system_node)
# automap_system_node 额外依赖 ament_index_cpp（config 路径）
ament_target_dependencies(automap_system_node ament_index_cpp)
add_ros2_node(sensor_sync_node)
add_ros2_node(fast_livo2_node)
add_ros2_node(submap_manager_node)
add_ros2_node(loop_detector_node)
add_ros2_node(optimizer_node)
add_ros2_node(map_builder_node)

# ──────────────────────────────────────────────────────────
# Install
# ──────────────────────────────────────────────────────────
install(TARGETS
  automap_core automap_robust automap_sensor automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map automap_visualization
  automap_system_node sensor_sync_node fast_livo2_node
  submap_manager_node loop_detector_node optimizer_node map_builder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})

# ──────────────────────────────────────────────────────────
# ament export
# ──────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(
  automap_core automap_sensor automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map automap_visualization
)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_data_types test/test_data_types.cpp)
    target_link_libraries(test_data_types automap_core ${PCL_LIBRARIES} ${OpenCV_LIBS})
    ament_target_dependencies(test_data_types rclcpp)
  endif()
endif()

ament_package()
