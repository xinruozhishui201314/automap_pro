cmake_minimum_required(VERSION 3.16)
project(automap_pro VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
# 与 GTSAM/PCL 同进程时避免 -march=native 导致 Eigen ABI 不一致引发 double free（见 docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md）
option(AUTOMAP_AVOID_MARCH_NATIVE "Build Release without -march=native to align ABI with GTSAM/PCL (avoid lago static-init double free)" ON)
if(AUTOMAP_AVOID_MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  message(STATUS "[automap_pro] AUTOMAP_AVOID_MARCH_NATIVE=ON: Release build without -march=native (GTSAM/PCL ABI alignment)")
else()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations")

# ──────────────────────────────────────────────────────────
# ROS2 基础依赖
# ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)   # Composable Node 支持
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(overlap_transformer_msgs QUIET)

# ──────────────────────────────────────────────────────────
# C++ 第三方库
# ──────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io features registration segmentation visualization)
find_package(OpenCV 4.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
# spdlog 需要 SPDLOG_ACTIVE_LEVEL 来控制编译期最低日志级别
# Debug 模式编译所有级别；Release 仅编译 INFO 以上（节省 CPU）
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
  add_definitions(-DCMAKE_BUILD_TYPE_STR="Debug")
else()
  add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
  add_definitions(-DCMAKE_BUILD_TYPE_STR="Release")
endif()

# GeographicLib
if(EXISTS "/usr/share/cmake/geographiclib")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()
find_package(GeographicLib REQUIRED)
if(NOT TARGET GeographicLib::GeographicLib)
  add_library(GeographicLib::GeographicLib INTERFACE IMPORTED)
  set_target_properties(GeographicLib::GeographicLib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES      "${GeographicLib_LIBRARIES}")
endif()

# nlohmann_json：优先使用本地 thrid_party/nlohmann-json3，避免 FetchContent 访问 GitHub 失败
set(_NLOHMANN_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/nlohmann-json3"
  "${CMAKE_CURRENT_SOURCE_DIR}/../thrid_party/nlohmann-json3"
  "${CMAKE_SOURCE_DIR}/../thrid_party/nlohmann-json3"
)
foreach(_try ${_NLOHMANN_CANDIDATES})
  if(EXISTS "${_try}/CMakeLists.txt")
    set(_NLOHMANN_CANDIDATE "${_try}")
    break()
  endif()
endforeach()
if(DEFINED _NLOHMANN_CANDIDATE)
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  add_subdirectory(${_NLOHMANN_CANDIDATE} ${CMAKE_CURRENT_BINARY_DIR}/nlohmann_json)
else()
  include(FetchContent)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2 GIT_SHALLOW TRUE)
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# GTSAM（iSAM2）
find_package(GTSAM REQUIRED)
message(STATUS "[automap_pro] GTSAM found (startup double free in lago? see docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md)")

# Ceres（可选）
find_package(Ceres QUIET)
if(Ceres_FOUND)
  add_definitions(-DUSE_CERES)
endif()

# LibTorch（OverlapTransformer 推理）
find_package(Torch QUIET)
if(Torch_FOUND)
  add_definitions(-DUSE_TORCH)
  message(STATUS "[automap_pro] LibTorch found: ${TORCH_LIBRARIES}")
else()
  message(STATUS "[automap_pro] LibTorch NOT found – OverlapTransformer will use fallback descriptor")
endif()

# CUDA
find_package(CUDA QUIET)
if(CUDA_FOUND AND Torch_FOUND)
  add_definitions(-DUSE_CUDA)
  message(STATUS "[automap_pro] CUDA ${CUDA_VERSION} found")
endif()

# ──────────────────────────────────────────────────────────
# TEASER++: 优先源码子目录编译，否则查找系统包
# ──────────────────────────────────────────────────────────
set(_TEASER_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/../thrid_party/TEASER-plusplus"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../thrid_party/TEASER-plusplus"
)
set(TEASERPP_SOURCE "")
foreach(_try ${_TEASER_CANDIDATES})
  if(EXISTS "${_try}/CMakeLists.txt")
    set(TEASERPP_SOURCE "${_try}")
    break()
  endif()
endforeach()

# TEASER++ 官方目标：teaser_registration + teaser_features（FPFH），无 teaser_fpfh
if(TEASERPP_SOURCE)
  message(STATUS "[automap_pro] TEASER++ source at ${TEASERPP_SOURCE} – inline build")
  set(BUILD_TEASER_TESTS    OFF CACHE BOOL "" FORCE)
  set(BUILD_TEASER_DEMOS    OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
  set(BUILD_DOC             OFF CACHE BOOL "" FORCE)
  set(BUILD_TEASER_FPFH     ON CACHE BOOL "" FORCE)  # 需要 FPFH 用于 loop closure
  add_subdirectory(${TEASERPP_SOURCE} ${CMAKE_CURRENT_BINARY_DIR}/teaserpp EXCLUDE_FROM_ALL)
  add_definitions(-DUSE_TEASER)
  set(TEASER_TARGET teaserpp::teaser_registration teaserpp::teaser_features)
else()
  find_package(teaserpp QUIET)
  if(teaserpp_FOUND)
    add_definitions(-DUSE_TEASER)
    # 官方导出名为 teaserpp::teaser_features（若系统包仅有 registration，可改为仅 teaser_registration）
    if(TARGET teaserpp::teaser_features)
      set(TEASER_TARGET teaserpp::teaser_registration teaserpp::teaser_features)
    else()
      set(TEASER_TARGET teaserpp::teaser_registration)
    endif()
    message(STATUS "[automap_pro] TEASER++ found from system")
  else()
    set(TEASER_TARGET "")
    message(WARNING "[automap_pro] TEASER++ not found – no robust registration")
  endif()
endif()

# HBA API 库（直接 C++ 调用，无文件IO）
# ament_export_libraries 可能导出 hba_api 或未创建 namespace 目标，需兼容
# 未找到时启用 GTSAM ISAM2 fallback，保证全局优化仍可执行
set(HBA_API_LINK_TARGET "")
find_package(hba_api QUIET)
if(hba_api_FOUND)
  if(TARGET hba_api::hba_api)
    set(HBA_API_LINK_TARGET hba_api::hba_api)
  elseif(TARGET hba_api)
    set(HBA_API_LINK_TARGET hba_api)
  endif()
  if(HBA_API_LINK_TARGET)
    add_definitions(-DUSE_HBA_API)
    message(STATUS "[automap_pro] hba_api found, linking ${HBA_API_LINK_TARGET}")
  else()
    message(WARNING "[automap_pro] hba_api package found but no CMake target (build hba_api first: colcon build --packages-select hba_api)")
  endif()
endif()
if(NOT HBA_API_LINK_TARGET)
  add_definitions(-DUSE_GTSAM_FALLBACK)
  message(STATUS "[automap_pro] hba_api not found, using GTSAM ISAM2 fallback for global optimization")
endif()

# overlap_transformer_msgs（可选，用于降级 Python Service）
if(overlap_transformer_msgs_FOUND)
  add_definitions(-DUSE_OVERLAP_TRANSFORMER_MSGS)
endif()

# ──────────────────────────────────────────────────────────
# ROS2 消息/服务/Action 生成
# ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSMeasurementMsg.msg"
  "msg/KeyFrameMsg.msg"
  "msg/KeyFrameInfoMsg.msg"
  "msg/LoopConstraintMsg.msg"
  "msg/SubMapEventMsg.msg"
  "msg/MappingStatusMsg.msg"
  "msg/SessionInfo.msg"
  "msg/SessionList.msg"
  "msg/CrossSessionConstraint.msg"
  "srv/SaveMap.srv"
  "srv/TriggerOptimize.srv"
  "srv/TriggerHBA.srv"
  "srv/LoadSession.srv"
  "srv/GetStatus.srv"
  "srv/TriggerGpsAlign.srv"
  "srv/CreateSession.srv"
  "srv/DeleteSession.srv"
  "srv/GetSessionInfo.srv"
  "srv/MergeSessions.srv"
  "action/RunHBA.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ──────────────────────────────────────────────────────────
# Include 路径
# ──────────────────────────────────────────────────────────
set(AUTOMAP_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
if(Torch_FOUND)
  list(APPEND AUTOMAP_INCLUDE_DIRS ${TORCH_INCLUDE_DIRS})
endif()

# ──────────────────────────────────────────────────────────
# 公共链接库集合（仅非 ament 库；ament 包必须用 ament_target_dependencies）
# ──────────────────────────────────────────────────────────
set(AUTOMAP_AMENT_DEPS
  rclcpp rclcpp_components rclcpp_action
  std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen
  pcl_ros pcl_conversions cv_bridge image_transport message_filters
)

set(AUTOMAP_LIBS
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  Threads::Threads
  yaml-cpp
  spdlog::spdlog
  gtsam
  nlohmann_json::nlohmann_json
  GeographicLib::GeographicLib
  ${TEASER_TARGET}
)
if(Torch_FOUND)
  list(APPEND AUTOMAP_LIBS ${TORCH_LIBRARIES})
endif()
if(Ceres_FOUND)
  list(APPEND AUTOMAP_LIBS Ceres::ceres)
endif()
if(HBA_API_LINK_TARGET)
  list(APPEND AUTOMAP_LIBS ${HBA_API_LINK_TARGET})
endif()

# ──────────────────────────────────────────────────────────
# 库：automap_core
# ──────────────────────────────────────────────────────────
add_library(automap_core SHARED
  src/core/config_manager.cpp
  src/core/map_frame_config.cpp
  src/core/logger.cpp
  src/core/load_trace.cpp
  src/core/utils.cpp
  src/core/error_code.cpp
  src/core/error_monitor.cpp
  src/core/health_monitor_impl.cpp
  src/core/crash_report.cpp
)
target_include_directories(automap_core PUBLIC ${AUTOMAP_INCLUDE_DIRS})
# 必须与 ament_target_dependencies 一致：全部用普通形式（无 PUBLIC/PRIVATE），否则 CMake 报 keyword/plain 混用
target_link_libraries(automap_core ${AUTOMAP_LIBS} ${cpp_typesupport_target})
ament_target_dependencies(automap_core ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# 库：automap_frontend（GPS Manager + KeyFrame Manager + LIVO Bridge）
# ──────────────────────────────────────────────────────────
add_library(automap_frontend SHARED
  src/frontend/keyframe_manager.cpp
  src/frontend/gps_manager.cpp
  src/frontend/livo_bridge.cpp
  src/sensor/attitude_estimator.cpp
)
target_include_directories(automap_frontend PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_frontend automap_core ${AUTOMAP_LIBS})
ament_target_dependencies(automap_frontend ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# 库：automap_submap（SubMap + Session + MS-Mapping）
# ──────────────────────────────────────────────────────────
add_library(automap_submap SHARED
  src/submap/submap_manager.cpp
  src/submap/session_manager.cpp
  src/submap/ms_mapping_wrapper.cpp
)
target_include_directories(automap_submap PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_submap automap_core automap_frontend ${AUTOMAP_LIBS})
ament_target_dependencies(automap_submap ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# 库：automap_loop_closure（OverlapTransformer + TEASER++ + ICP + ScanContext）
# ──────────────────────────────────────────────────────────
add_library(automap_loop_closure SHARED
  src/loop_closure/overlap_transformer_infer.cpp
  src/loop_closure/fpfh_extractor.cpp
  src/loop_closure/teaser_matcher.cpp
  src/loop_closure/icp_refiner.cpp
  src/loop_closure/loop_detector.cpp
  ${CMAKE_SOURCE_DIR}/thrid_party/scancontext_tro/cpp/module/Scancontext/Scancontext.cpp
)

# ============================================================
# ScanContext 详细调试日志
# ============================================================
message(STATUS "========================================")
message(STATUS "ScanContext CMake 配置调试")
message(STATUS "========================================")
message(STATUS "SCANCONTEXT_ROOT = '${SCANCONTEXT_ROOT}'")
message(STATUS "CMAKE_SOURCE_DIR = '${CMAKE_SOURCE_DIR}'")
message(STATUS "CMAKE_BINARY_DIR = '${CMAKE_BINARY_DIR}'")

# 重要修正：include 路径应该是 Scancontext 目录的父目录！
# 因为代码中使用的是 #include "Scancontext/Scancontext.h"
# 所以 include 路径需要包含 Scancontext 目录本身
if(SCANCONTEXT_ROOT)
  message(STATUS "[SC] 使用 SCANCONTEXT_ROOT 参数: ${SCANCONTEXT_ROOT}")
  # 设置为 Scancontext 目录的父目录，这样 #include "Scancontext/Scancontext.h" 才能找到
  set(SCANCONTEXT_INCLUDE_DIR "${SCANCONTEXT_ROOT}/cpp/module")
else()
  # 默认使用容器内路径
  message(STATUS "[SC] 未设置 SCANCONTEXT_ROOT，使用默认路径")
  set(SCANCONTEXT_INCLUDE_DIR "/root/automap_ws/src/thrid_party/scancontext_tro/cpp/module")
endif()
message(STATUS "SCANCONTEXT_INCLUDE_DIR = '${SCANCONTEXT_INCLUDE_DIR}'")

# 检查目录是否存在
if(EXISTS "${SCANCONTEXT_INCLUDE_DIR}/Scancontext")
  message(STATUS "[SC] ✓ ScanContext 目录存在: ${SCANCONTEXT_INCLUDE_DIR}/Scancontext")
  # 列出目录内容
  file(GLOB SC_SOURCES "${SCANCONTEXT_INCLUDE_DIR}/Scancontext/*.h")
  message(STATUS "[SC] 找到 ScanContext 头文件:")
  foreach(src ${SC_SOURCES})
    message(STATUS "  - ${src}")
  endforeach()
else()
  message(STATUS "[SC] ✗ ScanContext 目录不存在: ${SCANCONTEXT_INCLUDE_DIR}/Scancontext!")
  message(STATUS "[SC] 尝试搜索可能的 ScanContext 位置...")
  
  # 尝试搜索
  find_file(SC_HINTS "Scancontext.h" 
    PATHS 
      "${CMAKE_SOURCE_DIR}/thrid_party/scancontext_tro/cpp/module/Scancontext"
      "${CMAKE_SOURCE_DIR}/../scancontext_tro/cpp/module/Scancontext"
      "/root/automap_ws/src/thrid_party/scancontext_tro/cpp/module/Scancontext"
      "/root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext"
    NO_DEFAULT_PATH
  )
  if(SC_HINTS)
    message(STATUS "[SC] 找到 Scancontext.h: ${SC_HINTS}")
    get_filename_component(SC_DIR ${SC_HINTS} DIRECTORY)
    message(STATUS "[SC] 建议设置: -DSCANCONTEXT_ROOT=${SC_DIR}/../..")
  endif()
endif()

message(STATUS "ScanContext include dir: ${SCANCONTEXT_INCLUDE_DIR}")
if(NOT EXISTS "${SCANCONTEXT_INCLUDE_DIR}/Scancontext")
  message(FATAL_ERROR "ScanContext headers not found at: ${SCANCONTEXT_INCLUDE_DIR}/Scancontext")
endif()
# 需要同时添加两个目录：
# 1. cpp/module - 用于 #include "Scancontext/Scancontext.h"
# 2. cpp/module/Scancontext - 用于 #include <nanoflann.hpp>
target_include_directories(automap_loop_closure PUBLIC 
  ${AUTOMAP_INCLUDE_DIRS}
  ${SCANCONTEXT_INCLUDE_DIR}
  ${SCANCONTEXT_INCLUDE_DIR}/Scancontext
)
message(STATUS "[SC] target_include_directories 已设置（PUBLIC）: ${SCANCONTEXT_INCLUDE_DIR} 和 ${SCANCONTEXT_INCLUDE_DIR}/Scancontext")
target_link_libraries(automap_loop_closure automap_core automap_submap ${AUTOMAP_LIBS})
ament_target_dependencies(automap_loop_closure ${AUTOMAP_AMENT_DEPS})
# 与 PCL 构建对齐，避免 FPFHEstimation 析构时 Eigen aligned_free 崩溃（PCL #4877）
if(PCL_DEFINITIONS)
  target_compile_definitions(automap_loop_closure PRIVATE ${PCL_DEFINITIONS})
endif()
target_compile_definitions(automap_loop_closure PRIVATE EIGEN_MAKE_ALIGNED_OPERATION_NEW)
if(overlap_transformer_msgs_FOUND)
  ament_target_dependencies(automap_loop_closure overlap_transformer_msgs)
endif()

# ──────────────────────────────────────────────────────────
# 库：automap_backend（iSAM2 + HBA API 封装）
# ──────────────────────────────────────────────────────────
add_library(automap_backend SHARED
  src/backend/load_trace.cpp
  src/backend/gtsam_guard.cpp
  src/backend/incremental_optimizer.cpp
  src/backend/delayed_gps_compensator.cpp
  src/backend/hba_optimizer.cpp
  src/backend/pose_graph.cpp
  src/backend/optimizer.cpp
)
target_include_directories(automap_backend PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_backend automap_core automap_submap ${AUTOMAP_LIBS})
ament_target_dependencies(automap_backend ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# 库：automap_map
# ──────────────────────────────────────────────────────────
add_library(automap_map SHARED
  src/map/global_map.cpp
  src/map/map_builder.cpp
  src/map/map_exporter.cpp
)
target_include_directories(automap_map PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_map automap_core automap_submap ${AUTOMAP_LIBS})
ament_target_dependencies(automap_map ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# 库：automap_visualization（RvizPublisher）
# ──────────────────────────────────────────────────────────
add_library(automap_visualization SHARED
  src/visualization/rviz_publisher.cpp
)
target_include_directories(automap_visualization PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_visualization automap_core automap_submap ${AUTOMAP_LIBS})
ament_target_dependencies(automap_visualization ${AUTOMAP_AMENT_DEPS})

# ──────────────────────────────────────────────────────────
# Composable Node 组件库（供 Component Container 加载）
# ──────────────────────────────────────────────────────────
add_library(automap_system_component SHARED
  src/system/automap_system.cpp
)
target_include_directories(automap_system_component PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_link_libraries(automap_system_component
  automap_core automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map
  automap_visualization
  ${AUTOMAP_LIBS}
)
ament_target_dependencies(automap_system_component ${AUTOMAP_AMENT_DEPS})
rclcpp_components_register_node(automap_system_component
  PLUGIN "automap_pro::AutoMapSystem"
  EXECUTABLE automap_system_node
)

# ──────────────────────────────────────────────────────────
# 独立可执行文件（用于非Composable模式）
# ──────────────────────────────────────────────────────────
add_executable(automap_system_node_standalone src/nodes/automap_system_node.cpp)
target_include_directories(automap_system_node_standalone PRIVATE ${AUTOMAP_INCLUDE_DIRS})
# 显式链接 component 及其依赖，并允许 .so 中存在未定义符号（运行时由 loader 解析）
target_link_libraries(automap_system_node_standalone
  automap_system_component
  automap_submap automap_map automap_visualization automap_backend automap_loop_closure
  automap_frontend automap_core
  ${AUTOMAP_LIBS})
target_link_options(automap_system_node_standalone PRIVATE "LINKER:--allow-shlib-undefined")
ament_target_dependencies(automap_system_node_standalone ${AUTOMAP_AMENT_DEPS})
set_target_properties(automap_system_node_standalone PROPERTIES OUTPUT_NAME automap_system)

# ──────────────────────────────────────────────────────────
# 安装规则
# ──────────────────────────────────────────────────────────
install(TARGETS
  automap_core automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map
  automap_visualization automap_system_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS automap_system_node_standalone
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS launch/run_under_gdb.sh DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_export_include_directories(include)
ament_export_libraries(
  automap_core automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map
  automap_visualization automap_system_component)
ament_export_dependencies(${AUTOMAP_AMENT_DEPS} Eigen3 PCL OpenCV GTSAM)
ament_package()
