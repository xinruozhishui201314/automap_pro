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

# sloam_rec（SemanticProcessor）：仅使用本包内 thrid_party/sloam_rec（勿依赖仓库外 ../thrid_party）
set(SLOAM_REC_SLOAM_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/sloam_rec/sloam/include")
option(AUTOMAP_REQUIRE_SEMANTIC "Require non-stub semantic pipeline" ON)
if(EXISTS "${SLOAM_REC_SLOAM_INCLUDE}/segmentation/inference.h")
  message(STATUS "[automap_pro] sloam_rec sloam include: ${SLOAM_REC_SLOAM_INCLUDE}")
else()
  message(FATAL_ERROR
    "[automap_pro] sloam_rec not found. Expected:\n  ${SLOAM_REC_SLOAM_INCLUDE}/segmentation/inference.h\n"
    "Vendor copy: copy thrid_party/sloam_rec into ${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/sloam_rec")
endif()

# ONNX Runtime（SLOAM 语义分割）：与 GTSAM 一致优先 install_deps，未找到时使用 stub
set(_onnx_include "")
set(_onnx_lib "")
set(_sloam_segmentation_lib "")
set(_sloam_segmentation_target "")
set(_install_deps_onnx "")
if(DEFINED ENV{ONNXRUNTIME_HOME})
  set(_install_deps_onnx "$ENV{ONNXRUNTIME_HOME}")
endif()
if(NOT _install_deps_onnx AND CMAKE_SOURCE_DIR)
  get_filename_component(_ws_root "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
  if(EXISTS "${_ws_root}/install_deps/onnxruntime/lib/libonnxruntime.so")
    set(_install_deps_onnx "${_ws_root}/install_deps/onnxruntime")
  endif()
endif()
find_path(ONNX_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${_install_deps_onnx}/include/onnxruntime/core/session"
        /usr/local/include/onnxruntime/core/session
  PATH_SUFFIXES onnxruntime/core/session)
find_library(ONNX_LIBRARY onnxruntime
  HINTS "${_install_deps_onnx}/lib" /usr/local/lib /usr/lib/x86_64-linux-gnu)
if(ONNX_INCLUDE_DIR AND ONNX_LIBRARY)
  set(_onnx_include "${ONNX_INCLUDE_DIR}")
  set(_onnx_lib "${ONNX_LIBRARY}")
  set(_sloam_seg_hints "")
  if(DEFINED ENV{SLOAM_HOME})
    list(APPEND _sloam_seg_hints "$ENV{SLOAM_HOME}/lib")
  endif()
  if(DEFINED ENV{SLOAM_SEGMENTATION_LIB_DIR})
    list(APPEND _sloam_seg_hints "$ENV{SLOAM_SEGMENTATION_LIB_DIR}")
  endif()
  if(CMAKE_SOURCE_DIR)
    get_filename_component(_ws_root "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
    list(APPEND _sloam_seg_hints
      "${_ws_root}/install_deps/sloam/lib"
      "${_ws_root}/install/sloam/lib"
      "${_ws_root}/install/lib")
  endif()
  list(APPEND _sloam_seg_hints
    "${CMAKE_SOURCE_DIR}/../../install_deps/sloam/lib"
    "${CMAKE_SOURCE_DIR}/../install_deps/sloam/lib"
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu)
  list(REMOVE_DUPLICATES _sloam_seg_hints)
  find_library(SLOAM_SEGMENTATION_LIBRARY sloam_segmentation
    HINTS ${_sloam_seg_hints}
  )
  if(SLOAM_SEGMENTATION_LIBRARY)
    set(_sloam_segmentation_lib "${SLOAM_SEGMENTATION_LIBRARY}")
    set(_sloam_segmentation_target "${_sloam_segmentation_lib}")
    set(USE_SLOAM_SEMANTIC 1)
    message(STATUS "[automap_pro] ONNX Runtime + sloam_segmentation found: SLOAM semantic ENABLED")
  else()
    # 强制语义模式下的最小兜底：若预编译库缺失，则编译仓库内 sloam_rec segmentation 源码
    set(_sloam_seg_src_inference "${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/sloam_rec/sloam/src/segmentation/inference.cpp")
    set(_sloam_seg_src_trellis "${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/sloam_rec/sloam/src/segmentation/trellis.cpp")
    if(EXISTS "${_sloam_seg_src_inference}" AND EXISTS "${_sloam_seg_src_trellis}")
      add_library(automap_sloam_segmentation STATIC
        "${_sloam_seg_src_inference}"
        "${_sloam_seg_src_trellis}")
      target_include_directories(automap_sloam_segmentation PUBLIC
        "${SLOAM_REC_SLOAM_INCLUDE}"
        "${SLOAM_REC_SLOAM_INCLUDE}/segmentation"
        "${SLOAM_REC_SLOAM_INCLUDE}/helpers"
        "${_onnx_include}")
      # sloam_rec headers include ROS2 message/cv_bridge headers directly.
      ament_target_dependencies(automap_sloam_segmentation
        geometry_msgs
        sensor_msgs
        cv_bridge)
      target_link_libraries(automap_sloam_segmentation
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
        "${_onnx_lib}")
      set(_sloam_segmentation_target automap_sloam_segmentation)
      set(USE_SLOAM_SEMANTIC 1)
      message(WARNING
        "[automap_pro] Prebuilt sloam_segmentation not found. "
        "Using in-tree source fallback target: automap_sloam_segmentation")
    else()
      set(USE_SLOAM_SEMANTIC 0)
      if(AUTOMAP_REQUIRE_SEMANTIC)
        message(FATAL_ERROR
          "[automap_pro] Semantic pipeline is REQUIRED but sloam_segmentation library is missing.\n"
          "Searched paths: ${_sloam_seg_hints}\n"
          "Also checked in-tree fallback sources:\n"
          "  ${_sloam_seg_src_inference}\n"
          "  ${_sloam_seg_src_trellis}\n"
          "Please build/install sloam_segmentation first, or set SLOAM_HOME / SLOAM_SEGMENTATION_LIB_DIR.")
      else()
        message(WARNING "[automap_pro] ONNX Runtime found but sloam_segmentation library missing: semantic processor will run in stub mode")
      endif()
    endif()
  endif()
else()
  set(USE_SLOAM_SEMANTIC 0)
  if(AUTOMAP_REQUIRE_SEMANTIC)
    message(FATAL_ERROR
      "[automap_pro] Semantic pipeline is REQUIRED but ONNX Runtime was not found.\n"
      "Set ONNXRUNTIME_HOME or install onnxruntime in install_deps.")
  else()
    message(STATUS "[automap_pro] ONNX Runtime not found: SLOAM semantic DISABLED (stub mode)")
  endif()
endif()

# Boost（lockfree SPSC 队列，用于 ingress/frame 热路径无锁）
find_package(Boost 1.48 REQUIRED)
message(STATUS "[automap_pro] Boost found: ${Boost_VERSION} (lockfree spsc_queue for ingress/frame queues)")

# GTSAM（iSAM2）
find_package(GTSAM REQUIRED)
message(STATUS "[automap_pro] GTSAM found (startup double free in lago? see docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md)")

# Ceres（可选，须 ≥ 2.0 以支持 SphereManifold）
find_package(Ceres QUIET)
if(Ceres_FOUND)
  add_definitions(-DUSE_CERES)
  set(_ceres_ver "${Ceres_VERSION}")
  if(NOT _ceres_ver)
    set(_ceres_ver "${CERES_VERSION}")
  endif()
  if(DEFINED Ceres_VERSION AND Ceres_VERSION VERSION_LESS "2.0.0")
    message(FATAL_ERROR "[automap_pro] Ceres ${_ceres_ver} < 2.0，无法使用 SphereManifold。请使用 thrid_party/ceres-solver 或系统 Ceres ≥ 2.0")
  elseif(DEFINED CERES_VERSION AND CERES_VERSION VERSION_LESS "2.0.0")
    message(FATAL_ERROR "[automap_pro] Ceres ${_ceres_ver} < 2.0，无法使用 SphereManifold。请使用 thrid_party/ceres-solver 或系统 Ceres ≥ 2.0")
  endif()
  message(STATUS "[automap_pro] Ceres ${_ceres_ver}: SphereManifold 已启用（圆柱拟合）")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# LibTorch（OverlapTransformer 推理）：优先本地文件夹，便于从日志看到编译状态
# 查找顺序：1) LIBTORCH_HOME 环境变量  2) -DTorch_DIR  3) 本地候选路径
# ─────────────────────────────────────────────────────────────────────────────
message(STATUS "========== [automap_pro] LibTorch (OverlapTransformer) ==========")
set(_libtorch_resolved "")

# 1) 环境变量 LIBTORCH_HOME
if(DEFINED ENV{LIBTORCH_HOME})
  set(_try "$ENV{LIBTORCH_HOME}")
  if(EXISTS "${_try}/share/cmake/Torch")
    set(_libtorch_resolved "${_try}")
    message(STATUS "[automap_pro] LibTorch source: LIBTORCH_HOME (env) = ${_try}")
  else()
    message(STATUS "[automap_pro] LibTorch skip: LIBTORCH_HOME set but no share/cmake/Torch under ${_try}")
  endif()
endif()

# 2) CMake 已传入 Torch_DIR（命令行 -DTorch_DIR=...）
if(NOT _libtorch_resolved AND Torch_DIR AND EXISTS "${Torch_DIR}")
  set(_libtorch_resolved "from_cmake_arg")
  message(STATUS "[automap_pro] LibTorch source: Torch_DIR (cmake-arg) = ${Torch_DIR}")
endif()

# 3) 本地文件夹候选（仓库内、install_deps、或系统路径）
if(NOT _libtorch_resolved)
  # install_deps：与 GTSAM 一致，编译一次安装到 install_deps，不随工程重复编译
  set(_install_deps_libtorch "")
  if(DEFINED ENV{INSTALL_DEPS})
    set(_install_deps_libtorch "$ENV{INSTALL_DEPS}/libtorch")
  endif()
  if(NOT _install_deps_libtorch AND CMAKE_SOURCE_DIR)
    # colcon 下 CMAKE_SOURCE_DIR 为包源码目录（如 .../src/automap_pro），install_deps 在 workspace 根下
    get_filename_component(_ws_root "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
    if(EXISTS "${_ws_root}/install_deps/libtorch/share/cmake/Torch")
      set(_install_deps_libtorch "${_ws_root}/install_deps/libtorch")
    endif()
  endif()
  set(_libtorch_candidates
    "${CMAKE_CURRENT_SOURCE_DIR}/../libtorch"           # 仓库根目录 /libtorch
    "${CMAKE_CURRENT_SOURCE_DIR}/../thrid_party/libtorch"
    "${_install_deps_libtorch}"                         # install_deps/libtorch（与 GTSAM 一致）
    "${CMAKE_SOURCE_DIR}/libtorch"
    "${CMAKE_CURRENT_SOURCE_DIR}/libtorch"
    "/opt/libtorch"
    "/usr/local/libtorch"
  )
  list(REMOVE_ITEM _libtorch_candidates "")            # 去掉空串
  foreach(_dir IN LISTS _libtorch_candidates)
    if(EXISTS "${_dir}/share/cmake/Torch")
      set(_libtorch_resolved "${_dir}")
      message(STATUS "[automap_pro] LibTorch source: local path = ${_dir}")
      break()
    endif()
  endforeach()
  if(NOT _libtorch_resolved)
    message(STATUS "[automap_pro] LibTorch: no local path had share/cmake/Torch (tried repo root/libtorch, thrid_party/libtorch, etc.)")
  endif()
endif()

# 仅当解析到的是本地/环境变量路径时设置 Torch_DIR（Torch_DIR 已由 -D 传入时不再覆盖）
if(_libtorch_resolved AND _libtorch_resolved MATCHES "/")
  set(Torch_DIR "${_libtorch_resolved}/share/cmake/Torch" CACHE PATH "LibTorch CMake config")
endif()
find_package(Torch QUIET)
if(Torch_FOUND)
  add_definitions(-DUSE_TORCH)
  message(STATUS "[automap_pro] LibTorch BUILD STATUS: FOUND -> USE_TORCH=1 (OverlapTransformer .pt will be loaded at runtime)")
  message(STATUS "[automap_pro] LibTorch libs: ${TORCH_LIBRARIES}")
else()
  message(STATUS "[automap_pro] LibTorch BUILD STATUS: NOT FOUND -> USE_TORCH=0 (OverlapTransformer uses fallback descriptor only)")
  message(STATUS "[automap_pro] To enable: put libtorch in repo as libtorch/ or set LIBTORCH_HOME or -DTorch_DIR=.../libtorch/share/cmake/Torch")
endif()
message(STATUS "========== [automap_pro] LibTorch section end ==========")

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
  src/config/thread_config.cpp
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
ament_target_dependencies(automap_loop_closure ${AUTOMAP_AMENT_DEPS} ament_index_cpp)
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
  src/system/automap_system_component_plugin.cpp
  src/system/modules/system_init.cpp
  src/system/modules/data_callbacks.cpp
  src/system/modules/service_handlers.cpp
  src/system/modules/tasks_and_utils.cpp
  src/v3/map_registry.cpp
  src/v3/semantic_processor.cpp
  src/v3/semantic_module.cpp
  src/v3/frontend_module.cpp
  src/v3/dynamic_filter_module.cpp
  src/v3/mapping_module.cpp
  src/v3/optimizer_module.cpp
  src/system/frame_processor.cpp
)
target_include_directories(automap_system_component PUBLIC ${AUTOMAP_INCLUDE_DIRS})
target_include_directories(automap_system_component PRIVATE
  "${SLOAM_REC_SLOAM_INCLUDE}"
  "${SLOAM_REC_SLOAM_INCLUDE}/helpers")
if(USE_SLOAM_SEMANTIC)
  target_include_directories(automap_system_component PRIVATE "${_onnx_include}")
  target_compile_definitions(automap_system_component PRIVATE AUTOMAP_USE_SLOAM_SEMANTIC=1)
endif()
# 必须与 ament_target_dependencies 一致：全部用 plain 形式，否则 CMake 报 keyword/plain 混用
target_link_libraries(automap_system_component
  automap_core automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map
  automap_visualization
  Boost::boost
  ${AUTOMAP_LIBS}
)
if(USE_SLOAM_SEMANTIC)
  target_link_libraries(automap_system_component "${_onnx_lib}")
  target_link_libraries(automap_system_component ${_sloam_segmentation_target})
endif()
ament_target_dependencies(automap_system_component ${AUTOMAP_AMENT_DEPS} ament_index_cpp)
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
# VTK 轨迹曲线显示工具（可选；未安装 VTK 时跳过构建）
# ──────────────────────────────────────────────────────────
find_package(VTK QUIET)
if(VTK_FOUND)
  add_executable(vtk_trajectory_viewer src/tools/vtk_trajectory_viewer_node.cpp)
  target_include_directories(vtk_trajectory_viewer PRIVATE ${AUTOMAP_INCLUDE_DIRS})
  if(TARGET VTK::RenderingCore)
    target_link_libraries(vtk_trajectory_viewer VTK::RenderingCore VTK::RenderingUI VTK::IOImage)
  else()
    target_link_libraries(vtk_trajectory_viewer ${VTK_LIBRARIES})
    target_include_directories(vtk_trajectory_viewer PRIVATE ${VTK_INCLUDE_DIRS})
  endif()
  ament_target_dependencies(vtk_trajectory_viewer rclcpp nav_msgs)
  install(TARGETS vtk_trajectory_viewer DESTINATION lib/${PROJECT_NAME})
  message(STATUS "[automap_pro] VTK found: building vtk_trajectory_viewer")
else()
  message(STATUS "[automap_pro] VTK not found: skip vtk_trajectory_viewer (install libvtk9-dev)")
endif()

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
install(PROGRAMS scripts/plot_accuracy_curves.py DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/convert_ot_pth_to_torchscript.py DESTINATION share/${PROJECT_NAME}/scripts)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.pt" PATTERN "*.pth.tar")
endif()

ament_export_include_directories(include)
ament_export_libraries(
  automap_core automap_frontend automap_submap
  automap_loop_closure automap_backend automap_map
  automap_visualization automap_system_component)
ament_export_dependencies(${AUTOMAP_AMENT_DEPS} Eigen3 PCL OpenCV GTSAM)
ament_package()
