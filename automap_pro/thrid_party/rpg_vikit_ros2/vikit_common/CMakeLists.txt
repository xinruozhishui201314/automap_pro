SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.5)  # ROS2 Humble需要至少CMake 3.5
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

# Sophus 需要 C++14+（enable_if_t/optional 等），统一使用 C++17
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# Sophus: 优先使用工作空间内本地 thrid_party/Sophus（与 vikit_common 相对路径 ../../Sophus）
SET(SOPHUS_LOCAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Sophus")
IF(EXISTS "${SOPHUS_LOCAL_DIR}/CMakeLists.txt" AND EXISTS "${SOPHUS_LOCAL_DIR}/sophus/se3.hpp")
  message(STATUS "Using local Sophus: ${SOPHUS_LOCAL_DIR}")
  SET(SOPHUS_MASTER_PROJECT OFF)
  SET(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Build Sophus tests" FORCE)
  ADD_SUBDIRECTORY(${SOPHUS_LOCAL_DIR} ${CMAKE_CURRENT_BINARY_DIR}/sophus_build)
  SET(SOPHUS_USE_LOCAL TRUE)
ELSE()
  FIND_PACKAGE(Sophus REQUIRED)
  SET(SOPHUS_USE_LOCAL FALSE)
ENDIF()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
IF(NOT SOPHUS_USE_LOCAL)
  INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})
ENDIF()

# ROS2 build system
SET(USE_ROS2 TRUE)  # ROS2 specific build
IF(USE_ROS2)
  FIND_PACKAGE(ament_cmake REQUIRED)
  FIND_PACKAGE(rclcpp REQUIRED)  # ROS2 version of roscpp
  LIST(APPEND INCLUDE_DIRECTORIES ${rclcpp_INCLUDE_DIRS})
  # ament_package() moved to the end, after all ament_export_* calls
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
IF(SOPHUS_USE_LOCAL)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} Sophus::Sophus)
ENDIF()
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES})  # Link to rclcpp for ROS2
IF(NOT SOPHUS_USE_LOCAL)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Sophus_LIBRARIES})
ENDIF()

# Tests
ADD_EXECUTABLE(test_vk_common_camera test/test_camera.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_camera ${PROJECT_NAME})

ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME})

ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME})

################################################################################
# Install
IF(NOT USE_ROS2)
  GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ELSE()
  # ROS2 安装规则
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(OpenCV Eigen3 rclcpp)

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # ROS2 package declaration (must be called after all ament_export_* functions)
  ament_package()
ENDIF()
