cmake_minimum_required(VERSION 3.16)
project(hba_api VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters features)
find_package(GTSAM REQUIRED)
find_package(hba REQUIRED)

# HBA 算法头文件位于 automap_ws/src/hba/include/
set(HBA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../hba/include")
if(NOT EXISTS "${HBA_INCLUDE_DIR}/layer.hpp")
  message(FATAL_ERROR "[hba_api] HBA headers not found at ${HBA_INCLUDE_DIR}")
endif()

add_library(hba_api SHARED src/hba_api.cpp)
add_library(hba_api::hba_api ALIAS hba_api)

target_include_directories(hba_api
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${HBA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)
ament_target_dependencies(hba_api PUBLIC rclcpp visualization_msgs)
target_link_libraries(hba_api
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    gtsam
    ${hba_LIBRARIES}
)

install(TARGETS hba_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(hba_api)
ament_export_dependencies(Eigen3 PCL GTSAM rclcpp visualization_msgs)
ament_package()
