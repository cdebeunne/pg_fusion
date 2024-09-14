# SET(CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo)
SET(CMAKE_BUILD_TYPE DEBUG) # DEBUG / RELEASE

# set(CMAKE_BUILD_TYPE "DEBUG")
project(pg_fusion)
cmake_minimum_required(VERSION 3.14)

# -------------------------------------------------------------------
# Compiler Options
# -------------------------------------------------------------------
add_definitions("-Wall -std=c++17 -O3")

if(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  message(STATUS "Debug configuration")
  add_definitions("-g")
else(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
  message(STATUS "Release configuration")
  add_definitions("-O3")
endif(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

add_definitions(-D_USE_MATH_DEFINES)

# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(isae_slam CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)


# -------------------------------------------------------------------
# Library
# -------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED poseGraph.hpp pipeline.hpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${ISAE_SLAM_LIBRARIES} "/usr/local/lib/isae_slam/libisae_slam.so")

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Executables generation
# -------------------------------------------------------------------
add_executable(pg_main main.cpp)
ament_target_dependencies(pg_main isae_slam rclcpp sensor_msgs visualization_msgs tf2_ros tf2_geometry_msgs std_msgs ublox_msgs cv_bridge)
target_link_libraries(pg_main ${PROJECT_NAME})

install(TARGETS pg_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
