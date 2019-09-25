cmake_minimum_required(VERSION 2.8.3)
project(sele_path_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  roscpp
  message_filters
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  eigen_conversions
  elevation_mapping
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    grid_map_core
    grid_map_ros
    grid_map_msgs
    pcl_ros
    roscpp
    message_filters
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    eigen_conversions
    elevation_mapping
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(se_planner_node
  src/main.cpp
  src/se_planner.cpp
  src/Pioneer3AT.cpp
)

target_link_libraries(se_planner_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBS}
)

add_executable(savedata
  src/savedata.cpp
)

target_link_libraries(savedata
    ${catkin_LIBRARIES}
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBS}
)
