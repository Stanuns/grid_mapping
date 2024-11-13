cmake_minimum_required(VERSION 3.8)
project(grid_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  )

add_executable(grid_mapping_ros src/grid_mapping.cpp src/grid_mapping_ros.cpp)
ament_target_dependencies(
  grid_mapping_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(grid_mapping_ros Eigen3::Eigen)

add_library(${PROJECT_NAME}
  src/bresenham.cpp
  src/control.cpp
  src/grid_map.cpp
  src/motion.cpp
  src/observation.cpp
  src/particle.cpp
  src/particle_filter.cpp
  src/resample.cpp
  src/scan_matcher.cpp
  src/pose.cpp
  src/point.cpp
  )
target_link_libraries(grid_mapping_ros ${PROJECT_NAME})

add_executable(odom_tf_pub src/odom_tf_pub.cpp)
ament_target_dependencies(
  odom_tf_pub
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  grid_mapping_ros
  odom_tf_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME})

ament_package()
