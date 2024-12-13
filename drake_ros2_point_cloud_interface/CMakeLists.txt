cmake_minimum_required(VERSION 3.8)
project(drake_ros2_point_cloud_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drake_ros REQUIRED)

add_library(drake_ros2_point_cloud_interface src/drake_ros2_point_cloud_interface.cpp)
target_compile_features(drake_ros2_point_cloud_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(drake_ros2_point_cloud_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drake_ros2_point_cloud_interface
  drake::drake
  drake_ros::drake_ros_core
)
ament_target_dependencies(drake_ros2_point_cloud_interface
  rclcpp
  sensor_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(drake_ros2_point_cloud_interface PRIVATE "DRAKE_ROS2_INTERFACE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS drake_ros2_point_cloud_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  drake_ros2_point_cloud_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(sensor_msgs)
ament_export_dependencies(drake_ros)
ament_export_dependencies(drake)

ament_package()
