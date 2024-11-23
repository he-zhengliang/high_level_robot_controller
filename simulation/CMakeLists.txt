cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drake REQUIRED)
find_package(drake_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(simulation 
  src/simulation.cpp 
  src/robot_diagram.cpp
  src/abb_motion_planner.cpp
  src/svh_motion_planner.cpp
  src/oakd_s2_camera.cpp
  src/package_path.cpp
  src/ros_message_creator.cpp
)
target_include_directories(simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simulation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(simulation
  drake::drake
  drake_ros::drake_ros_core
)
ament_target_dependencies(simulation
  rclcpp
  control_msgs
  trajectory_msgs
  sensor_msgs
)

install(TARGETS simulation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY world svh abb
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
