cmake_minimum_required(VERSION 3.8)
project(abb_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Protobuf REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
set(PROTO_SRC egm/egm.pb.cc)
set(PROTO_HDR egm/egm.pb.h)

add_executable(abb_ros2_driver src/abb_ros2_driver.cpp ${PROTO_SRC})
target_include_directories(abb_ros2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(abb_ros2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/egm>
  $<INSTALL_INTERFACE:egm>)

target_compile_features(abb_ros2_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(abb_ros2_driver
  rclcpp
  geometry_msgs
  control_msgs
)
target_link_libraries(abb_ros2_driver ${Protobuf_LIBRARIES})

install(TARGETS abb_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

ament_package()
