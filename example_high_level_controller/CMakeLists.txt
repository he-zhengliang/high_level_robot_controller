cmake_minimum_required(VERSION 3.8)
project(example_high_level_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(drake REQUIRED)
find_package(abb_driver REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})

add_executable(example_high_level_controller src/example_high_level_controller.cpp)
target_include_directories(example_high_level_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_high_level_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example_high_level_controller
  drake::drake ${Protobuf_LIBRARIES}
)

ament_target_dependencies(example_high_level_controller
  abb_driver
)

install(TARGETS example_high_level_controller
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

ament_package()
