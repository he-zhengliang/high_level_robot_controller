cmake_minimum_required(VERSION 3.8)
project(abb_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(drake REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
set(PROTO_SRC egm/egm.pb.cc)
set(PROTO_HDR egm/egm.pb.h)

add_library(abb_driver
	src/abb_driver.cpp ${PROTO_SRC}
)

target_include_directories(abb_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(abb_driver PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/egm>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(abb_driver
	drake::drake
  ${Protobuf_LIBRARIES}
)

target_compile_definitions(abb_driver PRIVATE "ABB_DRIVER_PACKAGE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS abb_driver
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
  abb_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(Protobuf)

ament_package()
