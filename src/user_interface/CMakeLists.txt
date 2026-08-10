cmake_minimum_required(VERSION 3.8)
project(user_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
)

add_executable(keyboard src/keyboard.cpp)
add_executable(adapterToSimulation src/adapterToSimulation.cpp)
add_executable(kdlCartesianToJoint src/kdlCartesianToJoint.cpp)


target_link_libraries(keyboard SDL2::SDL2)

install(TARGETS
  keyboard
  adapterToSimulation
  kdlCartesianToJoint
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(keyboard rclcpp manipulator_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(adapterToSimulation rclcpp manipulator_msgs sensor_msgs)
ament_target_dependencies(kdlCartesianToJoint rclcpp manipulator_msgs sensor_msgs geometry_msgs kdl_parser)

ament_package()
