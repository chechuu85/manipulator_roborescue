cmake_minimum_required(VERSION 3.8)
project(manual_user_interface)

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
find_package(std_msgs REQUIRED)

#find_package(SDL2_ttf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_path(SDL2_TTF_INCLUDE_DIR SDL2/SDL_ttf.h)
find_library(SDL2_TTF_LIBRARY SDL2_ttf)
if(NOT SDL2_TTF_LIBRARY OR NOT SDL2_TTF_INCLUDE_DIR)
  message(FATAL_ERROR "No se encontro SDL2_ttf en el sistema.")
endif()

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
add_executable(kdlJointToCartesian src/kdlJointToCartesian.cpp)

# Dice donde buscar archivos
target_include_directories(keyboard PUBLIC ${SDL2_TTF_INCLUDE_DIR})

# Enlace todos las librerías compiladas
target_link_libraries(keyboard SDL2::SDL2 ${SDL2_TTF_LIBRARY})
target_link_libraries(kdlCartesianToJoint Eigen3::Eigen)

install(TARGETS
  keyboard
  adapterToSimulation
  kdlCartesianToJoint
  kdlJointToCartesian
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(keyboard rclcpp manipulator_msgs sensor_msgs geometry_msgs std_msgs)
ament_target_dependencies(adapterToSimulation rclcpp manipulator_msgs sensor_msgs)
ament_target_dependencies(kdlCartesianToJoint rclcpp manipulator_msgs sensor_msgs geometry_msgs kdl_parser)
ament_target_dependencies(kdlJointToCartesian rclcpp manipulator_msgs sensor_msgs geometry_msgs kdl_parser)

ament_package()
