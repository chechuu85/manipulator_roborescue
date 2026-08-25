cmake_minimum_required(VERSION 3.8)
project(advanced_user_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ==========================================
# CREAR LA LIBRERÍA MATEMÁTICA
# ==========================================

# Declarar trajectory_math como una librería compartida (SHARED)
add_library(trajectory_math SHARED 
  src/trajectory_math.cpp
)

# Añadir los directorios de cabecera de tu paquete a la librería
target_include_directories(trajectory_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Enlazar Eigen3 a tu librería.
target_link_libraries(trajectory_math Eigen3::Eigen yaml-cpp)
ament_target_dependencies(trajectory_math tf2 geometry_msgs)

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

add_executable(trajectoryPlanning src/trajectoryPlanning.cpp)

target_link_libraries(trajectoryPlanning trajectory_math yaml-cpp Eigen3::Eigen)

install(TARGETS trajectory_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS trajectoryPlanning
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(trajectoryPlanning rclcpp geometry_msgs std_msgs manipulator_msgs ament_index_cpp)

ament_package()
