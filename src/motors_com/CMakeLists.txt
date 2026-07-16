cmake_minimum_required(VERSION 3.8)
project(motors_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencias_externas
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Threads REQUIRED) # Para <thread> y <mutex>
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Ruta de la API de Rozum y su biblioteca compilada
set(ROZUM_API_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Rozum-Servo-Drives-API/c)
set(ROZUM_API_LIB ${ROZUM_API_ROOT}/build/libservo_api.a)
set(ROZUM_API_SO ${ROZUM_API_ROOT}/build/libservo_api.so)

# Compilar la API de Rozum con su Makefile nativo
add_custom_command(
  OUTPUT ${ROZUM_API_LIB} ${ROZUM_API_SO} 
  COMMAND ${CMAKE_MAKE_PROGRAM}
  WORKING_DIRECTORY ${ROZUM_API_ROOT}
  COMMENT "Compilando Rozum-Servo-Drives-API utilizando su Makefile nativo..."
)
add_custom_target(rozum_api_build ALL DEPENDS ${ROZUM_API_LIB} ${ROZUM_API_SO})

# Exponer la librería de Rozum al resto del paquete
add_library(rozum_api SHARED IMPORTED GLOBAL)
set_target_properties(rozum_api PROPERTIES
  IMPORTED_LOCATION "${ROZUM_API_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${ROZUM_API_ROOT}/include;${ROZUM_API_ROOT}/src"
  INTERFACE_LINK_LIBRARIES Threads::Threads
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

# Buscar archivos .hpp en el directorio include
include_directories(
  include
)

ament_package()
