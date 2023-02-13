cmake_minimum_required(VERSION 3.5)
project(warrior_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(warrior_common REQUIRED)
#find extern lib
link_directories (${PROJECT_SOURCE_DIR}/lib)
## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/test_hardware_interface.cpp
  src/MF9025_hardware_interface.cpp
  src/can_driver.cpp
  src/go1_hardware_interface.cpp
  src/bsp_go1.cpp
  src/bsp_LK_MF9025.cpp
  src/crc.cpp
#  src/imu_hardware_interface.cpp
)
#link extern lib
target_link_libraries (${PROJECT_NAME}  libcontrolcan_x86.so)
#target_link_libraries (${PROJECT_NAME}  libcontrolcan_aarch64.so)
target_link_libraries (
  ${PROJECT_NAME} Eigen3::Eigen
  warrior_common::warrior_common
  )
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface warrior_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
