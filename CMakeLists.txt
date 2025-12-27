cmake_minimum_required(VERSION 3.8)
project(minicar_diff_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(include)

# Create PCA9685 controller library
add_library(pca9685_controller SHARED
  src/pca9685_controller.cpp
)

# Create GPIO motor controller library
add_library(gpio_motor_controller SHARED
  src/gpio_motor_controller.cpp
)

target_include_directories(pca9685_controller PRIVATE
  include
)

target_include_directories(gpio_motor_controller PRIVATE
  include
)

# No external libraries needed for sysfs GPIO

ament_target_dependencies(pca9685_controller
  rclcpp
)

ament_target_dependencies(gpio_motor_controller
  rclcpp
)

# Create hardware interface library
add_library(minicar_hardware_interface SHARED
  src/minicar_hardware_interface.cpp
)

target_include_directories(minicar_hardware_interface PRIVATE
  include
)

target_link_libraries(minicar_hardware_interface
  pca9685_controller
)

ament_target_dependencies(minicar_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface minicar_hardware_interface_plugin.xml)

# Install libraries
install(TARGETS minicar_hardware_interface pca9685_controller gpio_motor_controller
  EXPORT export_minicar_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install urdf files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/
)

# Install plugin description file
install(FILES minicar_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_targets(export_minicar_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()