cmake_minimum_required(VERSION 3.10)
project(so_arm_100_hardware)

# Add SCServo subdirectory
add_subdirectory(include/SCServo_Linux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)



add_library(${PROJECT_NAME} SHARED
  src/so_arm_100_interface.cpp
)


target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  SCServo
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface so_arm_100_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add test executable
add_executable(test_servo
  test/test_servo.cpp
)

target_include_directories(test_servo PRIVATE
  include
)

target_link_libraries(test_servo
  SCServo
)

# Install test executable
install(
  TARGETS test_servo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()