cmake_minimum_required(VERSION 3.8)
project(igus_rebel)

# Set the required C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(igus_rebel_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
)

# Add hardware interface shared library
add_library(${PROJECT_NAME} SHARED
  src/Rebel.cpp
  src/RebelSocket.cpp
  src/CriMessages.cpp
)

target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC)

# Add library dependencies
ament_target_dependencies(${PROJECT_NAME}
  pluginlib hardware_interface igus_rebel_msgs
)

pluginlib_export_plugin_description_file(hardware_interface rebel.xml)

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  rebel.xml
  DESTINATION share/${PROJECT_NAME}
)

# Add executable target for the keyboard_input node
add_executable(keyboard_input_node src/keyboard_input.cpp)

ament_target_dependencies(keyboard_input_node
  rclcpp
  geometry_msgs
  moveit_msgs
  control_msgs)

# Install the keyboard_input executable
install(TARGETS keyboard_input_node
  DESTINATION lib/${PROJECT_NAME})

# Add executable target for the joystick_input node
add_executable(joystick_input_node src/joystick_input.cpp)

ament_target_dependencies(joystick_input_node
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  moveit_msgs
)

# Install the joystick_input executable
install(TARGETS joystick_input_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install package folders
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
