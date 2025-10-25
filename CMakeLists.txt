cmake_minimum_required(VERSION 3.8)
project(titanfall_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(titanfall_ai_lib
  src/sensor_fusion.cpp
  src/intent_model.cpp
  src/torque_controller.cpp
  src/energy_manager.cpp
  src/fault_detection.cpp
  src/thermal_manager.cpp
  src/main_controller.cpp
)

ament_target_dependencies(titanfall_ai_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Create executable
add_executable(titanfall_ai_node src/main.cpp)
target_link_libraries(titanfall_ai_node titanfall_ai_lib)

# Install targets
install(TARGETS
  titanfall_ai_lib
  titanfall_ai_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install data directory
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()
