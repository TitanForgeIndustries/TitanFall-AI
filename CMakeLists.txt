cmake_minimum_required(VERSION 3.8)
project(titanfall_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for advanced features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Optional: TensorFlow Lite
find_package(TensorflowLite QUIET)
if(TensorflowLite_FOUND)
  message(STATUS "TensorFlow Lite found - ML inference enabled")
  add_compile_definitions(USE_TFLITE)
  set(TFLITE_LIBS tensorflow-lite)
else()
  message(STATUS "TensorFlow Lite not found - using rule-based fallback")
  set(TFLITE_LIBS "")
endif()

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

# Create main executable
add_executable(titanfall_ai_node src/main.cpp)
target_link_libraries(titanfall_ai_node titanfall_ai_lib ${TFLITE_LIBS})

# Create example executable
add_executable(intent_recognition_example examples/intent_recognition_example.cpp)
target_link_libraries(intent_recognition_example titanfall_ai_lib ${TFLITE_LIBS})
ament_target_dependencies(intent_recognition_example
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  titanfall_ai_lib
  titanfall_ai_node
  intent_recognition_example
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

# Install scripts
install(PROGRAMS
  scripts/train_intent_model.py
  scripts/collect_training_data.py
  scripts/test_intent_model.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install documentation
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# Install examples
install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

ament_package()
