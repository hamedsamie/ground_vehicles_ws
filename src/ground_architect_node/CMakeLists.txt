cmake_minimum_required(VERSION 3.5)
project(ground_architect_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Optional: stricter warnings help keep production code clean
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories(include)

# ------------------------------------------------------------------------------
# Build the node implementation as a reusable library (so tests can link to it)
# ------------------------------------------------------------------------------
add_library(ground_node_lib
  src/middleware/ground_node.cpp
)
ament_target_dependencies(ground_node_lib
  rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces
)
target_include_directories(ground_node_lib PUBLIC include)

# ---------------------------------------------------------
# Small executable that just provides main() and links lib
# ---------------------------------------------------------
add_executable(ground_node
  src/middleware/main.cpp
)
# Use the PLAIN signature (no PRIVATE/PUBLIC/INTERFACE) to match ament style
target_link_libraries(ground_node ground_node_lib)
ament_target_dependencies(ground_node
  rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces
)

# -------------------------
# Install targets & headers
# -------------------------
install(TARGETS ground_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# --------------------------------
# Install config/launch if present
# --------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ---------------
# Tests (gtest)
# ---------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure C++ FSM test
  ament_add_gtest(test_state_machine tests/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)

  # Parameter handling test (links to node implementation)
  ament_add_gtest(test_params tests/test_params.cpp)
  if(TARGET test_params)
    target_include_directories(test_params PRIVATE include)
    ament_target_dependencies(test_params rclcpp rclcpp_lifecycle rcl_interfaces)
    target_link_libraries(test_params ground_node_lib)
  endif()

  # Pub/Sub integration-ish test using an executor + lifecycle transitions
  ament_add_gtest(test_pubsub tests/test_pubsub.cpp)
  if(TARGET test_pubsub)
    target_include_directories(test_pubsub PRIVATE include)
    ament_target_dependencies(test_pubsub
      rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs)
    target_link_libraries(test_pubsub ground_node_lib)
  endif()
endif()

ament_package()
