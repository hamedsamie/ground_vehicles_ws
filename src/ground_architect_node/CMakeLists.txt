cmake_minimum_required(VERSION 3.5)
project(ground_architect_node)

# ===================
# Compiler & warnings
# ===================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# ============
# Dependencies
# ============
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# For generating of custom action
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

# ===================
# Interfaces (action)
# ===================
set(ACTION_FILES
  "action/ExecuteMission.action"
)

# Generate action headers into the build tree
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces
)

set(GENERATED_ROSIDL_CPP_DIR ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_export_dependencies(rosidl_default_runtime)

# Path to generated C++ headers
set(GENERATED_ROSIDL_CPP_DIR ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Export runtime
ament_export_dependencies(rosidl_default_runtime)

# ====================
# Library & Executable
# ====================
# Node implementation as a reusable library
add_library(ground_node_lib
  src/middleware/ground_node.cpp
)
target_include_directories(ground_node_lib PUBLIC
  include
  ${GENERATED_ROSIDL_CPP_DIR}
)

# Small executable that just provides main() and links the lib
add_executable(ground_node
  src/middleware/main.cpp
)
target_link_libraries(ground_node ground_node_lib)

add_dependencies(ground_node_lib ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(ground_node     ${PROJECT_NAME}__rosidl_generator_cpp)

# Make sure generated typesupport libs are available before linking
add_dependencies(ground_node_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(ground_node     ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link against the C++ typesupport for generated action
target_link_libraries(ground_node_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(ground_node     ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link target dependencies
ament_target_dependencies(ground_node_lib
  rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces std_srvs
  rosidl_default_runtime
)
ament_target_dependencies(ground_node
  rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces std_srvs
  rosidl_default_runtime
)

# =======
# Install
# =======
install(TARGETS ground_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install config/launch if present
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

# =====
# Tests
# =====
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure C++ FSM test
  ament_add_gtest(test_state_machine tests/test_state_machine.cpp)
  if(TARGET test_state_machine)
    target_include_directories(test_state_machine PRIVATE include)
  endif()

  # Parameter handling test (links to node implementation)
  ament_add_gtest(test_params tests/test_params.cpp)
  if(TARGET test_params)
    target_include_directories(test_params PRIVATE include)
    ament_target_dependencies(test_params rclcpp rclcpp_lifecycle rcl_interfaces)
    target_link_libraries(test_params ground_node_lib)
  endif()

  # Pub/Sub test with executor + lifecycle transitions
  ament_add_gtest(test_pubsub tests/test_pubsub.cpp)
  if(TARGET test_pubsub)
    target_include_directories(test_pubsub PRIVATE include)
    ament_target_dependencies(test_pubsub
      rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces
    )
    target_link_libraries(test_pubsub ground_node_lib)
  endif()

  # Services + Action test
  ament_add_gtest(test_services_actions tests/test_services_actions.cpp)
  if(TARGET test_services_actions)
    target_include_directories(test_services_actions PRIVATE
      include
      ${GENERATED_ROSIDL_CPP_DIR}
    )
  ament_target_dependencies(test_services_actions
    rclcpp rclcpp_lifecycle rclcpp_action std_msgs lifecycle_msgs rcl_interfaces std_srvs
    rosidl_default_runtime
  )
  target_link_libraries(test_services_actions ground_node_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
  add_dependencies(test_services_actions ${PROJECT_NAME}__rosidl_generator_cpp ${PROJECT_NAME}__rosidl_typesupport_cpp)
endif()
endif()

# =======
# Package
# =======
ament_package()
