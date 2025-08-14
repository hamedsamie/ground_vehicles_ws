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

add_executable(ground_node
  src/middleware/main.cpp
  src/middleware/ground_node.cpp
)

# Link the executable against the ROS 2 libs it uses
ament_target_dependencies(ground_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  lifecycle_msgs
  rcl_interfaces
)

install(TARGETS ground_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_state_machine tests/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
endif()

ament_package()
