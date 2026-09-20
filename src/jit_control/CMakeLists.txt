cmake_minimum_required(VERSION 3.8)
project(jit_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(jit_msgs REQUIRED)

# local_guided_node parses its mission file itself, because ROS 2 parameters
# cannot express a list of maps.
find_package(yaml-cpp REQUIRED)

add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)
ament_target_dependencies(vehicle_interface_node
  rclcpp std_msgs geometry_msgs mavros_msgs jit_msgs)

add_executable(manual_control_node src/manual_control_node.cpp)
ament_target_dependencies(manual_control_node rclcpp std_msgs mavros_msgs jit_msgs)

add_executable(local_guided_node src/local_guided_node.cpp)
ament_target_dependencies(local_guided_node rclcpp std_msgs geometry_msgs jit_msgs)
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(local_guided_node yaml-cpp::yaml-cpp)
else()
  target_include_directories(local_guided_node PRIVATE ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(local_guided_node ${YAML_CPP_LIBRARIES})
endif()

add_executable(global_guided_node src/global_guided_node.cpp)
ament_target_dependencies(global_guided_node
  rclcpp std_msgs geometry_msgs sensor_msgs jit_msgs)

install(TARGETS
  vehicle_interface_node
  manual_control_node
  local_guided_node
  global_guided_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
