# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target jit_msgs::jit_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${jit_msgs_TARGETS}.
if(jit_msgs_TARGETS AND NOT TARGET jit_msgs::jit_msgs)
  add_library(jit_msgs::jit_msgs INTERFACE IMPORTED)
  set_target_properties(jit_msgs::jit_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${jit_msgs_TARGETS}")
endif()
