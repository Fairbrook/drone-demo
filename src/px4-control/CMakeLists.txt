cmake_minimum_required(VERSION 3.16)
project(px4_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DroneState.msg
  msg/DroneCommand.msg
  msg/PDDiagnostics.msg
  DEPENDENCIES std_msgs
)

add_executable(px4_control_node
  src/main.cpp
  src/px4_control_node.cpp
  src/pd_controller.cpp
)

target_include_directories(px4_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(px4_control_node
  rclcpp
  std_msgs
  geometry_msgs
  px4_msgs
)

# Link generated interfaces from this same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(px4_control_node ${cpp_typesupport_target})

install(TARGETS px4_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
