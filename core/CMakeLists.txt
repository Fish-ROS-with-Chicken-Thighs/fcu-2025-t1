cmake_minimum_required(VERSION 3.8)
project(core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros2_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME} "msg/BallPosition.msg" "msg/Calibrate.msg" "msg/BallDepth.msg" "srv/PickRelease.srv")
#ament_export_dependencies(rosidl_default_runtime)

add_executable(quad_node src/classes/flight_controller.cpp src/classes/quadcopter.cpp)

target_include_directories(quad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/src/classes
)

#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(main_node ${cpp_typesupport_target})

ament_target_dependencies(quad_node rclcpp geometry_msgs nav_msgs ros2_tools std_msgs mavros_msgs)
set(CMAKE_INSTALL_RPATH /usr/local/lib)

install(TARGETS quad_node DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY msg srv DESTINATION share/${PROJECT_NAME})

ament_package()