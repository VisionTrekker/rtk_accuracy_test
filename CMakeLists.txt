cmake_minimum_required(VERSION 3.8)
project(rtk_accuracy_test)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RtkStatus.idl"
  DEPENDENCIES std_msgs
)

add_executable(bx100_test_recorder src/bx100_test_recorder.cpp)
target_compile_features(bx100_test_recorder PUBLIC cxx_std_17)
ament_target_dependencies(bx100_test_recorder rclcpp std_msgs sensor_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bx100_test_recorder "${cpp_typesupport_target}")

install(TARGETS bx100_test_recorder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(PROGRAMS
  scripts/rtk_offline_analysis.py
  scripts/serial_capture.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
