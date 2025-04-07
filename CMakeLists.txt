cmake_minimum_required(VERSION 3.8)
project(gsoc_payload_visualization_and_metrics_)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-sign-compare)
  add_compile_options(-Wno-type-limits)

  endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(payload_visualization_and_metrics src/payload_visualization_and_metrics.cpp)
ament_target_dependencies(payload_visualization_and_metrics rclcpp std_msgs sensor_msgs)
target_link_libraries(payload_visualization_and_metrics pinocchio::pinocchio)

install(TARGETS
  payload_visualization_and_metrics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
