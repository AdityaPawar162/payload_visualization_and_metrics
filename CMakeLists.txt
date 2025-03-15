cmake_minimum_required(VERSION 3.8)
project(payload_visualization_and_metrics)

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


add_executable(pinocchio_task src/pinocchio_task.cpp)
ament_target_dependencies(pinocchio_task rclcpp std_msgs)
target_link_libraries(pinocchio_task pinocchio::pinocchio)

install(TARGETS
  pinocchio_task
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
