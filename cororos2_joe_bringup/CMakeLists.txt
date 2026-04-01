cmake_minimum_required(VERSION 3.8)
project(cororos2_joe_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sim_sensor_frame_republisher
  src/sim_sensor_frame_republisher.cpp)
target_compile_features(sim_sensor_frame_republisher PUBLIC cxx_std_17)
ament_target_dependencies(sim_sensor_frame_republisher rclcpp sensor_msgs)

install(
  TARGETS sim_sensor_frame_republisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
