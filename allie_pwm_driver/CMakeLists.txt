cmake_minimum_required(VERSION 3.8)
project(allie_pwm_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(allie_pwm_driver src/allie_pwm_driver.cpp)
target_compile_features(allie_pwm_driver PUBLIC cxx_std_17)
ament_target_dependencies(allie_pwm_driver geometry_msgs rclcpp std_msgs)

install(TARGETS allie_pwm_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
