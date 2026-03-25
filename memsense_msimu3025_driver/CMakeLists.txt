cmake_minimum_required(VERSION 3.8)
project(memsense_msimu3025_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(msimu3025_driver src/msimu3025_driver.cpp)
target_compile_features(msimu3025_driver PUBLIC cxx_std_17)
ament_target_dependencies(msimu3025_driver rclcpp sensor_msgs)

target_include_directories(msimu3025_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS msimu3025_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
