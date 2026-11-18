cmake_minimum_required(VERSION 3.8)
project(gears_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(gears_realtime_driver 
  src/gears_driver.cpp
  src/encoder_reader.cpp
)
ament_target_dependencies(gears_realtime_driver rclcpp geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(gears_realtime_driver gpiod)


install(TARGETS
  gears_realtime_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
