cmake_minimum_required(VERSION 3.8)
project(gmps_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gmps_msgs REQUIRED) 
find_package(can_msgs REQUIRED)

add_executable(gmps_driver
 src/gmps_driver.cpp
)
ament_target_dependencies(gmps_driver
 rclcpp
 gmps_msgs
 std_msgs
 geometry_msgs
 can_msgs
)

install(TARGETS
  gmps_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
