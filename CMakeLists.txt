cmake_minimum_required(VERSION 3.8)
project(robotic_bird)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(include)

# flight controller node
add_executable(flight_controller src/flight_controller.cpp)
ament_target_dependencies(flight_controller rclcpp geometry_msgs nav_msgs)

# navigator node
add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator rclcpp geometry_msgs)

# optional plugin
add_library(flap_plugin SHARED plugins/flap_plugin.cpp)
ament_target_dependencies(flap_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  gazebo_ros
)
target_link_libraries(flap_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  flight_controller
  navigator
  flap_plugin
  DESTINATION lib/${PROJECT_NAME})

# Install plugins
install(
  TARGETS flap_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch, config, urdf etc.
install(
  DIRECTORY launch urdf rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

