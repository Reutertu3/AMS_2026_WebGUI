cmake_minimum_required(VERSION 3.16)
project(minimal_edurob_node C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim_msgs REQUIRED)
# edurob_interfaces (gyro calibration service) is not part of ROS: build it once with
#   ./ros.sh colcon build --base-paths src --packages-select edurob_interfaces   (from the project folder)
find_package(edurob_interfaces REQUIRED)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/minimal_edurob_node.cpp)
  add_executable(minimal_edurob_node minimal_edurob_node.cpp)
  target_link_libraries(minimal_edurob_node rclcpp::rclcpp ${geometry_msgs_TARGETS} tf2_ros::tf2_ros)
endif()

# Arduino-style program: main.c (setup/loop) + edurobot.cpp (movement library)
add_executable(edurobot main.c edurobot.cpp)
target_link_libraries(edurobot rclcpp::rclcpp ${geometry_msgs_TARGETS} tf2_ros::tf2_ros ${turtlesim_msgs_TARGETS} ${edurob_interfaces_TARGETS} m)
