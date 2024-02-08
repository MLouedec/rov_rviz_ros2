//
// Created by morgan on 08/02/24.
// extract specific data from the bag file and save it in a csv file to be used in Matlab or python
//
#include <chrono>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
