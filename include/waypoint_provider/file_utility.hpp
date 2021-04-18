#pragma once
#include <pch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
std::vector<geometry_msgs::QuaternionStamped> file_read(const std::string& filename);