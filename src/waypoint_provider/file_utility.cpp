/*  File reader utility created in an effort to keep the
    ros nodes as clean as possible

    file format is csv

    -Gerson Uriarte, 6/15/2020
*/

#include "file_utility.hpp"
// #include <ros/ros.h>
// #include <string>
// #include <fstream>
// #include <vector>
// #include <sstream>
// #include <utility>
// #include <iostream>
// #include <limits.h>
// #include <unistd.h>


std::vector<geometry_msgs::QuaternionStamped> file_read(const std::string& filename){

    std::vector<geometry_msgs::QuaternionStamped> messages;



    std::ifstream file(filename);

    std::string line, col;
    float pn, pe, pd, vn, ve, vd, yaw, time, speed;

    while(file.good()){
        geometry_msgs::QuaternionStamped line_message;
        getline(file, line);
        std::stringstream ss(line);

        getline(ss,col,',');
        line_message.quaternion.x = stof(col);
        getline(ss,col,',');
        line_message.quaternion.y = stof(col);
        getline(ss,col,',');
        line_message.quaternion.z = stof(col);
        getline(ss,col,',');
        line_message.quaternion.w = stof(col);

        messages.insert(messages.begin(), line_message);
    }  
    file.close();
    ROS_INFO("Returning the messages!\n");
    return messages;
}

std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}