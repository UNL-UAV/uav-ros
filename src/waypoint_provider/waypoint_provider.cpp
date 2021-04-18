/**
 * Waypoint provider for Freyja, will provide the 
 * waypoints and Freyja will plan the trajectory and execute
 */

#include <ros/ros.h>
#include <termios.h>
#include "file_utility.hpp"
#include <geometry_msgs/QuaternionStamped.h>

#define NODE_NAME "waypoint_provider"
#define NODE_RATE 10
#define DELTA (1 / NODE_RATE)
#define KEYCODE_right 0x43
#define KEYCODE_down 0x42

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    int mode = 0;

    //n.getParam(n.getNamespace() + "/waypoint_mode", mode);

    ros::Publisher waypoint_publisher;
    waypoint_publisher = n.advertise<geometry_msgs::QuaternionStamped>("/reference_state", 10, true);

    ros::Rate update_rate(NODE_RATE);

    char c;
    int kfd = 0;
    struct termios cooked, raw;
    bool key_pressed = false;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    std::vector<geometry_msgs::QuaternionStamped> waypoints = file_read("/home/unl-uav/uav_workspace/waypoints.csv");

    while(1){

        if(!ros::ok()){
            tcsetattr(kfd, TCSANOW, &cooked);
            break;
        }

        if(waypoint_publisher.getNumSubscribers() == 0){
            ROS_WARN("No current subscribers\n");
        }
        ROS_INFO("Waiting for input from keyboard");
        if(read(kfd, &c, 1) < 0){
            ROS_ERROR("Error with keyboard reader\n");
        }

        if(waypoints.empty()){
            ROS_WARN("No more waypoints to send!\n");
        }else if(c == KEYCODE_right){
            key_pressed = true;

            waypoint_publisher.publish(waypoints.back());

            ROS_INFO("Waypoint published ");
            //ROS_INFO("The waypoint is %.2f %.2f %.2f\n", waypoints.back().terminal_pn, waypoints.back().terminal_pe, waypoints.back().terminal_pd);
            waypoints.pop_back();
        }else if(c == KEYCODE_down){
            
        }

        ros::spinOnce();
        update_rate.sleep();
    }

}