#include "safety_check.hpp"

state_msg drone_state;

void state_cb(const state_msg::ConstPtr& msg){
    drone_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "safety_test");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<state_msg>("mavros/state", 10, state_cb);

    ros::Rate rate(RATE);

    while(ros::ok() && !drone_state.connected){
        ros::spinOnce();
        rate.sleep();

        ROS_WARN("Waiting for flight controller to connect!");
    }

    ROS_WARN("Starting safety check test. All values below");
    safety_check drone_check(drone_state);

    while(ros::ok()){
        drone_check.update_state(drone_state);

        if(drone_check.armed()) ROS_INFO("Drone is armed");
        else ROS_INFO("Drone is not armed");
        
        if(drone_check.offboard_mode()) ROS_INFO("Drone is in offboard Mode");
        else ROS_INFO("Drone is not in offboard Mode");

        if(drone_check.connected()) ROS_INFO("Drone is connected");
        else ROS_INFO("Drone is not connected");


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}