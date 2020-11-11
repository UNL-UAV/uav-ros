#ifndef SAFETY_CHECK_H
#define SAFETY_CHECK_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

typedef mavros_msgs::State state_msg;

//defines the rate at which the safety checks should be ran
#define RATE 1

class safety_check{
    private:

    state_msg drone_state;

    public:

    safety_check( state_msg msg);

    bool armed();

    bool offboard_mode();

    void update_state(state_msg msg);

    bool connected();
};

#endif SAFETY_CHECK_H