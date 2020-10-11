#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

typedef mavros_msgs::State state_msg;

class safety_check{
    private:

    state_msg drone_state;

    public:

    safety_check(const state_msg::ConstPtr& msg)

    bool armed(const mavros_msgs::State::ConstPtr& msg);

    bool offboard_mode(const state_msg::ConstPtr& msg);

    void set_state(const state_msg::ConstPtr& msg)

}