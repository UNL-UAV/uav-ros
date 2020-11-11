#include "core/PX4.hpp"

PX4::PX4(){
}

void PX4::init(){
	this->_positionPublisher = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	this->_yawPublisher = _nh.advertise<std_msgs::Float32>("gi/set_pose/orientation", 10);
	this->_activityPublisher = _nh.advertise<std_msgs::String>("gi/set_activity/type", 10);
}

void PX4::move(float x, float y, float z, bool offset=true){
	_positionPublisher.publish(this->_pose(x, y, z, offset));
}
void PX4::turn(float yaw){
	std_msgs::Float32 val;
	val.data = yaw;
	_yawPublisher.publish(val);
}

void PX4::takeoff(){
	_action("HOVER");
}

void PX4::land(){
	_action("LAND");
}

void PX4::home(){
	this->move(0, 0, 32, false);
}

geometry_msgs::PoseStamped PX4::_pose(float x, float y, float z, bool flu=true){
	auto ret = geometry_msgs::PoseStamped();
	ret.header.stamp = ros::Time::now(); // NOW
	ret.header.frame_id = (flu)? "base_link" : "map";
	ret.pose.position.x = x;
	ret.pose.position.y = y;
	ret.pose.position.z = z;
	return ret;
}

void PX4::_action(const std::string& action){
	std_msgs::String val;
	val.data = action;
	_activityPublisher.publish(val);
}

PX4::~PX4(){
}
