#include "core/PX4.hpp"
#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"

PX4::PX4(){
}

void PX4::init(){
	this->_positionPublisher = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	this->_yawPublisher = _nh.advertise<std_msgs::Float32>("gi/set_pose/orientation", 10);
	this->_activityPublisher = _nh.advertise<std_msgs::String>("gi/set_activity/type", 10);
	this->_positionRawPublisher = _nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
	
	this->_stateSubscriber = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, [this](const mavros_msgs::State::ConstPtr& msg){
		_state = *msg;
	});

	this->_armingClient = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	this->_modeClient = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	this->_takeoffClient = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
}

void PX4::update(float delta){
	
}

const mavros_msgs::CommandBool PX4::setArmState(bool arm){
	mavros_msgs::CommandBool armMSG;
	armMSG.request.value = arm;
	this->_armingClient.call(armMSG);
	return armMSG;
}

const mavros_msgs::SetMode PX4::setMode(const std::string& mode){
	mavros_msgs::SetMode modeMSG;
	modeMSG.request.custom_mode = mode;
	this->_modeClient.call(modeMSG);
	return modeMSG;
}

std::string PX4::getMode(){
	return std::string(this->_state.mode);
}

void PX4::move(float x, float y, float z, bool offset){
	setPosition({x, y, z});
	_positionPublisher.publish(this->_pose(x, y, z, offset));
}

void PX4::moveRAW(float roll, float pitch, float yaw, float thrust){
	Vector3 _position;
	geometry_msgs::Quaternion q;
	mavros_msgs::AttitudeTarget attitude;
	geometry_msgs::Vector3 position;

	q.w = 0;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	attitude.header.stamp = ros::Time::now();
	attitude.orientation = q;
	attitude.thrust = thrust;

	position.x = roll;
	position.y = pitch;
	position.z = yaw;
	
	attitude.body_rate = position;
	attitude.type_mask = 0;

	_positionRawPublisher.publish(attitude);
}

void PX4::turn(float yaw){
	std_msgs::Float32 val;
	val.data = yaw;
	_yawPublisher.publish(val);
}

void PX4::takeoff(){
	_action("HOVER");
}

//requesting a takeoff of 1m of altitude
void PX4::takeoff_request(){
	mavros_msgs::CommandTOL takeoff_command;
	takeoff_command.request.altitude = 1;
	while(!takeoff_command.response.success){
		ros::Duration(1).sleep();
		_takeoffClient.call(takeoff_command);
	}
	ROS_INFO("Takeoff Initialized");
}

void PX4::land(){
	_action("LAND");
}

void PX4::home(){
	this->move(0, 0, 1, false);
}

bool PX4::isConnected(){
	return this->_state.connected;
}

bool PX4::isArmed(){
	return (this->_state.armed == 1)? true:false;
}

bool PX4::isOffboard(){
	return (this->_state.mode == "OFFBOARD") ? true : false;
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

PX4::~PX4(){}
