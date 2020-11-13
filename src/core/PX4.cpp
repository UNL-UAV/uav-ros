#include "core/PX4.hpp"

PX4::PX4(){
}

void PX4::init(){
	this->_positionPublisher = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	this->_yawPublisher = _nh.advertise<std_msgs::Float32>("gi/set_pose/orientation", 10);
	this->_activityPublisher = _nh.advertise<std_msgs::String>("gi/set_activity/type", 10);
	
	this->_stateSubscriber = _nh.subscribe<mavros_msgs::State>("mavros/state", 10, [this](const mavros_msgs::State::ConstPtr& msg){
		_state = *msg;
	});

	this->_armingClient = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	this->_modeClient = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
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
