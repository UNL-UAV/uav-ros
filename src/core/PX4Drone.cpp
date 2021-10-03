#include "core/PX4Drone.hpp"

#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Quaternion.h>
#include <stdexcept>

#include "core/exceptions/ArmFailed.hpp"
#include "core/exceptions/ServiceCallFailed.hpp"


PX4Drone::PX4Drone(const DroneSpecs& specs) : _nh(specs.nodeHandle){
	ROS_INFO("CREATED PX4 Drone");
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
};


void PX4Drone::arm(bool arm){
	if(isArmed()){ return; }
	
	mavros_msgs::CommandBool cmd;
	cmd.request.value = arm;

	if(_armingClient.call(cmd)){
		throw ServiceCallFailedException("Arming");
	}

	if(!cmd.response.success){
		throw ArmFailedExecption();
	}
}

void PX4Drone::connect() {
	ros::Rate tmpRate(25);
	while(ros::ok() && isConnected()){ ros::spinOnce(); tmpRate.sleep(); }
};

void PX4Drone::PX4Drone::update(float delta) {
};

void PX4Drone::mode(DroneFlightModes base, const std::string& custom){
	if(custom.length() && base != DroneFlightModes::ePreflight){
		throw std::invalid_argument("Custom mode only support DroneFlightModes::ePreflight");
	}

	mavros_msgs::SetMode modeMSG;
	modeMSG.request.base_mode = (uint8_t)base;
	if(custom.length())
		modeMSG.request.custom_mode = custom.c_str();
	
	if(!this->_modeClient.call(modeMSG)){
		throw ServiceCallFailedException("SetMode");
	}

	if(!modeMSG.response.mode_sent){
		throw ServiceCallFailedException("SetMode");
	}
};

void PX4Drone::PX4Drone::move(float x, float y, float z, bool offset) {
	_setPosition({x, y, z});
	_positionPublisher.publish(this->_pose(x, y, z, offset));
};

void PX4Drone::PX4Drone::moveRAW(float roll, float pitch, float yaw, float thrust) {
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
};

void PX4Drone::PX4Drone::turn(float yaw) {
	std_msgs::Float32 val;
	val.data = yaw;
	_yawPublisher.publish(val);
};

void PX4Drone::PX4Drone::takeoff() {
	mode(DroneFlightModes::ePreflight, "TAKEOFF");
};
void PX4Drone::PX4Drone::land() {
	mode(DroneFlightModes::ePreflight, "LAND");
};
void PX4Drone::PX4Drone::home() {
	this->move(0, 0, 1, false);
};

bool PX4Drone::isConnected(){
	return this->_state.connected;
}

bool PX4Drone::isArmed(){
	return (this->_state.armed == 1)? true:false;
}

bool PX4Drone::isOffboard(){
	return (this->_state.mode == "OFFBOARD") ? true : false;
}

std::string PX4Drone::getMode(){
	return std::string(this->_state.mode);
}

geometry_msgs::PoseStamped PX4Drone::_pose(float x, float y, float z, bool flu){
	auto ret = geometry_msgs::PoseStamped();
	ret.header.stamp = ros::Time::now(); // NOW
	ret.header.frame_id = (flu)? "base_link" : "map";
	ret.pose.position.x = x;
	ret.pose.position.y = y;
	ret.pose.position.z = z;
	return ret;
};

// Is this needed?
// void PX4Drone::_action(PX4DroneFlightModes action){
// 	std_msgs::String val;
// 	val.data = uav::to_string(action);
// 	_activityPublisher.publish(val);
// };

PX4Drone::~PX4Drone(){};