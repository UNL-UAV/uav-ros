#pragma once

#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "Vector3.hpp"
#include "Drone.hpp"

class PX4Drone : public Drone{
private:
	ros::NodeHandle _nh;
		
	ros::Publisher _positionPublisher;
	ros::Publisher _yawPublisher;
	ros::Publisher _activityPublisher;
	ros::Publisher _positionRawPublisher;

	ros::Subscriber _stateSubscriber;
	
	ros::ServiceClient _armingClient;
	ros::ServiceClient _modeClient;
	ros::ServiceClient _takeoffClient;
	
	mavros_msgs::State _state;
	Vector3 _position;
public:
	PX4Drone() = default;
	PX4Drone(const DroneSpecs& specs);
	
	virtual void arm(bool = true) override;
	virtual void connect() override;
	virtual void update(float delta) override;
	virtual void mode(DroneFlightModes base, const std::string& custom) override;
	virtual void move(float x, float y, float z, bool offset=true) override;
	virtual void moveRAW(float roll, float pitch, float yaw, float thrust) override;
	virtual void turn(float yaw) override;
	virtual void takeoff() override;
	virtual void land() override;
	virtual void home() override;

	bool isConnected();
	bool isArmed();
	bool isOffboard();

	const mavros_msgs::CommandBool setArmState(bool arm);
	const mavros_msgs::SetMode setMode(const std::string& mode);
	std::string getMode();

	inline const mavros_msgs::State& getState() {
		return this->_state;
	};

	~PX4Drone();
private:
	inline void _setPosition(const Vector3& v){
		this->_position = v;
	};
	geometry_msgs::PoseStamped _pose(float x, float y, float z, bool flu=true);
};