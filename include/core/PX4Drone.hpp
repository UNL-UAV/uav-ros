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

enum class PX4DroneFlightModes{
	eUnknown,
	ePosition,
	eAltitude,
	eManual,
	eTakeoff,
	eLand,
	eHold,
	eReturn,
	eMission,
	eOffboard
};

namespace uav{
	std::string to_string(PX4DroneFlightModes modes){
		switch (modes){
			case PX4DroneFlightModes::eUnknown:
				return "UNKOWN";
			case PX4DroneFlightModes::ePosition:
				return "Position";
			break;
			case PX4DroneFlightModes::eAltitude:
				return "Altitude";
			break;
			case PX4DroneFlightModes::eManual:
				return "Manual";
			break;
			case PX4DroneFlightModes::eTakeoff:
				return "Takeoff";
			break;
			case PX4DroneFlightModes::eLand:
				return "Land";
			break;
			case PX4DroneFlightModes::eHold:
				return "Hold";
			break;
			case PX4DroneFlightModes::eReturn:
				return "Return";
			break;
			case PX4DroneFlightModes::eMission:
				return "Mission";
			break;
			case PX4DroneFlightModes::eOffboard:
				return "Offboard";
			break;
		};
	};
}

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
	
	virtual void connect() override;
	virtual void update(float delta) override;
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
	void _action(PX4DroneFlightModes action);
};