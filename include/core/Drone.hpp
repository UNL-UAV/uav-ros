#pragma once

#include <ros/ros.h>

struct DroneSpecs{
	enum class DroneType{
		eArdupilot, ePX4
	} type;
	ros::NodeHandle nodeHandle;
};

class Drone{
private:
public:

	static Drone* createDrone(const DroneSpecs& specs);

	Drone() = default;
	
	virtual void connect() = 0;
	virtual void update(float delta) = 0;
	virtual void move(float x, float y, float z, bool offset=true) = 0;
	virtual void moveRAW(float roll, float pitch, float yaw, float thrust) = 0;
	virtual void turn(float yaw) = 0;
	virtual void takeoff() = 0;
	virtual void land() = 0;
	virtual void home() = 0;

	virtual ~Drone() = default;
private:
};