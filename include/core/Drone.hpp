#pragma once

#include <ros/ros.h>

struct DroneSpecs{
	enum class DroneType{
		eArdupilot, ePX4
	} type;
	ros::NodeHandle nodeHandle;
};

enum class DroneFlightModes : uint8_t{
	eUnknown = 0xF,
	ePreflight = 0,
	eStabilizeDisarmed = 80,
	eStabilizeArmed = 208,
	eManualDisarmed = 64,
	eManualArmed = 192,
	eGuidedDisarmed = 88,
	eGuidedArmed = 216,
	eAutoDisarmed = 92,
	eAutoArmed = 220,
	eTestDisarmed = 66,
	eTestArmed = 194,
};

namespace uav{
	std::string to_string(DroneFlightModes modes);
};

class Drone{
private:
public:

	static Drone* createDrone(const DroneSpecs& specs);

	Drone() = default;
	
	virtual void arm(bool = true) = 0;
	virtual void connect() = 0;
	virtual void update(float delta) = 0;
	virtual void mode(DroneFlightModes base, const std::string& custom) = 0;
	virtual void move(float x, float y, float z, bool offset=true) = 0;
	virtual void moveRAW(float roll, float pitch, float yaw, float thrust) = 0;
	virtual void turn(float yaw) = 0;
	virtual void takeoff() = 0;
	virtual void land() = 0;
	virtual void home() = 0;

	virtual ~Drone() = default;
private:
};