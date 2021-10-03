#include "core/Drone.hpp"

#include "core/PX4Drone.hpp"

std::string uav::to_string(DroneFlightModes modes){
	switch (modes){
		case DroneFlightModes::eUnknown:
			return "UNKOWN";
		case DroneFlightModes::ePreflight:
			return "MAV_MODE_PREFLIGHT";
		break;
		case DroneFlightModes::eStabilizeDisarmed:
			return "MAV_MODE_STABILIZE_DISARM";
		break;
		case DroneFlightModes::eStabilizeArmed:
			return "MAV_MODE_STABILIZE_ARM";
		break;
		case DroneFlightModes::eManualDisarmed:
			return "MAV_MODE_MANUAL_DISARMED";
		break;
		case DroneFlightModes::eManualArmed:
			return "MAV_MODE_MANUAL_ARMED";
		break;
		case DroneFlightModes::eGuidedDisarmed:
			return "MAV_MODE_GUIDED_DISARMED";
		break;
		case DroneFlightModes::eGuidedArmed:
			return "MAV_MODE_GUIDED_ARMED";
		break;
		case DroneFlightModes::eAutoDisarmed:
			return "MAV_MODE_AUTO_DISARMED";
		break;
		case DroneFlightModes::eAutoArmed:
			return "MAV_MODE_AUTO_ARMED";
		break;
		case DroneFlightModes::eTestDisarmed:
			return "MAV_MODE_TEST_DISARMED";
		break;
		case DroneFlightModes::eTestArmed:
			return "MAV_MODE_TEST_ARMED";
		break;
		default: return "UNKOWN"; break; 
	};
};

Drone* Drone::createDrone(const DroneSpecs& specs){
	Drone* drone = nullptr;
	switch(specs.type){
		case DroneSpecs::DroneType::eArdupilot:
		break;
		case DroneSpecs::DroneType::ePX4:
			drone = new PX4Drone(specs);
		break;
		default: break;
	}
	return drone;
};