#include "core/Drone.hpp"

#include "core/PX4Drone.hpp"

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