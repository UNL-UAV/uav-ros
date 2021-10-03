#include <pch.hpp>
#include "core/EntryPoint.hpp"
#include <ros/ros.h>
#include "core/PX4.hpp"

#include "core/Drone.hpp"
#include "core/PX4Drone.hpp"

#include "core/exceptions/ServiceCallFailed.hpp"

class NewTakeoffApp : public Application {
private:
	Drone* _drone;
	PX4Drone* _px4Drone;
	ros::Time _last;
	int _increment=0;
public:
	NewTakeoffApp(int argc, char** argv) : Application(argc, argv, "newTakeOff"){};
	void init() override{
		ROS_INFO("STARTED ROS");
		
		this->_drone = Drone::createDrone({
			DroneSpecs::DroneType::ePX4,
			*_nh
		});

		this->_px4Drone = static_cast<PX4Drone*>(_drone);
		
		ros::Rate tmpRate(25);
		this->_px4Drone->connect();
		ROS_INFO("CONNECTED TO DRONE");

		for(int i=0; i<100; i++){ 
			this->_px4Drone->move(0, 0, 2); 
			ros::spinOnce(); 
			tmpRate.sleep();
		}
		_last = ros::Time::now();
	};
	void update(float delta) override{
		this->_px4Drone->update(delta);
		if(ros::Time::now() - _last > ros::Duration(5.0)){		
			try{
				// this->_px4Drone->mode(DroneFlightModes::ePreflight, "GUIDED_NOGPS");
				this->_px4Drone->mode(DroneFlightModes::ePreflight, "OFFBOARD");
				this->_px4Drone->arm(true);
			} catch(const ServiceCallFailedException& e){
				ROS_WARN("%s", e.what());
			}
			//initialized takeoff
			//this->_px4Drone->takeoff_request();
			std::cout << _px4Drone->getState() << std::endl;
			_increment++;
			_last = ros::Time::now();
		}
		this->_px4Drone->move(_increment-1, 0, 2);
	};
	~NewTakeoffApp() override {
		delete _drone;
	}
};

Application* createApplication(int argc, char** argv){
	return new NewTakeoffApp(argc, argv);
}
