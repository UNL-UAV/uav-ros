#include <pch.hpp>
#include "core/EntryPoint.hpp"
#include <ros/ros.h>
#include "core/PX4.hpp"

class NewTakeoffApp : public Application {
private:
	PX4* _drone;
	ros::Time _last;
	int _increment=0;
public:
	NewTakeoffApp(int argc, char** argv) : Application(argc, argv, "newTakeOff"){};
	void init() override{
		ROS_INFO("STARTED ROS");
		
		this->_drone = new PX4(*_nh);
		this->_drone->init();
		
		ros::Rate tmpRate(25);
		while(ros::ok() && this->_drone->isConnected()){ ros::spinOnce(); tmpRate.sleep(); } // wait till we're connected to mavros
		ROS_INFO("CONNECTED TO DRONE");

		for(int i=0; i<100; i++){ 
			this->_drone->move(0, 0, 2); 
			ros::spinOnce(); 
			tmpRate.sleep();
		}
		_last = ros::Time::now();
	};
	void update(float delta) override{
		this->_drone->update(delta);
		if(ros::Time::now() - _last > ros::Duration(5.0)){
			if(!this->_drone->isOffboard()){
				auto resp = this->_drone->setMode("GUIDED_NOGPS");
				if(!resp.response.mode_sent){
					ROS_WARN("Could not set mode!");
				}else{
					ROS_INFO("MODE SET");
				}
			}
			if(!this->_drone->isArmed()){
				auto resp = this->_drone->setArmState(true);
				if(!resp.response.success){
					ROS_WARN("FAILED Armming: %d", resp.response.result);
				}else{
					ROS_INFO("ARM SET");
				}
			}
			//initialized takeoff
			//this->_drone->takeoff_request();
			std::cout << _drone->getState() << std::endl;
			_increment++;
			_last = ros::Time::now();
		}
		this->_drone->move(_increment-1, 0, 2);


	};
	~NewTakeoffApp() override {
		delete _drone;
	}
};

Application* createApplication(int argc, char** argv){
	return new NewTakeoffApp(argc, argv);
}
