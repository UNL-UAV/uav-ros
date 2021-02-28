#include "core/Application.hpp"
#include <ros/ros.h>
void Application::run(){
	ros::init(_argc, _argv, this->_name);
	this->_nh = new ros::NodeHandle();
	this->init();
	if(!ros::isInitialized()){
		ros::Time::init();
	}
	ros::Rate rate(25);
	ros::Time start = ros::Time::now();
	bool running = true;
	while(running){
		ros::Time end = ros::Time::now();
		ros::Duration diff = end-start;
		start = end;
		this->update(diff.toSec());
		if(ros::isInitialized()){
			running = ros::ok();
			ros::spinOnce();
		}
		rate.sleep();
	}
}

Application::~Application(){
	delete this->_nh;
}
