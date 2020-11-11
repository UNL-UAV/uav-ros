#include "core/Application.hpp"
#include <ros/ros.h>
void Application::run(){
	this->init();
	if(!ros::isInitialized()){
		ros::Time::init();
	}
	ros::Rate rate(20);
	ros::Time start = ros::Time::now();
	bool running = true;
	while(running){
		ros::Time end = ros::Time::now();
		ros::Duration diff = end-start;
		start = end;
		this->update(diff.toSec());
		if(ros::isInitialized()){
			running = ros::ok();
		}
		rate.sleep();
	}
}
