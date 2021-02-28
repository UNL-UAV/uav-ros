#pragma once
#include <pch.hpp>
#include <ros/ros.h>
class Application{
	protected:
		int _argc;
		char** _argv;
		std::string _name;
		ros::NodeHandle* _nh;
	public:
		Application(){};
		Application(int& argc, char**& argv, const std::string& name = "Application") : _name(name), _argc(argc), _argv(argv){};
		virtual void run();
		virtual void init()=0;
		virtual void update(float delta) =0;
		virtual ~Application();
};
