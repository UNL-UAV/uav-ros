#pragma once
#include <pch.hpp>
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

class PX4{
	private:
		ros::NodeHandle _nh;
		
		ros::Publisher _positionPublisher;
		ros::Publisher _yawPublisher;
		ros::Publisher _activityPublisher;

		ros::Subscriber _stateSubscriber;
		
		ros::ServiceClient _armingClient;
		ros::ServiceClient _modeClient;
		
		mavros_msgs::State _state;
	public:
		PX4();
		PX4(ros::NodeHandle& nh) : _nh(nh){};
		
		void init();
		virtual void update(float delta);
		
		bool isConnected();
		bool isArmed();
		bool isOffboard();
		
		const mavros_msgs::CommandBool setArmState(bool arm);
		const mavros_msgs::SetMode setMode(const std::string& mode);
		std::string getMode();
		
		void move(float x, float y, float z, bool offset=true);
		void turn(float yaw);
		void takeoff();
		void land();
		void home();

		inline const mavros_msgs::State& getState() {
			return this->_state;
		}

		~PX4();
	private:
		geometry_msgs::PoseStamped _pose(float x, float y, float z, bool flu);
		void _action(const std::string& str);
};
