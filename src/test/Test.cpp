#pragma once
#include <pch.hpp>
#include "core/Application.hpp"
#include "core/EntryPoint.hpp"

class TestApp : public Application{
public:
	TestApp(int argc, char** argv) : Application(argc, argv){}
	void init() {
		std::cout << "Hello World " << this->_argc << " ";
		for(int i =0; i < this->_argc; i++){
			std::cout << this->_argv[i] << " ";
		}
		std::cout << std::endl;
	};
	void update(float delta){
		std::cout << "Delta: " << delta << std::endl;
	};
};

Application* createApplication(int argc, char** argv){
	return new TestApp(argc, argv);
}
