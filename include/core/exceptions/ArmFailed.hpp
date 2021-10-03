#pragma once

#include <exception>

struct ArmFailedExecption : public std::exception{
	ArmFailedExecption() = default;
	~ArmFailedExecption() = default;
	const char* what() const throw(){
		return "Arming failed!";
	};
};