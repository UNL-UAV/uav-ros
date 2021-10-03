#pragma once

#include <exception>

struct ServiceCallFailedException : public std::exception{
	std::string whereA;
	std::string whatA;

	ServiceCallFailedException() = default;
	ServiceCallFailedException(const std::string& WhereA = "Unkown", const std::string& WhatA = "") : whereA(WhereA), whatA(WhatA){};
	~ServiceCallFailedException() = default;

	const char* what() const throw(){
		return std::string("Service Called Failed: "+whereA+": what:" + whatA).c_str();
	}
};