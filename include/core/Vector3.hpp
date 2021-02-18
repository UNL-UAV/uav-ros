#pragma once
#include <pch.hpp>
struct Vector3{
	float x, y, z;

	Vector3(){};
	Vector3(float a, float b, float c) : x(a), y(b), z(c){};
	
	friend std::ostream& operator << (std::ostream& os, const Vector3& v){
		os << "{x: "<< v.x << ",y: " << v.y << ", z: "<< v.z<<"}\n";
		return os;
	};
};
