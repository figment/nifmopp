#pragma once

#include <exception>

struct Point3
{
	float x, y, z;
	void Set(float x, float y, float z){
		this->x = x; this->y = y; this->z = z;
	}
};

struct Triangle
{
	unsigned int a, b, c;

	unsigned int operator[](size_t i) const {
		switch (i)
		{
		case 0: return a;
		case 1: return b;
		case 2: return c;
		default: throw std::exception("Invalid index");
		}
	}
	unsigned int& operator[](size_t i) {
		switch (i)
		{
		case 0: return a;
		case 1: return b;
		case 2: return c;
		default: throw std::exception("Invalid index");
		}
	}
};

struct Matrix3
{
	float m[3][3];
};

struct Matrix43
{
	float m[4][3];
};

struct Matrix44
{
	float m[4][4];
};

extern void InitializeHavok();
