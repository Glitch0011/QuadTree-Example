#pragma once

#include <ScreenSize.h>

class Quadtree;

struct Boid
{
public:
	double pressure;
	double density;
	double mass = 1.0;

	Vec3f accel;
	Vec3f normal;
	Point3f pos;
	Vec3f velocity;
	Vec3f nextPos;

	Boid(Point3f pos)
	{
		this->pos = pos;
		this->velocity = Vec3f(0, 0, 0);
		this->density = 0;
		this->pressure = 0;
		
	}

	void Update(float time);
};