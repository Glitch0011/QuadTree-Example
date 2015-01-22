#pragma once

#include <ScreenSize.h>

class Quadtree;

struct Boid
{
public:
	double pressure;
	double density;
	double mass = 0.02;

	Vec3d accel;
	Vec3d normal;
	Point3d pos;
	Vec3d velocity;
	Vec3d nextPos;

	Boid(Point3d pos)
	{
		this->pos = pos;
		this->velocity = Vec3d(0, 0, 0);
		this->density = 0;
		this->pressure = 0;
		
	}

	void Update(float time);
};