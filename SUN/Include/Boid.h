#pragma once

#include <ScreenSize.h>

struct Boid
{
public:
	Point3f target;
	Point3f pos;
	Vec3f velocity;

	Boid(Point3f pos, Point3f target)
	{
		this->pos = pos;
		this->velocity = Vec3f(0, 0, 0);
		this->target = target;
	}

	inline void Update(float time)
	{
		//Vector to target
		Vec3f targetForce = (this->target - this->pos);
		normalize(targetForce);

		//Smooth moving
		this->velocity += targetForce;
		this->velocity *= 0.998f;

		//Bounce off walls
		Point3f nextPos = this->pos + (this->velocity * time);
		if (!intersect(AABoxf(Point3f(0, 0, 0), Point3f(screenSize[0], screenSize[1], 1)), nextPos))
			this->velocity *= -1;
		else
			this->pos = nextPos;
	}
};