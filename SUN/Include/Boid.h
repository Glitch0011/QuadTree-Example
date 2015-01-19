#pragma once

#include <main.h>

struct Boid : IQuadtreeBoid
{
	Point3f pos;
	Vec3f velocity;
	Point3f target;

public:
	Boid(Point3f pos, Point3f target)
	{
		this->pos = pos;
		this->velocity = Vec3f(0, 0, 0);
		this->target = target;
	}

	inline Point3f getPos()
	{
		return this->pos;
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