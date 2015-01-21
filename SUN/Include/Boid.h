#pragma once

#include <ScreenSize.h>

class Quadtree;

struct Boid
{
public:
	Point3f target;
	Point3f pos;
	Vec3f velocity;
	Quadtree* quadTree;

	Boid(Point3f pos, Point3f target, Quadtree* quadTree)
	{
		this->pos = pos;
		this->velocity = Vec3f(0, 0, 0);
		this->target = target;
		this->quadTree = quadTree;
	}

	void Update(float time);
};