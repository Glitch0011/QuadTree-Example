#pragma once

#include <algorithm>

#include <gmtl/gmtl.h>
using namespace gmtl;

#include <Boid.h>

#define DIVISION_FACTOR 2.0
#define PER_QUAD_CAPACITY 32

class Quadtree
{
	//How many boids can be in a quad before it divides
	const int Capacity = PER_QUAD_CAPACITY;

public:
	AABoxd boundary;
	Quadtree* parent;

	std::vector<Boid*> points;
	std::vector<Quadtree> children;
	bool hasChildren = false;

	Quadtree(AABoxd boundary, Quadtree* parent = nullptr)
	{
		this->boundary = boundary;
		this->parent = parent;
		this->points.reserve(Capacity);
	}

	bool insert(Boid* vec)
	{
		//If it's within our boundary
		if (intersect(this->boundary, vec->pos))
		{
			//And we have the space to store it
			if (points.size() < Capacity)
			{
				//Add to our list
				points.push_back(vec);
				return true;
			}
			else
			{
				//If we haven't divided, divide
				if (!hasChildren)
					this->subdivide();

				//Find a child that'll accept it
				for (Quadtree& child : children)
					if (child.insert(vec))
						return true;
			}
		}
		return false;
	}

	void subdivide()
	{
		auto perNewPart = (this->boundary.getMax() - this->boundary.getMin()) / DIVISION_FACTOR;

		for (int x = 0; x < DIVISION_FACTOR; x++)
		{
			for (int y = 0; y < DIVISION_FACTOR; y++)
			{
				Point3d min(
					this->boundary.getMin() +
					Vec3d(
							x * perNewPart[0],
							y * perNewPart[1],
							0));

				Quadtree branch(
					AABoxd(
						min,
						min + Vec3d(
							perNewPart[0], 
							perNewPart[1],
							1)),
							this);

				this->children.push_back(branch);
			}
		}

		this->hasChildren = true;
	}

	std::vector<Boid*> queryRange(AABoxd box)
	{
		std::vector<Boid*> retPoints;
		this->queryRange(&retPoints, box);
		return retPoints;
	}

	void queryRange(std::vector<Boid*>* retPoints, AABoxd box)
	{
		//If we intersect with the box
		if (intersect(this->boundary, box))
		{
			//Add all the points in our quad
			for (Boid* point : this->points)
				if (intersect(box, point->pos))
					retPoints->push_back(point);

			//Add our children
			if (this->hasChildren)
				for (auto& child : this->children)
					child.queryRange(retPoints, box);
		}
	}

	void remove(Boid* boid)
	{
		std::vector<Boid*>::iterator position = std::find(this->points.begin(), this->points.end(), boid);
		if (position != this->points.end())
			this->points.erase(position);
	}

	Quadtree* trueParent()
	{
		return this->parent != nullptr ? this->parent->trueParent() : this;
	}

	void update()
	{
		std::vector<Boid*> toRemove;

		for (auto& boid : this->points)
			if (!intersect(this->boundary, boid->pos))		
				toRemove.push_back(boid);

		for (auto& boid : toRemove)
		{
			this->remove(boid);
			this->trueParent()->insert(boid);
		}

		for (auto& child : this->children)
			child.update();
	}

	void clear()
	{
		this->points.clear();
		this->children.clear();
		this->hasChildren = false;
	}
};