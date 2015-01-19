#pragma once

#include <gmtl/gmtl.h>
using namespace gmtl;

#define DIVISION_FACTOR 2.0f
#define PER_QUAD_CAPACITY 32

class IQuadtreeBoid
{
public:
	virtual ~IQuadtreeBoid() { }
	virtual Point3f getPos() = 0;
};


class Quadtree
{
	//How many boids can be in a quad before it divides
	const int Capacity = PER_QUAD_CAPACITY;

public:
	AABoxf boundary;

	std::vector<IQuadtreeBoid*> points;
	std::vector<Quadtree> children;
	bool hasChildren = false;

	Quadtree(AABoxf boundary)
	{
		this->boundary = boundary;
	}

	bool insert(IQuadtreeBoid* vec)
	{
		//If it's within our boundary
		if (intersect(this->boundary, vec->getPos()))
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
				Point3f min(
					this->boundary.getMin() +
						Vec3f(
							x * perNewPart[0],
							y * perNewPart[1],
							0));

				Quadtree branch(
					AABoxf(
						min,
						min + Vec3f(
							perNewPart[0], 
							perNewPart[1],
							1)));

				this->children.push_back(branch);
			}
		}

		this->hasChildren = true;
	}

	std::vector<IQuadtreeBoid*> queryRange(AABoxf box)
	{
		std::vector<IQuadtreeBoid*> retPoints;
		this->queryRange(&retPoints, box);
		return retPoints;
	}

	void queryRange(std::vector<IQuadtreeBoid*>* retPoints, AABoxf box)
	{
		//If we intersect with the box
		if (intersect(this->boundary, box))
		{
			//Add all the points in our quad
			for (IQuadtreeBoid* point : this->points)
				if (intersect(box, point->getPos()))
					retPoints->push_back(point);

			//Add our children
			if (this->hasChildren)
				for (auto& child : this->children)
					child.queryRange(retPoints, box);
		}
	}

	void clear()
	{
		this->points.clear();
		this->children.clear();
		this->hasChildren = false;
	}
};