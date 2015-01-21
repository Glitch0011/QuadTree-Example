#include <Boid.h>

#include <Quadtree.h>

void Boid::Update(float time)
{
	auto LINE_RANGE = 50;

	auto targetBox = AABoxf(
		Vec3f(this->pos[0] - LINE_RANGE, this->pos[1] - LINE_RANGE, 0),
		Vec3f(this->pos[0] + LINE_RANGE, this->pos[1] + LINE_RANGE, 1));

	auto boids = this->quadTree->queryRange(targetBox);

	Vec3f force = Vec3f(0, 0.1, 0);

	for (auto boid : boids)
	{
		force += this->pos - boid->pos;
	}

	normalize(force);

	//Smooth moving
	this->velocity += force;
	this->velocity *= 0.998f;

	//Bounce off walls
	Point3f nextPos = this->pos + (this->velocity * time * 2.0f);
	if (!intersect(AABoxf(Point3f(0, 0, 0), Point3f(screenSize[0], screenSize[1], 1)), nextPos))
		this->velocity *= -1;
	else
		this->pos = nextPos;
}