#include <main.h>

#include <thread>
#include <string>
#include <vector>

#include <random>
#include <chrono>
#include <mutex>

#include <gmtl/gmtl.h>
using namespace gmtl;

#include <conio.h>

#include <Boid.h>

#ifdef DEBUG
	#define BOID_COUNT 100
#else
	#define BOID_COUNT 500
#endif

#define UPDATE_FRAMERATE 10
#define RENDER_FRAMERATE 30
#define BOID_UPDATE_FRAMERATE 140
#define OUTPUT_CONSOLE
#define DRAW_QUADS
#define DRAW_RECTS
#define SECONDS_PER_MOUSE_UPDATE 0.25
#define SECONDS_PER_REBUILD 0.1
#define LINE_RANGE 30.0
//#define DRAW_LINES
//#define LINEAR_SEARCH
#define PI 3.14159265359
#define UPDATE_BOIDS

static sf::RectangleShape shape;
static sf::CircleShape circleShape;

std::mutex quadTreeMutexLock;

void draw(sf::RenderWindow* window, Quadtree branch)
{
	//Draw ourselves
	if (branch.children.size() == 0)
	{
		shape.setOutlineColor(sf::Color::White);
		shape.setOutlineThickness(0.5);
		shape.setFillColor(sf::Color::Transparent);

		auto a = branch.boundary.getMin();
		auto b = branch.boundary.getMax() - a;

		shape.setPosition(sf::Vector2f(a[0], a[1]));
		shape.setSize(sf::Vector2f(b[0], b[1]));

		window->draw(shape);
	}
	else
	{
		//Draw our children
		for (Quadtree& child : branch.children)
			draw(window, child);
	}
}

#include <Windows.h>

class FrameLimiter
{
private:
	std::chrono::system_clock::time_point last;
	double desiredFrameRate = 0;
	std::chrono::duration<double> timePerFrame;

	unsigned int frameRate = 0;
	double nextFrameRate = 1.0;

	bool output = false;
	
public:
	FrameLimiter(double _frameRate = 60.0, bool output = false)
	{
		this->desiredFrameRate = _frameRate;
		timePerFrame = std::chrono::duration<double>(1.0 / desiredFrameRate);
		this->output = output;
	}

	double Start()
	{
		auto timePassedInSeconds = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - last).count() * 1.0e-9;
		last = std::chrono::system_clock::now();

		if (nextFrameRate <= 0)
		{
			nextFrameRate = 1.0;

#ifdef OUTPUT_CONSOLE
			if (output)
			std::cout << ("Framerate: " + std::to_string(frameRate) + "\r\n").c_str();
#endif

			frameRate = 0;
		}
		else
		{
			nextFrameRate -= timePassedInSeconds;
			frameRate++;
		}

		return timePassedInSeconds;
	}

	void End()
	{
		std::chrono::duration<double> timePassed = std::chrono::system_clock::now() - last;
		auto timeToSleep = std::chrono::duration_cast<std::chrono::nanoseconds>(this->timePerFrame - timePassed).count();

		std::this_thread::sleep_for(std::chrono::nanoseconds(timeToSleep));
	}
};

static double h = 5.0;
#define GAS_STIFFNESS 3.0 //20.0 // 461.5  // Nm/kg is gas constant of water vapor
#define REST_DENSITY 998.29 // kg/m^3 is rest density of water particle
#define PARTICLE_MASS 0.02 // kg
#define VISCOSITY 3.5 // 5.0 // 0.00089 // Ns/m^2 or Pa*s viscosity of water
#define SURFACE_TENSION 0.0728 // N/m 
#define SURFACE_THRESHOLD 7.065
#define KERNEL_PARTICLES 20.0
#define GRAVITY_ACCELERATION 9.80665

#define WALL_K 10 //10000.0 // wall spring constant
#define WALL_DAMPING -0.9 // wall damping constant
#define BOX_SIZE 0.4

double W(Vec3f ij)
{
	auto q = length(ij) / h;
	if (q >= 0 || q <= 2)
		q = (2.0 / 3.0) - (9.0 / 8.0)*pow(q, 2) + (19.0 / 24.0)*pow(q, 3) - (5.0 / 32.0)*pow(q, 4);
	else
		q = 0;
	return q;
}

double Wpoly6(double radiusSquared)
{
	static double coefficient = 315.0 / (64.0*PI*pow(h, 9));
	static double hSquared = h*h;

	return coefficient * pow(hSquared - radiusSquared, 3);
}

void Wpoly6Gradient(Vec3f& diffPosition, double radiusSquared, Vec3f& gradient)
{
	static double coefficient = -945.0 / (32.0*PI*pow(h, 9));
	static double hSquared = h*h;

	gradient = (float)(coefficient * pow(hSquared - radiusSquared, 2)) * diffPosition;
}

double Wpoly6Laplacian(double radiusSquared)
{

	static double coefficient = -945.0 / (32.0*PI*pow(h, 9));
	static double hSquared = h*h;

	return coefficient * (hSquared - radiusSquared) * (3.0*hSquared - 7.0*radiusSquared);
}

void WspikyGradient(Vec3f& diffPosition, double radiusSquared, Vec3f& gradient)
{

	static double coefficient = -45.0 / (PI*pow(h, 6));

	double radius = sqrt(radiusSquared);

	gradient = (float)(coefficient * pow(h - radius, 2)) * diffPosition / (float)radius;
}


double WviscosityLaplacian(double radiusSquared)
{
	static double coefficient = 45.0 / (PI*pow(h, 6));

	double radius = sqrt(radiusSquared);

	return coefficient * (h - radius);
}

void collisionForce(Boid* boid, Vec3f f_collision)
{
	struct WALL
	{
	public:
		Vec3f point;
		Vec3f normal;
		WALL(Vec3f normal, Vec3f position){
			this->point = position; this->normal = normal;
		}
	};

	std::vector<WALL> _walls;
	/*_walls.push_back(WALL(Vec3f(0, 0, 1), Vec3f(0, 0, 0)));
	_walls.push_back(WALL(Vec3f(0, 0, -1), Vec3f(0, 0, 0)));*/
	_walls.push_back(WALL(Vec3f(1, 0, 0), Vec3f(0, 0, 0)));     // left
	_walls.push_back(WALL(Vec3f(-1, 0, 0), Vec3f(screenSize[0], 0, 0)));     // right
	_walls.push_back(WALL(Vec3f(0, -1, 0), Vec3f(0, screenSize[1] - 300, 0))); // bottom

	for (auto& wall : _walls)
	{
		float d = dot(wall.point - boid->pos, wall.normal) + 0.01; // particle radius

		if (d > 0.0)
		{
			boid->accel += (float)WALL_K * wall.normal * d;
			boid->accel += (float)WALL_DAMPING * dot(boid->velocity, wall.normal) * wall.normal;
		}
	}
}

void updateAccel(std::vector<Boid*>& boids, float timePassedInSeconds, Quadtree* quadTree)
{
	//Update density and pressure
	for (auto& boid : boids)
	{
		boid->density = 0;

		for (auto& otherBoid : boids)
		{
			Vec3f diffPos = boid->pos - otherBoid->pos;
			auto radSqr = lengthSquared(diffPos);
			if (radSqr < h*h)
				boid->density += Wpoly6(radSqr);
		}

		boid->density *= boid->mass;

		boid->pressure = GAS_STIFFNESS * (boid->density - REST_DENSITY);
	}

	for (auto& boid : boids)
	{
		Vec3f f_pressure, f_viscosity, f_surface, f_gravity(0.0, boid->density*GRAVITY_ACCELERATION, 0.0), n, colorFieldNormal;
		double colorFieldLaplacian = 0;

		for (auto& otherBoid : boids)
		{
			Vec3f diffPos = boid->pos - otherBoid->pos;
			double radiusSquared = lengthSquared(diffPos);

			if (radiusSquared <= h*h)
			{
				if (radiusSquared > 0.0)
				{
					Vec3f gradient;
					Wpoly6Gradient(diffPos, radiusSquared, gradient);

					float a = boid->pressure + otherBoid->pressure;
					float b = 2.0 * otherBoid->density;
					f_pressure += a / b * gradient;

					colorFieldNormal += gradient / (float)otherBoid->density;
				}

				f_viscosity += (otherBoid->velocity - boid->velocity) * (float)WviscosityLaplacian(radiusSquared) / (float)otherBoid->density;

				colorFieldLaplacian += Wpoly6Laplacian(radiusSquared) / (float)otherBoid->density;
			}
		}

		f_pressure *= -boid->mass;

		f_viscosity *= VISCOSITY * boid->mass;

		colorFieldNormal *= boid->mass;

		boid->normal = -1.0f * colorFieldNormal;

		colorFieldLaplacian *= boid->mass;

		//Surface tension
		double colorFieldNormalMagnitude = length(colorFieldNormal);
		if (colorFieldNormalMagnitude != 0)
		{
			if (colorFieldNormalMagnitude < SURFACE_THRESHOLD)
			{
				f_surface = (float)-SURFACE_TENSION * (float)colorFieldLaplacian * colorFieldNormal / (float)colorFieldNormalMagnitude;
			}
		}

		boid->accel = (f_pressure + f_viscosity + f_surface + f_gravity) / (float)(boid->density == 0 ? 1 : boid->density);

		Vec3f f_collision;
		collisionForce(boid, f_collision);
	}
}

void updateBoids(std::vector<Boid*>& boids, float timePassedInSeconds, Quadtree* quadTree)
{
	if (timePassedInSeconds < 0.001 || timePassedInSeconds > 1)
		return;

	updateAccel(boids, timePassedInSeconds, quadTree);

	Vec3f gravity = Vec3f(0, 1, 0);

	for (auto& boid : boids)
	{
		auto newPos = boid->pos + boid->velocity * timePassedInSeconds + boid->accel*timePassedInSeconds*timePassedInSeconds;
		auto newVel = (newPos - boid->pos) / timePassedInSeconds;

		boid->pos = newPos;
		boid->velocity = newVel;

		//boid->velocity += gravity;
		//boid->nextPos = boid->pos + (boid->velocity * timePassedInSeconds);
	}

	/*for (auto& boid : boids)
	{
		//Collisions
		auto targetBox = AABoxf(
			Vec3f(boid->pos[0] - LINE_RANGE, boid->pos[1] - LINE_RANGE, 0),
			Vec3f(boid->pos[0] + LINE_RANGE, boid->pos[1] + LINE_RANGE, 1));

		auto otherBoids = quadTree->queryRange(targetBox);

		for (auto& otherBoid : otherBoids)
		{
			if (otherBoid == boid)
				continue;

			auto l = length((Vec3f)(boid->nextPos - otherBoid->nextPos));
			if (l < 20)
			{
				Vec3f vector = (boid->nextPos - otherBoid->nextPos);
				normalize(vector);
				boid->velocity += vector * l*2.0f;
			}

		}

		//Work out floor collisions
		auto floorDiff = (screenSize[1] - 50) - boid->nextPos[1];
		if (floorDiff < 0)
			boid->velocity += Vec3f(0, floorDiff, 0);

		floorDiff = boid->nextPos[0] - 50;
		if (floorDiff < 0)
			boid->velocity += Vec3f(-floorDiff, 0, 0);

		floorDiff = (screenSize[0] - 50) - boid->nextPos[0];
		if (floorDiff < 0)
			boid->velocity += Vec3f(floorDiff, 0, 0);
	}

	for (auto& boid : boids)
	{
		boid->pos = boid->nextPos;
		boid->velocity *= 0.998f;
	}*/
}

int main()
{
	//Setup window
	sf::RenderWindow* window = new sf::RenderWindow(sf::VideoMode(screenSize[0], screenSize[1]), "Framerate: 0");

	//Create random generators
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(5.0, 1.0);
	std::uniform_real_distribution<double> uniform_distribution(0, 1.0);

	Quadtree quadTree(AABoxf(Vec3f(0, 0, 0), Vec3f(screenSize[0], screenSize[1], 1)));

	//Setup boids
	std::vector<Boid*> boids;
	for (auto x = 0; x < sqrt(BOID_COUNT); x++)
	{
		for (auto y = 0; y < sqrt(BOID_COUNT); y++)
		{
			boids.push_back(new Boid(Point3f(50,50, 0) + Point3f(x * 20, y * 20, 1)));
		}
	}

	bool mouseDown = false;

	for (Boid* _b : boids)
		quadTree.insert(_b);

	auto boidUpdateThread = std::thread([&]
	{
		FrameLimiter boidUpdateLimiter(BOID_UPDATE_FRAMERATE, true);
		while (window->isOpen())
		{
			auto timePassedInSeconds = boidUpdateLimiter.Start();

#ifdef UPDATE_BOIDS
			quadTreeMutexLock.lock();
			{
				updateBoids(boids, timePassedInSeconds, &quadTree);
			}
			quadTreeMutexLock.unlock();

#endif

			boidUpdateLimiter.End();
		}
	});

	auto quadRebuilderThread = std::thread([&]
	{
		FrameLimiter updateLimiter(UPDATE_FRAMERATE);
		double mouseDownLast = 0.5;
		double lastRebuild = SECONDS_PER_REBUILD;
		
		while (window->isOpen())
		{
			//Frame-rate control
			auto timePassedInSeconds = updateLimiter.Start();

			//If the mouse is down, tell the boids to go to that position
			if (mouseDown)
			{
				if (mouseDownLast <= 0)
				{
					auto mouse = sf::Mouse::getPosition(*window);
					/*for (auto b : boids)
						b->target = Vec3f(mouse.x, mouse.y, 1);*/
					mouseDownLast = SECONDS_PER_MOUSE_UPDATE;
				}
				else
					mouseDownLast -= timePassedInSeconds;
			}

			//Rebuild the quad-tree every SECONDS_PER_REBUILD
			if (lastRebuild <= 0)
			{
				quadTreeMutexLock.lock();
				{
					//quadTree.update();

					quadTree.clear();

					for (Boid* _b : boids)
						quadTree.insert(_b);
				}
				quadTreeMutexLock.unlock();

				lastRebuild = SECONDS_PER_REBUILD;
			}
			else
			{
				lastRebuild -= timePassedInSeconds;
			}

			updateLimiter.End();
		}
		return 0;
	});

	FrameLimiter renderLimiter(RENDER_FRAMERATE);

	while (window->isOpen())
	{
		renderLimiter.Start();

		//Process events
		sf::Event event;
		while (window->pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window->close();
			else if (event.type == sf::Event::MouseButtonPressed)
			{
				mouseDown = true;
			}
			else if (event.type == sf::Event::MouseButtonReleased)
			{
				mouseDown = false;
				for (auto b : boids)
				{
					auto angle = uniform_distribution(generator) * 2.0f * PI;
					auto range = (float)(200 + (uniform_distribution(generator) * 1));
					Vec2f target = Vec2f(screenSize[0] / 2.0f, screenSize[1] / 2.0f) + (Vec2f(sin(angle), cos(angle)) * range);

					//b->target = Vec3f(target[0], target[1], 1);
				}

				quadTreeMutexLock.lock();
				auto mousePos = sf::Mouse::getPosition(*window);
				boids.push_back(new Boid(Point3f(mousePos.x, mousePos.y, 1)));
				quadTreeMutexLock.unlock();
			}
		}
		
		//Clear the screen
		window->clear();

		//Draw the quad-tree using recursion
#ifdef DRAW_QUADS
		quadTreeMutexLock.lock();
		draw(window, quadTree);
		quadTreeMutexLock.unlock();
#endif

		//Draw boids
		circleShape.setOutlineThickness(1);
		circleShape.setRadius(10.0f);
		circleShape.setOutlineColor(sf::Color::White);
		circleShape.setFillColor(sf::Color::Transparent);

#ifdef DRAW_LINES
		std::vector<sf::Vertex> verticies;
		sf::Vertex v;
#endif

		quadTreeMutexLock.lock();
		for (Boid* boid : boids)
		{
			circleShape.setPosition(sf::Vector2f(boid->pos[0], boid->pos[1]));

			auto targetBox = AABoxf(
				Vec3f(boid->pos[0] - LINE_RANGE, boid->pos[1] - LINE_RANGE, 0),
				Vec3f(boid->pos[0] + LINE_RANGE, boid->pos[1] + LINE_RANGE, 1));

#ifdef DRAW_LINES
			//Find all boids nearby the first boid
			std::vector<Boid*> nearbyBoids;

	#ifdef LINEAR_SEARCH
			for (Boid* nearbyBoid : boids)
				if (nearbyBoid != child)
					if (intersect(targetBox, nearbyBoid->pos))
						nearbyBoids.push_back(nearbyBoid);
	#else
			nearbyBoids = quadTree.queryRange(targetBox);
	#endif

			//Add lines to every nearby boid
			v.color = sf::Color::White;
			v.color.a = 25.50f;
			for (auto nearbyBoid : nearbyBoids)
			{
				v.position = sf::Vector2f(boid->pos[0], boid->pos[1]);
				verticies.push_back(v);

				v.position = sf::Vector2f(nearbyBoid->pos[0], nearbyBoid->pos[1]);
				verticies.push_back(v);
			}
#endif

#ifdef DRAW_RECTS
			window->draw(circleShape);
#endif
		}
		quadTreeMutexLock.unlock();

#ifdef DRAW_LINES
		window->draw(verticies.data(), verticies.size(), sf::PrimitiveType::Lines);
#endif

		window->display();

		renderLimiter.End();
	}

	quadRebuilderThread.join();
	boidUpdateThread.join();

	for (auto boid : boids)
		delete boid;

	return 0;
}