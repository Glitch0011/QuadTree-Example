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

static double h = 4000.0;

double W(Vec3f ij)
{
	auto q = length(ij) / h;
	if (q >= 0 || q <= 2)
		q = (2.0 / 3.0) - (9.0 / 8.0)*pow(q, 2) + (19.0 / 24.0)*pow(q, 3) - (5.0 / 32.0)*pow(q, 4);
	else
		q = 0;
	return q;
}

void updateBoids(std::vector<Boid*> boids, float timePassedInSeconds, Quadtree* quadTree)
{
	if (timePassedInSeconds < 0.001 || timePassedInSeconds > 1)
		return;

	Vec3f gravity = Vec3f(0, 1, 0);

	for (auto& boid : boids)
	{
		boid->velocity += gravity;
		boid->nextPos = boid->pos + (boid->velocity * timePassedInSeconds);
	}

	for (auto& boid : boids)
	{
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

		boid->pressure = 0;
		boid->density = 0;

		double totalDensity = 0;
		//Approximation of density
		for (auto& otherBoid : otherBoids)
		{
			if (otherBoid == boid)
				continue;

			totalDensity += (float)(otherBoid->mass / otherBoid->density) * (Vec3f)(boid->velocity - otherBoid->velocity) * (float)W(boid->pos - otherBoid->pos);
			//auto w = W(boid->pos - otherBoid->pos);
			//boid->density += otherBoid->mass * w;
		}

		boid->density *= totalDensity;
	}

	for (auto& boid : boids)
	{
		auto targetBox = AABoxf(
			Vec3f(boid->pos[0] - LINE_RANGE, boid->pos[1] - LINE_RANGE, 0),
			Vec3f(boid->pos[0] + LINE_RANGE, boid->pos[1] + LINE_RANGE, 1));

		auto otherBoids = quadTree->queryRange(targetBox);

		//Correct velocity
		Vec3f totalVelocity;
		for (auto& otherBoid : otherBoids)
		{
			if (otherBoid == boid)
				continue;

			float left = (2 * otherBoid->mass) / (boid->density + otherBoid->density);
			Vec3f middle = (boid->velocity - otherBoid->velocity);
			float q = W(boid->pos - otherBoid->pos);

			auto right = (float)(315 / 208 * PI * pow(h, 3)) * (q);
			totalVelocity += left * middle * q;
		}
		if (otherBoids.size() > 0)
		{
			totalVelocity /= otherBoids.size();
			boid->velocity += totalVelocity;
		}

		boid->nextPos = boid->pos + (boid->velocity * timePassedInSeconds);
	}

	for (auto& boid : boids)
	{
		boid->pos = boid->nextPos;
		boid->velocity *= 0.998f;
	}
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