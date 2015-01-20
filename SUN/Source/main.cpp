#include <main.h>

#include <thread>
#include <string>
#include <vector>

#include <random>
#include <chrono>

#include <gmtl/gmtl.h>
using namespace gmtl;

#include <Boid.h>

#ifdef DEBUG
	#define BOID_COUNT 100
#else
	#define BOID_COUNT 1000
#endif

#define DRAW_QUADS
#define SECONDS_PER_REBUILD 0.1
#define LINE_RANGE 40.0
#define DRAW_LINES
//#define LINEAR_SEARCH
#define PI 3.14159265359

static sf::RectangleShape shape;

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

	//Draw our children
	for (Quadtree& child : branch.children)
		draw(window, child);
}

int main()
{
	//Setup window
	sf::RenderWindow* window = new sf::RenderWindow(sf::VideoMode(screenSize[0], screenSize[1]), "Framerate: 0");

	//Create random generators
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(5.0, 1.0);
	std::uniform_real_distribution<double> uniform_distribution(0, 1.0);

	//Setup boids
	std::vector<Boid*> boids;
	for (int i = 0; i < BOID_COUNT; ++i)
	{
		boids.push_back(
			new Boid(
			Point3f(
				(distribution(generator) / 10) * screenSize[0],
				(distribution(generator) / 10) * screenSize[1], 1),
			Vec3f(
				uniform_distribution(generator) * screenSize[0],
				uniform_distribution(generator) * screenSize[1], 1)));
	}

	
	std::chrono::system_clock::time_point last = std::chrono::system_clock::now();
	unsigned int frameRate = 0;
	double nextFrameRate = 1.0;
	double mouseDownLast = 0.5;
	double lastRebuild = SECONDS_PER_REBUILD;
	bool mouseDown = false;

	Quadtree quadTree(AABoxf(Vec3f(0, 0, 0), Vec3f(screenSize[0], screenSize[1], 1)));
	for (Boid* _b : boids)
		quadTree.insert(_b);

	while (window->isOpen())
	{
		//Frame-rate control
		auto timePassedInSeconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::system_clock::now() - last).count() * 1.0e-9;
		last = std::chrono::system_clock::now();

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

					b->target = Vec3f(target[0], target[1], 1);
				}
			}
		}

		//If the mouse is down, tell the boids to go to that position
		if (mouseDown)
		{
			if (mouseDownLast <= 0)
			{
				auto mouse = sf::Mouse::getPosition(*window);
				auto mouse_v = Vec3f(mouse.x, mouse.y, 1);
				for (auto b : boids)
					b->target = mouse_v;
				mouseDownLast = 0.5f;
			}
			else
				mouseDownLast -= timePassedInSeconds;
		}

		//Clear the screen
		window->clear();

		//Rebuild the quad-tree every SECONDS_PER_REBUILD
		if (lastRebuild <= 0)
		{
			quadTree.clear();

			for (Boid* _b : boids)
				quadTree.insert(_b);

			lastRebuild = SECONDS_PER_REBUILD;
		}
		else
		{
			lastRebuild -= timePassedInSeconds;
		}

		//Draw the quad-tree using recursion
#ifdef DRAW_QUADS
		draw(window, quadTree);
#endif

		//Draw boids
		shape.setOutlineThickness(1);
		shape.setSize(sf::Vector2f(2, 2));
		shape.setOutlineColor(sf::Color::White);
		shape.setFillColor(sf::Color::Transparent);

#ifdef DRAW_LINES
		std::vector<sf::Vertex> verticies;
		sf::Vertex v;
#endif

		for (Boid* child : boids)
		{
			shape.setPosition(sf::Vector2f(child->getPos()[0], child->getPos()[1]));

			auto targetBox = AABoxf(
				Vec3f(child->getPos()[0] - LINE_RANGE, child->getPos()[1] - LINE_RANGE, 0),
				Vec3f(child->getPos()[0] + LINE_RANGE, child->getPos()[1] + LINE_RANGE, 1));

#ifdef DRAW_LINES
			//Find all boids nearby the first boid
			std::vector<IQuadtreeBoid*> nearbyBoids;

	#ifdef LINEAR_SEARCH
			for (Boid* nearbyBoid : boids)
				if (nearbyBoid != child)
					if (intersect(targetBox, nearbyBoid->getPos()))
						nearbyBoids.push_back(nearbyBoid);
	#else
			nearbyBoids = quadTree.simpleQueryRange(targetBox);
	#endif

			//Add lines to every nearby boid
			for (auto nearbyBoid : nearbyBoids)
			{
				auto pos = child->getPos();
				v.position = sf::Vector2f(pos[0], pos[1]);
				v.color = sf::Color::White;
				v.color.a = 25.50f;
				verticies.push_back(v);

				pos = nearbyBoid->getPos();
				v.position = sf::Vector2f(pos[0], pos[1]);
				verticies.push_back(v);
			}
#endif

			window->draw(shape);
		}

#ifdef DRAW_LINES
		window->draw(verticies.data(), verticies.size(), sf::PrimitiveType::Lines);
#endif

		//Update boids
		for (auto b : boids)
			b->Update(timePassedInSeconds);
		
		//Frame-rate controls
		if (nextFrameRate <= 0)
		{
			nextFrameRate = 1.0;

			window->setTitle(
				sf::String(("Framerate: " + std::to_string(frameRate))
				.c_str()));

			frameRate = 0;
		}
		else
		{
			nextFrameRate -= timePassedInSeconds;
			frameRate++;
		}
		
		window->display();
	}

	for (auto boid : boids)
		delete boid;

	return 0;
}