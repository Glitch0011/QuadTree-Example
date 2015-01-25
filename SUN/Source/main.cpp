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
	#define BOID_COUNT 10
#else
	#define BOID_COUNT 500
#endif

#define UPDATE_FRAMERATE 60
#define RENDER_FRAMERATE 30
#define BOID_UPDATE_FRAMERATE 1000
#define OUTPUT_CONSOLE
#define DRAW_QUADS
#define DRAW_RECTS
#define SECONDS_PER_MOUSE_UPDATE 0.01
#define SECONDS_PER_REBUILD 0.1
#define LINE_RANGE 30.0
//#define RENDER_REAL
//#define DRAW_LINES
//#define LINEAR_SEARCH
#define PI 3.14159265359
#define UPDATE_BOIDS

#ifdef RENDER_REAL
	#undef DRAW_QUADS
#endif

static sf::RectangleShape shape;
static sf::CircleShape circleShape;

std::mutex quadTreeMutexLock;

void draw(sf::RenderWindow* window, Quadtree branch)
{
	//Draw ourselves
	if (branch.children.size() == 0)
	{
		shape.setOutlineColor(sf::Color::White);
		shape.setOutlineThickness(1.0);
		shape.setFillColor(sf::Color::Transparent);

		auto a = branch.boundary.getMin();
		auto b = branch.boundary.getMax() - a;

		shape.setPosition(sf::Vector2f(
			(a[0] + 1.0) * (screenSize[0] / 2.0),
			(a[1] + 1.0) * (screenSize[1] / 2.0)));

		auto c = sf::Vector2f(
			b[0] * (screenSize[0] / 2),
			b[1] * (screenSize[1] / 2));

		shape.setSize(c);

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

//
static double h = 0.035;//0.0457;

//
#define GAS_STIFFNESS 3.0//3.0 //20.0 // 461.5  // Nm/kg is gas constant of water vapor

//Lower = less compression
#define REST_DENSITY 998.29 //998.29 // kg/m^3 is rest density of water particle

#define PARTICLE_MASS 0.02 // kg

//Higher = gloopier
#define VISCOSITY 3.5 // 5.0 // 0.00089 // Ns/m^2 or Pa*s viscosity of water


#define SURFACE_TENSION 0.0728 // N/m 


#define SURFACE_THRESHOLD 0.01//7.065

#define KERNEL_PARTICLES 20.0

#define GRAVITY_ACCELERATION 9.80665

#define WALL_K 10000.0 //10000.0 // wall spring constant
#define WALL_DAMPING 30//-0.9 // wall damping constant
#define BOX_SIZE 0.4

#define PADDING 0.1

static Vec3d gravity = Vec3d(0, GRAVITY_ACCELERATION, 0);

double W(Vec3d ij)
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

void Wpoly6Gradient(Vec3d& diffPosition, double radiusSquared, Vec3d& gradient)
{
	static double coefficient = -945.0 / (32.0*PI*pow(h, 9));
	static double hSquared = h*h;

	gradient = (coefficient * pow(hSquared - radiusSquared, 2)) * diffPosition;
}

double Wpoly6Laplacian(double radiusSquared)
{

	static double coefficient = -945.0 / (32.0*PI*pow(h, 9));
	static double hSquared = h*h;

	return coefficient * (hSquared - radiusSquared) * (3.0*hSquared - 7.0*radiusSquared);
}

void WspikyGradient(Vec3d& diffPosition, double radiusSquared, Vec3d& gradient)
{
	static double coefficient = -45.0 / (PI*pow(h, 6));

	double radius = sqrt(radiusSquared);

	gradient = (coefficient * pow(h - radius, 2)) * diffPosition / radius;
}

double WviscosityLaplacian(double radiusSquared)
{
	static double coefficient = 45.0 / (PI*pow(h, 6));

	double radius = sqrt(radiusSquared);

	return coefficient * (h - radius);
}

void collisionForce(Boid* boid, Vec3d f_collision)
{
	struct WALL
	{
	public:
		Vec3d point;
		Vec3d normal;
		WALL(Vec3d normal, Vec3d position){
			this->point = position; this->normal = normal;
		}
	};

	std::vector<WALL> _walls;
	/*_walls.push_back(WALL(Vec3f(0, 0, 1), Vec3f(0, 0, 0)));
	_walls.push_back(WALL(Vec3f(0, 0, -1), Vec3f(0, 0, 0)));*/
	_walls.push_back(WALL(Vec3d(1, 0, 0), Vec3d(-(boundarySize[0]), 0, 0)));    // left
	_walls.push_back(WALL(Vec3d(-1, 0, 0), Vec3d((boundarySize[0]), 0, 0)));     // right
	_walls.push_back(WALL(Vec3d(0, -1, 0), Vec3d(0, (boundarySize[1]), 0))); // bottom
	_walls.push_back(WALL(Vec3d(0, 1, 0), Vec3d(0, -(boundarySize[1]), 0))); // bottom

	for (auto& wall : _walls)
	{
		double d = dot(wall.point - boid->pos, wall.normal) + 0.01; // particle radius

		if (d > 0.0)
		{
			//boid->pos += d * wall.normal;
			//boid->velocity -= dot(boid->velocity, wall.normal) * 1.9 * wall.normal;

			boid->accel += WALL_K * wall.normal * d;
			boid->accel -= WALL_DAMPING * dot(boid->velocity, wall.normal) * wall.normal;
		}
	}
}

void updateAccel(std::vector<Boid*>& boids, double timePassedInSeconds, Quadtree* quadTree)
{
	auto hh = h * 6;

	//Update density and pressure
	for (auto& boid : boids)
	{
		boid->density = 0;

		for (auto& otherBoid : boids)
		{
			auto radSqr = lengthSquared((Vec3d)(boid->pos - otherBoid->pos));
			if (radSqr <= h * h)
				boid->density += Wpoly6(radSqr);
		}

		boid->density *= boid->mass;

		boid->pressure = GAS_STIFFNESS * (boid->density - REST_DENSITY);
	}

	for (auto& boid : boids)
	{
		Vec3d f_pressure, f_viscosity, f_surface, f_gravity(gravity[0] * boid->density, gravity[1] * boid->density, gravity[2] * boid->density), n, colorFieldNormal;
		double colorFieldLaplacian = 0;

		for (auto& otherBoid : boids)
		{
			Vec3d diffPosition = boid->pos - otherBoid->pos;
			double radiusSquared = lengthSquared(diffPosition);

			if (radiusSquared <= h*h)
			{
				Vec3d poly6Gradient, spikyGradient;

				Wpoly6Gradient(diffPosition, radiusSquared, poly6Gradient);

				WspikyGradient(diffPosition, radiusSquared, spikyGradient);

				if (boid != otherBoid)
				{
					f_pressure += (boid->pressure / pow(boid->density, 2) + otherBoid->pressure / pow(otherBoid->density, 2)) * spikyGradient;
					f_viscosity += (otherBoid->velocity - boid->velocity) * WviscosityLaplacian(radiusSquared) / otherBoid->density;
				}

				colorFieldNormal += poly6Gradient / otherBoid->density;

				colorFieldLaplacian += Wpoly6Laplacian(radiusSquared) / otherBoid->density;
			}
		}

		f_pressure *= -boid->mass * boid->density;

		f_viscosity *= VISCOSITY * boid->mass;

		colorFieldNormal *= boid->mass;

		boid->normal = -1.0 * colorFieldNormal;

		colorFieldLaplacian *= boid->mass;

		//Surface tension
		double colorFieldNormalMagnitude = length(colorFieldNormal);
		
		if (colorFieldNormalMagnitude > SURFACE_THRESHOLD)
		{
			f_surface = -SURFACE_TENSION * colorFieldNormal / colorFieldNormalMagnitude * colorFieldLaplacian;
		}

		boid->accel = (f_pressure + f_viscosity + f_surface + f_gravity) / boid->density;

		Vec3d f_collision;
		collisionForce(boid, f_collision);
	}
}

static std::vector<double> timeAverage;
static std::default_random_engine generator;
static std::uniform_real_distribution<double> uniform_distribution(-0.2, 0.2);

void updateBoids(std::vector<Boid*>& boids, float _timePassedInSeconds, Quadtree* quadTree)
{
	quadTree->clear();

	auto timePassedInSeconds = 0.0;

	for (Boid* _b : boids)
		quadTree->insert(_b);

	_timePassedInSeconds *= 0.75;

	if (_timePassedInSeconds >= 1)
		return;

	if (timeAverage.size() == 0)
		timeAverage.insert(timeAverage.begin(), _timePassedInSeconds);

	for (auto time : timeAverage)
		timePassedInSeconds += time;

	timePassedInSeconds /= timeAverage.size();

	if (_timePassedInSeconds < timePassedInSeconds * 5)
	{
		timeAverage.insert(timeAverage.begin(), _timePassedInSeconds);
	}
	if (timeAverage.size() > 50)
		timeAverage.pop_back();

	//timePassedInSeconds = 1.0 / 100.0;

	updateAccel(boids, timePassedInSeconds, quadTree);

	Vec3d gravity = Vec3d(0, 1, 0);

	for (auto& boid : boids)
	{
		Vec3d newPos = boid->pos + (boid->velocity * timePassedInSeconds) + (boid->accel*timePassedInSeconds*timePassedInSeconds);

		Vec3d newVel = (newPos - boid->pos) / timePassedInSeconds;

		if (!intersect(quadTree->boundary, (Point3d)newPos) || !(newPos[0] < 0 || newPos[0] > 0 || newPos[0] == 0))
		{
			newPos = Vec3d(uniform_distribution(generator), uniform_distribution(generator), 1);
			newVel = Vec3d(0, 0, 0);
		}

		boid->pos = newPos;
		boid->velocity = newVel;
	}
}

int main()
{
	//Setup window
	sf::RenderWindow* window = new sf::RenderWindow(sf::VideoMode(screenSize[0], screenSize[1]), "Framerate: 0");

	//Create random generators
	std::normal_distribution<double> distribution(5.0, 1.0);

	Quadtree quadTree(AABoxd(Vec3d(quadPos[0] - boundarySize[0] - PADDING, quadPos[0] - boundarySize[0] - PADDING, 0), Vec3d(quadPos[0] + boundarySize[0] + PADDING, quadPos[1] + boundarySize[1] + PADDING, 1)));

	//Setup boids
	std::vector<Boid*> boids;
	for (auto x = 0; x < (BOID_COUNT); x++)
	{
		boids.push_back(
			new Boid(Point3d(0, 0, 0) +
			Point3d(
			0 + ((double)x/100.0f),
			0, 1)));
	}

	bool mouseDown = false;
	sf::Mouse::Button mouseButton;

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
				if (mouseButton == sf::Mouse::Button::Left)
				{
					if (mouseDownLast <= 0)
					{
						auto mouse = sf::Mouse::getPosition(*window);

						quadTreeMutexLock.lock();
						auto mousePos = sf::Mouse::getPosition(*window);
						boids.push_back(
							new Boid(Point3d(
							(mousePos.x / ((screenSize[0]) / 2)) - 1.0,
							(mousePos.y / ((screenSize[1]) / 2)) - 1.0, 1)));
						quadTreeMutexLock.unlock();

						mouseDownLast = SECONDS_PER_MOUSE_UPDATE;
					}
					else
						mouseDownLast -= timePassedInSeconds;
				}
				else
				{
					auto mousePos = sf::Mouse::getPosition(*window);
					Vec2f diff = Vec2f(mousePos.x, mousePos.y) - (screenSize / 2.0f);
					Vec3d v = Vec3d(diff[0], diff[1], 0); 
					normalize(v);
					gravity = v * GRAVITY_ACCELERATION;
				}
			}
			//Rebuild the quad-tree every SECONDS_PER_REBUILD
			if (lastRebuild <= 0)
			{
				/*quadTreeMutexLock.lock();
				{
					//quadTree.update();

					quadTree.clear();

					for (Boid* _b : boids)
						quadTree.insert(_b);
				}
				quadTreeMutexLock.unlock();*/

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
				mouseButton = event.mouseButton.button;
				mouseDown = true;
			}
			else if (event.type == sf::Event::MouseButtonReleased)
			{
				mouseDown = false;
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
		
		
		circleShape.setOutlineColor(sf::Color::White);
#ifdef RENDER_REAL
		circleShape.setFillColor(sf::Color::Blue);
		circleShape.setOutlineThickness(0);
		circleShape.setRadius(10.0f);
#else
		circleShape.setFillColor(sf::Color::Transparent);
		circleShape.setOutlineThickness(1);
		circleShape.setRadius(5.0f);
#endif
		quadTreeMutexLock.lock();
		for (Boid* boid : boids)
		{
			circleShape.setPosition(
				sf::Vector2f(
					(boid->pos[0] + 1.0) * (screenSize[0]/2.0),
					(boid->pos[1] + 1.0) * (screenSize[1]/2.0)));

			auto targetBox = AABoxf(
				Vec3f(boid->pos[0] - LINE_RANGE, boid->pos[1] - LINE_RANGE, 0),
				Vec3f(boid->pos[0] + LINE_RANGE, boid->pos[1] + LINE_RANGE, 1));

#ifdef DRAW_RECTS
			window->draw(circleShape);
#endif
		}
		quadTreeMutexLock.unlock();

		window->display();

		renderLimiter.End();
	}

	quadRebuilderThread.join();
	boidUpdateThread.join();

	for (auto boid : boids)
		delete boid;

	return 0;
}