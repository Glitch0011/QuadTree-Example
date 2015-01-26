#include <thread>
#include <chrono>

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