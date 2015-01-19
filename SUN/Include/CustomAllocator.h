#include <string>
#include <vector>

struct Allocation
{
	unsigned int size;
	std::string fileName;
	int line;
};

std::vector<Allocation> allocations;

void* operator new (unsigned int size, const char* filename, int line)
{
	void* ptr = new char[size];

	Allocation a;
	a.fileName = filename;
	a.size = size;
	a.line = line;

	allocations.push_back(a);

	return ptr;
}

#define new new(__FILE__, __LINE__)