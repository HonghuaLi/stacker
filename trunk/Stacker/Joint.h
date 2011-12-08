#pragma once
#include <vector>

class Primitive;

class Joint{
public:
	Joint(Primitive *a, Primitive *b, std::vector<double> loc):A(a), B(b), location(loc) {}
	Primitive *A, *B;
	std::vector<double> location;
};
