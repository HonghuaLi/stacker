#pragma once
#include <vector>
#include "Vector.h"

class cuboidDeformParam
{
public:
	// Constructor
	cuboidDeformParam();

	// Ranges for each parameter
	void setRanges(std::vector< double > &new_ranges);

	// Step forward along i-th axis in the deformation space
	bool stepForward(int i, double step);

	// Random sample in the deformation space
	void randomSample();

	// Set param directly
	bool setParam(int i, double val);

	// GETs
	Vec3d getT();
	Vec3d getR();
	Vec3d getS();
	void print();

private:
	// T(0:2) R(3:5) S(6:8)
	std::vector< double > params;
	std::vector< double > infs;
	std::vector< double > sups;

};