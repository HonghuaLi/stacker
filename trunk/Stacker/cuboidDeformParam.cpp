#include "cuboidDeformParam.h"

#include <cstdlib>
#include <iostream>


cuboidDeformParam::cuboidDeformParam()
{
	double p[9] = {0, 0, 0, 0, 0, 0, 1, 1, 1};
	params.clear();
	params.insert(params.begin(), p, p+9);

	double ranges[6] = {-0.5, 0.5, -90, 90, 0.5, 2};
	setRanges(std::vector< double > (ranges, ranges+6));
}

// the range is normalized
bool cuboidDeformParam::stepForward( int i, double step )
{
	if (i<0 || i>8)
		return false;

	bool isOkay = false;	// Check if tmp is in the range	
	double tmp = params[i] + step * (sups[i]-infs[i]);
	if (tmp>=infs[i] && tmp<=sups[i])
	{
		params[i] = tmp;
		isOkay = true;
	}

	return isOkay;
}

void cuboidDeformParam::setRanges( std::vector< double > &ranges )
{
	infs.clear();
	sups.clear();
	for (int i=0;i<3;i++)
	{
		std::vector< double > tmp1(3, ranges[2*i]);
		infs.insert(infs.end(), tmp1.begin(), tmp1.end());

		std::vector< double > tmp2(3, ranges[2*i+1]);
		sups.insert(sups.end(), tmp2.begin(), tmp2.end());
	}
}

void cuboidDeformParam::randomSample()
{
	for (int i=0;i<9;i++)
	{
		double t = ((double)rand()/(double)RAND_MAX);
		params[i] += t/10 *(sups[i] - infs[i]);
	}
}

Vec3d cuboidDeformParam::getT()
{
	return Vec3d(params[0], params[1], params[2]);
}

Vec3d cuboidDeformParam::getR()
{
	return Vec3d(params[3], params[4], params[5]);
}

Vec3d cuboidDeformParam::getS()
{
	return Vec3d(params[6], params[7], params[8]);
}

bool cuboidDeformParam::setParam( int i, double val )
{
	if (val>=infs[i] && val <=sups[i])
	{
		params[i] = val;
		return true;
	}
	else
		return false;
	 
}

void cuboidDeformParam::print()
{
	printf("T: %.3f, %.3f, %.3f,\nR: %.3f %.3f %.3f \nS: %.3f %.3f %.3f", params[0], params[1], 
		params[2], params[3], params[4], params[5], params[6], params[7], params[8]);
	std::cout << std::endl<< std::endl;
}
