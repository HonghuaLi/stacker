#include "CuboidParam.h"

#include <cstdlib>
#include <iostream>
#include <math.h>

CuboidParam::CuboidParam( QString ID ) : PrimitiveParam(ID)
{
	double p[9] = {0, 0, 0, 0, 0, 0, 1, 1, 1};
	params.clear();
	params.insert(params.begin(), p, p+9);

	double ranges[6] = {-0.5, 0.5, -90, 90, 0.5, 2};
	setRanges(std::vector< double > (ranges, ranges+6));
}

// the range is normalized
bool CuboidParam::stepForward( int i, double step )
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

void CuboidParam::setRanges( std::vector< double > &ranges )
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

void CuboidParam::randomSample()
{
	for (int i=0;i<9;i++)
	{
		double t = ((double)rand()/(double)RAND_MAX);
		params[i] += t/10 *(sups[i] - infs[i]);
	}
}

Vec3d CuboidParam::getT()
{
	return Vec3d(params[0], params[1], params[2]);
}

Vec3d CuboidParam::getR()
{
	return Vec3d(params[3], params[4], params[5]);
}

Vec3d CuboidParam::getS()
{
	return Vec3d(params[6], params[7], params[8]);
}

bool CuboidParam::setParam( int i, double val )
{
	// Val is a percentage [0, 1]
	if (val<0 || val>1) return false;

	// T(0:2) R(3:5) S(6:8)
	if (i<6)
		params[i] = infs[i] + val * (sups[i] - infs[i]);
	else
		params[i] = exp(log(infs[i]) + val*log(sups[i]/infs[i]));

	return true;	 
}

bool CuboidParam::forceParam( int i, double val )
{
	params[i] = val;
	return true;
}

bool CuboidParam::setParams( std::vector< double >& newParams )
{
	bool result = true;

	for(int i = 0; i < newParams.size(); i++)
	{
		result = result && setParam(i, newParams[i]);
	}

	return result;
}


void CuboidParam::print()
{
	printf("T: %.3f, %.3f, %.3f,\nR: %.3f %.3f %.3f \nS: %.3f %.3f %.3f", params[0], params[1], 
		params[2], params[3], params[4], params[5], params[6], params[7], params[8]);
	std::cout << std::endl<< std::endl;
}


std::vector< double > CuboidParam::getDefaulParam()
{
	std::vector< double > pp;
	int i = 0;

	// T, R
	for (; i<6; i++)	{
		pp.push_back( (0-infs[i])/(sups[i]-infs[i]) );
	}

	//S
	for (; i<9;i++)	{
		pp.push_back( (log(1/infs[i]))/log(sups[i]/infs[i]) );
	}

	return pp;
}

int CuboidParam::numParams()
{
	return 9;
}

PrimitiveParam* CuboidParam::clone()
{
	CuboidParam* result = new CuboidParam( static_cast<CuboidParam&> (*this));
	return (PrimitiveParam*)result;
}
