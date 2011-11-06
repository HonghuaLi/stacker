#pragma once

#include "SkeletonNode.h"

class SkeletonEdge
{
public:

	uint index;

	SkeletonNode * n1;
	SkeletonNode * n2;

	double length;

	SkeletonEdge(SkeletonNode * N1, SkeletonNode * N2, uint newIndex){
		n1 = N1;
		n2 = N2;

		index = newIndex;
	}

	inline double calculateLength()
	{	return n1->distanceTo(*n2);	}

	inline Vec3d direction()
	{	return n2 - n1;	}
};
