#pragma once

#include "Vector.h"

typedef unsigned int uint;

class SkeletonNode : public Vec3d
{
public:

	uint index;

	SkeletonNode(double X, double Y, double Z, int newIndex)
	{
		x() = X; y() = Y; z() = Z;
		index = newIndex;
	}

	SkeletonNode(const SkeletonNode & n, int newIndex)
	{
		x() = n.x();	y() = n.y();	z() = n.z();
		index = newIndex;
	}

	SkeletonNode(const SkeletonNode & from)
	{
		x() = from.x();	y() = from.y();	z() = from.z();
		index = from.index;
	}

	static SkeletonNode Midpoint(const SkeletonNode & n1, const SkeletonNode & n2, int index)
	{
		double x = (n1.x() + n2.x()) / 2;
		double y = (n1.y() + n2.y()) / 2;
		double z = (n1.z() + n2.z()) / 2;

		return SkeletonNode(x, y, z, index);
	}

	void set(double X, double Y, double Z) {x() = X; y() = Y; z() = Z; }
	void set(double * pos){ x() = pos[0]; y() = pos[1]; z() = pos[2]; }

	double distanceTo(const SkeletonNode & n2)
	{	return (*this - n2).norm();	}
};
