#pragma once

#include "QSurfaceMesh.h"
#include "Skeleton.h"

// For rotations..
#include "QGLViewer/quaternion.h"

#include "RMF.h"

class GeneralizedCylinder{
public:
	GeneralizedCylinder( Skeleton * skeleton, QSurfaceMesh * mesh );

	QSurfaceMesh geometry;
	std::vector<Point> spine;

	RMF frames;

	void draw();

	// debug
	std::vector<Point> debugPoints;

public:
	class Circle { 
	public: 
		Point center; double radius; Normal n; uint index;

		Circle(Point newCenter, double newRadius, Normal newNormal, uint newIndex)
		{ center = newCenter; radius = newRadius; n = newNormal; index = newIndex; };

		std::vector<Point> toSegments(int numSegments, const Vec3d& startVec);
	};

	std::vector<Circle> crossSection;
};
