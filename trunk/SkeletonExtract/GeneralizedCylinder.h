#pragma once

#include "QSurfaceMesh.h"
#include "Skeleton.h"

#include "RMF.h"

class GeneralizedCylinder{
private:
	QSurfaceMesh * src_mesh;

public:
	GeneralizedCylinder( Skeleton * skeleton, QSurfaceMesh * mesh );

	QSurfaceMesh geometry;
	RMF frames;

	void realignCrossSections();

	void draw();
	bool isDrawFrames;

	// debug
	std::vector<Point> debugPoints;
	
public:
	class Circle { 
	public: 
		Point center; double radius; Normal n; uint index;

		Circle(Point newCenter, double newRadius, Normal newNormal, uint newIndex)
		{ center = newCenter; radius = newRadius; n = newNormal; index = newIndex; };

		std::vector<Point> toSegments( int numSegments, const Vec3d& startVec, double delta = 1.0);
	};

	std::vector<Circle> crossSection;

};
