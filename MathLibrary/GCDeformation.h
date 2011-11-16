#pragma once

#include "QSurfaceMesh.h"

class GCDeformation{

public:
	GCDeformation(QSurfaceMesh * forShape, QSurfaceMesh * usingCage);
	
	void deform();

	QSurfaceMesh * shape;
	QSurfaceMesh * cage;

	std::vector<Point> orginalCagePos, deformedCagePos;
	std::vector<Normal> orginalCageNormal, deformedCageNormal;
	std::vector<double> S;

	std::vector< std::vector<double> > coord_v;
	std::vector< std::vector<double> > coord_n;

private:
	void computeCoordinates(const Vec3d& point, uint vIndex);
	double GCTriInt(const Vec3d& p, const Vec3d& v1, const Vec3d& v2, const Vec3d& e);

	void initDeform();
	void deformPoint(Point & p, uint vIndex);
};
