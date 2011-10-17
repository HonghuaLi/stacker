#pragma once

#include "QSurfaceMesh.h"
#include "QControlPoint.h"

enum FFD_FitType {BoundingBoxFFD, VolumeFFD};

class FFD
{
public:
	FFD(QSurfaceMesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD);

	void bbFit(Vec3i res);

	QSurfaceMesh * mesh;
	double width, length, height;

	Vec3i resolution;

	StdVector<QControlPoint*> points;
	StdVector< StdVector< StdVector < int > > > pointsGridIdx; 
};
