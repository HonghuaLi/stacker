#pragma once

#include "QSurfaceMesh.h"
#include "QControlPoint.h"

enum FFD_FitType {BoundingBoxFFD, VolumeFFD};

class FFD
{
public:
	FFD(QSurfaceMesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD);

	StdVector<Vec3d> bbFit();

	QSurfaceMesh * mesh;
	double width, length, height;

	StdVector<QControlPoint> points;
};
