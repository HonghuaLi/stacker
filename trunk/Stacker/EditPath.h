#pragma once

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"

class EditPath{
public:
	EditPath();
	EditPath( Point Center, Vec3d Direction, double Value);

	void draw();
	QSurfaceMesh getGeometry();

	Point	center;
	Vec3d	move;
	double	value;
};
