#pragma once

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"

class EditingSuggestion{
public:
	EditingSuggestion();
	EditingSuggestion( Point Center, Vec3d Direction, double Value);

	void draw();
	QSurfaceMesh getGeometry();

	Point center;
	Vec3d direction;
	double value;

	// debug values
	int side;
	double deltaS;
	double deltaV;
};
