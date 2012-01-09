#pragma once

#include "QSurfaceMesh.h"

class EditSuggestion{
public:
	EditSuggestion();
	EditSuggestion( Point Center, Vec3d Direction, double Value);

	void draw(double scale, Vec3d normal);
	QSurfaceMesh getGeometry();

	Point center;
	Vec3d direction;
	double value;

	// debug values
	double deltaS;
	double deltaV;
};
