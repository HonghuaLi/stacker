#pragma once

#include "QSurfaceMesh.h"
#include "Wire.h"

class AnalyzeWires
{
public:
	static std::vector<Wire> fromMesh( QSurfaceMesh * src_mesh, double sharp_threshold, double strength_threshold );
};
