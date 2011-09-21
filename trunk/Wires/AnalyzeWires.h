#pragma once

#include "QSurfaceMesh.h"
#include "Wire.h"

class AnalyzeWires
{
public:
	static std::vector<Wire> fromMesh( QSurfaceMesh * m, double dihedralThreshold );
	static std::vector<Wire> fromMesh2( QSurfaceMesh * m, double dihedralThreshold ); // experimental: from curvature
};
