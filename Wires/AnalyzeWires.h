#pragma once

#include "Surface_mesh.h"
#include "Wire.h"

class AnalyzeWires
{
public:
	static std::vector<Wire> fromMesh( Surface_mesh * m, double dihedralThreshold );
	static std::vector<Wire> fromMesh2( Surface_mesh * m, double dihedralThreshold ); // experimental: from curvature
};