#pragma once

#include "ColorMap.h"
#include <vector>
#include <set>
#include "Scene.h"
#include "QSurfaceMesh.h"


class Offset
{
public:
	Offset(Scene* scene);
	~Offset();

public:
	std::vector< std::vector<double> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction, std::vector< std::vector<double> > &offset, double O_max);
	void run();

private:
	Scene * activeScene;
};