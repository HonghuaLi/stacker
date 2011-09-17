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
	std::vector< std::vector<float> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction, std::vector< std::vector<float> > &offset, float O_max);
	void run();

private:
	Scene * activeScene;
};