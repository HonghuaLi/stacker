#pragma once

#include "Surface_mesh.h"
#include "SimpleEdge.h"

#include <map>

class Wire
{
private:
	std::map<uint, Point> points;
	SimpleEdgeSet edges;

	bool isClosedLoop;

public:
	Wire();

	void addPoint( const Point& p, uint pindex );
	void addEdge( const Point& p1, const Point& p2, uint pi1, uint pi2, double weight );
	void draw();
};
