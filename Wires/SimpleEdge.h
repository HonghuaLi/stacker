#pragma once

#include <set>

typedef unsigned int uint;

class SimpleEdge
{
public:
	uint vIndex[2];
	double weight;

	SimpleEdge(){ vIndex[0] = vIndex[1] = -1; weight = 0; }

	SimpleEdge(uint v1, uint v2, double edgeWeight = 0)
	{
		vIndex[0] = v1;
		vIndex[1] = v2;

		this->weight = edgeWeight;
	}

	SimpleEdge(const SimpleEdge& newEdge)
	{
		vIndex[0] = newEdge.vIndex[0];
		vIndex[1] = newEdge.vIndex[1];
		weight = newEdge.weight;
	}

	inline uint operator[](uint i) const	{ 	return vIndex[i];	}
	inline uint operator()(uint i) const	{ 	return vIndex[i];	}

	inline uint neighbor() const			{	return vIndex[1];	}
};

struct CompareSimpleEdge{
	bool operator()(const SimpleEdge &a, const SimpleEdge &b) 
	{
		if (a.vIndex[0] < b.vIndex[0]) return true;
		if (a.vIndex[0] > b.vIndex[0]) return false;
		return a.vIndex[1] < b.vIndex[1];
	}
};

typedef std::set<SimpleEdge, CompareSimpleEdge> SimpleEdgeSet;
