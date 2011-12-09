#pragma once
#include <vector>
#include <QSet>
#include "QSurfaceMesh.h"
#include "Primitive.h"

class Controller;

enum GroupType{ SYMMETRY, CONCENTRIC, COPLANNAR };

class Group{
public:
	Group(Controller * controller, GroupType newType);

	// Add
	void addNodes(std::vector<int> newNodes);
	void addNode(int nodeId);
	void addEdge(int nodeA, int nodeB);

	// Remove
	void removeNode(int nodeId);
	void removeEdge(int nodeA, int nodeB);

	// Compute symmetry, cocentric, coplanar, ..., etc.
	virtual void process(std::vector<int> segments) = 0;

	// Primitives
	Primitive * getPrimitive(int node);

	// Variables
	GroupType type;
	Controller * ctrl;
	QSet<int> nodes;

	// Edge structure
	struct Edge{
		int first, second, idx;
		Edge(int a, int b, int index = -1){
			a = b = -1; idx = index;
			if(a > b) {second = a; first = b;}
			if(b > a) {second = b; first = a;}
		}
		bool operator ==(const Edge& other) const { return first == other.first; }
		bool operator !=(const Edge& other) const { return first != other.first; }
	};
		
	QSet<Edge> edges;

	int id;

	// Visualization
	virtual void draw() = 0;
	std::vector<Point> debugPoints;
};

inline uint qHash(Group::Edge key) {     
	uint h1 = qHash(key.first);
	uint h2 = qHash(key.second);
	return ((h1 << 16) | (h1 >> 16)) ^ h2; 
}
