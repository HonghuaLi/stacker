#include "Group.h"
#include "Controller.h"
#include "SimpleDraw.h"

int GroupUniqueID = 0;

Group::Group( Controller * controller, GroupType newType )
{
	this->ctrl = controller;
	this->type = newType;
	this->id = GroupUniqueID++;
}

void Group::addNode( int nodeId )
{
	nodes.insert(nodeId);
}

void Group::addNodes( std::vector<int> newNodes )
{
	foreach(int node, newNodes)
		addNode(node);
}

void Group::addEdge( int nodeA, int nodeB )
{
	addNode(nodeA);
	addNode(nodeB);

	edges.insert(Group::Edge(nodeA, nodeB, edges.size()));
}

void Group::removeNode( int nodeId )
{
	nodes.remove(nodeId);
}

void Group::removeEdge( int nodeA, int nodeB )
{
	edges.remove(Edge(nodeA, nodeB));
}

void Group::draw()
{
	foreach(int node, nodes)
	{
		std::vector<Point> pnts = ctrl->getPrimitive(node)->points();

		Point centerPoint(0,0,0);

		foreach(Point p, pnts) centerPoint += p;

		centerPoint /= pnts.size();

		SimpleDraw::IdentifyPoint(centerPoint, 0,0,1);
	}
}
