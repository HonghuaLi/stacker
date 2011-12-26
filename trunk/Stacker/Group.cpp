#include "Group.h"
#include "Controller.h"
#include "SimpleDraw.h"

int GroupUniqueID = 0;

Group::Group( Controller * controller, GroupType newType )
{
	this->ctrl = controller;
	this->type = newType;
	this->id = QString("%1").arg(GroupUniqueID++);

	this->isDraw = true;
}

void Group::addNode( QString nodeId )
{
	nodes[nodes.size()] = nodeId;
}

void Group::addNodes( QVector<QString> newNodes )
{
	foreach(QString node, newNodes)
		addNode(node);
}

void Group::addEdge( QString nodeA, QString nodeB )
{
	addNode(nodeA);
	addNode(nodeB);

	edges.insert(Group::Edge(nodeIdNum(nodeA), nodeIdNum(nodeB), edges.size()));
}

void Group::removeNode( QString nodeId )
{
	nodes.remove(nodeIdNum(nodeId));
}

void Group::removeEdge( QString nodeA, QString nodeB )
{
	edges.remove(Edge(nodeIdNum(nodeA), nodeIdNum(nodeB)));
}

int Group::nodeIdNum(QString stringId)
{
	QMapIterator<int, QString> i(nodes);
	while (i.hasNext()) {
		i.next();
		if(i.value() == stringId) return i.key();
	}
	return -1;
}

Primitive * Group::getPrimitive(QString nodeId)
{
	return ctrl->getPrimitive(nodeId);
}

void Group::draw()
{
	if(!isDraw) return;

	foreach(QString node, nodes)
	{
		SimpleDraw::IdentifyPoint(getPrimitive(node)->centerPoint(), 0,0,1);
	}

	drawDebug();
}

void Group::drawDebug()
{
	glClear(GL_DEPTH_BUFFER_BIT);

	foreach(Point p, debugPoints) SimpleDraw::IdentifyPoint(p, 1,1,1, 20);

	for(int i = 0; i < debugLines.size(); i++)
		SimpleDraw::IdentifyLine(debugLines[i].first, debugLines[i].second, 1,1,1,false, 6);
}

void Group::save( std::ofstream &outF )
{
	outF << nodes.size() << "\t";
	foreach(QString node, nodes)
		outF << qPrintable(node) << "\t";
}

void Group::load( std::ifstream &inF )
{
	int n;
	inF >> n;
	std::string str;
	QVector<QString> segments;
	for (int i=0;i<n;i++)
	{
		inF >> str;
		segments.push_back(str.c_str());
	}

	process(segments);
}

bool Group::has( QString node )
{
	foreach(QString nodeId, nodes.values())
		if(nodeId == node) return true;
	return false;
}

QVector<Primitive *> Group::regroup()
{
	// Return the propagated primitives
	return QVector<Primitive *>();
}
