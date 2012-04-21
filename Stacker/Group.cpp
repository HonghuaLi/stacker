#include "Group.h"
#include "SimpleDraw.h"
#include "Primitive.h"

int GroupUniqueID = 0;

Group::Group(GroupType newType )
{
	this->type = newType;
	this->id = QString("%1").arg(GroupUniqueID++);

	this->isDraw = true;
}

void Group::process( QVector< Primitive* > segments )
{
	// Store the nodes
	nodes = segments;
}


void Group::regroup()
{
	// Set the nodes frozen
	foreach(Primitive* p, nodes)
		p->isFrozen = true;
}


void Group::loadParameters( std::ifstream &inF )
{
	// Please reload this method if there are parameters to load
}

void Group::saveParameters( std::ofstream &outF )
{
	// Please reload this method if there are parameters to save
}

void Group::draw()
{
	if(!isDraw) return;

	foreach(Primitive* node, nodes)
	{
	//	SimpleDraw::IdentifyPoint(node->centerPoint(), 0,0,1);
	}

	drawDebug();
}

void Group::drawDebug()
{
	foreach(Point p, debugPoints) SimpleDraw::IdentifyPoint(p, 1,1,1, 20);

	for(int i = 0; i < debugLines.size(); i++)
		SimpleDraw::IdentifyLine(debugLines[i].first, debugLines[i].second, Color(1,1,1,1),false, 6);
}

bool Group::has( QString id )
{
	foreach(Primitive* p, nodes){
		if (p->id == id) return true;
	}

	return false;
}

QVector<QString> Group::getNodes()
{
	QVector<QString> ids;
	foreach(Primitive* p, nodes)
		ids.push_back(p->id);

	return ids;
}
