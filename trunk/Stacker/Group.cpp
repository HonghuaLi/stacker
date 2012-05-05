#include "Group.h"
#include "Utility/SimpleDraw.h"
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

void Group::loadParameters( std::ifstream &inF, Vec3d translation, double scaleFactor )
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
		SimpleDraw::IdentifyLine(debugLines[i].first, debugLines[i].second, Color(1,0,1,1),false, 1);
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


bool Group::getRegroupDirection( Primitive* &frozen, Primitive* &non_frozen )
{
	frozen = nodes.first();
	non_frozen = nodes.last();

	// Both are frozen or unfrozen
	if(frozen->isFrozen == non_frozen->isFrozen) return false;

	// Match the pointer with the correct primitive
	if(!frozen->isFrozen) 
	{
		Primitive * temp = frozen;
		frozen = non_frozen;
		non_frozen = temp;
	}
	 return true;
}
