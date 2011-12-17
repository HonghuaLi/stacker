#include "SelfSymmetryOne.h"


void SelfSymmetryOne::process( QVector< QString > segments )
{
	addNodes(segments);
	Primitive * prim = getPrimitive(nodes[0]); 
	selectedPartId = prim->selectedPartId;
}

void SelfSymmetryOne::draw()
{
	Primitive * prim = getPrimitive(nodes[0]); 
	prim->selectedPartId = selectedPartId;

	std::vector<Plane> planes = prim->getSymmetryPlanes(1);
	planes[0].draw();
}

void SelfSymmetryOne::save( std::ofstream &outF )
{
	Group::save(outF);

	outF << selectedPartId << "\t";
}

void SelfSymmetryOne::load( std::ifstream &inF )
{
	int n;
	inF >> n;
	std::string str;
	QVector<QString> segments;
	inF >> str;
	segments.push_back(str.c_str());

	int id;
	inF >> id;
	Primitive * prim = getPrimitive(segments[0]); 
	prim->selectedPartId = id;

	process(segments);
}


