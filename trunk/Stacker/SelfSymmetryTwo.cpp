#include "SelfSymmetryTwo.h"


void SelfSymmetryTwo::process( QVector< QString > segments )
{
	addNodes(segments);

	Primitive * prim = getPrimitive(nodes[0]); 
	selectedPartId = prim->selectedPartId;
}

void SelfSymmetryTwo::draw()
{
	Primitive * prim = getPrimitive(nodes[0]); 
	prim->selectedPartId = selectedPartId;

	std::vector<Plane> planes = prim->getSymmetryPlanes(2);
	planes[0].draw();
	planes[1].draw();
}

void SelfSymmetryTwo::save( std::ofstream &outF )
{
	Group::save(outF);

	outF << selectedPartId << "\t";
}

void SelfSymmetryTwo::load( std::ifstream &inF )
{
	int n;
	inF >> n;
	std::string str;
	QVector<QString> segments;
	inF >> str;
	segments.push_back(str.c_str());

	int id;
	inF >> id;
	id = RANGED(0, id, 5);
	Primitive * prim = getPrimitive(segments[0]); 
	prim->selectedPartId = id;

	process(segments);
}


