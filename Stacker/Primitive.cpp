#include "Primitive.h"
#include "SimpleDraw.h"

Primitive::Primitive( QSurfaceMesh* mesh )
{
	m_mesh = mesh;

	isSelected = false;
}

void Primitive::drawDebug()
{
	foreach(Vec3d p, debugPoints)
		SimpleDraw::IdentifyPoint(p);

	for(int i = 0; i < debugLines.size(); i++)
		SimpleDraw::IdentifyLine(debugLines[i].first, debugLines[i].second);
}
