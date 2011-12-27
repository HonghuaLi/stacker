#include "Primitive.h"
#include "SimpleDraw.h"

Primitive::Primitive( QSurfaceMesh* mesh, QString newId )
{
	m_mesh = mesh;

	id = newId;

	isSelected = false;
	isAvailable = true;
	isFrozen = false;
}

void Primitive::drawDebug()
{
	foreach(Vec3d p, debugPoints)
		SimpleDraw::IdentifyPoint(p);

	for(int i = 0; i < debugLines.size(); i++)
		SimpleDraw::IdentifyLine(debugLines[i].first, debugLines[i].second);

	foreach(std::vector<Vec3d> poly, debugPoly)
		SimpleDraw::DrawPoly(poly);
}

Vec3d Primitive::centerPoint()
{
	Point centerPoint(0,0,0);

	std::vector<Point> pnts = points();
	foreach(Point p, pnts) centerPoint += p;
	return centerPoint /= pnts.size();
}

void Primitive::addFixedPoint( Point fp )
{
	if (!symmPlanes.empty() || !fixedPoints.empty())
		isFrozen = true;

	fixedPoints.push_back(fp);
}
