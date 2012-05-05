#include "Primitive.h"
#include "Utility/SimpleDraw.h"

Primitive::Primitive( QSurfaceMesh* mesh, QString newId )
{
	m_mesh = mesh;

	id = newId;

	isSelected = false;
	isFrozen = false;

	isDraw = true;

	selectedCurveId = -1;
}

void Primitive::drawDebug()
{
	// Debug points
	foreach(Point p, debugPoints)	SimpleDraw::IdentifyPoint(p, 1,0,0);
	foreach(Point p, debugPoints2)	SimpleDraw::IdentifyPoint(p, 0,1,0);
	foreach(Point p, debugPoints3)	SimpleDraw::IdentifyPoint(p, 0,0,1);

	// Debug lines
	foreach(std::vector<Point> line, debugLines) SimpleDraw::IdentifyConnectedPoints(line, 1.0,0,0);
	foreach(std::vector<Point> line, debugLines2) SimpleDraw::IdentifyConnectedPoints(line, 0,1.0,0);
	foreach(std::vector<Point> line, debugLines3) SimpleDraw::IdentifyConnectedPoints(line, 0,0,1.0);

	foreach(std::vector<Vec3d> poly, debugPoly)		SimpleDraw::DrawPoly(poly, 1, 0, 0);
	foreach(std::vector<Vec3d> poly, debugPoly2)	SimpleDraw::DrawPoly(poly, 1, 0, 0);

	// Outline shape
	/*if(isSelected)
	{
		glPushAttrib( GL_ALL_ATTRIB_BITS );
		glEnable( GL_LIGHTING );

		glClearStencil(0);
		glClear( GL_STENCIL_BUFFER_BIT );
		glEnable( GL_STENCIL_TEST );

		glStencilFunc( GL_ALWAYS, 1, 0xFFFF );
		glStencilOp( GL_KEEP, GL_KEEP, GL_REPLACE );

		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		glColor3f( 0.0f, 0.0f, 0.0f );
		m_mesh->simpleDraw(true);
		glDisable( GL_LIGHTING );

		glStencilFunc( GL_NOTEQUAL, 1, 0xFFFF );
		glStencilOp( GL_KEEP, GL_KEEP, GL_REPLACE );

		glLineWidth( 20.0f );
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		glColor3f( 1.0f, 1.0f, 0.0f );
		m_mesh->simpleDraw(false);

		glPointSize( 10.0f );
		m_mesh->simpleDraw(false, true);

		glPopAttrib();
	}*/
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
	fixedPoints.push_back(fp);
}

double Primitive::similarity( void* state1, void* state2 )
{
	// Save the current state
	void* state = getState();
	
	std::vector<Vec3d> points1, points2;
	setState(state1);
	points1 = points();
	setState(state2);
	points2 = points();

	double result = 0;
	for (int i=0; i<points1.size();i++)
		result += (points1[i] - points2[i]).norm();
	
	// Restore the current state
	setState(state);

	return result;
}


void Primitive::addFixedCurve( int cid )
{

}
