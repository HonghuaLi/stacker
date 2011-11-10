#include "GCylinder.h"
#include "SkeletonExtract.h"
#include "SimpleDraw.h"

GCylinder::GCylinder( QSurfaceMesh* mesh ) : Primitive(mesh)
{
	fit();
}

void GCylinder::fit()
{
	// Extract and save skeleton
	SkeletonExtract skelExt( m_mesh );
	Skeleton skel;
	skelExt.SaveToSkeleton( &skel );

	// Select part of skeleton
	skel.selectLongestPath();

	// Compute generalized cylinder
	gc = new GeneralizedCylinder( &skel, m_mesh );
}

void GCylinder::deformMesh()
{

}

void GCylinder::draw()
{
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glColor3d(0, 0.5, 1);

	double delta = 1.1;

	// Cross-sections
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		std::vector<Point> pnts = c.toSegments(30, gc->frames.U[c.index].s, delta);
		pnts.push_back(pnts.front());
		glBegin(GL_LINE_STRIP);
		foreach(Vec3d p, pnts) glVertex3dv(p);
		glEnd();
	}

	// Along height side, connected
	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.point.size(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * gc->crossSection[i].radius * delta));
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.point.size(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * -gc->crossSection[i].radius * delta));
	glEnd();

	// Along height side, dashed
	glLineStipple(1, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);

	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.point.size(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].s * gc->crossSection[i].radius * delta));
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.point.size(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].s * -gc->crossSection[i].radius * delta));
	glEnd();

	glDisable(GL_LINE_STIPPLE);
	glEnable(GL_LIGHTING);

	//gc->draw();
}
