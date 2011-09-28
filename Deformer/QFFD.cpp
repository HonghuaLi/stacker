#include "QFFD.h"
#include "SimpleDraw.h"

FFD current_ffd;

QFFD::QFFD( QSurfaceMesh * src_mesh, FFD_FitType fit_type )
{
	isReady = false;

	current_ffd = FFD(src_mesh, fit_type);

	// Fit control points using bounding box
	StdVector<Vec3d> fitPoints = ffd()->bbFit();

	// Add control points from that fit
	uint i = 0;
	foreach(Vec3d p, fitPoints)
	{
		ffd()->points.push_back(QControlPoint(p, i++));
	}

	isReady = true;
}

void QFFD::draw()
{
	if(!isReady) return;

	// Visual representation
	foreach(QControlPoint cp, this->ffd()->points)
		SimpleDraw::IdentifyPoint(cp.pos);

	glColor4dv(Color(1,1,0,1));

	foreach(int idx, selectedPoints)
		SimpleDraw::DrawSphere(ffd()->points[idx].pos, Min(ffd()->width, Min(ffd()->length, ffd()->width)) * 0.05);
}

void QFFD::drawNames()
{
	foreach(QControlPoint cp, this->ffd()->points)
	{
		glPushName(cp.idx);
		glBegin(GL_POINTS);
		glVertex3dv(cp.pos);
		glEnd();
		glPopName();
	}
}

void QFFD::postSelection(int idx)
{
	if(idx == -1){
		selectedPoints.clear();
		return;
	}

	selectedPoints.push_back(idx);
}

QControlPoint & QFFD::getQControlPoint( int index )
{
	return ffd()->points[index];
}

FFD * QFFD::ffd()
{
	return &current_ffd;
}
