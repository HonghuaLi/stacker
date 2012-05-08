#include "QManualDeformer.h"

#include "Primitive.h"
#include "Propagator.h"
#include "Utility/SimpleDraw.h"

QManualDeformer::QManualDeformer(Controller * usingController)
{
	this->frame = new qglviewer::ManipulatedFrame;

	this->ctrl = usingController;
	
	this->lastScale = 1.0;

	this->connect(frame, SIGNAL(manipulated()), SLOT(updateController()));
}

qglviewer::ManipulatedFrame * QManualDeformer::getFrame()
{
	return frame;
}

Vec3d QManualDeformer::pos()
{
	qglviewer::Vec q = frame->position();
	return Vec3d (q.x,q.y,q.z);
}

void QManualDeformer::updateController()
{
	// unfreeze all
	ctrl->setPrimitivesFrozen(false);

	Primitive * prim = ctrl->getSelectedPrimitive();
	if(!prim) return;
	prim->isFrozen = true;

	Vec3d delta = pos() - prim->getSelectedCurveCenter();

	if(delta.norm() > 1e-9 && delta.norm() < prim->getMesh()->radius * 0.5)
	{
		// Translation
		prim->moveCurveCenter( -1,  delta );
	}
	else
	{
		// Rotation
		std::vector<Point> pnts = originalMesh;
		Point center = prim->centerPoint();
		
		for(int i = 0; i < pnts.size(); i++)
		{
			// move to zero
			pnts[i] -= center;

			// rotate
			qglviewer::Vec v(pnts[i]);
			v = (frame->rotation()) * v;
			pnts[i] = Point(v[0], v[1], v[2]);

			// move back
			pnts[i] += center;
		}

		prim->reshape(pnts, prim->scales());
		prim->getMesh()->buildUp();
	}

	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;
	emit( objectModified() );
}

void QManualDeformer::scaleUp( double s )
{
	// unfreeze all
	ctrl->setPrimitivesFrozen(false);

	Primitive * prim = ctrl->getSelectedPrimitive();
	if(!prim) return;

	prim->isFrozen = true;
	prim->scaleCurve(-1, s);
	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;

	emit( objectModified() );
}

void QManualDeformer::scale( Vec3d delta )
{	// unfreeze all
	ctrl->setPrimitivesFrozen(false);

	Primitive * prim = ctrl->getSelectedPrimitive();
	if(!prim) return;

	prim->isFrozen = true;

	//==============================
	// Scaling:
	std::vector<Point> pnts = originalMesh;
	Point center = prim->centerPoint();

	for(int i = 0; i < pnts.size(); i++)
	{
		// move to zero
		pnts[i] -= center;

		// rotate
		pnts[i] = Vec3d(pnts[i].x() * delta.x(),
						pnts[i].y() * delta.y(),
						pnts[i].z() * delta.z());

		// move back
		pnts[i] += center;
	}

	prim->reshape(pnts, prim->scales());
	prim->getMesh()->buildUp();
	//==============================

	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;

	emit( objectModified() );
}

void QManualDeformer::draw()
{
	SimpleDraw::IdentifyPoint(pos(), 1,1,0,20);
}

void QManualDeformer::saveOriginal()
{
	Primitive * prim = ctrl->getSelectedPrimitive();
	if(!prim) return;

	originalMesh = prim->points();
}
