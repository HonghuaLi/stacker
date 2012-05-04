#include "QManualDeformer.h"

#include "Primitive.h"
#include "Propagator.h"
#include "SimpleDraw.h"

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

	prim->isFrozen = true;
	prim->moveCurveCenter( -1,  pos() - prim->getSelectedCurveCenter() );
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

	prim->isFrozen = true;
	prim->scaleCurve(-1, s);
	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;

	emit( objectModified() );

}

void QManualDeformer::draw()
{
	SimpleDraw::IdentifyPoint(pos(), 1,1,0,20);
}
