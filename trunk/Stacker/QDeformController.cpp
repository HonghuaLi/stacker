#include "QDeformController.h"
#include "SimpleDraw.h"

QDeformController::QDeformController(Controller * usingController)
{
	this->frame = new qglviewer::ManipulatedFrame;
	this->ctrl = usingController;

	this->connect(frame, SIGNAL(manipulated()), SLOT(updateController()));
}

qglviewer::ManipulatedFrame * QDeformController::getFrame()
{
	return frame;
}

Vec3d QDeformController::pos()
{
	qglviewer::Vec q = frame->position();
	return Vec3d (q.x,q.y,q.z);
}

void QDeformController::updateController()
{
	// unfreeze all
	ctrl->setPrimitivesFrozen(false);

	Primitive * prim = ctrl->getSelectedPrimitive();
	prim->isFrozen = true;

	prim->moveCurveCenter( -1, pos() - prim->selectedPartPos() );
	ctrl->propagate();

	prim->isFrozen = false;

	emit( objectModified() );
}

void QDeformController::draw()
{
	SimpleDraw::IdentifyPoint(pos(), 1,1,0,20);
}
