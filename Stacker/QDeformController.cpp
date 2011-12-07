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
	ctrl->getSelectedPrimitive()->moveCurveCenter( -1, pos() - ctrl->getPrimPartPos() );
	emit( primitiveReshaped() );
}
