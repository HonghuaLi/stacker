#include "QDeformController.h"

#include "Primitive.h"
#include "Propagator.h"
#include "SimpleDraw.h"

QDeformController::QDeformController(Controller * usingController)
{
	this->frame = new qglviewer::ManipulatedFrame;
	this->ctrl = usingController;
	
	this->lastScale = 1.0;

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

	//std::cout << "Moving primitive: " << qPrintable(prim->id) << "\n";

	prim->isFrozen = true;
	prim->moveCurveCenter( -1,  pos() - prim->selectedPartPos() );
	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;

	emit( objectModified() );
}

void QDeformController::scaleUp( double s )
{
	// unfreeze all
	ctrl->setPrimitivesFrozen(false);

	Primitive * prim = ctrl->getSelectedPrimitive();

	if(prim != NULL)
	{
		prim->isFrozen = true;
		prim->scaleCurve(-1, s);
		Propagator propagator(ctrl);
		propagator.execute();
		prim->isFrozen = false;

		emit( objectModified() );
	}
}

void QDeformController::draw()
{
	SimpleDraw::IdentifyPoint(pos(), 1,1,0,20);
}
