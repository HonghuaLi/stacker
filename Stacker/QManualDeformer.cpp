#include "QManualDeformer.h"

#include "Primitive.h"
#include "GCylinder.h"
#include "Propagator.h"
#include "Utility/SimpleDraw.h"

QManualDeformer::QManualDeformer(Controller * usingController)
{
	this->frame = new qglviewer::ManipulatedFrame;
	this->ctrl = usingController;
	this->lastScale = 1.0;
	this->frame->setSpinningSensitivity(100.0);

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

	Vec3d delta(0,0,0);

	// Check if a curve is selected or not
	Point p = pos();
	if(prim->selectedCurveId < 0)
		delta = p - prim->centerPoint();
	else
		delta = p - prim->getSelectedCurveCenter();

	if(delta.norm() > 1e-9)
	{
		// Translation
		prim->moveCurveCenter( -1,  delta );
	}
	else
	{
		// Rotation
		// Restrict the rotation according to symmetrys
		// How?

		std::vector<Point> pnts, pnts2;
		Point center(0);

		// Get structure points
		if(prim->primType == CUBOID)
		{
			// Apply to points
			pnts = originalMesh;
			prim->centerPoint();
		}
		else
		{
			GCylinder * gc = (GCylinder *) prim;
			int N = gc->basicGC.crossSection.size();
			pnts.resize(N);

			for(int i = 0; i < N; i++)
			{
				pnts[i] = gc->basicGC.crossSection[i].center;
				center += pnts[i];
				//pnts2.push_back(pnts.back());
			}
			center /= N;
		}

		// Rotate structure
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

		// Reshape
		if(prim->primType == CUBOID)
			prim->reshape(pnts, prim->scales());
		else
		{
			GCylinder * gc = (GCylinder *) prim;
			int N = gc->basicGC.crossSection.size();

			for(int i = 0; i < N; i++)
			{
				gc->basicGC.crossSection[i].center = pnts[i];
			}
			
			gc->update();
		}

		prim->getMesh()->buildUp();
	}

	Propagator propagator(ctrl);
	propagator.slide(prim->id);
	prim->isFrozen = true;
	propagator.execute();

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
	/*ctrl->setPrimitivesFrozen(false);

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

		// scale
		pnts[i] = Vec3d(pnts[i].x() * delta.x(),
						pnts[i].y() * delta.y(),
						pnts[i].z() * delta.z());

		// move back
		pnts[i] += center;
	}

	std::vector<double> oldScales = prim->scales();

	if(prim->primType == GCYLINDER)
	{
		double usedScale = (delta.x() != 1.0) ? delta.x() : delta.y();
		for(int i = 0; i < (int)oldScales.size(); i++)
		{
			oldScales[i] += (usedScale - 1) * 0.075;
		}
	}

	prim->reshape(pnts, oldScales);
	prim->getMesh()->buildUp();
	//==============================

	Propagator propagator(ctrl);
	propagator.execute();
	prim->isFrozen = false;

	emit( objectModified() );*/
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
