#include "PointJointGroup.h"

#include "Primitive.h"
#include "Controller.h"
#include "Utility/SimpleDraw.h"
#include "GraphicsLibrary/Voxel/Voxeler.h"


PointJointGroup::PointJointGroup( GroupType newType ): Group(newType)
{
	Point p(0., 0., 0.);
	pos = p;
}

void PointJointGroup::process( QVector< Primitive* > segments )
{
	Group::process(segments);

	// Calculate the joint coordinates in both primitives
	Primitive * a = nodes.first();
	Primitive * b = nodes.last();

	jointCoords[a->id] = a->getCoordinate(pos);
	jointCoords[b->id] = b->getCoordinate(pos);
}

void PointJointGroup::regroup()
{
	Primitive *frozen,  *non_frozen;
	if (!getRegroupDirection(frozen, non_frozen)) return;

	// Modify the \non_frozen to fix the joint
	Vec3d newPos = frozen->fromCoordinate(jointCoords[frozen->id]);
	Vec3d oldPos = non_frozen->fromCoordinate(jointCoords[non_frozen->id]);

	non_frozen->movePoint(oldPos, newPos - oldPos);

	// Fix the regrouped point joint
	non_frozen->addFixedPoint(newPos);
}

void PointJointGroup::draw()
{
	// Draw debug
	Group::draw();
	
	// Show joints
	if(isDraw && !nodes.isEmpty())
	{
		Primitive * a = nodes.first();
		Primitive * b = nodes.last();
		SimpleDraw::IdentifyPoint( a->fromCoordinate(jointCoords[a->id]), 1,0,1,20 );
		SimpleDraw::IdentifyPoint( b->fromCoordinate(jointCoords[b->id]), 0,1,0, 20 );
	}

}

void PointJointGroup::saveParameters( std::ofstream &outF )
{
	outF << getJointPos();
}

void PointJointGroup::loadParameters( std::ifstream &inF, Vec3d translation, double scaleFactor )
{
	inF >> pos;
	pos += translation;
	pos *= scaleFactor;
}

Point PointJointGroup::getJointPosOnPrimitive( Primitive* prim )
{
	return prim->fromCoordinate(jointCoords[prim->id]);
}

Point PointJointGroup::getJointPos()
{
	Primitive * a = nodes.first();
	return a->fromCoordinate(jointCoords[a->id]);
}

void PointJointGroup::slide( QString sliderID )
{
	// Get the \slider and \track
	Primitive * slider = nodes.first();
	Primitive * track = nodes.last();

	// Swap if needed
	if (slider->id != sliderID)
	{
		Primitive * tmp = slider;
		slider = track;
		track = tmp;
	}

	// The joint position on \slider	
	Point oldPos = slider->fromCoordinate(jointCoords[slider->id]);

	// The \slider should be the '|' of the 'T' junction
	bool at_end = slider->atEnd(1, oldPos);
	if (at_end)
	{	
//		std::cout << qPrintable(track->id) << "is regarded as track.\n";

		//slider->debugPoints.clear();
		//slider->debugPoints.push_back(oldPos);

		// Project \oldPos to the track
		Point newPos = track->closestProjection(oldPos);
	/*	track->debugPoints2.clear();
		track->debugPoints2.push_back(newPos);*/

		slider->movePoint(oldPos, newPos - oldPos);
		slider->fixedPoints.push_back(newPos);

		// Recompute the coordinates of joint
		pos = newPos;
		process(nodes);

		// Freeze both slider and track
		slider->isFrozen = true;
		track->isFrozen = true;
	}

}

void PointJointGroup::rejoint(QString sliderID)
{
	// Get the \slider and \track
	Primitive * slider = nodes.first();
	Primitive * track = nodes.last();

	// Swap if needed
	if (slider->id != sliderID)
	{
		Primitive * tmp = slider;
		slider = track;
		track = tmp;
	}

	// The joint position on \slider	
	Point p = slider->fromCoordinate(jointCoords[slider->id]);

	// Update the joint coordinates for \track
	if (slider->atEnd(1, p))
	{
		jointCoords[track->id] = track->getCoordinate(slider->fromCoordinate(jointCoords[sliderID]));
	}

	// Freeze both slider and track
	/*slider->isFrozen = true;
	track->isFrozen = true;*/
}

Group* PointJointGroup::clone()
{
	PointJointGroup* g = new PointJointGroup(POINTJOINT);

	g->id = this->id;
	g->nodes = this->nodes;
	g->jointCoords = this->jointCoords;

	return g;
}
