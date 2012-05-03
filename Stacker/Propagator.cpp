#include "Propagator.h"

#include <QQueue>

#include "Group.h"
#include "PointJointGroup.h"
#include "LineJointGroup.h"
#include "Primitive.h"
#include "Cuboid.h"
#include "Controller.h"
#include "ShapeState.h"
#include "ConstraintGraph.h"
#include "JointDetector.h"

#include <Eigen/Core>
using namespace Eigen;

Propagator::Propagator( Controller* ctrl )
{
	mCtrl = ctrl;
	mGraph = new ConstraintGraph(mCtrl);
}


void Propagator::regroupPair( QString id1, QString id2 )
{
	// For sure there is no group
	if (id1 == id2) return;

	// Get the constraint group(s)
	QVector<Group*> constraints;
	QVector<Group*> groups = mCtrl->groupsOf(id1);
	for (int i=0;i<groups.size();i++)
	{
		if (groups[i]->has(id2))
			constraints.push_back(groups[i]);
	}

	// If there are constraints, regroup them
	// Warning: more than two = problems?
	foreach(Group* c, constraints)
	{
		// One has to be frozen
		// So the one having most fixed points is frozen
		Primitive* node1 = c->nodes.first();
		Primitive* node2 = c->nodes.last();

		if (node1->fixedPoints.size() > node2->fixedPoints.size())
			node1->isFrozen = true;
		else
			node2->isFrozen = true;

		c->regroup(); 
	}
}

void Propagator::execute()
{
	// The next propagation target
	QString target = mGraph->nextTarget();

	while (!target.isEmpty())
	{
		// DEBUG
		//std::cout << "Current target = " << qPrintable(target) << "\t";

		// All the constrains for the target
		QVector<ConstraintGraph::Edge> constrains = mGraph->getConstraints(target);

		// Solving
		// Apply symmetry constraint no matter how
		bool hasSymmetry= false;
		foreach(ConstraintGraph::Edge e, constrains)
		{
			Group* group = mCtrl->groups[e.id];
			if ( group->type == SYMMETRY )
			{
				group->regroup();
				hasSymmetry = true;
				break;
			}
		}

		// The challenging part
		if ( !hasSymmetry )
			solveConstraints(target, constrains);


		// The next
		target = mGraph->nextTarget();
	}
}

void Propagator::solveConstraints( QString target, QVector<ConstraintGraph::Edge> constraints )
{
	// \constraints are only positional: joint and hight defining constraints

	// There is only one constraint
	if (!constraints.isEmpty())
		mCtrl->groups[constraints.first().id]->regroup();

	// set target as frozen
	mGraph->node(target)->isFrozen = true;
}