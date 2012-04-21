#include "Propagator.h"

#include <QQueue>

#include "Group.h"
#include "Primitive.h"
#include "Controller.h"
#include "ShapeState.h"
#include "ConstraintGraph.h"

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
		c->regroup(); 
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
		QVector<QString> constrains = mGraph->getConstraints(target);

		// Solving
		// Apply symmetry constraint no matter how
		bool hasSymmetry= false;
		foreach(QString c, constrains)
		{
			Group* group = mCtrl->groups[c];
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

void Propagator::solveConstraints( QString target, QVector<QString> constraints )
{
	// \constraints are only positional	

	// There is only one constraint
	if (constraints.size() == 1)
	{
		mCtrl->groups[constraints.first()]->regroup();
		return;
	}

	// Solve multiple constraints
	

	// set target as frozen
	mGraph->node(target)->isFrozen = true;
}
