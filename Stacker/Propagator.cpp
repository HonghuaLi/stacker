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
	if (id1 == id2) return;

	Group *pairGrp = NULL;
	QVector<Group*> groups = mCtrl->groupsOf(id1);
	for (int i=0;i<groups.size();i++){
		if (groups[i]->has(id2))
		{
			pairGrp = groups[i];
			break;
		}
	}

	if (pairGrp)
		pairGrp->regroup();
}

void Propagator::execute()
{
	// The next propagation target
	QString target = mGraph->nextTarget();

	while (!target.isEmpty())
	{
		// All the constrains
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
		{			
			if (constrains.size() == 1)
			{
				// There is only one constraint
				mCtrl->groups[constrains.first()]->regroup();
			}
			else
			{
				// There are multiple constraints
				solveConstraints(target, constrains);
			}
		}


		// The next
		target = mGraph->nextTarget();
	}
}

void Propagator::solveConstraints( QString target, QVector<QString> constraints )
{

}
