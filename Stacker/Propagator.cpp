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
	// The hot spots pair is on the same primitive
	if (id1 == id2) 
	{
		mGraph->node(id1)->isFrozen = true;
		return;
	}

	// This function is called only once at the very beginning of the propagation
	// At this time, all primitives are unfrozen
	// To regroup the pair, one primitive has to be frozen
	Primitive* node1 = mGraph->node(id1);
	Primitive* node2 = mGraph->node(id2);
	QString target;

	// A more sophisticated method might be needed (?)
	if (  node1->fixedPoints.size() > node2->fixedPoints.size() ||
		( node1->fixedPoints.size() == node2->fixedPoints.size() &&
		node1->symmPlanes.size() >= node2->symmPlanes.size() )  ) 
	{
		target = id2;
		node1->isFrozen = true;
	}
	else
	{
		target = id1;
		node2->isFrozen = true;
	}

	// Solve the constraints
	propagateTo(target);

	// Freeze the propagated target
	mGraph->node(target)->isFrozen = true;
}

void Propagator::execute()
{
	// The next propagation target
	QString target = mGraph->nextTarget();

	while (!target.isEmpty())
	{
		// Solve the constraints
		propagateTo(target);

		// Freeze the propagated target
		mGraph->node(target)->isFrozen = true;

		// The next
		target = mGraph->nextTarget();
	}
}

void Propagator::propagateTo( QString target )
{
	// All the constrains for the target
	QVector<ConstraintGraph::Edge> constraints = mGraph->getConstraints(target);
	int N = constraints.size();

	// This would happen if the graph is not connected
	// In this case, do nothing
	if (constraints.isEmpty())	return;

	// Priority: Symmetry > Line joint > Point joint
	// 1. Symmetry (suppose only one)
	foreach(ConstraintGraph::Edge e, constraints)
	{
		Group* group = mCtrl->groups[e.id];
		if ( group->type == SYMMETRY )
		{
			group->regroup();
			return;
		}
	}

	// 2. Line joint (suppose only one)
	foreach(ConstraintGraph::Edge e, constraints)
	{
		Group* group = mCtrl->groups[e.id];
		if ( group->type == LINEJOINT )
		{
			group->regroup();
			return;
		}
	}

	// 3. Point joint(s)
	solvePointJointConstraints(target, constraints);
}

void Propagator::solvePointJointConstraints( QString target, QVector<ConstraintGraph::Edge> &constraints )
{
	Primitive* targetPrim = mGraph->node(target);
	int N = constraints.size();

	// If the \target is GC, apply all constraints
	if (targetPrim->primType == GCYLINDER)
	{
		foreach(ConstraintGraph::Edge e, constraints)
			mCtrl->groups[e.id]->regroup();
		return;
	}

	// If the \target is cuboid
	if (targetPrim->primType == CUBOID)
	{
		// If there are more than one fixed points already
		// And no symmetry planes
		// Do nothing
		if (targetPrim->fixedPoints.size() > 1 
			&& targetPrim->symmPlanes.isEmpty()) 
			return;


		// == Only one constraint
		// Or there are symmetry planes
		if (N == 1 || !targetPrim->symmPlanes.isEmpty())
		{
			mCtrl->groups[constraints.first().id]->regroup();
			return;
		}

		// == More than one constraint
		// And no symmetry planes
		QVector<Point> constraint_points;
		foreach(ConstraintGraph::Edge e, constraints)
		{
			PointJointGroup* group = (PointJointGroup*)mCtrl->groups[e.id];
			Primitive *frozen = mCtrl->getPrimitive(e.to);
			Point p = frozen->fromCoordinate(group->jointCoords[e.to]);
			constraint_points.push_back(p);
		}

		if (targetPrim->fixedPoints.isEmpty())
		{
			// Find the furthest two constraints
			int idx1 = 0, idx2 = 1;
			double maxDis = 0.0;
			for (int i = 0; i < N-1; i++){
				for (int j = i+1; j < N; j++)
				{
					double dis = (constraint_points[i] - constraint_points[j]).norm();

					if (dis > maxDis){
						maxDis = dis;	idx1 = i;	idx2 = j;
					}
				}
			}

			mCtrl->groups[constraints[idx1].id]->regroup();
			mCtrl->groups[constraints[idx2].id]->regroup();
		}
		else
		{
			// Find the furthest constraint from the fixed point
			Point fixed_point = targetPrim->fixedPoints.first();

			int idx;
			double maxDis = 0.0;
			for (int i = 0; i < N; i++)
			{
				double dis = (constraint_points[i] - fixed_point).norm();

				if (dis > maxDis){
					maxDis = dis;	idx = i;
				}
			}

			mCtrl->groups[constraints[idx].id]->regroup();
		}
	}

}

void Propagator::slide( QString id )
{
	// The main part doesn't slide
	Primitive* slider = mGraph->node(id);
	if(slider->symmPlanes.size() == 2) return;

	// The \id has been deformed 
	// To make sliding happen, \id has to be deformed again respect to its neighbors
	QVector<ConstraintGraph::Edge> constraints = mGraph->getEdges(id);

	foreach(ConstraintGraph::Edge e, constraints)
	{
		Group* group = mCtrl->groups[e.id];
		if ( group->type == POINTJOINT )
		{
			((PointJointGroup*) group)->slide(id);
			std::cout << "Sliding " << qPrintable(id) << std::endl;
		}
	}

	// In case the \slider has symmetry peer	
	QVector<QString> peers;
	foreach(ConstraintGraph::Edge e, constraints)
	{
		// find the peers
		Group* group = mCtrl->groups[e.id];
		if ( group->type == SYMMETRY )
		{
			group->regroup();
			peers.push_back(e.to); 
		}
	}


	foreach(QString peerID, peers)
	{
		// get all constraints for peer
		QVector<ConstraintGraph::Edge> pc = mGraph->getEdges(peerID);

		foreach(ConstraintGraph::Edge e, pc)
		{
			Group* group = mCtrl->groups[e.id];
			if ( group->type == POINTJOINT )
			{
				((PointJointGroup*) group)->rejoint(peerID);
			}
		}
	}

}