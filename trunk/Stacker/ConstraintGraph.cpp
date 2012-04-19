#include "ConstraintGraph.h"
#include "Primitive.h"
#include "Group.h"
#include "Controller.h"

ConstraintGraph::ConstraintGraph( Controller * controller )
{
	this->ctrl = controller;

	if(!ctrl) return;

	// Add nodes
	foreach(Primitive * prim, ctrl->getPrimitives())
	{
		adjacency_map[prim->id] = QList<Edge>();
	}

	// Add edges
	foreach(Group * group, ctrl->groups.values())
	{
		QString n1 = group->nodes.first()->id;
		QString n2 = group->nodes.last()->id;

		Edge edge1(group->id, n1, n2);
		Edge edge2(group->id, n2, n1);

		adjacency_map[n1].push_back(edge1);
		adjacency_map[n2].push_back(edge2);
	}
}

Primitive * ConstraintGraph::node( QString id )
{
	return ctrl->getPrimitive(id);
}

QString ConstraintGraph::nextTarget()
{
	// First propagate via all symmetry constrain edges
	// If there is no symmetry constrains, rank the nodes by percentage
	return "";
}

QVector<QString> ConstraintGraph::constraints( QString target )
{
	return QVector<QString>();
}

GroupType ConstraintGraph::edgeType( QString id )
{
	return ctrl->groups[id]->type;
}
