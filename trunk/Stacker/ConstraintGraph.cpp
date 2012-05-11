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
	// Symmetry first
	Primitive * n1, * n2;
	foreach(Group* p, ctrl->groups)
	{
		if (p->type == SYMMETRY)
		{
			n1 = p->nodes.first();
			n2 = p->nodes.last();
			if (n1->isFrozen != n2->isFrozen)
				return (n1->isFrozen) ? n2->id : n1->id;
		}
	}

	// Find the most constrained free node
	QString target = "";
	double maxScore = 0.0;
	foreach(Primitive * p, ctrl->getPrimitives())
	{
		// Skip frozen nodes
		if(p->isFrozen) continue;

		// Get all (non-symmetric) neighbours
		QVector<QString> neighbours = getNeighbours(p->id);
		int numFrozen = 0;
		foreach(QString nei, neighbours)
			if (node(nei)->isFrozen) numFrozen++;

		// Update score
		double score = (double)numFrozen / neighbours.size();
		if (maxScore < score)
		{
			maxScore = score;
			target = p->id;
		}
	}

	return target;
}

// Non-symmetric neighbours
QVector<QString> ConstraintGraph::getNeighbours( QString node )
{
	QVector<QString> neighbours;

	foreach(Edge e, adjacency_map[node])
	{
		if (ctrl->groups[e.id]->type != SYMMETRY)
		{
			neighbours.push_back(e.to);
		}
	}

	return neighbours;
}

QVector<ConstraintGraph::Edge> ConstraintGraph::getConstraints( QString target )
{
	QVector<ConstraintGraph::Edge> constrains;

	foreach(ConstraintGraph::Edge e, adjacency_map[target])
	{
		if (node(e.to)->isFrozen)
			constrains.push_back(e);
	}

	return constrains;
}

QVector<ConstraintGraph::Edge> ConstraintGraph::getEdges( QString node )
{
	return adjacency_map[node].toVector();
}

GroupType ConstraintGraph::edgeType( QString id )
{
	return ctrl->groups[id]->type;
}
