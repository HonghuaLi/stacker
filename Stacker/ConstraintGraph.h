#pragma once

#include <QString>
#include <QMap>
#include <QList>

#include "Group.h"
class Primitive;
class Controller;

class ConstraintGraph{

public:
	// Edge structure
	struct Edge{
		Edge(){}
		QString id, from, to;
		int flag;
		Edge(QString ID, QString node1, QString node2, int Flag = -1){
			id = ID; from = node1; to = node2; flag = Flag;
		}
		bool operator == (const Edge & rhs) const{
			return id == rhs.id;
		}
	};

	ConstraintGraph(Controller * controller = 0);

	QString nextTarget();
	QVector<QString> getNeighbours(QString node);
	QVector<Edge> getConstraints(QString target);

	GroupType edgeType(QString id);

public:
	QMap< QString, QList<Edge> > adjacency_map;
	Controller * controller() { return ctrl; }

public:
	Controller * ctrl;
	Primitive * node(QString id);
};
