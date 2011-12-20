#pragma once
#include <vector>
#include <fstream>
#include <QSet>
#include <QVector>
#include <QString>
#include "QSurfaceMesh.h"
#include "Primitive.h"

class Controller;

enum GroupType{ SYMMETRY, JOINT, CONCENTRIC, COPLANNAR };

class Group{
public:
	Group(Controller * controller, GroupType newType);

	// Add
	void addNodes( QVector<QString> newNodes );
	void addNode( QString nodeId );
	void addEdge( QString nodeA, QString nodeB );
	// Remove
	void removeNode( QString nodeId );
	void removeEdge( QString nodeA, QString nodeB );

	bool has(QString node);

	// Compute symmetry, cocentric, coplanar, ..., etc.
	virtual void process(QVector< QString > segments) = 0;
	virtual QVector<Primitive *> regroup();

	// Primitives
	Primitive * getPrimitive(QString nodeId);

	// Variables
	GroupType type;
	Controller * ctrl;
	QMap<int, QString> nodes;
	int nodeIdNum(QString stringId);
	QMap< QString, QVector<int> > correspondence;

	// Edge structure
	struct Edge{
		int first, second, idx;
		Edge(int a, int b, int index = -1){
			a = b = -1; idx = index;
			if(a > b) {second = a; first = b;}
			if(b > a) {second = b; first = a;}
		}
		bool operator ==(const Edge& other) const { return first == other.first; }
		bool operator !=(const Edge& other) const { return first != other.first; }
	};
		
	QSet<Edge> edges;

	QString id;

	// Visualization
	virtual void draw();
	virtual void drawDebug();
	std::vector<Point> debugPoints;
	std::vector< std::pair<Point,Point> > debugLines;

	// Save and load
	virtual void save(std::ofstream &outF);
	virtual void load(std::ifstream &inF);
};

inline uint qHash(Group::Edge key) {     
	uint h1 = qHash(key.first);
	uint h2 = qHash(key.second);
	return ((h1 << 16) | (h1 >> 16)) ^ h2; 
}
