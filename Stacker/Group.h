#pragma once

#include <vector>
#include <fstream>

#include <QMap>
#include <QVector>
#include <QString>

#include "GraphicsLibrary/Mesh/SurfaceMesh/Surface_mesh.h"

class Primitive;

enum GroupType{ SYMMETRY, POINTJOINT, LINEJOINT, CONCENTRIC, COPLANNAR, SELF_SYMMETRY, SELF_ROT_SYMMETRY };

class Group{
public:
	Group(GroupType newType);

	// Compute group properties
	virtual void process(QVector< Primitive* > segments);

	// Regroup
	virtual QVector<QString> regroup() = 0;

	// Visualization
	virtual void draw();
	void drawDebug();

	// Save and load parameters
	virtual void saveParameters(std::ofstream &outF);
	virtual void loadParameters(std::ifstream &inF);

	// Others
	bool has(QString id);
	QVector<QString> getNodes();

public:

	QString id;
	GroupType type;
	QVector< Primitive* > nodes;
	QMap< QString, QVector<int> > correspondence;

	// Visualization
	bool isDraw;
	std::vector<Point> debugPoints;
	std::vector< std::pair<Point,Point> > debugLines;
};