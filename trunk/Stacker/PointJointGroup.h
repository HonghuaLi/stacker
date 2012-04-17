#pragma once

#include "Group.h"

class PointJointGroup : public Group{

public:
	PointJointGroup(GroupType newType);

	// Inherited methods
	void process(QVector< Primitive* > segments);
	QVector<QString> regroup();
	void draw();	
	void saveParameters( std::ofstream &outF );
	void loadParameters( std::ifstream &inF );

public:
	Point joint;
    QMap<QString, std::vector<double> > jointCoords;
};
