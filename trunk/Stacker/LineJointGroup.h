#pragma once

#include "Group.h"

class LineJointGroup : public Group{

public:

	LineJointGroup(GroupType newType) : Group( newType ){}

	// Inherited methods
	void process(QVector< Primitive* > segments);
	QVector<QString> regroup();
	void draw();	
	void saveParameters( std::ofstream &outF );
	void loadParameters( std::ifstream &inF );

public:
	QVector<Point> lineEnds;
	QMap< QString, QVector< std::vector<double> > > lineEndsCoords;
};