#pragma once

#include "Group.h"

class LineJointGroup : public Group{

public:

	LineJointGroup(GroupType newType) : Group( newType ){}

	// Inherited methods
	void process(QVector< Primitive* > segments);
	void regroup();
	void draw();	
	void saveParameters( std::ofstream &outF );
	void loadParameters(std::ifstream &inF, Vec3d translation, double scaleFactor);
	Group* clone();


	// 
	void updateLineEnds();

public:
	QVector<Point> lineEnds;
	QMap< QString, QVector< std::vector<double> > > lineEndsCoords;
};