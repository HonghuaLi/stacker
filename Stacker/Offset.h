#pragma once

#include "ColorMap.h"
#include <vector>
#include <set>
#include "QSegMesh.h"

class HiddenViewer;

class Offset
{
public:
	Offset(HiddenViewer* viewer);
	~Offset();

	QSegMesh* activeObject();
	void computeOffset();
	void saveOffsetAsImage(QString fileName);

	double getStackability();

	void run();
	std::vector< std::vector<double> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction, std::vector< std::vector<double> > &offset, double O_max);


	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvolope;
	std::vector< std::vector<double> > lowerEnvolope;
	std::vector< std::vector<double> > offset; 
	double O_max;
	double objectH;
};