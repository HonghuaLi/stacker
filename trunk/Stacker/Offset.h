#pragma once

#include "ColorMap.h"
#include <vector>
#include <set>
#include "QSegMesh.h"

class HiddenViewer;

class Offset
{
public:
	// Constructor
	Offset(HiddenViewer* viewer);

	// Active object
	QSegMesh* activeObject();

	// Compute offset function and stackability
	void computeOffset();

	// Save offset funciton as an color mapped image
	void saveOffsetAsImage(QString fileName);

	// Stackability = 1 - O_max/H
	double getStackability();

	// Detect hot spots (vertices on the mesh), where O_max occurs
	std::vector<int> hotSegments();

	void run();
	std::vector< std::vector<double> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction);


	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvolope;
	std::vector< std::vector<double> > lowerEnvolope;
	std::vector< std::vector<double> > offset; 
	double O_max;
	double objectH;
};