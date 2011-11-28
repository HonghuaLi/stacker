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

	// Get active object
	QSegMesh* activeObject();

	// Compute the upper/lower envelops
	std::vector< std::vector<double> > computeEnvelope(int direction);	
	
	// Compute offset function and stackability
	void computeOffset();

	// Detect hot spots
	void hotspotsFromDirection( int direction, double threshold );
	void detectHotspots();
	void showHotVertices();
	void showHotSegments();

	// Save offset function as an color mapped image
	void saveOffsetAsImage(QString fileName);

	// Stackability = 1 - O_max/H
	double getStackability();


	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvelope;
	std::vector< std::vector<double> > lowerEnvelope;
	std::vector< std::vector<double> > offset; 
	double O_max;
	double objectH;
	std::vector< uint > hotVertices;
	std::set< uint > hotSegments;
};