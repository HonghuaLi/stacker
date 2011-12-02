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
	void computeEnvelope(int direction, std::vector< std::vector<double> > &envelope, std::vector< std::vector<double> > &depth);	
	
	// Compute offset function and stackability
	void computeOffset();

	// Detect hot spots
	void hotspotsFromDirection( int direction, double threshold );
	void detectHotspots();
	void showHotSegments();
	std::set<uint> getHotSegment();

	// Save offset function as an color mapped image
	void saveOffsetAsImage(QString fileName);

	// Stackability = 1 - O_max/H
	double getStackability();

	// Guassian kernel
	double getValue( std::vector< std::vector < double > >& image, uint x, uint y );

	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvelope;
	std::vector< std::vector<double> > lowerEnvelope;	
	std::vector< std::vector<double> > upperDepth;
	std::vector< std::vector<double> > lowerDepth;
	std::vector< std::vector<double> > offset; 
	double O_max;
	double objectH;
	std::map< uint, std::set<uint> > hotFaces;
	std::map< uint, std::set<Vec3d> > hotPoints;
};