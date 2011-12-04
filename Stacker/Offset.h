#pragma once

#include "ColorMap.h"
#include "QSegMesh.h"


#include <vector>
#include <set>
#include <functional>

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
	void hotspotsFromDirection( int direction );
	void detectHotspots(int useFilterSize = 1, double hotRange = 0.99);
	std::set<uint> getHotSegment();
	void showHotSpots();

	// Save offset function as an color mapped image
	void saveOffsetAsImage(QString fileName);

	// Stackability = 1 - O_max/H
	double getStackability();

	// Numeric
	double getValue( std::vector< std::vector < double > >& image, uint x, uint y );
	double getMinValue( std::vector< std::vector < double > >& image );
	double getMaxValue( std::vector< std::vector < double > >& image );	
	template< typename PREDICATE >
	std::vector< Vec2ui > getRegion( std::vector< std::vector < double > >& image, 
									 std::vector< std::vector < bool > >& mask, 
									 Vec2ui seed, PREDICATE predicate );
	template< typename PREDICATE >
	std::vector< std::vector< Vec2ui > > getRegions(std::vector< std::vector < double > >& image, 
																			PREDICATE predicate);


public:
	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvelope;
	std::vector< std::vector<double> > lowerEnvelope;	
	std::vector< std::vector<double> > upperDepth;
	std::vector< std::vector<double> > lowerDepth;
	std::vector< std::vector<double> > offset; 
	std::vector< std::vector<Vec2ui> > hotRegions;
	
	double O_max;
	double objectH;
	std::map< uint, std::set<Vec3d> > hotSpots;
	
	// Paramters
	int filterSize;
	double hotRangeThreshold;
};