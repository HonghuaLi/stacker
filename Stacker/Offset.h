#pragma once

#include "ColorMap.h"
#include "QSegMesh.h"

#include <functional>

class HiddenViewer;

class Offset
{
public:
	class HotSpot
	{
	public:
		uint hotRegionID;
		uint segmentID;
		bool defineHeight;
		std::vector< Vec3d > hotSamples;

		void print(){
			std::cout << " hotRegionID="   << hotRegionID 
					  << " segmentID="	  << segmentID 
					  << " defineHeight=" << defineHeight << std::endl; 
		}
	};

public:
	// Constructor
	Offset(HiddenViewer* viewer);

	// Shorteners
	void clear();
	QSegMesh* activeObject();
	
	// Compute offset function and stackability (1 - O_max/objectH)
	void computeEnvelope(int direction, std::vector< std::vector<double> > &envelope, std::vector< std::vector<double> > &depth);	
	void computeOffset();
	double getStackability();

	// Detect hot spots
	void hotspotsFromDirection( int direction );
	void detectHotspots(int useFilterSize = 1, double hotRange = 0.99);
	std::set<uint> getHotSegment();
	void showHotSpots();
	bool defineHeight( int direction, std::vector< Vec2ui >& region);

	// Improve stackability
	void applyHeuristics();

	// Numeric
	double getValue( std::vector< std::vector < double > >& image, uint x, uint y, uint r );
	double getMinValue( std::vector< std::vector < double > >& image );
	double getMaxValue( std::vector< std::vector < double > >& image );	
	std::vector< double > getValuesInRegion( std::vector< std::vector < double > >& image, 
											 std::vector< Vec2ui >& region, bool xFlipped = false );	
	template< typename PREDICATE >
	std::vector< Vec2ui > getRegion( std::vector< std::vector < double > >& image, 
									 std::vector< std::vector < bool > >& mask, 
									 Vec2ui seed, PREDICATE predicate );
	template< typename PREDICATE >
	std::vector< std::vector< Vec2ui > > getRegions(std::vector< std::vector < double > >& image, 
																			PREDICATE predicate);
	
	// Utilities 
	template< typename T >
	void makeImage( std::vector< std::vector < T > >& image, int w, int h, T intial);
	void saveAsImage( std::vector< std::vector < double > >& image, double maxV, QString fileName );

public:
	HiddenViewer * activeViewer;
	int filterSize;
	double hotRangeThreshold;

	double O_max;
	double objectH;

	std::vector< std::vector<double> > upperEnvelope;
	std::vector< std::vector<double> > lowerEnvelope;	
	std::vector< std::vector<double> > upperDepth;
	std::vector< std::vector<double> > lowerDepth;
	std::vector< std::vector<double> > offset; 	

	std::map< uint, std::vector<Vec3d> > hotPoints;
	std::vector< std::vector<Vec2ui> > hotRegions;
	std::vector < HotSpot >  upperHotSpots;
	std::vector < HotSpot >  lowerHotSpots;
	

};
