#pragma once

#include "ColorMap.h"
#include "QSegMesh.h"
#include <QColor>

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
	bool defineHeight( int direction, std::vector< Vec2i >& region);

	// Improve stackability
	void applyHeuristics();
	void applyHeuristicsOnHotspot( uint hid, int side );
	Vec3d getHorizontalMove( uint hid, int side );

	// Numeric
	double getValue( std::vector< std::vector < double > >& image, uint x, uint y, uint r );
	double getMinValue( std::vector< std::vector < double > >& image );
	double getMaxValue( std::vector< std::vector < double > >& image );	
	std::vector< double > getValuesInRegion( std::vector< std::vector < double > >& image, 
											 std::vector< Vec2i >& region, bool xFlipped = false );	
	template< typename PREDICATE >
	std::vector< Vec2i > getRegion( std::vector< std::vector < double > >& image, 
									 std::vector< std::vector < bool > >& mask, 
									 Vec2i seed, PREDICATE predicate );
	template< typename PREDICATE >
	std::vector< std::vector< Vec2i > > getRegions(std::vector< std::vector < double > >& image, 
																			PREDICATE predicate);

	std::vector< Vec2i > deltaVectorsToKRing(int deltaX, int deltaY, int K);
	std::vector< Vec2i > shiftRegionInBB( std::vector< Vec2i >& region, Vec2i delta, Vec2i bbmin, Vec2i bbmax );
	Vec2i sizeofRegion( std::vector< Vec2i >& region );

	// Utilities 
	template< typename T >
	std::vector< std::vector < T > > createImage( int w, int h, T intial);
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int direction );

	// Useful for debugging
	void saveAsImage( std::vector< std::vector < double > >& image, double maxV, QString fileName );
	void saveAsImage( std::vector< std::vector < bool > >& image, QString fileName );
	void setRegionColor( std::vector< std::vector < double > >& image, std::vector< Vec2i >& region, double color );
	void setPixelColor( std::vector< std::vector < double > >& image, Vec2i pos, double color );
	QRgb jetColor( double val, double min, double max );
	void visualizeHotRegions( QString filename );
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
	std::vector< std::vector<Vec2i> > hotRegions;
	std::vector < HotSpot >  upperHotSpots;
	std::vector < HotSpot >  lowerHotSpots;
	

};
