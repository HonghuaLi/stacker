#pragma once

#include "ColorMap.h"
#include "QSegMesh.h"
#include "Controller.h"
#include <QColor>
#include <QRect>

#include <queue>
#include <functional>

class HiddenViewer;

extern int FILTER_SIZE;
extern double HOT_RANGE;

class Offset
{
public:
	class HotSpot
	{
	public:
		int side;
		int hotRegionID;
		QString segmentID;
		bool defineHeight;
		std::vector< Vec3d > hotSamples;

		void print(){
			std::cout << "side="   << side 
					  << "\tsegmentID="	  << qPrintable(segmentID) 
					  << "\tdefineHeight=" << defineHeight << std::endl; 
		}

		Point hotPoint(){return hotSamples[0];}
	};

public:
	// Constructor
	Offset(HiddenViewer* viewer);

	// Shorteners
	void clear();
	QSegMesh* activeObject();
	
	// Compute offset function and stackability (1 - O_max/objectH)
	void computeEnvelope(int direction);
	void computeEnvelopeOfShape(int direction);
	void computeEnvelopeOfRegion( int direction , Vec3d bbmin, Vec3d bbmax);
	void computeOffset();
	void computeOffsetOfShape();
	void computeOffsetOfRegion( std::vector< Vec2i >& region );
	double getStackability();

	// Detect hot spots
	void hotspotsFromDirection( int direction );
	void detectHotspots();
	HotSpot detectHotspotInRegion(int direction, std::vector<Vec2i>& hotRegion);
	std::set<uint> getHotSegment();
	void showHotSpots();
	// Improve stackability
	void improveStackabilityTo(double targetS);
	void improveStackability();
	void applyHeuristics();
	void applyHeuristicsOnHotspot( HotSpot& HS, HotSpot& opHS );
	std::vector< Vec3d > getHorizontalMoves( HotSpot& HS );
	double preStackability;
	Vec3d pre_bbmin, pre_bbmax;
	bool satisfyBBConstraint();

	// Numeric
	double getValue( std::vector< std::vector < double > >& image, uint x, uint y, uint r );
	double getMinValue( std::vector< std::vector < double > >& image );
	double getMaxValue( std::vector< std::vector < double > >& image );	
	double maxValueInRegion( std::vector< std::vector < double > >& image,  std::vector< Vec2i >& region);
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
	void BBofRegion( std::vector< Vec2i >& region, Vec2i &bbmin, Vec2i &bbmax );
	// Utilities 
	template< typename T >
	std::vector< std::vector < T > > createImage( int w, int h, T intial);
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int pathID );
	// Useful for debugging
	void saveAsImage( std::vector< std::vector < double > >& image, double maxV, QString fileName );
	void saveAsImage( std::vector< std::vector < bool > >& image, QString fileName );
	void setRegionColor( std::vector< std::vector < double > >& image, std::vector< Vec2i >& region, double color );
	void setPixelColor( std::vector< std::vector < double > >& image, Vec2i pos, double color );
	QRgb jetColor( double val, double min, double max );
	void visualizeHotRegions( QString filename );
	void showHotSolution(int i);
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
	std::vector< double > maxOffsetInHotRegions;
	std::vector< HotSpot >  upperHotSpots;
	std::vector< HotSpot >  lowerHotSpots;
	std::set< QString> hotSegments;

	std::queue< ShapeState > candidateSolutions;
	std::vector< ShapeState > solutions;
	std::vector< ShapeState > hotSolutions;
};
