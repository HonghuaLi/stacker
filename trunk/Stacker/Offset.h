#pragma once

#include "ColorMap.h"
#include "QSegMesh.h"
#include "Controller.h"
#include <QColor>
#include <QRect>
#include <QQueue>
#include <functional>

class HiddenViewer;

enum STACKING_TYPE
{
	STRAIGHT_LINE, ROT_AROUND_AXIS, ROT_FREE_FORM
};

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
		bool isRing;
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
	void computeEnvelopeOfShape(Vec3d pos, Vec3d upVector = Vec3d(0,1,0), Vec3d horizontalShift = Vec3d(0,0,0));
	void computeEnvelopeOfRegion( int direction , Vec3d bbmin, Vec3d bbmax);
	void computeOffset();
	double computeOffsetOfShape( STACKING_TYPE type = STRAIGHT_LINE, int rotDensity = 1);
	void computeOffsetOfRegion( std::vector< Vec2i >& region );
	double getStackability();

	// Detect hot spots
	void hotspotsFromDirection( int direction );
	void detectHotspots();
	HotSpot detectHotspotInRegion(int direction, std::vector<Vec2i>& hotRegion);
	std::set<QString> getHotSegment();
	void showHotSpots();

	// Improve stackability
	void improveStackabilityTo(double targetS);
	void improveStackability();
	void applyHeuristics();
	void applyHeuristicsOnHotspot( HotSpot& HS, HotSpot& opHS );
	void applyHeuristicsOnHotRing( HotSpot& HS );
	std::vector< Vec3d > getHorizontalMoves( HotSpot& HS );
	double preStackability;
	Vec3d pre_bbmin, pre_bbmax;
	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );

	// Numeric
	double getValue( std::vector< std::vector < double > >& image, int x, int y, int r );
	static double getMinValue( std::vector< std::vector < double > >& image );
	static double getMaxValue( std::vector< std::vector < double > >& image );	
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
	Vec2i centerOfRegion( std::vector< Vec2i >& region );


	// Utilities 
	template< typename T >
	std::vector< std::vector < T > > createImage( int w, int h, T intial);
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int pathID );
	// Useful for debugging
	static void saveAsImage( std::vector< std::vector < double > >& image, double maxV, QString fileName );
	static void saveAsImage( std::vector< std::vector < bool > >& image, QString fileName );
	static void saveAsData( std::vector< std::vector < double > >& image, double maxV, QString fileName );
	void saveHotSpots( QString filename, int direction = 1, double percent = 1.0 );

	void setRegionColor( std::vector< std::vector < double > >& image, std::vector< Vec2i >& region, double color );
	void setPixelColor( std::vector< std::vector < double > >& image, Vec2i pos, double color );
	static QRgb jetColor( double val, double min, double max );
	void visualizeRegions( std::vector< std::vector<Vec2i> >& regions, QString filename );
	void showSolution( int i );
	double HOT_RANGE;

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

	std::map< QString, std::vector<Vec3d> > hotPoints;
	std::vector< std::vector<Vec2i> > hotRegions;
	std::vector< double > maxOffsetInHotRegions;
	std::vector< HotSpot >  upperHotSpots;
	std::vector< HotSpot >  lowerHotSpots;
	std::set< QString> hotSegments;

	QVector< ShapeState > usedCandidateSolutions;
	QQueue< ShapeState > candidateSolutions;
	QVector< ShapeState > solutions;
};
