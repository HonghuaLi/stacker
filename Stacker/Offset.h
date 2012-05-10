#pragma once

#include <QQueue>
#include <QObject>


#include "GraphicsLibrary/Mesh/QSegMesh.h"
#include "Controller.h"
#include "HotSpot.h"
#include "Numeric.h"

#define ZERO_TOLERANCE 0.001

class HiddenViewer;

enum SEARCH_TYPE
{
	NONE, ROT_AROUND_X, ROT_AROUND_Y, ROT_AROUND_X_AND_Y, SAMPLE_UPPER_HEMESPHERE
};


class Offset: public QObject
{
	Q_OBJECT

public:
	Offset(HiddenViewer* viewer);

	void		computeOffsetOfShape();
	void		detectHotspots();

	double		getStackability(bool recompute = false);
	HotSpot&	getHotspot( int side, int id );
	void		showHotSpots();
	std::vector<HotSpot> getHotspots(int side);

	// Shortener
	QSegMesh*	activeObject();
	Controller* ctrl();
	void		clear();
	
public:
	// Compute offset function and stackability
	void computeEnvelope(int side);
	void computeEnvelopeOfShape( int side, Vec3d up, Vec3d stacking_direction );
	void computeEnvelopeOfRegion( int side , Vec3d bbmin, Vec3d bbmax);
	
	void computeOffset();
	void computeOffsetOfRegion( std::vector< Vec2i >& region );

	// Hot spots
	HotSpot detectHotspotInRegion(int side, std::vector<Vec2i>& hotRegion);
	void	saveHotSpots( QString filename, int direction = 1, double percent = 1.0 );

	// Stacking directions
	QVector<Vec3d> getDirectionsOnXYPlane();
	QVector<Vec3d> getDirectionsInCone(double cone_size);

	// Utilities 
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int pathID );
	Vec3d computeShapeExtents(Vec3d up);

public:
	HiddenViewer * activeViewer;

	// Stackability
	double O_max;

	// Parameters
	SEARCH_TYPE searchType;
	double coneSize;
	int searchDensity;			// Number of samples in [0, PI]

	// Buffers
	Buffer2d upperEnvelope;
	Buffer2d lowerEnvelope;	
	Buffer2d upperDepth;
	Buffer2d lowerDepth;
	Buffer2d offset; 	

	// Hot stuff
	std::map< QString, std::vector<Vec3d> > hotPoints;
	std::vector< std::vector<Vec2i> > hotRegions;
	std::vector< double > maxOffsetInHotRegions;
	std::vector< HotSpot >  upperHotSpots;
	std::vector< HotSpot >  lowerHotSpots;
	std::set< QString> hotSegments;


public slots:
	void setSearchType(int type);
	void setSearchDensity(int density);
	void setConeSize(double size);
};
