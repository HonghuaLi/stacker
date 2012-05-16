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

	// Stackability
	void	computeStackability();
	void	computeStackability(Vec3d direction);
	double	getStackability(bool recompute = false);
	
	// Compute offset function and stackability
	void computeEnvelope(int side);
	void computeEnvelopeOfRegion( int side , Vec3d up, Vec3d direction, Vec3d bbmin, Vec3d bbmax );
	void computeEnvelopeOfShape( int side, Vec3d up, Vec3d stacking_direction );
	
	void computeOffset();
	void computeOffsetOfRegion( Vec3d direction, std::vector< Vec2i >& region );
	void computeOffsetOfShape( Vec3d direction );

	// Hot spots
	void		detectHotspots();
	HotSpot		detectHotspotInRegion(int side, std::vector<Vec2i>& hotRegion);
	HotSpot&	getHotspot( int side, int id );
	void		showHotSpots();
	std::vector<HotSpot> getHotspots(int side);
	void	saveHotSpots( QString filename, int direction = 1, double percent = 1.0 );

	// Stacking directions
	QVector<Vec3d> getDirectionsOnXYPlane();
	QVector<Vec3d> getDirectionsInCone(double cone_size);

	// Shortener
	QSegMesh*	activeObject();
	Controller* ctrl();
	void		clear();

	// Utilities 
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int pathID );
	Vec3d computeShapeExtents(Vec3d direction);
	Vec3d computeCameraUpVector(Vec3d newZ);

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
	Buffer2v2i hotRegions;
	std::vector< double > maxOffsetInHotRegions;
	std::vector< HotSpot >  upperHotSpots;
	std::vector< HotSpot >  lowerHotSpots;
	std::set< QString> hotSegments;


public slots:
	void setSearchType(int type);
	void setSearchDensity(int density);
	void setConeSize(double size);
};
