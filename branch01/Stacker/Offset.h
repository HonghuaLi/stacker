#pragma once

#include "GraphicsLibrary/Mesh/QSegMesh.h"
#include "Controller.h"
#include <QQueue>
#include "StackerGlobal.h"
#include "EditSuggestion.h"
#include "HotSpot.h"
#include "Numeric.h"


extern QVector<EditSuggestion> suggestions;

class HiddenViewer;

enum STACKING_TYPE
{
	STRAIGHT_LINE, ROT_AROUND_AXIS, ROT_FREE_FORM
};


class Offset
{
public:

public:
	// Constructor
	Offset(HiddenViewer* viewer);

	// Shortener
	void clear();
	QSegMesh* activeObject();
	
	// Compute offset function and stackability (1 - O_max/objectH)
	void computeEnvelope(int direction);
	void computeEnvelopeOfShape(int direction);
	void computeEnvelopeOfShape(int direction, Vec3d pos, Vec3d upVector = Vec3d(0,1,0), Vec3d horizontalShift = Vec3d(0,0,0));
	void computeEnvelopeOfRegion( int direction , Vec3d bbmin, Vec3d bbmax);
	void computeOffset();
	double computeOffsetOfShape( STACKING_TYPE type = STRAIGHT_LINE, int rotDensity = 1);
	void computeOffsetOfRegion( std::vector< Vec2i >& region );
	double getStackability();

	// Detect hot spots
	void detectHotspots();
	HotSpot detectHotspotInRegion(int direction, std::vector<Vec2i>& hotRegion);
	std::set<QString> getHotSegment();
	void showHotSpots();
	void saveHotSpots( QString filename, int direction = 1, double percent = 1.0 );

	// Improve stackability
	void improveStackabilityToTarget();
	void improveStackability();
	void applyHeuristics();
	void applyHeuristicsOnHotspot( HotSpot& HS, HotSpot& opHS );
	void applyHeuristicsOnHotRing( HotSpot& HS );
	std::vector< Vec3d > getHorizontalMoves( HotSpot& HS );
	std::vector< Vec3d > getLocalMoves( HotSpot& HS );
	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );

	// Suggestions
	QVector<EditSuggestion> getSuggestions();
	void normalizeSuggestions();

	// Utilities 
	Vec3d unprojectedCoordinatesOf( uint x, uint y, int direction);
	Vec2i projectedCoordinatesOf( Vec3d point, int pathID );

	
	// Show results
	void showSolution( int i );
	void showSuggestion( int i );
	

public:
	HiddenViewer * activeViewer;
	int filterSize;
	double hotRangeThreshold;

	double O_max;
	double objectH;

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

	// Suggestion
	bool isSuggesting;
	QVector<EditSuggestion> suggestions;
	PQShapeShateLessEnergy suggestSolutions;

	// Beat Searching
	double orgStackability;
	Vec3d org_bbmin, org_bbmax;
	ShapeState currentCandidate;
	QVector< ShapeState > usedCandidateSolutions;
	PQShapeShateLessEnergy candidateSolutions;
	PQShapeShateLessDistortion solutions;
};
