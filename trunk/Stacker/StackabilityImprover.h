#pragma once

#include "HotSpot.h"
#include "ShapeState.h"
#include "EditingSuggestion.h"

class Offset;
class QSegMesh;
class Controller;
class Propagator;

class StackabilityImprover : public QObject
{
	Q_OBJECT

public:
	StackabilityImprover(Offset *offset);

	// Execute improving
	void executeImprove(int level = -1);

	// Show results
	void showSolution( int i );
	void showSuggestion( int i );
	void normalizeSuggestions();

	// Clear
	void clear();

private:
	QVector<Vec3d> getLocalMoves( HotSpot& HS );
	QVector<double> getLocalScales( HotSpot& HS );
	void deformNearPointLineHotspot( int side );
	void deformNearRingHotspot( int side );
	void deformNearHotspot( int side );
	void recordSolution(Point handleCenter, Vec3d localMove, int side);
	void localSearch();

	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );
	void setPositionalConstriants( HotSpot& fixedHS );


public:
	// Suggestion
	QVector<EditingSuggestion> suggestions;

	// Best first Searching
	double origStackability;
	Vec3d constraint_bbmin, constraint_bbmax;
	ShapeState currentCandidate;
	QVector< ShapeState > usedCandidateSolutions;
	PQShapeStateLessEnergy candidateSolutions;
	PQShapeStateLessDistortion solutions;

private:
	Offset* activeOffset;

	QSegMesh* activeObject();
	Controller* ctrl();

signals:
	void printMessage( QString );
};