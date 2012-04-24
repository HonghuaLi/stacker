#pragma once

#include "HotSpot.h"
#include "ShapeState.h"
#include "EditingSuggestion.h"

class Offset;
class QSegMesh;
class Controller;
class Propagator;

extern QVector<EditingSuggestion> suggestions;

class StackabilityImprover
{
public:
	StackabilityImprover(Offset *offset);

	// Execute improving
	void executeImprove();

	// Suggestions
	QVector<EditingSuggestion> getSuggestions();
	void normalizeSuggestions();

	// Show results
	void showSolution( int i );
	void showSuggestion( int i );

private:
	std::vector< Vec3d > getLocalMoves( HotSpot& HS );
	void deformNearPointHotspot( int side );
	void deformNearLineHotspot( int side );
	void deformNearRingHotspot( int side );
	void deformNearHotspot( int side );
	void localSearch();

	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );
	void setPositionalConstriants(int fixedSide);


public:
	// Suggestion
	bool isSuggesting;
	QVector<EditingSuggestion> suggestions;
	PQShapeShateLessEnergy suggestSolutions;

	// Beam Searching
	double orgStackability;
	Vec3d constraint_bbmin, constraint_bbmax;
	ShapeState currentCandidate;
	QVector< ShapeState > usedCandidateSolutions;
	PQShapeShateLessEnergy candidateSolutions;
	PQShapeShateLessDistortion solutions;

private:
	Offset* activeOffset;

	QSegMesh* activeObject();
	Controller* ctrl();

};