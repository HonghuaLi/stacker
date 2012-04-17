#pragma once

#include "Offset.h"
#include "EditingSuggestion.h"
extern QVector<EditingSuggestion> suggestions;

class StackabilityImprover
{
public:
	// Constructor
	StackabilityImprover(Offset *offset);

	// Shortener
	QSegMesh* activeObject();


	// Improve stackability
	void improveStackabilityToTarget();
	void improveStackability();
	void applyHeuristics();
	void applyHeuristicsOnHotspot( HotSpot& HS, HotSpot& opHS );
	void applyHeuristicsOnHotRing( HotSpot& HS );
	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );
	std::vector< Vec3d > getHorizontalMoves( HotSpot& HS );
	std::vector< Vec3d > getLocalMoves( HotSpot& HS );

	// Suggestions
	QVector<EditingSuggestion> getSuggestions();
	void normalizeSuggestions();

	// Show results
	void showSolution( int i );
	void showSuggestion( int i );


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
	Offset *activeOffset;
};