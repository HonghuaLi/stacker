#pragma once

#include "HotSpot.h"
#include "ShapeState.h"
#include "EditPath.h"

class Offset;
class QSegMesh;
class Controller;
class Propagator;

#define IMPROVER_MAGIC_NUMBER -99999

class Improver : public QObject
{
	Q_OBJECT

public:
	Improver(Offset *offset);

	// Parameters
	int NUM_EXPECTED_SOLUTION;
	double BB_TOLERANCE;
	double TARGET_STACKABILITY;
	int LOCAL_RADIUS;

	// Execute improving
	void executeImprove(int level = IMPROVER_MAGIC_NUMBER);

private:
	QVector<Vec3d> getLocalMoves( HotSpot& HS );
	QVector<double> getLocalScales( HotSpot& HS );
	void deformNearPointLineHotspot( int side );
	void deformNearRingHotspot( int side );
	void deformNearHotspot( int side );
	void recordSolution(Point handleCenter, Vec3d localMove);
	void localSearch();

	bool satisfyBBConstraint();
	bool isUnique( ShapeState state, double threshold );
	void setPositionalConstriants( HotSpot& fixedHS );


public:
	// Best first Searching
	double origStackability;
	Vec3d constraint_bbmin, constraint_bbmax;
	ShapeState currentCandidate;
	PQShapeStateLessEnergy candidateSolutions;
	QVector<ShapeState> usedCandidateSolutions;
	QVector<ShapeState> solutions;

private:
	Offset* activeOffset;
	QSegMesh* activeObject();
	Controller* ctrl();

public slots:
	void setTargetStackability(double s);
	void setBBTolerance(double tol);
	void setNumExpectedSolutions(int num);

signals:
	void printMessage( QString );
};