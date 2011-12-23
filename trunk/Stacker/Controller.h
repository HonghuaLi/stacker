#pragma once
#include <QMap>
#include "Primitive.h"
#include "CuboidParam.h"
#include "Voxeler.h"
#include "Group.h"

class QSegMesh;

typedef QMap< QString, void* > ShapeState;

class Controller
{
public:
	Controller(QSegMesh* mesh);
	~Controller();

public:
	// SET and GET
	Primitive * getPrimitive( QString id );
	Primitive * getPrimitive( uint id );
	Primitive * getSelectedPrimitive();
	std::vector<Primitive*> getPrimitives();

	// Fitting
	void fitPrimitives();
	void fitOBBs();	
	void convertToGC( QString primitiveId, bool isUsingSkeleton = true, int cuboidAxis = 0 );
	// Joints
	void findJoints(double threshold);

	// Interaction
	void select(int id);
	void select(QString id);
	bool selectPrimitivePart( int id );
	Vec3d getPrimPartPos();

	// Grouping
	QMap<QString, Group*> groups;
	std::set< QString > getRidOfRedundancy( std::set< QString > Ids );
	QVector< Group * > groupsOf( QString id );

	// Deformation
	void deformShape( PrimitiveParamMap& primParams, bool isPermanent = false );
	void recoverShape();


	uint numPrimitives();
	int numHotPrimitives();

	// OpenGL stuff
	void draw();
	void drawNames(bool isDrawParts = false);

	// Shape state
	ShapeState getShapeState();
	void setShapeState( ShapeState &shapeState );

	// Propagation
	void propagate();
	void regroupPair(QString id1, QString id2);


	// Debug items
	std::vector<Point> debugPoints;
	std::vector< std::vector<Point> > debugLines;

	// Flags
	void setSegmentsVisible(bool isVisible = true);
	void setPrimitivesFrozen(bool isFrozen = false);
	void setPrimitivesAvailable(bool isAvailable = true);

	// Computations
	struct Stat{
		// stats that can be easily computed in run time
		double volumeBB;
		std::vector< double > volumePrim;
		std::map< std::pair<int, int>, double > proximity;
		std::map< std::pair<int, int>, bool > coplanarity;

		// stats that are stored and keep updating by other member functions
		PrimitiveParamMap params;
	};

	Stat& getStat();
	Stat& getOriginalStat();

	QVector<QString> stringIds(QVector<int> numericalIds);
	int getPrimitiveIdNum(QString stringId);
	QMap<int, QString> primitiveIdNum;

private:
	QMap<QString, Primitive*> primitives;
	QSegMesh* m_mesh;
	Stat originalStat;
	Stat currStat;

	void assignIds();
};

