#pragma once
#include <QMap>
#include "Primitive.h"
#include "CuboidParam.h"
#include "Voxeler.h"
#include "Group.h"

class QSegMesh;

class Controller
{
public:
	Controller(QSegMesh* mesh);
	~Controller();

public:
	// Fitting
	void fitPrimitives();
	void fitOBBs();	
	void convertToGC(QString primitiveId, bool isUsingSkeleton = true);

	// Joints
	void findJoints(double threshold);

	// Interaction
	void select(int id);
	void select(QString id);
	bool selectPrimitivePart( int id );
	Vec3d getPrimPartPos();
	void reshapePrimitive(Vec3d q);

	// Grouping
	std::map<QString, Group*> groups;

	// Deformation
	void deformShape( PrimitiveParamMap& primParams, bool isPermanent = false );
	void recoverShape();

	// SET and GET
	Primitive * getPrimitive( QString id );
	Primitive * getPrimitive( uint id );
	Primitive * getSelectedPrimitive();
	std::vector<Primitive*> getPrimitives();

	uint numPrimitives();
	int numHotPrimitives();

	// OpenGL stuff
	void draw();
	void drawNames(bool isDrawParts = false);

	// Debug items
	std::vector<Point> debugPoints;
	std::vector< std::vector<Point> > debugLines;

	// Testing
	void test1();

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

