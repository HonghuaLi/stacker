#pragma once
#include "Primitive.h"
#include "CuboidParam.h"

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
	void convertToGC(int primitiveId, bool isUsingSkeleton = true);

	// OpenGL stuff
	void draw();
	void drawNames(bool isDrawParts = false);
	
	// Interaction
	void select(int id);
	bool selectPrimitivePart( int id );
	Vec3d getPrimPartPos();
	void reshapePrimitive(Vec3d q);

	// Deformation
	void deformShape( PrimitiveParamMap& primParams, bool isPermanent = false );
	void recoverShape();

	// SET and GET
	Primitive * getPrimitive(int id);
	Primitive * getSelectedPrimitive();

	uint numPrimitives();
	int numHotPrimitives();

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
		PrimitiveParamVector params;
	};

	Stat& getStat();
	Stat& getOriginalStat();

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
	Stat originalStat;
	Stat currStat;
};

