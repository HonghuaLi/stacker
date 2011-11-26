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
	void drawNames();
	void select(int id);

	// Deformation
	void deformShape( PrimitiveParamMap primParams, bool isPermanent = false );
	void recoverShape();

	// SET and GET
	Primitive * getPrimitive(int id);

	uint numPrimitives();
	int numHotPrimitives();

	// Testing
	void test1();

	// Computations
	struct Stat{
		double volumeBB;
		std::vector< double > volumePrim;
		std::map< std::pair<int, int>, double > proximity;
		std::map< std::pair<int, int>, bool > coplanarity;
		std::vector< PrimitiveParam * > params;
	};

	Stat getStat();
	Stat getOriginalStat();

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
	Stat originalStat;


};

