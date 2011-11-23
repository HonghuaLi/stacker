#pragma once
#include "Primitive.h"

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
	void deformShape(Vec3d scale, Vec3d transl, Vec3d angles);
	void recoverShape();

	// SET and GET
	Primitive * getPrimitive(int id);
	int numPrimitives();
	int numHotPrimitives();

	// Testing
	void test1();


private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
};

