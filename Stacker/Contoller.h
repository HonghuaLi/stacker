#pragma once
#include "Primitive.h"

class QSegMesh;

class Controller
{
public:
	Controller(QSegMesh* mesh);
	~Controller();

public:
	void fitPrimitives();
	void fitOBBs();
	
	void convertToGC(int primitiveId, bool isUsingSkeleton = true);

	void draw();
	void drawNames();

	void select(int id);

	void test1();
	void test2(Vec3d scale, Vec3d transl, Vec3d angles);
	void undo();
	Primitive * getPrimitive(int id);

	int numPrimitives();

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
};

