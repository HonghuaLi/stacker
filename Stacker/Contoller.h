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
	void draw();
	void test1();
	void test2();

	Primitive * getPrimitive(int id);

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
};

