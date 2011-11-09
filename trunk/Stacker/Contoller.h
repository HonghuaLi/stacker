#pragma once
#include "QSegMesh.h"
#include "Primitive.h"


class Controller
{
public:
	Controller(QSegMesh* mesh);
	~Controller();


public:
	void fitPrimitives();
	void draw();
	void test1();
	void test2();

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
};


