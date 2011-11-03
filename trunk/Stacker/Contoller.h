#pragma once
#include "QSegMesh.h"
#include "Primitive.h"
#include "Cuboid.h"


class Controller
{
public:
	Controller(QSegMesh* mesh);
	~Controller();


public:
	void fitPrimitives();
	void draw();

private:
	std::vector<Primitive*> primitives;
	QSegMesh* m_mesh;
};


