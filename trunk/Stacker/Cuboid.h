#pragma once
#include "Primitive.h"
#include "QSurfaceMesh.h"
#include "MinOBB3.h"

class Cuboid : public Primitive
{
public:
	Cuboid(QSurfaceMesh* segment);
	~Cuboid(void);

public:
	void draw();

private:
	MinOBB3* m_mobb;
};

