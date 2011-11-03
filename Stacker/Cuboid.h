#pragma once
#include "Primitive.h"
#include "MinOBB3.h"

class Cuboid : public Primitive
{
public:
	Cuboid(QSurfaceMesh* segment);
	virtual ~Cuboid(void);

public:
	virtual void fit();

	virtual void defromMesh();

	virtual void draw();

private:
	MinOBB3* m_mobb;
};

