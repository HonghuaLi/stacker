#pragma once

#include "Primitive.h"
#include "GeneralizedCylinder.h"

class GCylinder : public Primitive
{
public:
	GCylinder(QSurfaceMesh* segment);

public:
	virtual void fit();
	virtual void deformMesh();
	virtual void draw();

	void translate(Vec3d T);

private:
	GeneralizedCylinder * gc;

private:
	
};
