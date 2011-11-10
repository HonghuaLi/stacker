#pragma once
#include "Primitive.h"
#include "MinOBB3.h"

class Cuboid : public Primitive
{
public:
	Cuboid(QSurfaceMesh* segment);

public:
	virtual void fit();
	virtual void deformMesh();
	virtual void draw();
	void scaleAlongAxis(int axisId, double scale);
	void translate(Vector3 T);

private:
	Vector3 getCoordinatesInBox(MinOBB3::Box3 &box, Vector3 &p);
	Vector3 getPositionInBox(MinOBB3::Box3 &box, Vector3 &coord);
	std::vector<Vector3> getBoxConners(MinOBB3::Box3 box);

private:
	MinOBB3::Box3 currBox;
	MinOBB3::Box3 preBox;
};

