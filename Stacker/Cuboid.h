#pragma once
#include "Primitive.h"
#include "MinOBB3.h"
#include <Eigen/Dense>

class Cuboid : public Primitive
{
public:
	Cuboid(QSurfaceMesh* segment);

public:
	virtual void fit();
	virtual void deformMesh();
	virtual void draw();
	virtual void drawNames();

	void translate( Vector3 &T );
	void scaleAlongAxis( Vector3 &scales );
	void rotateAroundAxes(Vector3 &angles );
	void transform( Vector3 &T, Vector3 &scales, Vector3 &angles );
	void recoverMesh();

private:
	Vector3 getCoordinatesInBox(MinOBB3::Box3 &box, Vector3 &p);
	Vector3 getPositionInBox(MinOBB3::Box3 &box, Vector3 &coord);
	std::vector<Vector3> getBoxConners(MinOBB3::Box3 box);
	Eigen::Matrix3d rotationMatrixAroundAxis(int axisId, double theta);
	
	void drawCube(double lineWidth, Vec4d color, bool isOpaque = false);
	Eigen::Vector3d V2E(Vector3 &vec);
	Vector3 E2V(Eigen::Vector3d &vec);

private:
	std::vector< Vector3 > coordinates;
	MinOBB3::Box3 originalBox, currBox;
	Vector3 preT, preS, preR;
};
