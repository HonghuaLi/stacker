#pragma once
#include "Primitive.h"
#include "MinOBB3.h"
#include <Eigen/Dense>
#include "CuboidParam.h"

class Cuboid : public Primitive
{
public:
	Cuboid(QSurfaceMesh* segment);

public:
	virtual void fit();
	virtual void deform( PrimitiveParam* params, bool isPermanent = false);
	virtual void deformMesh();
	virtual void draw();
	virtual	void drawNames(bool isDrawParts = false);

	virtual double volume();
	virtual std::vector <Vec3d> points();

	void translate( Vector3 &T );
	void scaleAlongAxis( Vector3 &scales );
	void rotateAroundAxes(Vector3 &angles );
	void recoverMesh();

	virtual Vec3d selectedPartPos();
	virtual void reshapePart( Vec3d q );

private:
	Vector3 getCoordinatesInBox(MinOBB3::Box3 &box, Vector3 &p);
	Vector3 getPositionInBox(MinOBB3::Box3 &box, Vector3 &coord);	
	Eigen::Matrix3d rotationMatrixAroundAxis(int axisId, double theta);
	std::vector<Vector3> getBoxConners(MinOBB3::Box3 box);
	std::vector< std::vector<Vector3> > getBoxFaces(MinOBB3::Box3 fromBox);

	void drawCube(double lineWidth, Vec4d color, bool isOpaque = false);
	Eigen::Vector3d V2E(Vector3 &vec);
	Vector3 E2V(Eigen::Vector3d &vec);

public:
	std::vector< Vector3 > coordinates;
	MinOBB3::Box3 originalBox, currBox;
	Vector3 preT, preS, preR;
};
