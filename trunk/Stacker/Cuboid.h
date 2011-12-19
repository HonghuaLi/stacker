#pragma once
#include "Primitive.h"
#include "MinOBB3.h"
#include <Eigen/Dense>
#include "CuboidParam.h"

//		  7-----------6                     Y
//		 /|          /|                   f2^   /f5
//		4-+---------5 |                     |  / 
//		| |         | |                     | /
//		| |         | |             f1      |/     f0
//		| 3---------+-2            ---------+-------> X 
//		|/          |/                     /|
//		0-----------1                     / |
//								       f4/  |f3
//	                                    Z


class Cuboid : public Primitive
{
public:
	Cuboid( QSurfaceMesh* segment, QString newId );

public:
	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit();

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deform( PrimitiveParam* params, bool isPermanent = false);
	virtual void deformMesh();

	// Visualize the primitive and potential actions
	virtual void draw();
	virtual	void drawNames(int name, bool isDrawParts = false);

	// Hot curves
	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples );
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respected = -1 );

	// Reshaping
	virtual void translate( Vector3 &T );
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	virtual void moveCurveCenter( uint fid, Vec3d T);
	virtual bool excludePoints( std::vector< Vec3d >& pnts );
	virtual void reshapeFromCorners( std::vector<Vec3d>& corners);

	// Primitive coordinate system
	virtual std::vector<double> getCoordinate( Point v );
	virtual Point fromCoordinate(std::vector<double> coords);

	// Primitive state
	virtual void* getState();
	virtual void setState( void* state );

	// Primitive geometry
	virtual double volume();
	virtual std::vector <Vec3d> points();
	virtual QSurfaceMesh getGeometry();
	virtual std::vector <Vec3d> majorAxis();
	virtual std::vector < std::vector <Vec3d> > getCurves();

	// Symmetry
	virtual std::vector<Plane> getSymmetryPlanes(int opt);
	virtual void setSelectedPartId( Vec3d normal );

	void scaleAlongAxis( Vector3 &scales );
	void rotateAroundAxes(Vector3 &angles );
	void recoverMesh();
	Vec3d rotatePointByMatrix( Eigen::Matrix3d &R, Vec3d p );

	virtual Vec3d selectedPartPos();



private:
	Vector3 getCoordinatesInBox(MinOBB3::Box3 &box, Vector3 &p);
	Vector3 getPositionInBox(MinOBB3::Box3 &box, Vector3 &coord);	
	Eigen::Matrix3d rotationMatrixAroundAxis(Vec3d u, double theta);
	std::vector<Vector3> getBoxConners(MinOBB3::Box3 &box);
	std::vector< std::vector<Vector3> > getBoxFaces(MinOBB3::Box3 &fromBox);
	Vec3d faceCenterOfBox( MinOBB3::Box3 &box, uint fid );

	void drawCube(double lineWidth, Vec4d color, bool isOpaque = false);
	Eigen::Vector3d V2E(Vector3 &vec);
	Vector3 E2V(Eigen::Vector3d &vec);
public:
	std::vector< Vector3 > coordinates;
	MinOBB3::Box3 originalBox, currBox;
	Vector3 preT, preS, preR;
};
