#pragma once
#include "Primitive.h"
#include "MathLibrary/Bounding/MinOBB3.h"
#include <Eigen/Dense>

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
	Cuboid( QSurfaceMesh* segment, QString newId);
	Cuboid( QSurfaceMesh* segment, QString newId, bool useAABB, int fit_method);
public:
	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit(){}
	virtual void computeMeshCoordiantes();
	void fit(bool useAABB = true, int obb_method = 0);

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh();

	// Visualize the primitive and potential actions
	virtual void draw();
	virtual	void drawNames(int name, bool isDrawParts = false);

	// Hot curves
	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples );
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respect );

	// Reshaping
	virtual void translate( Vec3d &T );
	virtual void moveCurveCenter( int cid, Vec3d T);
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	virtual bool excludePoints( std::vector< Vec3d > pnts );
	virtual void reshapeFromPoints( std::vector<Vec3d>& pnts );
	virtual void movePoint(Point p, Vec3d T);
	virtual void scaleCurve(int cid, double s);
	void moveLineJoint(Point A, Point B, Point newA, Point newB);

	// Primitive coordinate system
	virtual std::vector<double> getCoordinate( Point v );
	virtual Point fromCoordinate(std::vector<double> coords);
	virtual bool containsPoint(Point p);
	virtual Vec3d closestPoint(Point p);

	// Primitive state
	virtual void* getGeometryState();
	virtual void setGeometryState( void* );

	// Primitive geometry
	virtual std::vector <Vec3d> points();
	virtual QSurfaceMesh getGeometry();
	virtual double volume();
	virtual std::vector<Vec3d> majorAxis();
	virtual std::vector < std::vector <Vec3d> > getCurves();

	// Joint, symmetry
	virtual void setSymmetryPlanes(int nb_fold);

	// Selecting
	virtual Vec3d selectedPartPos();
	virtual void setSelectedPartId( Vec3d normal );
	virtual Point getSelectedCurveCenter();

	void scaleAlongAxis( Vector3 &scales );
	void rotateAroundAxes(Vector3 &angles );
	void recoverMesh();

	// Save and load
	virtual void save(std::ofstream &outF);
	virtual void load(std::ifstream &inF, Vec3d translation, double scaleFactor);

private:
	QSurfaceMesh getBoxGeometry( Box3 &box, bool isUniform = false );
	Vector3 getCoordinatesInUniformBox(Box3 &box, Vector3 &p);
    Vector3 getPositionInUniformBox(const Box3 &box, const Vector3 &coord);
	std::vector<Vec3d> getUniformBoxConners( Box3 &box );

	Vector3 getPositionInBox( Box3 &box, int vidx );

	std::vector<Vector3> getBoxConners(Box3 &box);
	std::vector< std::vector<Vector3> > getBoxFaces(Box3 &fromBox);
	Vector3 faceCenterOfUniformBox( Box3 &box, uint fid );

	void drawCube(double lineWidth, Vec4d color, bool isOpaque = false);

	// Debug
	bool isDrawAxis;
	bool isUsedAABB;

public:
	std::vector< std::vector<double> > coordinates;

	Box3 originalBox, currBox;
};

// Face corners
static uint cubeIds[6][4] = 
	{1, 2, 6, 5,
	 0, 4, 7, 3,
	 4, 5, 6, 7,
	 0, 3, 2, 1,
	 0, 1, 5, 4,
	 2, 3, 7, 6};
