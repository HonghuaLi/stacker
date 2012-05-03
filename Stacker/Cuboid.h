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
	void fit(){}
	void computeMeshCoordinates();
	void fit(bool useAABB = true, int obb_method = 0);

	// Deform the underlying geometry according to the \pre_state and current state
	void deformMesh();

	// Visualize the primitive and potential actions
	void draw();
	void drawNames(int name, bool isDrawParts = false);

	// Hot curves
	int detectHotCurve( Point hotSample);
	int detectHotCurve( QVector<Point> &hotSamples );

	// Reshaping
	void translate( Vec3d &T );
	void moveCurveCenter( int cid, Vec3d T);
	void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	bool excludePoints( std::vector< Vec3d > pnts );
	void reshapeFromPoints( std::vector<Vec3d>& pnts );
	void movePoint(Point p, Vec3d T);
	void scaleCurve(int cid, double s);
	void moveLineJoint(Point A, Point B, Vec3d deltaA, Vec3d deltaB);
	void translateCurve( uint cid, Vec3d T, uint sid_respect );

	// Primitive coordinate system
	std::vector<double> getCoordinate( Point v );
	Point fromCoordinate(std::vector<double> coords);
	bool containsPoint(Point p);
	Vec3d closestPoint(Point p);

	// Primitive state
	void* getGeometryState();
	void setGeometryState( void* );

	// Primitive geometry
	double volume();
	Point curveCenter(int cid);
	double curveRadius(int cid);
	QSurfaceMesh getGeometry();
	std::vector<Vec3d> points();
	std::vector<Vec3d> majorAxis();
	std::vector< std::vector <Vec3d> > getCurves();

	// Joint, symmetry
	void setSymmetryPlanes(int nb_fold);

	// Selecting
	Vec3d selectedPartPos();
	void setSelectedPartId( Vec3d normal );
	Point getSelectedCurveCenter();

	void scaleAlongAxis( Vector3 &scales );
	void rotateAroundAxes(Vector3 &angles );
	void recoverMesh();

	// Save and load
	void save(std::ofstream &outF);
	void load(std::ifstream &inF, Vec3d translation, double scaleFactor);

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
