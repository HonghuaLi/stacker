#pragma once

#include "QSurfaceMesh.h"
#include "Plane.h"
#include <QVector>
#include <Eigen/Dense>
#include "ShapeState.h"

enum PrimType{ CUBOID, GC, WIRE};


class Primitive
{
public:
	Primitive(QSurfaceMesh* m_mesh, QString newId);

	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit() = 0;
	virtual void computeMeshCoordiantes() = 0;

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh() = 0;

	// Visualize the primitive and potential actions
	virtual void draw() = 0;
	virtual	void drawNames(int name, bool isDrawParts = false) = 0;

	// Hot curves
	virtual uint detectHotCurve( Vec3d hotSample );
	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples ) = 0;
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respect ) = 0;

	// Reshaping
	virtual void translate( Vec3d &T ) = 0;
	virtual void moveCurveCenter( int cid, Vec3d T) = 0;
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T) = 0;
	virtual bool excludePoints( std::vector< Vec3d >& pnts ) = 0;
	virtual void reshapeFromPoints( std::vector<Vec3d>& pnts ) = 0;
	virtual void movePoint(Point p, Vec3d T) = 0;
	virtual void scaleCurve(int cid, double s) = 0;

	// Primitive coordinate system
	virtual std::vector<double> getCoordinate( Point v ) = 0;
	virtual Point fromCoordinate(std::vector<double> coords) = 0;
	virtual bool containsPoint(Point p) = 0;
	virtual Vec3d closestPoint(Point p) = 0;

	// Primitive state
	virtual PrimitiveState getState();
	virtual void setState( PrimitiveState state);
	virtual void* getGeometryState() = 0;
	virtual void setGeometryState( void* ) = 0;

	// Primitive geometry
	virtual std::vector <Vec3d> points() = 0;
	virtual QSurfaceMesh getGeometry() = 0;
	double originalVolume;
	virtual double volume() = 0;
	virtual std::vector<Vec3d> majorAxis() = 0;
	virtual std::vector < std::vector <Vec3d> > getCurves() = 0;
	virtual Vec3d centerPoint();

	// The underlying geometry
	QSurfaceMesh*	m_mesh;			
	QSurfaceMesh* getMesh(){ return m_mesh; }

	// Symmetry, joints, fixed points
	bool isRotationalSymmetry;
	QVector<Point> fixedPoints;
	QVector<Plane> symmPlanes;
	virtual void setSymmetryPlanes(int nb_fold) = 0;
	virtual void addFixedPoint(Point fp);

	// Similarity between two primitives
	double similarity(PrimitiveState state1, PrimitiveState state2);


	// Helpful for debugging
	std::vector<Vec3d> debugPoints, debugPoints2, debugPoints3;
	std::vector< std::vector<Vec3d> > debugLines, debugLines2, debugLines3;
	std::vector< std::vector<Vec3d> > debugPoly, debugPoly2;
	void drawDebug();

	// Selecting
	bool isSelected;
	int selectedPartId;
	virtual Vec3d selectedPartPos() = 0;
	virtual void setSelectedPartId( Vec3d normal ) = 0;

	// Save and load
	virtual void save(std::ofstream &outF) = 0;
	virtual void load(std::ifstream &inF) = 0;

	// Rotation
	Eigen::Matrix3d rotationMatrixAroundAxis(Vec3d u, double theta);
	Vec3d rotatePointByMatrix( Eigen::Matrix3d &R, Vec3d p );
	Eigen::Vector3d V2E(Vec3d &vec);
	Vec3d E2V(Eigen::Vector3d &vec);

	QString id;
	PrimType primType;
	bool				isHot;			// Is this hot component?
	bool				isDirty;		// Has the underlying geometry been updated?
	bool				isFrozen;		// The seed of propagation
	bool				isDraw;
};
