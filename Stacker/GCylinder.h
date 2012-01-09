#pragma once

#include "Primitive.h"
#include "GeneralizedCylinder.h"
#include "Skeleton.h"
#include "GCDeformation.h"
#include "QGLViewer/manipulatedFrame.h"

extern int skeletonJoints;

class GCylinder : public QObject, public Primitive
{
	Q_OBJECT

public:

	GCylinder( QSurfaceMesh* segment, QString newId);
	GCylinder( QSurfaceMesh* segment, QString newId, bool doFit);

public:
	virtual void fit();
	virtual void createGC( std::vector<Point> spinePoints, bool computeRadius = true );

	virtual void computeMeshCoordiantes();
	virtual void deform( PrimitiveParam* params, bool isPermanent = false);
	virtual void deformMesh();
	virtual void draw();
	virtual	void drawNames(int name, bool isDrawParts = false);

	// Hot curves
	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples );
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respect );

	// Reshaping
	virtual void translate( Vec3d &T );
	virtual void moveCurveCenter( int cid, Vec3d T);
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	virtual bool excludePoints( std::vector< Vec3d >& pnts );
	virtual void reshapeFromPoints( std::vector<Vec3d>& pnts );
	virtual void movePoint(Point p, Vec3d T);
	virtual void scaleCurve(int cid, double s);

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
	
	// Save and load
	virtual void save(std::ofstream &outF);
	virtual void load(std::ifstream &inF);

	void buildCage();

	qglviewer::ManipulatedFrame *mf1, *mf2;

public slots:
	void update();

private:
	void moveCurveCenterRanged(int cid, Vec3d delta, int start = 0, int finish = -1);

	GeneralizedCylinder * gc;
	Skeleton * skel;

	QSurfaceMesh * cage;
	void updateCage();
	GCDeformation * gcd;

	double 	deltaScale;
	double cageScale;
	int cageSides;

	bool isFitted;
	std::vector<Point> originalSpine;
};
