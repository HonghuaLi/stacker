#pragma once

#include "Primitive.h"
#include "GeneralizedCylinder.h"
#include "Skeleton.h"
#include "MathLibrary/Coordiantes/GCDeformation.h"
#include "GUI/Viewer/libQGLViewer/QGLViewer/manipulatedFrame.h"
#include "MathLibrary/Deformer/Skinning.h"

enum DEFORMER { GREEN_COORDIANTES, SKINNING };

class GCylinder : public QObject, public Primitive
{
	Q_OBJECT

public:

	GCylinder( QSurfaceMesh* segment, QString newId);
	GCylinder( QSurfaceMesh* segment, QString newId, bool doFit);

public:
	void fit();
	void createGC( std::vector<Point> spinePoints, bool computeRadius = true );

	void computeMeshCoordiantes();
	void deformMesh();
	void draw();
	void drawNames(int name, bool isDrawParts = false);

	// Hot curves
	int detectHotCurve( QVector< Vec3d > &hotSamples );
	Point getSelectedCurveCenter();

	// Reshaping
	void translate( Vec3d &T );
	void translateCurve( uint cid, Vec3d T, uint sid_respect );
	void moveCurveCenter( int cid, Vec3d T);
	void scaleCurve(int cid, double s);
	void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	void reshapeFromPoints( std::vector<Vec3d>& pnts );
	void movePoint(Point p, Vec3d T);
	void moveLineJoint(Point A, Point B, Vec3d deltaA, Vec3d deltaB);

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
	
	// Save and load
	void save(std::ofstream &outF);
	void load(std::ifstream &inF, Vec3d translation, double scaleFactor);

	void buildCage();

	qglviewer::ManipulatedFrame *mf1, *mf2;

public slots:
	void update();

private:
	void moveCurveCenterRanged(int cid, Vec3d delta, int start = 0, int finish = -1);

	QVector<double> origRadius; 
	QVector<double> scales;
	GeneralizedCylinder * gc;
	Skeleton * skel;

	QSurfaceMesh * cage;
	void updateCage();
	GCDeformation * gcd;
	Skinning * skinner;

	double 	deltaScale;
	double cageScale;
	int cageSides;

	DEFORMER deformer;

	bool isFitted;
	std::vector<Point> originalSpine;
};
