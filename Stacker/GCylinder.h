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
	// Build up
	void fit();
	void createGC( std::vector<Point> spinePoints, bool computeRadius = true );
	void buildCage();			// Build the cage from the skeleton for the first time
	void build_up();			// Do all the necessary things

	// Coordinate system
	std::vector<double> getCoordinate( Point v );
	Point				fromCoordinate(std::vector<double> coords);
	void				computeMeshCoordinates();

	// Primitive geometry
	double	volume();
	Point	curveCenter(int cid);
	double	curveRadius(int cid);
	bool	containsPoint(Point p);
	Vec3d	closestPoint(Point p);
	QSurfaceMesh		getGeometry();
	std::vector<Vec3d>	points();
	std::vector<Vec3d>	majorAxis();
	std::vector< std::vector <Vec3d> > getCurves();

	// Hot curves
	int detectHotCurve( Point hotSample);
	int detectHotCurve( QVector<Point> &hotSamples );
	Point	getSelectedCurveCenter();

	// Weights
	double computeWeight( double x, bool useGaussian = false );

	// Reshaping
	void deformMesh();
	void translate( Vec3d &T );
	void moveCurveCenter( int cid, Vec3d T);
	void scaleCurve(int cid, double s);
	void movePoint(Point p, Vec3d T);
	void moveLineJoint(Point A, Point B, Vec3d deltaA, Vec3d deltaB);
	void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T);
	void reshapeFromPoints( std::vector<Vec3d>& pnts );
	void moveCurveCenterRanged(int cid, Vec3d delta, int start = -1, int finish = -1);

	// Draw
	void draw();
	void drawNames(int name, bool isDrawParts = false);

	// Primitive state
	void*	getGeometryState();
	void	setGeometryState( void* );

	// Symmetry
	void	setSymmetryPlanes(int nb_fold);

	// Selecting
	Vec3d	selectedPartPos();
	void	setSelectedPartId( Vec3d normal );
	
	// Save and load
	void save(std::ofstream &outF);
	void load(std::ifstream &inF, Vec3d translation, double scaleFactor);

	qglviewer::ManipulatedFrame *mf1, *mf2;

public slots:
	void update();

private:
	Skeleton * skel;				// Skeleton of GC
	std::vector<Point> originalSpine;

	GeneralizedCylinder * gc;		// The underlying GC
	QVector<double> origRadius;		// Original radius
	QVector<double> scales;			// Scales for each cross section

	DEFORMER deformer;				// Deformer switcher
	Skinning * skinner;				// Skinning deformer
	GCDeformation * gcd;			// Green Coordinates deformer

	QSurfaceMesh * cage;			// Wrapping cage
	double 	deltaScale;
	double cageScale;
	int cageSides;
	void updateCage();				// Update the cage after deforming the skeleton

	bool isFitted;					// Fitting tag
};
