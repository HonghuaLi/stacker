#pragma once

#include "Primitive.h"
#include "GraphicsLibrary/Skeleton/GeneralizedCylinder.h"
#include "GraphicsLibrary/Skeleton/Skeleton.h"
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
	void fit();						// Extract skeleton first, then build GC
	void buildGC( std::vector<Point> spinePoints, bool computeRadius = true ); 
									// Build GC from spin points
	void buildCage();				// Build the cage from the skeleton for the first time
	void computeMeshCoordinates();	// Set up the underlying deformer
	void buildUp();				// Include the two step above

	// Update
	void updateGC();			// Update frame, cross sections
	void updateCage();			// If the GC is changed
	void deformMesh();			// Deform the underlying geometry
	void update();				// Include the three steps above

	// Coordinate system
	std::vector<double> getCoordinate( Point v );
	Point				fromCoordinate(std::vector<double> &coords);

	// Primitive geometry
	double	volume();
	Point	curveCenter(int cid);
	double	curveRadius(int cid);
	bool	containsPoint(Point p);
	Vec3d	closestPoint(Point p);
	QSurfaceMesh		getGeometry();
	std::vector<Vec3d>	majorAxis();
	std::vector< std::vector <Vec3d> > getCurves();
	std::vector<Point>	points();
	std::vector<double> scales();
	Point closestProjection( Point p );
	bool atEnd(int dimensions, Point p);

	// Hot curves
	int detectHotCurve( Point hotSample);
	int detectHotCurve( QVector<Point> &hotSamples );

	// Weights
	double computeWeight( double x, bool useGaussian = false );

	// Reshaping
	void reshape( std::vector<Point>& pnts, std::vector<double>& scales);

	// Deformation
	void translate( Vec3d &T );
	void movePoint(Point p, Vec3d T);
	void moveLineJoint(Point A, Point B, Vec3d deltaA, Vec3d deltaB);
	void moveCurveCenter( int cid, Vec3d T);
	void moveCurveCenterRanged(int cid, Vec3d delta, int start = -1, int finish = -1);
	void scaleCurve(int cid, double s);
	void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T); // not used

	// Draw
	void draw();
	void drawNames(int name, bool isDrawCurves = false);

	// Primitive state
	void*	getState();
	void	setState( void* toState);

	// Symmetry
	void	setSymmetryPlanes(int nb_fold);

	// Selecting
	Point	getSelectedCurveCenter();
	
	// Save and load
	void save(std::ofstream &outF);
	void load(std::ifstream &inF, Vec3d translation, double scaleFactor);
	void	serialize( QTextStream &out);
	void	unserialize( QTextStream &in);
public:
	GeneralizedCylinder	*	gc;					// The underlying GC
	
	// \gc is computed by the following \basicGC and additional scales and translations
	GeneralizedCylinder		basicGC;			// The basic GC (only center and radius are used)
	std::vector<double>		curveScales;		// Scales for each cross section of \basicGC
	std::vector<Vec3d>		curveTranslation;	// Translation for each cross section of \basicGC

	DEFORMER		deformer;		// Deformer switcher
	Skinning *		skinner;		// Skinning deformer
	GCDeformation * gcd;			// Green Coordinates deformer

	QSurfaceMesh *	cage;			// Wrapping cage
	double			cageScale;		// Scale to the radius
	int				cageSides;		// Number of sides

	double			deltaScale;		// Used for selecting
};
