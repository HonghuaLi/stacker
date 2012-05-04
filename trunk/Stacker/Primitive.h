#pragma once

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"
#include "GraphicsLibrary/Basic/Plane.h"
#include <QVector>
#include <Eigen/Dense>
#include "ShapeState.h"

enum PrimType{ CUBOID, GCYLINDER, WIRE};


class Primitive
{
public:
	Primitive(QSurfaceMesh* m_mesh, QString newId);

	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit() = 0;
	virtual void computeMeshCoordinates() = 0;

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh() = 0;

	// Visualize the primitive and potential actions
	virtual void draw() = 0;
	virtual	void drawNames(int name, bool isDrawParts = false) = 0;

	// Hot curves
	virtual int detectHotCurve( Point hotSample) = 0;
	virtual int detectHotCurve( QVector<Point> &hotSamples ) = 0;

	// Reshaping
	virtual void translate( Vec3d &T ) = 0;
	virtual void scaleCurve(int cid, double s) = 0;
	virtual void movePoint(Point p, Vec3d T) = 0;
	virtual void moveLineJoint(Point A, Point B, Vec3d deltaA, Vec3d deltaB) = 0;
	virtual void moveCurveCenter( int cid, Vec3d T) = 0;
	virtual void reshape( std::vector<Point>& pnts, std::vector<double>& scales) = 0;
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T) = 0;

	// Primitive coordinate system
	virtual std::vector<double> getCoordinate( Point v ) = 0;
	virtual Point fromCoordinate(std::vector<double> coords) = 0;

	// Primitive state
	virtual PrimitiveState	getState();
	virtual void			setState( PrimitiveState state);
	virtual void*			getGeometryState() = 0;
	virtual void			setGeometryState( void* ) = 0;

	// Primitive geometry
	double	originalVolume;
	virtual double	volume() = 0;
	virtual Vec3d	centerPoint();
	virtual double	curveRadius(int cid) = 0;
	virtual Point	curveCenter(int cid) = 0;
	virtual QSurfaceMesh		getGeometry() = 0;
	virtual std::vector<Vec3d>	majorAxis() = 0;
	virtual std::vector< std::vector<Vec3d> > getCurves() = 0;
	virtual bool containsPoint(Point p) = 0;
	virtual Vec3d closestPoint(Point p) = 0;
	virtual std::vector<Point>	points() = 0;
	virtual std::vector<double> scales() = 0;

	// The underlying geometry
	QSurfaceMesh* m_mesh;			
	QSurfaceMesh* getMesh(){ return m_mesh; }

	// Symmetry, joints, fixed points
	QVector<Point>	fixedPoints;
	QVector<Plane>	symmPlanes;
	virtual void	setSymmetryPlanes(int nb_fold) = 0;
	virtual void	addFixedPoint(Point fp);
	virtual void	addFixedCurve(int cid);

	// Similarity between two primitives
	double similarity(PrimitiveState state1, PrimitiveState state2);

	// Helpful for debugging
	std::vector<Vec3d> debugPoints, debugPoints2, debugPoints3;
	std::vector< std::vector<Vec3d> > debugLines, debugLines2, debugLines3;
	std::vector< std::vector<Vec3d> > debugPoly, debugPoly2;
	void drawDebug();

	// Selecting
	bool	isSelected;
	int		selectedPartId;
	virtual Point	getSelectedCurveCenter() = 0;

	// Save and load
	virtual void save(std::ofstream &outF) = 0;
	virtual void load(std::ifstream &inF, Vec3d translation, double scaleFactor) = 0;

	QString		id;
	PrimType	primType;
	bool		isDraw;

	bool		isHot;			// For hot segment
	bool		isFrozen;		// For propagation
};
