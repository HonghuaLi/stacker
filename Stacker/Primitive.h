#pragma once

#include "QSurfaceMesh.h"
#include "PrimativeParam.h"
#include "Plane.h"
#include <QVector>


class Primitive
{
public:
	Primitive(QSurfaceMesh* m_mesh, QString newId);

	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit() = 0;
	virtual void computeMeshCoordiantes() = 0;

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh() = 0;
	virtual void deform( PrimitiveParam* params, bool isPermanent = false) = 0;

	// Visualize the primitive and potential actions
	virtual void draw() = 0;
	virtual	void drawNames(int name, bool isDrawParts = false) = 0;

	// Hot curves
	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples ) = 0;
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respect ) = 0;

	// Reshaping
	virtual void translate( Vec3d &T ) {}
	virtual void moveCurveCenter( uint fid, Vec3d T) {}
	virtual void deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T) {}
	virtual bool excludePoints( std::vector< Vec3d >& pnts ) = 0;
	virtual void reshapePart( Vec3d q ) {}
	virtual void reshapeFromCorners( std::vector<Vec3d>& corners) {}
	virtual void movePoint(Point p, Vec3d T){}

	// Primitive coordinate system
	virtual std::vector<double> getCoordinate( Point v ) = 0;
	virtual Point fromCoordinate(std::vector<double> coords) = 0;
	virtual bool containsPoint(Point p){return true;}
	virtual Vec3d closestPoint(Point p){return Point();}

	// Primitive state
	virtual void* getState() = 0;
	virtual void setState( void* ) = 0;

	// Primitive geometry
	virtual std::vector <Vec3d> points() = 0;
	virtual QSurfaceMesh getGeometry() = 0;
	virtual double volume() = 0;
	Vec3d centerPoint();
	virtual std::vector<Vec3d> majorAxis() = 0;
	virtual std::vector < std::vector <Vec3d> > getCurves() = 0;

	// The underlying geometry
	QSurfaceMesh*		m_mesh;			
	QSurfaceMesh* getMesh(){ return m_mesh; }

	// Symmetry, joints, fixed points
	struct Joint{
		Point pos;
		bool frozen;
	};
	QVector<Joint> joints;
	QVector<Point> fixedPoints;
	QVector<Plane> symmPlanes;
	virtual void setSymmetryPlanes(int nb_fold){}
	virtual void addFixedPoint(Point fp){}



	// Helpful for debugging
	std::vector<Vec3d> debugPoints;
	std::vector< std::pair<Vec3d,Vec3d> > debugLines;
	std::vector< std::vector<Vec3d> > debugPoly;
	void drawDebug();

	// Selecting
	bool isSelected;
	int selectedPartId;
	virtual Vec3d selectedPartPos() {return Vec3d(0,0,0);}
	virtual void setSelectedPartId( Vec3d normal ){}

	QString id;
	bool				isHot;			// Is this hot component?
	bool				isDirty;		// Has the underlying geometry been updated?
	bool				isAvailable;	// if propagation affect this primitive
	bool				isFrozen;		// The seed of propagation

};
