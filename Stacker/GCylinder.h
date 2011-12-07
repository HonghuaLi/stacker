#pragma once

#include "Primitive.h"
#include "GeneralizedCylinder.h"
#include "GCDeformation.h"
#include "QGLViewer/manipulatedFrame.h"

class GCylinder : public QObject, public Primitive
{
	Q_OBJECT

public:
	GCylinder(QSurfaceMesh* segment);

public:
	virtual void fit();
	virtual void deform( PrimitiveParam* params, bool isPermanent = false);
	virtual void deformMesh();
	virtual void draw();
	virtual	void drawNames(bool isDrawParts = false);

	virtual double volume();
	virtual std::vector <Vec3d> points();

	virtual uint detectHotCurve( std::vector< Vec3d > &hotSamples );
	virtual void translateCurve( uint cid, Vec3d T, uint sid_respect );

	void translate(Vec3d T);

	qglviewer::ManipulatedFrame *mf1, *mf2;

public slots:
	void update();

private:
	GeneralizedCylinder * gc;
	Skeleton * skel;

	QSurfaceMesh * cage;
	void buildCage();
	void updateCage();
	GCDeformation * gcd;
};
