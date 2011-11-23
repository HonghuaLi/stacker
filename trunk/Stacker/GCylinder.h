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
	virtual void deformMesh();
	virtual void draw();

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