#pragma once
#include <QObject>

#include "Surface_mesh.h"
#include "VBO.h"

class QSurfaceMesh : public QObject, public Surface_mesh
{
	Q_OBJECT

public:
	QSurfaceMesh();

	void compute_bounding_box();

	void draw();
	void drawFaceNames();

	void update();
	void set_color_vertices(double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0);
	bool isReady;

	Point bbmin, bbmax, center;
	Scalar radius;

private:
	bool isDirty;

	VBO<Point, Normal_, Color>* vbo;
	Vector<unsigned int> triangles, edges;
};
