#pragma once
#include <QObject>

#include "Surface_mesh.h"
#include "VBO.h"

class QSurfaceMesh : public QObject, public Surface_mesh
{
	Q_OBJECT

public:
	QSurfaceMesh();


	std::vector<Vertex_iterator> vertex_array;
	std::vector<Face_iterator> face_array;

	void assignVertexIndices();
	void assignFaceIndices();
	std::vector<uint> vertexIndicesAroundFace( uint f_id );
	Point getVertex( uint v_id );



	void compute_bounding_box();
	void moveCenterToOrigin();

	void draw();
	void drawFaceNames();
	void drawFacesUnique();

	void update();
	void set_color_vertices(double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0);
	void setVertexColor( uint v_id, const Color& newColor );
	bool isReady;

	Point bbmin, bbmax, center;
	Scalar radius;

private:
	bool isDirty;

	VBO<Point, Normal_, Color>* vbo;
	Vector<unsigned int> triangles, edges;
};
