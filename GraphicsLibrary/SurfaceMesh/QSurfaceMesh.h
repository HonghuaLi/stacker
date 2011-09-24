#pragma once
#include <QObject>
#include "Macros.h"

#include "Surface_mesh.h"

class QSurfaceMesh : public QObject, public Surface_mesh
{
	Q_OBJECT

public:
	QSurfaceMesh();
	QSurfaceMesh(const QSurfaceMesh& from);
	QSurfaceMesh& operator=(const QSurfaceMesh& rhs);

	std::vector<Vertex_iterator> vertex_array;
	std::vector<Face_iterator> face_array;

	void assignVertexArray();
	void assignFaceArray();

	std::vector<uint> vertexIndicesAroundFace( uint f_id );
	
	Point getVertexPos( uint v_id );
	Point getVertexPos( const Vertex & v );

	void computeBoundingBox();
	void moveCenterToOrigin();

	double getAverageEdgeLength();
	double averageEdgeLength;

	void draw();
	void drawFaceNames();
	void drawFacesUnique();

	void setColorVertices(double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0);
	void setVertexColor( uint v_id, const Color& newColor );

	void collectEnoughRings(Vertex v, const size_t min_nb, std::vector <Vertex>& all);
	void resetVistedVertices(std::vector <Vertex>& all);
	void resetVistedVertices(uint toState = false); // for entire mesh

	// Properties
	bool isReady;
	Point bbmin, bbmax, center;
	Scalar radius;

	// Debug items
	std::vector<Point> debug_points, debug_points2, debug_points3;
	std::vector< std::vector<Point> > debug_lines, debug_lines2, debug_lines3;

	// Face Utility
	Vec3d fn( Face f );
	Vec3d faceCenter ( Face f );
	double faceArea( Face f );
	std::vector<Vec3d> pointsFace( Face f );
	Vec3d getBaryFace( Face f, double U, double V );

	std::vector<unsigned int> triangles, edges;

private:
	bool isDirty;
};
