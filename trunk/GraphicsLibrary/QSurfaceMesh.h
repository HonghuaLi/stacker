#pragma once
#include <QObject>
#include <QString>
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
	Point getVertexPos( const Vertex v );
	Vertex getVertex( uint v_id);
	Face getFace( uint f_id);

	std::set<uint> faceIndicesAroundVertex(const Vertex& v);
	std::set<uint> vertexIndicesAroundVertex(const Vertex& v);

	void computeBoundingBox();
	void moveCenterToOrigin();

	double getAverageEdgeLength();
	double averageEdgeLength;

	double volume();
	double normalize();
	double scalingFactor;

	double closestDistancePointVertices(const Point & p);

	std::vector<Point> clonePoints();
	void setFromPoints(const std::vector<Point>& fromPoints);
	std::vector< std::pair<Point, Point> > cloneEdges();

	std::vector<Normal> cloneFaceNormals();

	void drawFaceNames();
	void drawFacesUnique(uint offset);
	void drawDebug();
	void simpleDraw();

	void setColorVertices(double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0);
	void setColorVertices( Vec4d color );
	void setVertexColor( uint v_id, const Color& newColor );

	void collectEnoughRings(Vertex v, const size_t min_nb, std::vector <Vertex>& all);
	void resetVistedVertices(std::vector <Vertex>& all);
	void resetVistedVertices(uint toState = false); // for entire mesh

	// Load the mesh from file
	void read(const std::string& filename);

	// Build up
	void buildUp();

	// Properties
	bool isReady;
	Point bbmin, bbmax, center;
	Scalar radius;

	// Debug items
	std::vector<Point> debug_points, debug_points2, debug_points3;
	std::vector< std::vector<Point> > debug_lines, debug_lines2, debug_lines3;
	bool isDrawBB;

	// Face Utility
	Vec3d fn( Face f );
	Vec3d faceCenter ( Face f );
	double faceArea( Face f );
	std::vector<Vec3d> facePoints( Face f );
	std::vector<uint> faceVerts( const Face& f );
	Vec3d getBaryFace( Face f, double U, double V );
	void fillTrianglesList();
	std::vector<unsigned int> cloneTriangleIndices();
	std::vector<unsigned int> triangles, edges;

	QString id;
	bool isVisible;

private:
	bool isDirty;
};
