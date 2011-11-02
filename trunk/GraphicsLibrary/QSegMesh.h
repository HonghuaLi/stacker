#pragma once
#include <QObject>
#include "QSurfaceMesh.h"
#include <vector>

class QSegMesh : public QObject
{
	Q_OBJECT

public:
	QSegMesh();
	QSegMesh(const QSegMesh& from);
	QSegMesh& operator=(const QSegMesh& rhs);


	// Get segment
	QSurfaceMesh* operator [] (int i);
	QSurfaceMesh* getSegment(int i);
	int nbSegments();

	// Draw
	void simpleDraw();
	void drawFacesUnique();

	// Load the mesh from file
	void read(QString fileName);

	// Build up the mesh
	void build_up();
	void moveCenterToOrigin();
	void computeBoundingBox();
	void setColorVertices();

	// Form Mesh
	void update_face_normals();
	void update_vertex_normals();

	// Properties
	bool isReady;
	Point bbmin, bbmax, center;
	Scalar radius;

private:
	std::vector<QSurfaceMesh*> segment;
};


