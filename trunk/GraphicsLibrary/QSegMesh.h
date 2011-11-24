#pragma once
#include <QObject>
#include <QString>
#include "QSurfaceMesh.h"
#include <vector>

class Controller;

class QSegMesh : public QObject
{
	Q_OBJECT

public:
	QSegMesh();
	QSegMesh(const QSegMesh& from);
	QSegMesh& operator=(const QSegMesh& rhs);

	// Face, vertex
	uint nbVertices();
	uint nbFaces();
	std::vector<uint> vertexIndicesAroundFace(uint fid);
	Point getVertexPos( uint vid );
	void setVertexColor( uint vid, const Color& newColor );

	// Get segment
	QSurfaceMesh* operator [] (uint i);
	QSurfaceMesh* getSegment(uint i);
	std::vector<QSurfaceMesh*> getSegments();
	uint nbSegments();
	uint vertexInSegment( uint vid );

	// Draw
	void simpleDraw();
	void drawFacesUnique();

	// Load the mesh from file
	void read(QString fileName);

	// Save the mesh
	void saveObj(QString fileName);

	// Build up the mesh
	void build_up();
	void moveCenterToOrigin();
	void computeBoundingBox();
	void setColorVertices( double r = 1.0, double g = 1.0, double b = 1.0, double a = 1.0);
	void update_face_normals();
	void update_vertex_normals();
	void normalize();

	// Properties
	bool isReady;
	Point bbmin, bbmax, center;
	Scalar radius;
	Scalar scaleFactor;

	// Set global unique name for this and all its segments
	void setObjectName(const QString &name);

	// Controller
	Controller* controller;

	// Stackability
	Scalar O_max;
	Scalar stackability;

private:
	std::vector<QSurfaceMesh*> segment;
	void global2local_fid(uint fid, uint& sid, uint& fid_local);
	void global2local_vid(uint vid, uint& sid, uint& vid_local);

	// This is useful for segmented OBJs
	void checkObjSegmentation ( QString fileName, QString segFilename);
};
