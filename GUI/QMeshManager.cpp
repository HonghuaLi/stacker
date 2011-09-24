#include "QMeshManager.h"

QMap<QString, QSurfaceMesh> all_objects;

void addNewObject( QString fileName )
{
	QFileInfo fInfo (fileName);
	QString newObjName = fInfo.fileName();
	newObjName.chop(4);

	// Create a new QSurfaceMesh
	all_objects[ newObjName ] = QSurfaceMesh();
	QSurfaceMesh * newMesh = &all_objects[ newObjName ];

	// Using Surface_mesh library
	newMesh->read(qPrintable(fileName));

	// Default pre-processing
	newMesh->moveCenterToOrigin();
	newMesh->computeBoundingBox();
	newMesh->setColorVertices(); // white
	newMesh->assignFaceArray();
	newMesh->assignVertexArray();

	// From Surface_mesh
	newMesh->update_face_normals();
	newMesh->update_vertex_normals();
}

QSurfaceMesh * getObject( QString objectId )
{
	if(all_objects.find(objectId) != all_objects.end())
		return &all_objects[objectId];
	else
		return NULL;
}
