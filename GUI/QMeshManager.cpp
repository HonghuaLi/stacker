#include "QMeshManager.h"

QMap<QString, QSurfaceMesh> all_objects;
uint global_id;

QString addNewObject( QString fileName )
{
	QFileInfo fInfo (fileName);
	QString newObjId = fInfo.fileName();
	newObjId.chop(4);

	global_id++;

	newObjId += QString("-%1").arg(global_id);

	// Create a new QSurfaceMesh
	all_objects[ newObjId ] = QSurfaceMesh();
	QSurfaceMesh * newMesh = &all_objects[ newObjId ];

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

	newMesh->isReady = true;

	return newObjId;
}

QSurfaceMesh * getObject( QString objectId )
{
	if(all_objects.find(objectId) != all_objects.end())
		return &all_objects[objectId];
	else
		return NULL;
}
