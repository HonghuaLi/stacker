#include "QMeshManager.h"

QMap<QString, QSegMesh> all_objects;
uint global_id;

QString addNewObject( QString fileName )
{
	QFileInfo fInfo (fileName);
	QString newObjId = fInfo.fileName();
	newObjId.chop(4);

	global_id++;

	newObjId += QString("-%1").arg(global_id);

	// Create a new QSegMesh
	all_objects[ newObjId ] = QSegMesh();
	QSegMesh * newMesh = &all_objects[ newObjId ];

	// Reading QSegMesh
	newMesh->read(fileName);

	return newObjId;
}

QSegMesh * getObject( QString objectId )
{
	if(all_objects.find(objectId) != all_objects.end())
		return &all_objects[objectId];
	else
		return NULL;
}
