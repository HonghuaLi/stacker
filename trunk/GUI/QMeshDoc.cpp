#include "QMeshDoc.h"
#include <QFileInfo>


QMeshDoc::QMeshDoc()
{
	global_id = 0;
}

void QMeshDoc::importObject( QString fileName )
{
	// Get object name from file path
	QFileInfo fInfo (fileName);

	if(!fInfo.exists())
	{
		emit(printMessage(QString("Error: invalid file (%1).").arg(fileName)));
		return;
	}

	QString newObjId = fInfo.fileName();
	newObjId.chop(4);
	global_id++;
	newObjId += QString("-%1").arg(global_id);

	// Create a new QSegMesh
	all_objects[ newObjId ] = QSegMesh();
	QSegMesh * newMesh = &all_objects[ newObjId ];

	// Reading QSegMesh
	newMesh->read(fileName);
	newMesh->setObjectName(newObjId);

	emit(objectImported(newMesh));
}

QSegMesh * QMeshDoc::getObject( QString objectId )
{
	if(all_objects.find(objectId) != all_objects.end())
		return &all_objects[objectId];
	else
		return NULL;
}
