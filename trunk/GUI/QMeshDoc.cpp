#include "QMeshDoc.h"
#include <QFileDialog>

QMeshDoc::QMeshDoc()
{
	global_id = 0;
}


QMeshDoc::~QMeshDoc()
{
	foreach (const QString id, all_objects.keys())
		delete all_objects.value(id);
}


void QMeshDoc::importObject()
{
	QString fileName = QFileDialog::getOpenFileName(0, "Import Mesh", "", "Mesh Files (*.obj *.off *.stl)"); 

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
	QSegMesh * newMesh = new QSegMesh();
	all_objects[ newObjId ] = newMesh;

	// Reading QSegMesh
	newMesh->read(fileName);

	// Set global ID for the mesh and all its segments
	newMesh->setObjectName(newObjId);

	emit(objectImported(newMesh));
	emit(printMessage(newObjId+" has been imported."));
}

QSegMesh * QMeshDoc::getObject( QString objectId )
{
	if(all_objects.contains(objectId))
		return all_objects[objectId];
	else
		return NULL;
}

void QMeshDoc::deleteObject( QString objectId )
{
	if(all_objects.contains(objectId))
	{
		delete all_objects[objectId];
		all_objects.remove(objectId);
	}
}

void QMeshDoc::exportObject(QSegMesh * mesh)
{
	if(!mesh){ 
		emit(printMessage("Nothing to export."));
		return;
	}

	QString fileName = QFileDialog::getSaveFileName(0, "Export Mesh", "", "Mesh Files (*.obj *.off *.stl)"); 

	// Based on file extension
	QString ext = fileName.right(3).toLower();

	if(ext == "obj")
	{
		mesh->saveObj(fileName);
	}

	emit(printMessage(mesh->objectName() + " has been exported."));
}
