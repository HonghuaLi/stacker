#include "global.h"
#include "QMeshDoc.h"
#include <QFileDialog>
#include "Workspace.h"
#include "MeshBrowser/MeshBrowserWidget.h"
#include "Stacker/Controller.h"

QMeshDoc::QMeshDoc( QObject * parent ) : QObject(parent)
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
	Workspace * workspace = (Workspace *) parent();
	if(workspace->activeScene == NULL) return;

	// The dialog
	QString fileName = QFileDialog::getOpenFileName(0, "Import Mesh", DEFAULT_FILE_PATH, "Mesh Files (*.obj *.off *.stl)"); 
	
	// Read the file
	QSegMesh * newMesh = importObject(fileName);

	if (newMesh)
	{
		// Save the default path
		DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();

		// Emit signals
		emit(printMessage(newMesh->objectName() + " has been imported."));
		emit(objectImported(newMesh));
	}
}

QSegMesh * QMeshDoc::importObject(QString fileName)
{
	// Get object name from file path
	QFileInfo fInfo (fileName);

	if(!fileName.size() || !fInfo.exists())
	{
		emit(printMessage(QString("Error: invalid file (%1).").arg(fileName)));
		return NULL;
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
	
	// Try to load the controller and groups with the same filename
	// Setup controller file name
	fileName.chop(3);fileName += "ctrl";

	if(QFileInfo(fileName).exists())
	{
		// Load controller
		newMesh->ptr["controller"] = new Controller(newMesh, true, fileName);
		Controller * ctrl = (Controller *)newMesh->ptr["controller"];

		fileName.chop(4);fileName += "grp";
		if(QFileInfo(fileName).exists())
		{
			std::ifstream inF(qPrintable(fileName), std::ios::in);
			ctrl->loadGroups(inF);
			inF.close();
		}
	}
	else
	{
		newMesh->ptr["controller"] = new Controller(newMesh);
	}

	return newMesh;
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

	QString fileName = QFileDialog::getSaveFileName(0, "Export Mesh", DEFAULT_FILE_PATH, "Mesh Files (*.obj *.off *.stl)"); 

	// Based on file extension
	QString ext = fileName.right(3).toLower();

	if(ext == "obj")
	{
		mesh->saveObj(fileName);
	}

	emit(printMessage(mesh->objectName() + " has been exported."));

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();
}

void QMeshDoc::importObjectBrowser()
{
	MeshBrowserWidget * browser = new MeshBrowserWidget;

	if(browser->exec())
		importObject(browser->selectedFile());
}
