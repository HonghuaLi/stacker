#include <QFileInfo>

#include "Workspace.h"
#include "Scene.h"

#include "QMeshManager.h"
#include "SimpleDraw.h"

// Debug OBB code
#include "OBB.h"
OBB * testOBB;

#include "OBB2.h"
OBB2 * testOBB2;

Scene::Scene( QString loadObject, QWidget *parent)
{
	activeFrame = new ManipulatedFrame();
	setManipulatedFrame(activeFrame);

	activeDeformer = NULL;

	// GLViewer options
	setGridIsDrawn();

	// TEXT ON SCREEN
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), SLOT(dequeueLastMessage()));

	// Other events
	connect(this, SIGNAL(objectInserted(QSurfaceMesh *)), SLOT(update()));

	if(loadObject.size())
		insertObject(loadObject);

	displayMessage(tr("New scene created."));

	// Mouse selection window
	this->setSelectRegionHeight(10);
	this->setSelectRegionWidth(10);

	emit(newSceneCreated());

	testOBB = NULL;
	testOBB2 = NULL;
}

void Scene::insertObject( QString fileName )
{
	// Get object name from file path
	QFileInfo fInfo (fileName);

	if(!fInfo.exists()){
		displayMessage(QString("Error: invalid file (%1).").arg(fileName));
		return;
	}

	// Load mesh into memory
	activeObjectId = addNewObject(fileName);

	// Change title of scene
	setWindowTitle(activeObjectId);

	QSurfaceMesh * newMesh = getObject(activeObjectId);

	camera()->setSceneRadius(newMesh->radius);
	camera()->showEntireScene();

	emit(objectInserted(newMesh));
}

void Scene::updateVBOs()
{
	QSurfaceMesh * mesh = activeObject();

	if(mesh && mesh->isReady && vboCollection.find(activeObjectId) == vboCollection.end())
	{
		Surface_mesh::Vertex_property<Point>  points   = mesh->vertex_property<Point>("v:point");
		Surface_mesh::Vertex_property<Point>  vnormals = mesh->vertex_property<Point>("v:normal");
		Surface_mesh::Face_property<Point>    fnormals = mesh->face_property<Point>("f:normal");
		Surface_mesh::Vertex_property<Color>  vcolors  = mesh->vertex_property<Color>("v:color");
		Surface_mesh::Vertex_property<float>  vtex     = mesh->vertex_property<float>("v:tex1D", 0.0);

		// Create VBO
		mesh->fillTrianglesList();
		vboCollection[activeObjectId] = VBO(mesh->n_vertices(), points.data(), vnormals.data(), vcolors.data(), mesh->triangles);
	}
}

void Scene::updateActiveObject()
{
	vboCollection[activeObjectId].setDirty(true);
}

uint Scene::numObjects()
{
	return vboCollection.size();
}

void Scene::init()
{
	// Options
	this->viewMode = VIEW;
	this->selectMode = NONE;
	this->modifyMode = DEFAULT;
	this->specialRenderMode = REGULAR;

	// Background
	setBackgroundColor(backColor = QColor(50,50,60));

	// Lights
	setupLights();

	// Camera
	setupCamera();

	// Material
	float mat_ambient[] = {0.1745f, 0.01175f, 0.01175f, 1.0f};
	float mat_diffuse[] = {0.65f, 0.045f, 0.045f, 1.0f};
	float mat_specular[] = {0.09f, 0.09f, 0.09f, 1.0f};
	float high_shininess = 100;

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, high_shininess);

}

void Scene::setupCamera()
{
	camera()->setUpVector(Vec(0,0,1));
	camera()->setPosition(Vec(2,-2,2));
	camera()->lookAt(Vec());
}

void Scene::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
}

void Scene::draw()
{
	glEnable(GL_MULTISAMPLE);

	// Background color
	this->setBackgroundColor(backColor);

	// Update if needed
	updateVBOs();

	// Draw objects using VBO
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
		i->render();

	if(activeObjectId.size())
		activeObject()->drawDebug();

	// Wires
	foreach(Wire w, activeWires)
	{
		w.draw();
	}

	// Deformer
	if(activeDeformer) activeDeformer->draw();

	// For depth buffer, selection, anything extraordinary
	specialDraw();

	if(testOBB) testOBB->draw();
	if(testOBB2) testOBB2->draw();
}

void Scene::specialDraw()
{
	switch(specialRenderMode)
	{
	case REGULAR:
		break;

	case DEPTH:
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_MULTISAMPLE);

		activeObject()->simpleDraw();
		break;

	case UNIQUE_FACES:
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_MULTISAMPLE);

		activeObject()->drawFacesUnique();
		break;
	}
}

void Scene::drawWithNames()
{
	if(activeDeformer) activeDeformer->drawNames();
}

void Scene::mousePressEvent( QMouseEvent* e )
{
	// Regular behavior
	QGLViewer::mousePressEvent(e);
}

void Scene::mouseReleaseEvent( QMouseEvent* e )
{
	// Regular behavior
	QGLViewer::mouseReleaseEvent(e);
}

void Scene::mouseMoveEvent( QMouseEvent* e )
{
	// Regular behavior
	QGLViewer::mouseMoveEvent(e);
}

void Scene::keyPressEvent( QKeyEvent *e )
{
	if(e->key() == Qt::Key_O)
	{
		testOBB = new OBB();
		testOBB->build_from_mesh( activeObject() );
	}

	if(e->key() == Qt::Key_P)
	{
		testOBB2 = new OBB2( activeObject() );
	}

	// Regular behavior
	QGLViewer::keyPressEvent(e);
}

void Scene::postSelection( const QPoint& point )
{
	// Regular behavior
	//QGLViewer::postSelection(point);

	int selected = selectedName();

	print(QString("Selected %1").arg(selected));

	if(activeDeformer) 
	{
		activeDeformer->postSelection(selected);

		if(selected >= 0)
			setManipulatedFrame( activeDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );
	}
}

void Scene::setViewMode(ViewMode toMode)
{
	viewMode = toMode;
}

void Scene::setSelectMode(SelectMode toMode)
{
	selectMode = toMode;
}

void Scene::setModifyMode(ModifyMode toMode)
{
	modifyMode = toMode;
}

void Scene::postDraw()
{
	if(specialRenderMode != REGULAR)
		return;

	// Textual log messages
	for(int i = 0; i < osdMessages.size(); i++){
		int margin = 20; //px
		int x = margin;
		int y = (i * QFont().pointSize() * 1.5f) + margin;

		qglColor(Qt::white);
		renderText(x, y, osdMessages.at(i));
	}

	QGLViewer::postDraw();

	SimpleDraw::drawCornerAxis(camera()->orientation().inverse().matrix());
}

void Scene::print(QString message, long age)
{
	osdMessages.enqueue(message);
	timer->start(age);
	updateGL();
}

void Scene::dequeueLastMessage()
{
	if(!osdMessages.isEmpty()){
		osdMessages.dequeue();
		updateGL();
	}
}

void Scene::focusInEvent( QFocusEvent * event )
{
	emit(focusChanged(this));
}

void Scene::focusOutEvent( QFocusEvent * event )
{
	emit(focusChanged(this));
}

void Scene::setActiveWires( QVector<Wire> newWires )
{
	activeWires = newWires;
}

void Scene::setActiveDeformer( QFFD * newFFD )
{
	activeDeformer = newFFD;
	updateGL();
}

void* Scene::readBuffer( GLenum format, GLenum type )
{
	int w = width();
	int h = height();

	void * data = NULL;

	switch(format)
	{
	case GL_DEPTH_COMPONENT:
		data = new GLfloat[w*h];
		break;

	case GL_RGBA:
		data = new GLubyte[w*h*4];
		break;
	}

	glReadPixels(0, 0, w, h, format, type, data);

	return data;
}

QSurfaceMesh * Scene::activeObject()
{
	if(all_objects.find(activeObjectId) != all_objects.end())
		return &all_objects[activeObjectId];
	else
		return NULL;
}

VBO * Scene::activeVBO()
{
	if(vboCollection.find(activeObjectId) != vboCollection.end())
		return &vboCollection[activeObjectId];
	else
		return NULL;
}
