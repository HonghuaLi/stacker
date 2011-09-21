#include "Scene.h"

#include <QFileInfo>

#include "SimpleDraw.h"

Scene::Scene( QWidget *parent ) : QGLViewer(parent)
{
	// GLViewer options
	setGridIsDrawn();

	// TEXT ON SCREEN
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), SLOT(dequeueLastMessage()));

	// Other events
	connect(this, SIGNAL(objectInserted(QSurfaceMesh *)), SLOT(update()));

	activeObject = NULL;

	displayMessage(tr("New scene created."));
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

void Scene::preDraw()
{
	this->makeCurrent();
	QGLViewer::preDraw();
}

void Scene::draw()
{
	glEnable(GL_MULTISAMPLE);

	// Background color
	this->setBackgroundColor(backColor);

	// Draw objects normally
	foreach(QSurfaceMesh * mesh, objects)
	{
		mesh->draw();
	}

	// Wires
	foreach(Wire w, activeWires)
	{
		w.draw();
	}

	// For depth buffer, selection, anything extraordinary
	specialDraw();
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

		foreach(QSurfaceMesh * mesh, objects)
			mesh->draw();
		break;

	case UNIQUE_FACES:
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_MULTISAMPLE);

		foreach(QSurfaceMesh * mesh, objects)
			mesh->drawFacesUnique();
		break;
	}
}

void Scene::drawWithNames()
{
	foreach(QSurfaceMesh * mesh, objects)
	{
		mesh->drawFaceNames();
	}
}

void Scene::insertObject( QString fileName )
{
	// Get object name from file path
	QFileInfo fInfo (fileName);
	QString newObjName = fInfo.fileName();
	newObjName.chop(4);

	// Title of scene
	setWindowTitle(newObjName);

	// Legacy mesh data structure
	/*QMesh * newMesh = new QMesh;
	newMesh->id = qPrintable(newObjName);
	newMesh->loadFromFile(qPrintable(fileName));
	newMesh->normalizeScale();*/

	// Using Surface_mesh library
	QSurfaceMesh * newMesh = new QSurfaceMesh;
	newMesh->read(qPrintable(fileName));
	newMesh->moveCenterToOrigin();
	newMesh->setColorVertices(); // white
	newMesh->assignFaceArray();
	newMesh->assignVertexArray();

	camera()->setSceneRadius(newMesh->radius);
	camera()->showEntireScene();

	newMesh->update();

	// Add to list of scene objects
	objects[ newObjName ] = newMesh;

	activeObject = newMesh;

	emit(objectInserted(newMesh));
}

uint Scene::numObjects()
{
	return objects.size();
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
	// Regular behavior
	QGLViewer::keyPressEvent(e);
}

void Scene::postSelection( const QPoint& point )
{
	// Regular behavior
	//QGLViewer::postSelection(point);

	int selected = selectedName();

	print(QString("Selected %1").arg(selected));
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

void Scene::setActiveWires( QVector<Wire> newWires )
{
	activeWires = newWires.toStdVector();
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
