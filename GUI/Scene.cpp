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

	displayMessage(tr("New scene created."));
}

void Scene::init()
{
	// Options
	this->viewMode = VIEW;
	this->selectMode = NONE;
	this->modifyMode = DEFAULT;

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

	// Draw objects
	foreach(QSurfaceMesh * mesh, objects)
	{
		mesh->draw();
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
	newMesh->compute_bounding_box();
	newMesh->set_color_vertices();

	camera()->setSceneCenter(Vec(newMesh->center.data()));
	camera()->setSceneRadius(newMesh->radius);
	camera()->showEntireScene();

	newMesh->update();

	// Add to list of scene objects
	objects[ newObjName ] = newMesh;

	emit(objectInserted(newMesh));
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
	update();
}

void Scene::dequeueLastMessage()
{
	if(!osdMessages.isEmpty()){
		osdMessages.dequeue();
		update();
	}
}

void Scene::focusInEvent( QFocusEvent * event )
{
	emit(focusChanged(this));
}

void Scene::doStacking()
{
	print("Start stacking...");
	Offset offset(objects.begin().value());
}
