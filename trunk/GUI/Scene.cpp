#include <QFileInfo>

#include "Workspace.h"
#include "Scene.h"

#include "QMeshManager.h"
#include "SimpleDraw.h"

Scene::Scene( QString loadObject, QWidget *parent)
{
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

	emit(newSceneCreated());
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

		// get face indices
		Surface_mesh::Face_iterator fit, fend = mesh->faces_end();
		Surface_mesh::Vertex_around_face_circulator fvit, fvend;
		Surface_mesh::Vertex v0, v1, v2;
		for (fit = mesh->faces_begin(); fit != fend; ++fit){
			fvit = fvend = mesh->vertices(fit);
			v0 = fvit;
			v2 = ++fvit;

			do{
				v1 = v2;
				v2 = fvit;
				mesh->triangles.push_back(v0.idx());
				mesh->triangles.push_back(v1.idx());
				mesh->triangles.push_back(v2.idx());
			} while (++fvit != fvend);
		}

		// Create VBO
		vboCollection[activeObjectId] = VBO(mesh->n_vertices(), points.data(), vnormals.data(), vcolors.data(), mesh->triangles);
	}
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

	// Update if needed
	updateVBOs();

	// Draw objects normally
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
		i->render(true);

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

		activeObject()->draw();
		break;

	case UNIQUE_FACES:
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_MULTISAMPLE);

		activeObject()->draw();
		break;
	}
}

void Scene::drawWithNames()
{

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

void Scene::focusOutEvent( QFocusEvent * event )
{
	emit(focusChanged(this));
}

void Scene::setActiveWires( QVector<Wire> newWires )
{
	activeWires = newWires;
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
