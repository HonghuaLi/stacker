#include <QFileInfo>
#include "Workspace.h"
#include "Scene.h"
#include "SimpleDraw.h"
#include "Controller.h"

// Debugging codes
#include "SkeletonExtract.h"
SkeletonExtract * skelExt;
Skeleton * skel;

#include "GeneralizedCylinder.h"
GeneralizedCylinder * gc;

#include "QDeformController.h"
QDeformController * defCtrl;

Scene::Scene( QWidget *parent)
{
	activeMesh = NULL;

	activeFrame = new ManipulatedFrame();
	setManipulatedFrame(activeFrame);

	activeDeformer = NULL;

	// GLViewer options
	setGridIsDrawn();

	// TEXT ON SCREEN
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), SLOT(dequeueLastMessage()));

	// Update the inserted object
	connect(this, SIGNAL(objectInserted()), SLOT(updateActiveObject()));

	// Mouse selection window
	this->setSelectRegionHeight(10);
	this->setSelectRegionWidth(10);
	displayMessage(tr("New scene created."));

	// Testing
	skel = NULL;
	skelExt = NULL;
	gc = NULL;
	defCtrl = NULL;
}

void Scene::setActiveObject(QSegMesh* newMesh)
{
	if (!this->hasFocus()) return;

	activeMesh = newMesh;

	// Change title of scene
	setWindowTitle(activeMesh->objectName());

	// Set camera
	camera()->setSceneRadius(activeMesh->radius);
	camera()->showEntireScene();

	emit(objectInserted());
}

void Scene::updateVBOs()
{
	QSegMesh * mesh = activeObject();

	if(mesh && mesh->isReady)
	{
		// Create VBO for each segment if needed
		for (int i=0;i<mesh->nbSegments();i++)
		{			
			QSurfaceMesh* seg = mesh->getSegment(i);
			QString objId = seg->objectName();

			if (VBO::isVBOSupported() && !vboCollection.contains(objId))
			{
				Surface_mesh::Vertex_property<Point>  points   = seg->vertex_property<Point>("v:point");
				Surface_mesh::Vertex_property<Point>  vnormals = seg->vertex_property<Point>("v:normal");
				Surface_mesh::Vertex_property<Color>  vcolors  = seg->vertex_property<Color>("v:color");			
				seg->fillTrianglesList();

				// Create VBO 
				vboCollection[objId] = VBO( seg->n_vertices(), points.data(), vnormals.data(), vcolors.data(), seg->triangles );		
			}
		}
	}
}

void Scene::updateActiveObject()
{
	vboCollection.clear();
	updateGL();
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

	// Update VBO if needed
	updateVBOs();

	// Draw objects using VBO
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
		i->render();

	// Fall back
	if(!isEmpty() && vboCollection.isEmpty())
		activeObject()->simpleDraw();

	// Draw the controllers if exist
	if (!isEmpty() && activeObject()->controller)
		activeObject()->controller->draw();

	// Wires
	foreach(Wire w, activeWires)
		w.draw();

	// Deformer
	if(activeDeformer) activeDeformer->draw();

	// Debug
	if (!isEmpty())
		activeObject()->drawDebug();

	// DEBUG
	if(gc) gc->draw();
	if(skel) skel->draw();

	if(defCtrl)
		defCtrl->drawDebug();
}

void Scene::drawWithNames()
{
	if(activeDeformer) activeDeformer->drawNames();

	// Draw the controllers if exist
	if (!isEmpty() && activeObject()->controller)
	{
		bool isDrawParts = false;

		if(this->selectMode == CONTROLLER_ELEMENT)
			isDrawParts = true;

		activeObject()->controller->drawNames(isDrawParts);
	}
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
	if(e->key() == Qt::Key_K)
	{
		skel = new Skeleton();
		skelExt = new SkeletonExtract(activeObject()->getSegment(0));
		skelExt->SaveToSkeleton(skel);

		std::list<int> path = skel->getGraph().GetLargestConnectedPath();

		if(path.size())
		{	
			skel->selectNode(path.front());	
			skel->selectNode(path.back());
			skel->selectEdges(path.front(), path.back());

			gc = new GeneralizedCylinder(skel, activeObject()->getSegment(0));
		}

		activeObject()->setColorVertices(1,1,1,0.5);
		updateActiveObject();
	}

	if(e->key() == Qt::Key_W)
	{
		this->setRenderMode(RENDER_WIREFRAME);
	}

	if(e->key() == Qt::Key_P)
	{
		this->setRenderMode(RENDER_POINT);
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

	switch (selectMode)
	{
	case CONTROLLER:
		if (!isEmpty() && activeObject()->controller)
			activeObject()->controller->select(selected);
		break;

	case CONTROLLER_ELEMENT:
		if (!isEmpty() && activeObject()->controller)
		{
			if(activeObject()->controller->selectPrimitivePart(selected))
			{
				defCtrl = new QDeformController();
				defCtrl->setController(activeObject()->controller);

				setManipulatedFrame( defCtrl->getFrame() );

				Vec3d q = activeObject()->controller->getPrimPartPos();
				Vec p(q.x(), q.y(), q.z());

				manipulatedFrame()->setPosition(p);
			}
			else
			{
				setSelectMode(CONTROLLER);
				setManipulatedFrame( activeFrame );
			}
		}
		break;
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
	QGLViewer::postDraw();

	SimpleDraw::drawCornerAxis(camera()->orientation().inverse().matrix());

	// Textual log messages
	for(int i = 0; i < osdMessages.size(); i++){
		int margin = 20; //px
		int x = margin;
		int y = (i * QFont().pointSize() * 1.5f) + margin;

		qglColor(Qt::white);
		renderText(x, y, osdMessages.at(i));
	}
}

void Scene::print( QString message, long age )
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
	emit(gotFocus(this));
}

void Scene::closeEvent( QCloseEvent * event )
{
	emit(sceneClosed(NULL));
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

QSegMesh * Scene::activeObject()
{
	return activeMesh;
}

bool Scene::isEmpty()
{
	return activeMesh == NULL;
}

void Scene::exportActiveObject()
{
	emit( exportActiveObject(activeObject()) );
}

void Scene::setRenderMode( RENDER_MODE toMode )
{
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
	{
		if(i->render_mode == toMode)
			i->render_mode = RENDER_REGULAR;
		else
			i->render_mode = toMode;
	}

	updateGL();
}
