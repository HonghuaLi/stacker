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

#include "SymmetryGroup.h"
#include "ConcentricGroup.h"
#include "CoplanarGroup.h"

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
	if(!this->context()->isValid())
	{
		printf("");

	}

	glEnable(GL_MULTISAMPLE);
	glEnable (GL_LINE_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

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
	//if(defCtrl) defCtrl->draw();
	
	if (!isEmpty() && activeObject()->controller)
	{
		Controller * ctrl = activeObject()->controller;

		foreach(Group* g, ctrl->groups)
		{
			g->draw();
		}
	}
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

void Scene::setActiveObject(QSegMesh* newMesh)
{
	if (!this->hasFocus()) return;

	activeMesh = newMesh;

	// Change title of scene
	setWindowTitle(activeMesh->objectName());

	// Set camera
	camera()->setSceneRadius(activeMesh->radius);
	camera()->showEntireScene();

	setGridIsDrawn(false);

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

void Scene::mousePressEvent( QMouseEvent* e )
{
	// Regular behavior
	QGLViewer::mousePressEvent(e);

	if ((e->button() == Qt::RightButton) && (e->modifiers() == Qt::ShiftModifier))
	{
		switch (selectMode){
			case CONTROLLER:
			case CONTROLLER_ELEMENT:
				if(selection.isEmpty()){
					print("Please select some controllers.");
					break;
				}

				Controller * ctrl = activeObject()->controller;

				QMenu menu( this );

				QAction* symmGrp = menu.addAction("Create Symmetry group..");
				QAction* concentricGrp = menu.addAction("Create Concentric group..");
				QAction* coplanGrp = menu.addAction("Create Coplanar group..");
				menu.addSeparator();
				QAction* self1foldSymm = menu.addAction("Create 1-fold Self-Symmetry..");
				QAction* self2foldSymm = menu.addAction("Create 2-fold Self-Symmetry..");

				QAction* action = menu.exec(e->globalPos()); // show menu

				Group* newGroup = NULL;

				int opt = 0;
				if(action == symmGrp)		newGroup = new SymmetryGroup(ctrl, SYMMETRY);
				if(action == concentricGrp) newGroup = new ConcentricGroup(ctrl, CONCENTRIC);
				if(action == coplanGrp)		newGroup = new CoplanarGroup(ctrl, COPLANNAR);
				if(action == self1foldSymm) ctrl->getPrimitive(selection[0])->setSymmetryPlanes(1);
				if(action == self2foldSymm) ctrl->getPrimitive(selection[0])->setSymmetryPlanes(2);

				if(newGroup)
				{
					newGroup->process( ctrl->stringIds(selection) );
					ctrl->groups[newGroup->id] = newGroup;
					emit(groupsChanged());

					print("New group added.");
				}

				break;
		}
	}
}

void Scene::wheelEvent( QWheelEvent* e )
{
	if(selectMode != CONTROLLER_ELEMENT)
		QGLViewer::wheelEvent(e);

	switch (selectMode)
	{
	case CONTROLLER_ELEMENT:
		{
			if(!defCtrl) break;

			double s = 0.1 * (e->delta() / 120.0);
			defCtrl->scaleUp(s);
		}
		break;
	}
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

	// General selection
	if(selected == -1)
		selection.clear();
	else
	{
		if(selection.contains( selected ))
			selection.remove(selection.indexOf(selected));
		else
			selection.push_back(selected); // to start from 0
	}

	// FFD and such deformer
	if(activeDeformer) 
	{
		activeDeformer->postSelection(selected);

		if(selected >= 0)
			setManipulatedFrame( activeDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );
	}

	// Selection mode cases
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
				defCtrl = new QDeformController(activeObject()->controller);

				this->connect(defCtrl, SIGNAL(objectModified()), SLOT(updateActiveObject()));
				this->connect(defCtrl, SIGNAL(objectModified()), sp, SLOT(updateActiveObject()));

				emit(objectInserted());

				setManipulatedFrame( defCtrl->getFrame() );

				Vec3d q = activeObject()->controller->getPrimPartPos();
				Vec p(q.x(), q.y(), q.z());

				manipulatedFrame()->setPosition(p);
			}

			if(selected == -1)
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

	bool currGLcontext = isValid();

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

void Scene::toggleCameraProjection()
{
	if(camera()->type() == Camera::PERSPECTIVE)
		camera()->setType(Camera::ORTHOGRAPHIC);
	else
		camera()->setType(Camera::PERSPECTIVE);

	updateGL();
}
