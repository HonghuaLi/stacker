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

#include "SubScene.h"


Scene::Scene( QWidget *parent)
{
	activeMesh = NULL;

	activeFrame = new ManipulatedFrame();
	setManipulatedFrame(activeFrame);

	activeDeformer = NULL;
	activeVoxelDeformer = NULL;

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

	isShowStacked = false;
	isDrawOffset = false;

	sp = NULL;

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
	setBackgroundColor(backColor = QColor(226,226,226));

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

	// Draw stacking
	if(!isEmpty() && isShowStacked)
	{
		int stackCount = 3;

		double O_max = activeObject()->O_max;
		double S = activeObject()->stackability;

		Vec3d stackDirection = Vec3d(0., 0., 1.);
		Vec3d delta = O_max * stackDirection;
		double theta = activeObject()->theta;
		double phi = activeObject()->phi;
		
		Vec3d shift = activeObject()->translation;

		glPushMatrix();
		glTranslated(delta[0],delta[1],delta[2]);

		for(int i = 1; i < stackCount; i++)
		{
			glColor4dv(Color(0.45,0.72,0.43,0.8));
			
			glEnable(GL_CULL_FACE);
			activeObject()->simpleDraw(false);
			glDisable(GL_CULL_FACE);

			glTranslated(delta[0],delta[1],delta[2]);
			glRotated(theta, 1, 0, 0);
			glRotated(phi, 0, 0, 1);
			glTranslated(shift[0], shift[1], shift[2]);
		}

		glPopMatrix();
	}

	// Draw groups
	if (!isEmpty() && activeObject()->controller)
	{
		Controller * ctrl = activeObject()->controller;

		foreach(Group* g, ctrl->groups)
		{
			g->draw();
		}
	}

	// Deformers
	if(activeDeformer) activeDeformer->draw();
	if(activeVoxelDeformer) activeVoxelDeformer->draw();

	// Wires
	foreach(Wire w, activeWires)
		w.draw();

	// Draw the controllers if exist
	if (!isEmpty() && activeObject()->controller)
		activeObject()->controller->draw();

	// DEBUG
	if (!isEmpty())	activeObject()->drawDebug();
	//if(defCtrl) defCtrl->draw();

	// Suggestions
	Vec p = camera()->position();
	Vec3d pos(p.x, p.y, p.z);
	double scaling = 0.05;//pos.norm() / 100.0;
	pos.normalize();	
	foreach(EditSuggestion sg, suggestions)
		sg.draw(scaling);
}

void Scene::drawWithNames()
{
	if(activeDeformer) activeDeformer->drawNames();
	if(activeVoxelDeformer) 
	{
		activeVoxelDeformer->drawNames();
		return;
	}

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

	// Clear
	suggestions.clear();

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
		QMenu menu( this );

		QAction* selectControllerAction = menu.addAction("Select controller");
		QAction* selectCurveAction = menu.addAction("Select curve");
		menu.addSeparator();

		switch (selectMode){
			case CONTROLLER:
			case CONTROLLER_ELEMENT:
				if(selection.isEmpty()){
					print("Please select some controllers.");
					break;
				}

				Controller * ctrl = activeObject()->controller;

				QAction* symmGrp = menu.addAction("Create Symmetry group..");
				QAction* concentricGrp = menu.addAction("Create Concentric group..");
				QAction* coplanGrp = menu.addAction("Create Coplanar group..");
				menu.addSeparator();
				QAction* self1foldSymm = menu.addAction("Create 1-fold Self-Symmetry..");
				QAction* self2foldSymm = menu.addAction("Create 2-fold Self-Symmetry..");
				QAction* selfRotSymm = menu.addAction("Create Rotational Self-Symmetry..");
				menu.addSeparator();
				QAction* addFixedPoint = menu.addAction("Add fixed Point");

				QAction* action = menu.exec(e->globalPos()); // show menu

				Group* newGroup = NULL;

				int opt = 0;
				if(action == symmGrp)		newGroup = new SymmetryGroup(ctrl, SYMMETRY);
				if(action == concentricGrp) newGroup = new ConcentricGroup(ctrl, CONCENTRIC);
				if(action == coplanGrp)		newGroup = new CoplanarGroup(ctrl, COPLANNAR);

				if(action == self1foldSymm) ctrl->getSelectedPrimitive()->setSymmetryPlanes(1);
				if(action == self2foldSymm) ctrl->getSelectedPrimitive()->setSymmetryPlanes(2);
				if(action == selfRotSymm)	ctrl->getSelectedPrimitive()->isRotationalSymmetry = true;

				if(action == selectControllerAction)	setSelectMode(CONTROLLER);
				if(action == selectCurveAction)			setSelectMode(CONTROLLER_ELEMENT);

				if(action == addFixedPoint)	
				{
					Primitive * prim = ctrl->getSelectedPrimitive();
					Point pos = prim->getSelectedCurveCenter();
					prim->addFixedPoint(pos);
				}

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
	if(e->key() == Qt::Key_R)
	{
		updateActiveObject();
	}

	if(e->key() == Qt::Key_W)
	{
		this->setRenderMode(RENDER_WIREFRAME);
	}

	if(e->key() == Qt::Key_O)
	{
		isDrawOffset = !isDrawOffset;
	}

	if(e->key() == Qt::Key_P)
	{
		this->setRenderMode(RENDER_POINT);
	}

	if(e->key() == Qt::Key_I)
	{
		addSubScene(Min(300,width() * (1.0/Max(1,subScenes.size()))));
	}

	if(e->key() == Qt::Key_U)
	{
		isShowStacked = !isShowStacked;
	}

	// Regular behavior
	QGLViewer::keyPressEvent(e);

	updateGL();
}

void Scene::addSubScene(int scene_width)
{
	int offsetX = 0;
	int offsetY = height() - scene_width;

	foreach(SubScene * s, subScenes){
		offsetX += s->width;
		//offsetY += s->height;
	}

	subScenes.push_back(new SubScene(this, offsetX, offsetY, scene_width, scene_width));

	resizeSubScenes();
}

void Scene::resizeEvent( QResizeEvent * event )
{
	QGLViewer::resizeEvent(event);
	resizeSubScenes();
}

void Scene::resizeSubScenes()
{
	int scene_width = Min(300,width() * (1.0/Max(1,subScenes.size())));

	int offsetX = 0;
	int offsetY = height() - scene_width;

	foreach(SubScene * s, subScenes){
		s->x = offsetX;
		s->y = offsetY;
		s->width = scene_width;
		s->height = scene_width;

		offsetX += s->width;
		//offsetY += s->height;
	}
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
	if(activeDeformer){
		activeDeformer->postSelection(selected);

		if(selected >= 0)
			setManipulatedFrame( activeDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );
	}

	if(activeVoxelDeformer){
		if(selected >= 0)
			setManipulatedFrame( activeVoxelDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );

		activeVoxelDeformer->select(selected);
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

	foreach(SubScene * s, subScenes){
		if(s->contains(Vec2i(point.x(), point.y())))
		{
			s->isSelected = ! s->isSelected;
		}
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
	// Textual log messages
	for(int i = 0; i < osdMessages.size(); i++){
		int margin = 20; //px
		int x = margin;
		int y = (i * QFont().pointSize() * 1.5f) + margin;

		qglColor(Qt::white);
		renderText(x, y, osdMessages.at(i));
	}

	// Draw offset function
	if(isDrawOffset && activeObject() && activeObject()->data2D.contains("offset"))
	{
		setupSubViewport(0,0, 400, 400);

		// Draw graph
		glTranslated(-0.5, -0.5, 0);
		SimpleDraw::DrawGraph2D(activeObject()->data2D["offset"]);

		// black line
		Vec3d p(0, 0, 0); Vec3d q = p + Vec3d(0,0,1.0);
		SimpleDraw::IdentifyLine(p,q, Color(0,0,0,1), false, 1.0);

		endSubViewport();
	}

	if(activeObject() && activeObject()->isReady && activeObject()->controller)
	{
		Controller *ctrl = activeObject()->controller;

		ShapeState oldState = ctrl->getShapeState();

		PQShapeShateLessEnergy solutionsCopy = sp->activeOffset->suggestSolutions;
		
		for (int i = 0; i < Min(subScenes.size(), solutionsCopy.size()); i++)
		{
			SubScene * s = subScenes[i];

			s->draw();

			s->caption = QString::number(i);

			// BEGIN
			setupSubViewport(s->x, s->y, s->width, s->height);
			glClear(GL_DEPTH_BUFFER_BIT);

			ShapeState sln = solutionsCopy.top();
			solutionsCopy.pop();

			ctrl->setShapeState(sln);

			// Draw shape in current state
			glColor3dv(Color(0.96, 0.71, 0.30, 1));
			activeObject()->simpleDraw(false);

			// END
			endSubViewport();
		}

		// Restore state;
		ctrl->setShapeState(oldState);
	}

	// To avoid aliasing in text
	for (int i = 0; i < subScenes.size(); i++)
		subScenes[i]->postDraw();

	QGLViewer::postDraw();

	//SimpleDraw::drawCornerAxis(camera()->orientation().inverse().matrix());
}

void Scene::setupSubViewport( int x, int y, int w, int h )
{
	glViewport( x, height() - y - h, w, h);
	glScissor( x, height() - y - h, w, h);

	glClear(GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);glPushMatrix();	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();glLoadIdentity();glMultMatrixd(camera()->orientation().inverse().matrix());
}

void Scene::endSubViewport()
{
	glMatrixMode(GL_PROJECTION);glPopMatrix();
	glMatrixMode(GL_MODELVIEW);glPopMatrix();

	glViewport(0,0,width(),height());
	glScissor(0,0,width(),height());
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

void Scene::setActiveVoxelDeformer( VoxelDeformer * newFFD )
{
	activeVoxelDeformer = newFFD;

	this->connect(activeVoxelDeformer, SIGNAL(meshDeformed()), sp, SLOT(updateActiveObject()));

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
