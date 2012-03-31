#include "global.h"
#include <QFileInfo>
#include "Workspace.h"
#include "Scene.h"
#include "SubScene.h"
#include "Utility/SimpleDraw.h"

// Stacker stuff
#include "QDeformController.h"
#include "Stacker/SymmetryGroup.h"
#include "Stacker/ConcentricGroup.h"
#include "Stacker/CoplanarGroup.h"

Scene::Scene( QWidget * parent, const QGLWidget * shareWidget, Qt::WFlags flags) : QGLViewer(parent, shareWidget, flags)
{
	activeMesh = NULL;

	activeFrame = new ManipulatedFrame();
	setManipulatedFrame(activeFrame);

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

	// Take focus
	this->setFocus();
	emit(gotFocus(this));

	// Stacker stuff
	sp = NULL;
	activeDeformer = NULL;
	activeVoxelDeformer = NULL;
	isShowStacked = false;
	isDrawOffset = false;

	displayMessage(tr("New scene created."));
}

void Scene::init()
{
	// Options
	this->viewMode = VIEW;
	this->selectMode = SELECT_NONE;
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
	glEnable (GL_LINE_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

	// Background color
	this->setBackgroundColor(backColor);

	// DEBUG
	if (!isEmpty())	activeObject()->drawDebug();
	//if(defCtrl) defCtrl->draw();

	// Update VBO if needed
	updateVBOs();

	// Draw objects using VBO
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
		i->render();

	// Fall back
	if(!isEmpty() && vboCollection.isEmpty())
		activeObject()->simpleDraw();

	// point cloud
	if(!isEmpty() && activeObject()->nbFaces() == 0)
		SimpleDraw::IdentifyPoints(activeObject()->getSegment(0)->clonePoints(), Vec4d(1,1,1,1), 1.0f);

	// == Stacker ==
	// Draw stacking
	if(!isEmpty() && isShowStacked)
	{
		int stackCount = 3;

		double O_max = activeObject()->val["O_max"];
		double S = activeObject()->val["stackability"];

		Vec3d stackDirection = Vec3d(0., 0., 1.);
		Vec3d delta = O_max * stackDirection;
		double theta = activeObject()->val["theta"];
		double phi = activeObject()->val["phi"];

		double tranX = activeObject()->val["tranX"];
		double tranY = activeObject()->val["tranY"];
		double tranZ = activeObject()->val["tranZ"];

		Vec3d shift (tranX, tranY, tranZ);

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
	if (!isEmpty())
	{
		Controller * ctrl = ((Controller *)activeObject()->ptr["controller"]);
		if (ctrl){
			foreach(Group* g, ctrl->groups)
				g->draw();
		}

		// deformer
		if(activeDeformer) activeDeformer->draw();
		if(activeVoxelDeformer) activeVoxelDeformer->draw();

		// Draw the controllers if exist
		if (ctrl) ctrl->draw();

		// DEBUG
		activeObject()->drawDebug();

		// Suggestions
		Vec p = camera()->position();
		Vec3d pos(p.x, p.y, p.z);
		double scaling = 0.05;//pos.norm() / 100.0;
		pos.normalize();	
		foreach(EditSuggestion sg, suggestions)
			sg.draw(scaling);
	}
}

void Scene::drawWithNames()
{
	if(activeDeformer) activeDeformer->drawNames();
	if(activeVoxelDeformer) activeVoxelDeformer->drawNames();

	// Draw the controllers if exist
	if (!isEmpty() && activeObject()->ptr["controller"] && 
		(selectMode == CONTROLLER || selectMode ==CONTROLLER_ELEMENT))
	{
		bool isDrawParts = false;

		if(this->selectMode == CONTROLLER_ELEMENT)
			isDrawParts = true;

		((Controller *)activeObject()->ptr["controller"])->drawNames(isDrawParts);
	}
}

void Scene::setActiveObject(QSegMesh* newMesh)
{
	//if (!this->hasFocus()) return;
	activeMesh = newMesh;

	activeMesh->ptr["controller"] = NULL;

	// Change title of scene
	setWindowTitle(activeMesh->objectName());

	// Set camera
	resetView();

	emit(gotFocus(this));
	emit(objectInserted());
}

void Scene::resetView()
{
	camera()->setSceneRadius(activeMesh->radius);
	camera()->showEntireScene();
}

void Scene::updateVBOs()
{
	QSegMesh * mesh = activeObject();

	if(mesh && mesh->isReady)
	{
		// Create VBO for each segment if needed
		for (int i=0;i<(int)mesh->nbSegments();i++)
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

		QAction* action = NULL;

		/*switch (selectMode){
			case CONTROLLER:
			case CONTROLLER_ELEMENT:
				if(selection.isEmpty()){
					print("Please select some controllers.");
					break;
				}*/

				Controller * ctrl = (Controller *)activeObject()->ptr["controller"];

				QAction* symmGrp = menu.addAction("Create Symmetry group..");
				QAction* concentricGrp = menu.addAction("Create Concentric group..");
				QAction* coplanGrp = menu.addAction("Create Coplanar group..");
				menu.addSeparator();
				QAction* self1foldSymm = menu.addAction("Create 1-fold Self-Symmetry..");
				QAction* self2foldSymm = menu.addAction("Create 2-fold Self-Symmetry..");
				QAction* selfRotSymm = menu.addAction("Create Rotational Self-Symmetry..");
				menu.addSeparator();
				QAction* addFixedPoint = menu.addAction("Add fixed Point");

				// == Deformer stuff ===
				menu.addSeparator();
				QAction* selectFFD = menu.addAction("FFD");
				QAction* selectVoxelDeformer = menu.addAction("Voxel Deformer");
				// == end deformer ===

				action = menu.exec(e->globalPos()); // show menu

				// Selections
				if(action == selectControllerAction)	setSelectMode(CONTROLLER);
				if(action == selectCurveAction)			setSelectMode(CONTROLLER_ELEMENT);
				if(action == selectFFD)					setSelectMode(FFD_DEFORMER);
				if(action == selectVoxelDeformer)		setSelectMode(VOXEL_DEFORMER);

				Group* newGroup = NULL;

				int opt = 0;
				if(action == symmGrp)		newGroup = new SymmetryGroup(ctrl, SYMMETRY);
				if(action == concentricGrp) newGroup = new ConcentricGroup(ctrl, CONCENTRIC);
				if(action == coplanGrp)		newGroup = new CoplanarGroup(ctrl, COPLANNAR);

				if(action == self1foldSymm) ctrl->getSelectedPrimitive()->setSymmetryPlanes(1);
				if(action == self2foldSymm) ctrl->getSelectedPrimitive()->setSymmetryPlanes(2);
				if(action == selfRotSymm)	ctrl->getSelectedPrimitive()->isRotationalSymmetry = true;

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

				/*break;
		}*/
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

	// FFD and such deformers
	if(activeDeformer && selectMode == FFD_DEFORMER){
		activeDeformer->postSelection(selected);

		if(selected >= 0)
			setManipulatedFrame( activeDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );
	}

	if(activeVoxelDeformer && selectMode == VOXEL_DEFORMER){
		if(selected >= 0)
			setManipulatedFrame( activeVoxelDeformer->getQControlPoint(selected) );
		else
			setManipulatedFrame( activeFrame );

		activeVoxelDeformer->select(selected);
	}

	Controller * ctrl = ((Controller *)activeObject()->ptr["controller"]);

	// Selection mode cases
	switch (selectMode)
	{
	case CONTROLLER:
		if (!isEmpty() && ctrl)
			ctrl->select(selected);
		break;

	case CONTROLLER_ELEMENT:
		if (!isEmpty() && ctrl)
		{
			if(ctrl->selectPrimitivePart(selected))
			{
				defCtrl = new QDeformController(ctrl);

				this->connect(defCtrl, SIGNAL(objectModified()), SLOT(updateActiveObject()));
				this->connect(defCtrl, SIGNAL(objectModified()), sp, SLOT(updateActiveObject()));

				emit(objectInserted());

				setManipulatedFrame( defCtrl->getFrame() );

				Vec3d q = ctrl->getPrimPartPos();
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
	//print(QString("got! (%1)").arg(event->reason()));
	emit(gotFocus(this));
}

void Scene::focusOutEvent( QFocusEvent * event )
{
	if(event->reason() != Qt::PopupFocusReason)
		emit(lostFocus(this));
}

void Scene::closeEvent( QCloseEvent * event )
{
	// Alert others
	this->activeMesh = NULL;
	emit(objectInserted());

	emit(sceneClosed(this));

	event->accept();
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

void Scene::setActiveDeformer( QFFD * newFFD )
{
	activeDeformer = newFFD;
	updateGL();
	this->setSelectMode(FFD_DEFORMER);

	this->connect(newFFD, SIGNAL(meshDeformed()), sp, SLOT(updateActiveObject()));
}

void Scene::setActiveVoxelDeformer( VoxelDeformer * newFFD )
{
	activeVoxelDeformer = newFFD;
	updateGL();
	this->setSelectMode(VOXEL_DEFORMER);

	this->connect(activeVoxelDeformer, SIGNAL(meshDeformed()), sp, SLOT(updateActiveObject()));
}
