#include "StackerPreview.h"
#include "StackerPanel.h"


StackerPreview::StackerPreview( QWidget * parent ) : QGLViewer (parent)
{
	// Default size of the preview window
	this->resize(200,200);

	// No active scene when initializing
	this->activeScene = NULL;

	// Stacking direction is always along z axis
	stackDirection = Vec3d(0., 0., 1.);
}

void StackerPreview::init()
{
	glEnable(GL_MULTISAMPLE);

	setBackgroundColor(backColor = palette().color(QPalette::Window));

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

void StackerPreview::setupCamera()
{
	camera()->setUpVector(Vec(0,0,1));
	camera()->setPosition(Vec(2,-2,2));
	camera()->lookAt(Vec());
}

void StackerPreview::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
}

void StackerPreview::preDraw()
{
	QGLViewer::preDraw();
}

void StackerPreview::draw()
{
	// Anti aliasing 
	glEnable(GL_MULTISAMPLE);

	// Background
	setBackgroundColor(backColor);

	if (!activeScene || activeScene->isEmpty()) return;

	// Update VBO is needed
	updateVBOs();

	int stackCount = 3;
	double O_max = activeObject()->O_max;
	double S = activeObject()->stackability;

	Vec3d delta = O_max * stackDirection;

	glPushMatrix();

	for(int i = 0; i < stackCount; i++)
	{
		// Draw object using VBO
		for (QMap<QString, VBO>::iterator itr = vboCollection.begin(); itr != vboCollection.end(); ++itr)
			itr->render();

		// Fall back
		if(vboCollection.isEmpty() && activeObject())
			activeObject()->simpleDraw();

		glTranslated(delta[0],delta[1],delta[2]);
	}

	glPopMatrix();
}

void StackerPreview::postDraw()
{
	QGLViewer::postDraw();

	qglColor(Qt::white);

	if(activeObject())
	{
		QString message = QString("O_max = %1; S = %2").arg(activeObject()->O_max).arg(
			activeObject()->stackability);
		renderText(0, this->height()- 10, message);
	}
}

void StackerPreview::setActiveScene( Scene * toScene )
{
	this->activeScene = toScene;

	updateActiveObject();
}

void StackerPreview::updateVBOs()
{
	QSegMesh * mesh = activeScene->activeObject();

	if(mesh && mesh->isReady)
	{
		// Create VBO for each segment if needed
		for (int i=0;i<mesh->nbSegments();i++)
		{			
			QSurfaceMesh* seg = mesh->getSegment(i);
			QString objId = seg->objectName();

			if (VBO::isVBOSupported() && vboCollection.find(objId) == vboCollection.end())
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

void StackerPreview::updateActiveObject()
{
	if(activeScene && !activeScene->isEmpty())
	{
		camera()->setSceneRadius(activeScene->activeObject()->radius);
		camera()->showEntireScene();
		camera()->addKeyFrameToPath(0);
		camera()->setSceneRadius(activeScene->activeObject()->radius * 4);
		camera()->showEntireScene();
		camera()->resetPath(0);
	}
	
	vboCollection.clear();
	updateGL();
}

QSegMesh* StackerPreview::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
	else
		return NULL;
}

void StackerPreview::keyPressEvent( QKeyEvent *e )
{
	if(e->key() == Qt::Key_W)
	{
		this->setRenderMode(RENDER_WIREFRAME);
	}

	if(e->key() == Qt::Key_R)
	{
		this->setRenderMode(RENDER_REGULAR);
	}

	if(e->key() == Qt::Key_P)
	{
		this->setRenderMode(RENDER_POINT);
	}

	// Regular behavior
	QGLViewer::keyPressEvent(e);
}

void StackerPreview::setRenderMode( RENDER_MODE toMode )
{
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
		i->render_mode = toMode;

	updateGL();
}