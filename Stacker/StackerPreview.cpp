#include "StackerPreview.h"


StackerPreview::StackerPreview( QWidget * parent ) : QGLViewer (parent)
{
	// Restrict the size of the preview window
	setMaximumWidth(200);

	// No active scene when initializing
	this->activeScene = NULL;

	// Stacking direction is always along z axis
	stackDirection = Vec3d(0., 0., 1.);

	// check if FBO is supported by your video card
	glInfo glInfo;
	glInfo.getInfo();
	if(glInfo.isExtensionSupported("GL_EXT_framebuffer_object"))
	{
		fobSupported = true;
		this->displayMessage("FBO is supported ;)");
	}
	else
	{
		fobSupported = false;
		this->displayMessage("FBO isn't supported ;(");
	}
}

void StackerPreview::init()
{
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
	// Background
	setBackgroundColor(backColor);

	// Update VBO is needed
	updateVBOs();

	int stackCount = 3;
	double O_max = activeScene->m_offset->getMaxOffset();
	double S = activeScene->m_offset->getStackability();

	this->displayMessage(QString("O_max = %1; S = %2").arg(O_max).arg(S));
	Vec3d delta = O_max * stackDirection;

	glPushMatrix();
	glRotated(10, 0, 0, 1);

	for(int i = 0; i < stackCount; i++)
	{
		// Draw object using VBO
		for (QMap<QString, VBO>::iterator itr = vboCollection.begin(); itr != vboCollection.end(); ++itr)
			itr->render();

		glTranslated(delta[0],delta[1],delta[2]);
	}

	glPopMatrix();
}

void StackerPreview::setActiveScene( Scene * changedScene )
{
	this->activeScene = changedScene;

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

			if (vboCollection.find(objId) == vboCollection.end())
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
	}
		
	vboCollection.clear();
}
