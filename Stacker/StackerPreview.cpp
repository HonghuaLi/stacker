#include "StackerPreview.h"


StackerPreview::StackerPreview( QWidget * parent ) : QGLViewer (parent)
{
	setMaximumWidth(200);

	this->activeScene = NULL;
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

	if(!activeScene || !activeScene->activeObject())
		return;

	if(!activeObjectVBO.isReady || activeObjectVBO.objectId != qPrintable(activeScene->activeObjectId)){
		QSurfaceMesh * mesh = activeScene->activeObject();

		Surface_mesh::Vertex_property<Point>  points   = mesh->vertex_property<Point>("v:point");
		Surface_mesh::Vertex_property<Point>  vnormals = mesh->vertex_property<Point>("v:normal");
		Surface_mesh::Face_property<Point>    fnormals = mesh->face_property<Point>("f:normal");
		Surface_mesh::Vertex_property<Color>  vcolors  = mesh->vertex_property<Color>("v:color");
		Surface_mesh::Vertex_property<float>  vtex     = mesh->vertex_property<float>("v:tex1D", 0.0);

		// Create VBO
		mesh->fillTrianglesList();
		activeObjectVBO = VBO(mesh->n_vertices(), points.data(), vnormals.data(), vcolors.data(), mesh->triangles);
		activeObjectVBO.objectId = qPrintable(activeScene->activeObjectId);
	}

	int stackCount = 3;
	Vec3d stackDirection(0, 0, 1.0);
	double delta = activeScene->activeObject()->radius / 2.0;

	glPushMatrix();
	glRotated(10, 0, 0, 1);

	for(int i = 0; i < stackCount; i++)
	{
		activeObjectVBO.render();

		glTranslated(0,0,delta);
	}

	glPopMatrix();
}

void StackerPreview::setActiveScene( Scene * changedScene )
{
	this->activeScene = changedScene;

	if(activeScene && activeScene->activeObject())
		camera()->setSceneRadius(activeScene->activeObject()->radius);

	camera()->showEntireScene();

	this->updateGL();
}
