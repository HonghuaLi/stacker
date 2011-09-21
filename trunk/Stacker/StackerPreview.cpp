#include "StackerPreview.h"


StackerPreview::StackerPreview( QWidget * parent ) : QGLViewer (parent)
{
	setMaximumWidth(150);

	this->activeObject = NULL;
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
	this->makeCurrent();
	QGLViewer::preDraw();
}

void StackerPreview::draw()
{
	if(!activeObject)
		return;

	// Background
	setBackgroundColor(backColor);

	activeObject->draw();
}

void StackerPreview::setActiveObject( QSurfaceMesh * newObject )
{
	if(!newObject)
		return;

	if(activeObject)
		delete activeObject;

	this->activeObject = new QSurfaceMesh(*newObject);
	this->activeObject->setColorVertices(); // white

	camera()->setSceneRadius(activeObject->radius);
	camera()->showEntireScene();
}
