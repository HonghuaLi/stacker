#include "HiddenViewer.h"


HiddenViewer::HiddenViewer( QWidget * parent ) : QGLViewer (parent)
{
	// Restrict the size of the window
	setFixedSize(50,50);

	// No active scene when initializing
	this->activeScene = NULL;

	// Initial render mode as HV_NONE
	mode = HV_NONE;
}

void HiddenViewer::init()
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

void HiddenViewer::setupCamera()
{
	camera()->setUpVector(Vec(0,0,1));
	camera()->setPosition(Vec(2,-2,2));
	camera()->lookAt(Vec());
}

void HiddenViewer::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
}

void HiddenViewer::draw()
{
	if (!activeObject()) return;

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_MULTISAMPLE);

	switch (mode)
	{
	case HV_NONE:
		break;
	case HV_DEPTH:
		activeObject()->simpleDraw();
		break;
	case HV_UNIQUE_FACES:
		activeObject()->drawFacesUnique();
		break;
	}

	setMode(HV_NONE);
}

void HiddenViewer::setActiveScene( Scene * changedScene )
{
	this->activeScene = changedScene;
	this->updateGL();
}

void HiddenViewer::setMode( HVMode toMode )
{
	mode = toMode;
}

QSegMesh* HiddenViewer::activeObject()
{
	if(activeScene != NULL)
		return activeScene->activeObject();
	else
		return NULL;
}

void* HiddenViewer::readBuffer( GLenum format, GLenum type )
{
	void * data = NULL;

	int w = this->width();
	int h = this->height();

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
