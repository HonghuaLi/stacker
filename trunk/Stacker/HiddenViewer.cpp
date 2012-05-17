#include "GraphicsLibrary/Mesh/QSegMesh.h"

#include "HiddenViewer.h"
#include "Numeric.h"

HiddenViewer::HiddenViewer( QWidget * parent ) : QGLViewer (parent)
{
	// Restrict the size of the window
	setFixedSize(200, 200);

	// No active scene when initializing
	this->_activeObject = NULL;

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
	camera()->setType(Camera::ORTHOGRAPHIC);
	camera()->setUpVector(Vec(0,0,1));
	camera()->setPosition(Vec(0,0,10));
	camera()->lookAt(Vec());
}

void HiddenViewer::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
}

void HiddenViewer::preDraw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(activeObject())
	{
		// Compute the AABB
		Vec3d bbmin = objectTransformation.bbmin;
		Vec3d bbmax = objectTransformation.bbmax;
		Point center = (bbmin + bbmax) / 2;

		std::vector<Point> corner = cornersOfAABB(bbmin - center, bbmax - center);
		for (int i = 0; i < 8; i++){
			Vec p = objectTransformation.rot * Vec(corner[i]);
			corner[i] = Point(p.x, p.y, p.z);
		}
		Point new_bbmin(-1), new_bbmax(1);	
		computeAABB(corner, new_bbmin, new_bbmax);
		double s = 1.5;

		camera()->fitBoundingBox(Vec(new_bbmin) * s, Vec(new_bbmax) * s);
		camera()->setSceneRadius(10);
	}


	// GL_PROJECTION matrix
	camera()->loadProjectionMatrix();
	// GL_MODELVIEW matrix
	camera()->loadModelViewMatrix();

	Q_EMIT drawNeeded();
}

void HiddenViewer::draw()
{
	glPushMatrix();

	if(activeObject())
	{
		// Place object
		glMultMatrixd(objectTransformation.rot.matrix());
		glTranslated(objectTransformation.t.x, objectTransformation.t.y, objectTransformation.t.z);
	}

	switch (mode)
	{
	case HV_NONE:
		break;
	case HV_DEPTH:
//		std::cout << "Hidden Viewer: DEPTH\n";
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		activeObject()->simpleDraw();
		break;
	case HV_FACEUNIQUE:
//		std::cout << "Hidden Viewer: FACEUNIQUE\n";
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		activeObject()->drawFacesUnique();
		break;
	}

	glPopMatrix();

	//setMode(HV_NONE);
}

QSegMesh* HiddenViewer::activeObject()
{
	return _activeObject;
}

void HiddenViewer::setActiveObject( QSegMesh * changedObject )
{
	_activeObject = changedObject;
	this->updateGL();
}

void HiddenViewer::setMode( HVMode toMode )
{
	mode = toMode;
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

void HiddenViewer::setResolution( int newRes )
{
	setFixedSize(newRes,newRes);
}
