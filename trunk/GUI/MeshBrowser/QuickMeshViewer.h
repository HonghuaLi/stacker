#pragma once

#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"
using namespace qglviewer;

#include "QuickMesh.h"

class QuickMeshViewer : public QGLViewer{
	Q_OBJECT
public:
	QuickMeshViewer(QWidget * parent = 0):QGLViewer(parent)
	{
		this->setMaximumSize(200,200);

		this->isActive = false;
	}

	virtual void init()
	{
		QGLViewer::init();

		// Light
		GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

		// Camera
		camera()->setType(Camera::ORTHOGRAPHIC);

		resetView();
	}

	void resetView()
	{
		camera()->setSceneRadius(2.0);
		camera()->setUpVector(Vec(0,0,1));
		camera()->setSceneCenter(Vec(0,0,0));
		camera()->setPosition(Vec(1,1,1));
		camera()->lookAt(Vec(0,0,0));
	}

	virtual void draw()
	{
		if(!this->isActive) return;

		QGLViewer::draw();

		mesh.draw();
	}

	virtual void postDraw()
	{
		if(!this->isActive) return;

		if(this->hasFocus())
		{
			int w = width(), h = height();
			startScreenCoordinatesSystem();
			glColor3d(0.8,0.2,0.2);
			glLineWidth(10);
			glBegin(GL_LINE_STRIP);
			glVertex3dv(Vec(0,0,0));
			glVertex3dv(Vec(w,0,0));
			glVertex3dv(Vec(w,h,0));
			glVertex3dv(Vec(0,h,0));
			glVertex3dv(Vec(0,0,0));
			glEnd();
			stopScreenCoordinatesSystem();
		}

		if(this->mesh.isLoading){
			glColor3d(1,1,1);
			renderText(15, 15, "Loading...");
		}

		QGLViewer::postDraw();
	}

	void focusInEvent( QFocusEvent * event )
	{
		emit(gotFocus(this));
	}

	void clearMesh()
	{
		mesh.isLoading = true;
		mesh.clear();
		isActive = false;
	}

	QString meshFileName()
	{
		return mesh.fileName;
	}

	bool isActive;

public slots:
	void loadMesh(QString fileName)
	{
		mesh.isLoading = true;
		mesh.load(fileName);
		mesh.isLoading = false;

		isActive = true;
	}

signals:
	void gotFocus(QuickMeshViewer*);

private:
	QuickMesh mesh;
};

