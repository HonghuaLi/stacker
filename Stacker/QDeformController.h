#pragma once

#include "Controller.h"
#include "QGLViewer/qglviewer.h"

class QDeformController: public QObject{
	Q_OBJECT
public:
	QDeformController(Controller * usingController);

	qglviewer::ManipulatedFrame * getFrame();
	Vec3d pos();
	void draw();

	double lastScale;

public slots:
	void updateController();
	void scaleUp(double s);

signals:
	void objectModified();

private:
	qglviewer::ManipulatedFrame * frame;
	Controller * ctrl;
};
