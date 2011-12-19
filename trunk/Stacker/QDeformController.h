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

public slots:
	void updateController();

signals:
	void objectModified();

private:
	qglviewer::ManipulatedFrame * frame;
	Controller * ctrl;
};
