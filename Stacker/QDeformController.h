#pragma once

#include "Controller.h"
#include "QGLViewer/qglviewer.h"

class QDeformController: public QObject{
	Q_OBJECT
public:
	QDeformController();

	qglviewer::ManipulatedFrame * getFrame();
	Vec3d pos();

signals:
	void primitiveReshaped();

private:
	qglviewer::ManipulatedFrame * frame;
};
