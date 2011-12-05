#pragma once

#include "Controller.h"
#include "QGLViewer/qglviewer.h"

class QDeformController: public QObject{
	Q_OBJECT
public:
	QDeformController(Controller * usingController);

	qglviewer::ManipulatedFrame * getFrame();
	Vec3d pos();

public slots:
	void updateController();

signals:
	void primitiveReshaped();

private:
	qglviewer::ManipulatedFrame * frame;
	Controller * ctrl;
};
