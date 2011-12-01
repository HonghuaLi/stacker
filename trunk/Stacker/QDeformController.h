#pragma once

#include "Controller.h"
#include "QGLViewer/qglviewer.h"

class QDeformController: public QObject{
	Q_OBJECT
public:
	QDeformController();

	qglviewer::ManipulatedFrame * getFrame();
	Vec3d pos();

	void setController(Controller * c);

	void drawDebug();

public slots:
	void updateController();

private:
	qglviewer::ManipulatedFrame * frame;
	Controller * ctrl;

	// Helpful for debugging
	std::vector<Vec3d> debugPoints;
	std::vector< std::pair<Vec3d,Vec3d> > debugLines;
};
