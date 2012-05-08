#pragma once

#include "Controller.h"
#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"

class QManualDeformer: public QObject{
	Q_OBJECT
public:
	QManualDeformer(Controller * usingController);

	qglviewer::ManipulatedFrame * getFrame();

	Vec3d pos();
	void draw();

	double lastScale;
	
	// For rotation
	void saveOriginal();
	std::vector<Point> originalMesh;

public slots:
	void updateController();
	void scaleUp(double s);
	void scale(Vec3d delta);

signals:
	void objectModified();

private:
	qglviewer::ManipulatedFrame *frame;
	Controller * ctrl;
};
