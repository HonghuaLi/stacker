#pragma once
#include "GUI/Viewer/libQGLViewer/QGLViewer/qglviewer.h"

#include "ConstraintGraph.h"

class ConstraintGraphViewer: public QGLViewer {
	Q_OBJECT
public:
	ConstraintGraphViewer(ConstraintGraph * fromGraph = 0, QWidget * parent = 0) : QGLViewer (parent)
	{
		this->graph = fromGraph;

		this->setMaximumSize(800,800);
		this->setMinimumSize(800,800);

		setBackgroundColor(QColor(255,255,255));
	}

	virtual void draw();

	ConstraintGraph * graph;
};

#include <QVBoxLayout>
#include <QDockWidget>

class ConstraintGraphVis: public QDockWidget{
	Q_OBJECT
public:
	ConstraintGraphViewer * viewer;

	ConstraintGraphVis(ConstraintGraph * fromGraph = 0, QWidget * parent = 0) : QDockWidget(parent){
		this->setVisible(false);
		viewer = new ConstraintGraphViewer(fromGraph);
		setWidget (viewer);
	}

	void setGraph(ConstraintGraph * fromGraph);
};
