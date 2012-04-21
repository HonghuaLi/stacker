#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"
#include "ConstraintGraphVis.h"
#include "Primitive.h"
#include "Controller.h"
#include "Utility/SimpleDraw.h"
#include "GraphicsLibrary/Basic/Circle.h"

using namespace qglviewer;

ConstraintGraphViewer::ConstraintGraphViewer( ConstraintGraph * fromGraph /*= 0*/, QWidget * parent /*= 0*/ ) : QGLViewer (parent)
{
	this->graph = fromGraph;

	//this->setMaximumSize(400,400);
	//this->setMinimumSize(400,400);

	setBackgroundColor(QColor(255,255,255));
}

void ConstraintGraphViewer::draw()
{
	if(!graph || !graph->controller()) return;

	camera()->setType(Camera::ORTHOGRAPHIC);

	glEnable(GL_MULTISAMPLE);

	Controller * ctrl = graph->controller();

	double s = 3;
	double r = ctrl->meshRadius() * s;
	double circleRad = r * 0.1;

	// Compute nodes positions
	QMap<QString, Point> nodePos;

	// Find node positions in 3D
	foreach(QString node, graph->adjacency_map.keys())
	{
		nodePos[node] = ctrl->getPrimitive(node)->centerPoint() * s;
	}

	// Draw edges
	foreach(QList<ConstraintGraph::Edge> edges, graph->adjacency_map.values())
	{
		foreach(ConstraintGraph::Edge edge, edges)
		{
			Vec4d color(0,0,0,1);

			switch(graph->edgeType(edge.id))
			{
			case LINEJOINT: case POINTJOINT:
				SimpleDraw::IdentifyLine(nodePos[edge.from], nodePos[edge.to], 
					Vec4d(1,0,0,1), false); // red
				break;

			case SYMMETRY:
				SimpleDraw::IdentifyDashedLine(nodePos[edge.from], nodePos[edge.to], 
					Vec4d(1,0.2,0.4,0.5), false);
				break;
			}
		}
	}

	glDisable(GL_LIGHTING);
	glClear(GL_DEPTH_BUFFER_BIT);

	// Draw nodes
	foreach(QString node, graph->adjacency_map.keys())
	{
		Vec3d pos (nodePos[node]);

		// Draw node circle
		Circle c(pos, V2V(camera()->viewDirection()), circleRad);
		c.drawFilled(Vec4d(1,1,1,1), 2, Vec4d(0,0,0,1));
	}

	glDisable(GL_LIGHTING);
	glClear(GL_DEPTH_BUFFER_BIT);

	// Draw node id string
	glColor4dv(Vec4d(0,0,0,1));

	foreach(QString node, graph->adjacency_map.keys())
	{
		Vec3d pos (nodePos[node]);

		double fontSize = Max(8, (width() / abs(camera()->position().norm()) / node.size()));
		QFont font("Monospace");
		font.setStyleHint(QFont::TypeWriter);
		font.setPixelSize(fontSize);

		Vec proj(camera()->projectedCoordinatesOf(Vec3d2V(pos)));
		drawText(proj.x - fontSize * node.size() * 0.25, proj.y + fontSize * 0.25, node, font);
	}
}

void ConstraintGraphVis::setGraph( ConstraintGraph * fromGraph )
{
	this->viewer->graph = fromGraph;

	if(!fromGraph || !fromGraph->controller()) return;

	this->viewer->camera()->setSceneRadius(fromGraph->controller()->meshRadius() * 4);
	this->viewer->camera()->showEntireScene();
}
