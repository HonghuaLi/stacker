#include "GraphicsLibrary/Mesh/QSegMesh.h"
#include <QKeyEvent>

#include "Previewer.h"
#include "SimpleDraw.h"

Previewer::Previewer( QWidget * parent ) : QGLViewer (parent)
{
	// No active scene when initializing
	this->_activeObject = NULL;

	// Stack count
	stackCount = 3;
}

void Previewer::init()
{
	glEnable(GL_MULTISAMPLE);

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

void Previewer::setupCamera()
{
	camera()->setUpVector(Vec(0,0,1));
	camera()->setPosition(Vec(1,-2,1));
	camera()->lookAt(Vec());
}

void Previewer::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
}

void Previewer::preDraw()
{
	QGLViewer::preDraw();
}

void Previewer::draw()
{
	// Anti aliasing 
	glEnable(GL_MULTISAMPLE);
	glEnable (GL_LINE_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);

	// Background
	setBackgroundColor(backColor);

	if (!activeObject()) return;

	// Update VBO if needed
	updateVBOs();

	double S = activeObject()->val["stackability"];
	Vec3d stacking_shift = activeObject()->vec["stacking_shift"];

	glPushMatrix();
	Vec3d initPos = - stacking_shift * (stackCount-1) / 2.0;
	glTranslated(initPos[0],initPos[1],initPos[2]);

	for(int i = 0; i < stackCount; i++)
	{
		// Draw object using VBO
		for (QMap<QString, VBO>::iterator itr = vboCollection.begin(); itr != vboCollection.end(); ++itr)
			itr->render();

		// Fall back
		if(vboCollection.isEmpty() && activeObject())
			activeObject()->simpleDraw();

		glTranslated(stacking_shift[0],stacking_shift[1],stacking_shift[2]);
	}

	glPopMatrix();

	// Draw stacking direction
	glColor4dv(Color(0,0,0.8,0.8));
	SimpleDraw::DrawArrowDirected(initPos, stacking_shift, stackCount);
}

void Previewer::postDraw()
{
	QGLViewer::postDraw();

	qglColor(Qt::black);

	if(activeObject())
	{
		QString message = QString("O_max = %1; S = %2").arg(activeObject()->val["O_max"]).arg(activeObject()->val["stackability"]);
		renderText(0, this->height()- 10, message);

		QString message2 = QString("Theta = %1; Phi = %2").arg(activeObject()->val["theta"]).arg(activeObject()->val["phi"]);
		renderText(0, this->height()- 30, message2);

		double tranX = activeObject()->val["tranX"];
		double tranY = activeObject()->val["tranY"];
		double tranZ = activeObject()->val["tranZ"];
		Vec3d shift (tranX, tranY, tranZ);

		QString message3 = QString("Shift = %1,%2,%3").arg(shift[0]).arg(shift[1]).arg(shift[2]);
		renderText(0, this->height()- 50, message3);
	}

}


void Previewer::updateVBOs()
{
	QSegMesh * mesh = activeObject();

	if(mesh && mesh->isReady)
	{
		// Create VBO for each segment if needed
		for (int i=0;i<(int)mesh->nbSegments();i++)
		{			
			QSurfaceMesh* seg = mesh->getSegment(i);
			QString objId = seg->objectName();

			if (VBO::isVBOSupported() && !vboCollection.contains(objId))
			{
				Surface_mesh::Vertex_property<Point>  points   = seg->vertex_property<Point>("v:point");
				Surface_mesh::Vertex_property<Point>  vnormals = seg->vertex_property<Point>("v:normal");
				Surface_mesh::Vertex_property<Color>  vcolors  = seg->vertex_property<Color>("v:color");			
				seg->fillTrianglesList();

				// Create VBO 
				vboCollection[objId] = VBO( seg->n_vertices(), points.data(), vnormals.data(), vcolors.data(), seg->triangles );		
			}
		}
	}
}

void Previewer::updateActiveObject()
{
	if(activeObject())
	{
		Vec3d bbmin = activeObject()->bbmin;
		Vec3d bbmax = activeObject()->bbmax;
		double O_max = activeObject()->val["O_max"];
		double delta = (stackCount-1) * O_max / 2;
		bbmin[2] -= delta;
		bbmax[2] += delta;

		double radius = (bbmax - bbmin).norm()/2;
		camera()->setSceneRadius(radius);
		double s = 1.5;
		camera()->fitBoundingBox(Vec(bbmin * s), Vec(bbmax * s));
	}

	vboCollection.clear();
	updateGL();
}

QSegMesh* Previewer::activeObject()
{
	return _activeObject;
}

void Previewer::keyPressEvent( QKeyEvent *e )
{
	if(e->key() == Qt::Key_W)
	{
		this->setRenderMode(RENDER_WIREFRAME);
	}

	if(e->key() == Qt::Key_R)
	{
		this->setRenderMode(RENDER_REGULAR);
	}

	if(e->key() == Qt::Key_P)
	{
		this->setRenderMode(RENDER_POINT);
	}

	// Regular behavior
	QGLViewer::keyPressEvent(e);
}

void Previewer::setRenderMode( RENDER_MODE toMode )
{
	QMap<QString, VBO>::iterator i;
	for (i = vboCollection.begin(); i != vboCollection.end(); ++i)
	{
		if(i->render_mode == toMode)
			i->render_mode = RENDER_REGULAR;
		else
			i->render_mode = toMode;
	}

	updateGL();
}

void Previewer::saveStackObj( QString fileName, int numStack, double scaleFactor)
{
	int stackCount = numStack;
	double O_max = activeObject()->val["O_max"];
	double S = activeObject()->val["stackability"];

	Vec3d delta = O_max * activeObject()->vec["stacking_direction"];

	QSegMesh outputMesh;
	QSurfaceMesh * deltaMesh = activeObject()->flattenMesh();

	if(scaleFactor < 0) scaleFactor = activeObject()->scaleFactor;

	outputMesh.scaleFactor = scaleFactor;

	QString singleFileName = fileName;
	singleFileName.replace(".obj", "_single.obj");

	double phi =  activeObject()->val["phi"];

	for(int i = 0; i < stackCount; i++)
	{
		outputMesh.insertCopyMesh(deltaMesh);

		deltaMesh->translate(delta);
		deltaMesh->rotateAroundUp(RADIANS(phi));

		// Output single mesh first
		if(i == 0) outputMesh.saveObj(singleFileName);
	}

	outputMesh.saveObj(fileName);
	// delete deltaMesh?
}

void Previewer::setStackCount( int num )
{
	stackCount = num;
	updateActiveObject();
}

void Previewer::setActiveObject( QSegMesh * newObject )
{
	_activeObject = newObject;
}
