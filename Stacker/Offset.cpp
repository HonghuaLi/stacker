#include "Offset.h"


Offset::Offset( Scene *scene )
{
	activeScene = scene;
}

Offset::~Offset()
{

}

std::vector< std::vector<float> > Offset::computeEnvelope( int direction )
{
	QSurfaceMesh * activeObject = activeScene->activeObject();

	// Set camera
	activeScene->camera()->setType(Camera::ORTHOGRAPHIC);
	activeScene->camera()->setPosition(Vec(0,0, direction * activeObject->radius));
	activeScene->camera()->lookAt(Vec());	
	activeScene->camera()->setUpVector(Vec(1,0,0));
	activeScene->camera()->fitBoundingBox(Vec(activeObject->bbmin), Vec(activeObject->bbmax));

	// Save new camera state
	activeScene->camera()->addKeyFrameToPath(direction + 2);

	// Compute the envelop (z value)
	float zCamera = (activeScene->camera()->position()).z;

	activeScene->specialRenderMode = DEPTH;
	activeScene->updateGL(); // draw
	activeScene->specialRenderMode = REGULAR;

	GLfloat* depthBuffer = (GLfloat*)activeScene->readBuffer(GL_DEPTH_COMPONENT, GL_FLOAT);

	int w = activeScene->width();
	int h = activeScene->height();
	float zNear = activeScene->camera()->zNear();
	float zFar = activeScene->camera()->zFar();
	std::vector< std::vector<float> > envelop(h);

	for(int y = 0; y < h; y++)
	{
		envelop[y].resize(w);

		for(int x = 0;x < w; x++)
		{
			float zU = depthBuffer[(y*w) + x];
			if (zU == 1.0)
				envelop[y][x] = FLOAT_INFINITY;
			else
				envelop[y][x] = zCamera - direction * ( zU * zFar + (1-zU) * zNear );
		}
	}

	delete[] depthBuffer;

	return envelop;
}

std::set<uint> Offset::verticesOnEnvelope( int direction )
{
	//return value
	std::set<uint> vertices;	
	QSurfaceMesh * activeObject = activeScene->activeObject();	

	// restore camera
	activeScene->camera()->playPath(direction + 2);

	//rend the faces with unique color
	int w = activeScene->width();
	int h = activeScene->height();	

	activeScene->specialRenderMode = UNIQUE_FACES;
	activeScene->updateGL();
	activeScene->specialRenderMode = REGULAR;

	GLubyte* colormap = (GLubyte*)activeScene->readBuffer(GL_RGBA, GL_UNSIGNED_BYTE);

	//get the indices back
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{

			uint r = (uint)colormap[((y*w)+x)*4+0];
			uint g = (uint)colormap[((y*w)+x)*4+1];
			uint b = (uint)colormap[((y*w)+x)*4+2];
			uint a = (uint)colormap[((y*w)+x)*4+3];

			uint f_id = ((255-a)<<24) + (r<<16) + (g<<8) + b;

			if(f_id > 0 && f_id < (activeObject->n_faces() + 1))
			{
				std::vector<uint> cur_vertices = activeObject->vertexIndicesAroundFace(f_id - 1);
				vertices.insert(cur_vertices.begin(), cur_vertices.end());
			}
		}
	}		

	delete colormap;
	return vertices;
}

void Offset::setOffsetColors( int direction, std::vector< std::vector<float> > &offset, float O_max )
{
	QSurfaceMesh * activeObject = activeScene->activeObject();

	// restore camera
	activeScene->camera()->playPath(direction + 2);

	// Assign each vertex with offset color

	int h = offset.size();
	int w = offset[0].size();
	uchar * rgb = new uchar[3];	

	std::set<uint> vindices = verticesOnEnvelope(direction);

	float objectH = (activeObject->bbmax - activeObject->bbmin).z();

	for (std::set<uint>::iterator it = vindices.begin(); it!=vindices.end(); it++)
	{
		Point src = activeObject->getVertexPos(*it);
		Vec vpixel = activeScene->camera()->projectedCoordinatesOf(Vec(src));

		// For flipping
		int _x = (direction) == 1 ? vpixel.x : (w-1)-vpixel.x;

		int x = RANGED(0, _x, (w-1));
		int y = RANGED(0, (h-1)-vpixel.y, (h - 1));

		ColorMap::jetColorMap(rgb, Max(0, offset[y][x] / O_max), 0, 1);

		double r = rgb[0] / 255.0;
		double g = rgb[1] / 255.0;
		double b = rgb[2] / 255.0;

		activeObject->setVertexColor(*it, Color(r,g,b,1));
	}

	delete[] rgb;
}

void Offset::run()
{
	if(!activeScene) return;

	QSurfaceMesh * activeObject = activeScene->activeObject();

	// Save original camera state
	activeScene->camera()->addKeyFrameToPath(0);

	// Compute the offset function
	std::vector< std::vector<float> > upperEnvolope = computeEnvelope(1);	 //upper
	std::vector< std::vector<float> > lowerEnvolope = computeEnvelope(-1);  //lower
	std::vector< std::vector<float> > offset = upperEnvolope; 
	int h = upperEnvolope.size();
	int w = upperEnvolope[0].size();
	std::vector<float> row_max;	

	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			if (upperEnvolope[y][x]==FLOAT_INFINITY | lowerEnvolope[y][(w-1)-x]==FLOAT_INFINITY)
				offset[y][x] = 0.0; //out the shape domain
			else
				offset[y][x] = upperEnvolope[y][x] - lowerEnvolope[y][(w-1)-x]; //in the shape domain
		}

		row_max.push_back(*max_element(offset[y].begin(), offset[y].end()));
	}

	float O_max = *max_element(row_max.begin(), row_max.end());


	// Assign each vertex with offset color
	setOffsetColors(1, offset, O_max);
	setOffsetColors(-1, offset, O_max);

	activeObject->update();

	//// Save the offset function to an image
	uchar * rgb = new uchar[3];	
	float objectH = (activeObject->bbmax - activeObject->bbmin).z();
	QImage offset_img(w, h, QImage::Format_ARGB32);
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			ColorMap::jetColorMap(rgb, Max(0., offset[y][x] / O_max), 0., 1.);
			offset_img.setPixel(x, (h-1) - y, QColor::fromRgb(rgb[0],rgb[1],rgb[2]).rgba());			
		}
	}
	delete[] rgb;
	offset_img.save("offset_function.png");

	// Restore camera state
	activeScene->camera()->setType(Camera::PERSPECTIVE);
	activeScene->camera()->resetPath(0);

	activeScene->displayMessage(QString("O_max / objectH = %1").arg(O_max / objectH));
	activeScene->print(QString("Offset function computing has done!"));
}
