#include "Offset.h"
#include "HiddenViewer.h"

Offset::Offset( HiddenViewer *viewer )
{
	activeViewer = viewer;
}

std::vector< std::vector<double> > Offset::computeEnvelope( int direction )
{
	// Set camera
	activeViewer->camera()->setType(Camera::ORTHOGRAPHIC);
	activeViewer->camera()->setPosition(Vec(0,0, direction * activeObject()->radius));
	activeViewer->camera()->lookAt(Vec());	
	activeViewer->camera()->setUpVector(Vec(1,0,0));
	activeViewer->camera()->fitBoundingBox(Vec(activeObject()->bbmin), Vec(activeObject()->bbmax));

	// Save this new camera settings
	activeViewer->camera()->addKeyFrameToPath(direction + 2);

	// Compute the envelop (z value)
	double zCamera = (activeViewer->camera()->position()).z;

	activeViewer->setMode(HV_DEPTH);
	activeViewer->updateGL(); // draw

	GLfloat* depthBuffer = (GLfloat*)activeViewer->readBuffer(GL_DEPTH_COMPONENT, GL_FLOAT);

	int w = activeViewer->width();
	int h = activeViewer->height();
	double zNear = activeViewer->camera()->zNear();
	double zFar = activeViewer->camera()->zFar();
	std::vector< std::vector<double> > envelop(h);

	for(int y = 0; y < h; y++)
	{
		envelop[y].resize(w);

		for(int x = 0;x < w; x++)
		{
			double zU = depthBuffer[(y*w) + x];
			if (zU == 1.0)
				envelop[y][x] = FLOAT_INFINITY;
			else
				envelop[y][x] = zCamera - direction * ( zU * zFar + (1-zU) * zNear );
		}
	}

	delete[] depthBuffer;

	return envelop;
}

void Offset::computeOffset()
{
	if (!activeObject()) return;

	// Compute the height of the shape
	objectH = (activeObject()->bbmax - activeObject()->bbmin).z();

	// Save original camera settings
	activeViewer->camera()->addKeyFrameToPath(0);

	// Compute the offset function
	upperEnvolope = computeEnvelope(1);
	lowerEnvolope = computeEnvelope(-1);
	offset = upperEnvolope; 
	int h = upperEnvolope.size();
	int w = upperEnvolope[0].size();
	std::vector<double> row_max;	

	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			if (upperEnvolope[y][x]==DOUBLE_INFINITY | lowerEnvolope[y][(w-1)-x]==DOUBLE_INFINITY)
				offset[y][x] = 0.0; //out the shape domain
			else
				offset[y][x] = upperEnvolope[y][x] - lowerEnvolope[y][(w-1)-x]; //in the shape domain
		}

		row_max.push_back(*max_element(offset[y].begin(), offset[y].end()));
	}

	O_max = *max_element(row_max.begin(), row_max.end());

	// Update the stackability in QSegMesh
	activeObject()->O_max = O_max;
	activeObject()->stackability = 1 - O_max/objectH;
}

std::set<uint> Offset::verticesOnEnvelope( int direction )
{
	//return value
	std::set<uint> vertices;	
	QSegMesh * activeObject = activeViewer->activeObject();	

	// restore camera
	activeViewer->camera()->playPath(direction + 2);

	//rend the faces with unique color
	int w = activeViewer->width();
	int h = activeViewer->height();	

	activeViewer->setMode(HV_UNIQUE_FACES);
	activeViewer->updateGL();

	GLubyte* colormap = (GLubyte*)activeViewer->readBuffer(GL_RGBA, GL_UNSIGNED_BYTE);

	//get the indices of visable vertices
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{

			uint indx = ((y*w)+x)*4;

			uint r = (uint)colormap[indx+0];
			uint g = (uint)colormap[indx+1];
			uint b = (uint)colormap[indx+2];
			uint a = (uint)colormap[indx+3];

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

void Offset::setOffsetColors( int direction )
{
	// Restore camera settings
	activeViewer->camera()->playPath(direction + 2);

	int h = offset.size();
	int w = offset[0].size();
	uchar * rgb = new uchar[3];	

	// Get all the vertices on the current envelope 
	std::set<uint> vindices = verticesOnEnvelope(direction);

	// Assign each vertex with offset color
	for (std::set<uint>::iterator it = vindices.begin(); it!=vindices.end(); it++)
	{
		Point src = activeObject()->getVertexPos(*it);
		Vec vpixel = activeViewer->camera()->projectedCoordinatesOf(Vec(src));

		// For flipping
		int _x = (direction) == 1 ? vpixel.x : (w-1) - vpixel.x;

		int x = RANGED(0, _x, (w-1));
		int y = RANGED(0, (h-1)-vpixel.y, (h - 1));

		// Average color around pixel
		double vertexColor = offset[y][x] / O_max;

		ColorMap::jetColorMap(rgb, vertexColor, 0, 1);

		double r = rgb[0] / 255.0;
		double g = rgb[1] / 255.0;
		double b = rgb[2] / 255.0;

		activeObject()->setVertexColor(*it, Color(r,g,b,1));
	}

	delete[] rgb;
}


std::vector<int> Offset::hotSegments()
{
	// Get all the vertices on the current envelope 
	std::set<uint> vindices = verticesOnEnvelope(1);

	std::vector<int> segIds;
	return segIds;
}



void Offset::run()
{
	// Assign each vertex with offset color
	setOffsetColors(1);
//	setOffsetColors(-1);

	// Restore the camera
	activeViewer->camera()->resetPath(0);
}



void Offset::saveOffsetAsImage( QString fileName )
{
	int h = offset.size();
	int w = offset[0].size();
	QImage offset_img(w, h, QImage::Format_ARGB32);

	uchar * rgb = new uchar[3];		
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			ColorMap::jetColorMap(rgb, Max(0., offset[y][x] / O_max), 0., 1.);
			offset_img.setPixel(x, (h-1) - y, QColor::fromRgb(rgb[0],rgb[1],rgb[2]).rgba());			
		}
	}

	delete[] rgb;
	offset_img.save(fileName);
}

double Offset::getStackability()
{
	return 1 - O_max/objectH;
}

QSegMesh* Offset::activeObject()
{
	return activeViewer->activeObject();
}
