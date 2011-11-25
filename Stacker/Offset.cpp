#include "Offset.h"
#include "HiddenViewer.h"

#define ZERO_TOLERANCE 0.01

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
	upperEnvelope = computeEnvelope(1);
	lowerEnvelope = computeEnvelope(-1);
	offset = upperEnvelope; 
	int h = upperEnvelope.size();
	int w = upperEnvelope[0].size();
	std::vector<double> row_max;	

	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			// Two envelopes are horizontally flipped
			if (upperEnvelope[y][x]==DOUBLE_INFINITY | lowerEnvelope[y][(w-1)-x]==DOUBLE_INFINITY)
				offset[y][x] = 0.0; 
			else
				offset[y][x] = upperEnvelope[y][x] - lowerEnvelope[y][(w-1)-x];
		}

		row_max.push_back(*max_element(offset[y].begin(), offset[y].end()));
	}

	O_max = *max_element(row_max.begin(), row_max.end());

	// Update the stackability in QSegMesh
	activeObject()->O_max = O_max;
	activeObject()->stackability = 1 - O_max/objectH;

	activeViewer->updateGL();
}

// Still not very confident on screen coordinates converting
void Offset::hotspotsFromDirection( int direction, double threshold )
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 2 );

	// Envelop
	std::vector< std::vector<double> >& envelope 
		= (direction == 1)? upperEnvelope : lowerEnvelope;

	// The size of current viewer
	int w = activeViewer->width();
	int h = activeViewer->height();	

	// Go through the mesh
	uint i, nbV = activeObject()->nbVertices();
	for (i=0; i<nbV; i++)
	{
		Point src = activeObject()->getVertexPos(i);
		Vec vpixel = activeViewer->camera()->projectedCoordinatesOf(Vec(src));

		// Get the 2D projected coordinate
		int x = ceil((w - 1) - vpixel.x);
		int y = ceil((h - 1) - vpixel.y);
		x = RANGED(0, x, w-1);
		y = RANGED(0, y, h-1);

		// Check whether it is visible
		bool isVisible;
		if (direction == 1)
			isVisible = src.z() > (envelope[y][x] - ZERO_TOLERANCE);
		else
			isVisible = src.z() < (envelope[y][x] + ZERO_TOLERANCE);

		// Check whether it is hot
		if (isVisible)
		{
			double off = offset[y][x];

			if (off > threshold)
			{
				hotVertices.push_back(i);
				hotSegments.insert(activeObject()->vertexInSegment(i));			
			}
		}
	}
}


void Offset::detectHotspots()
{
	// Initialization
	hotVertices.clear();
	hotSegments.clear();

	// Set the threshold for hot spots
	double threshold = 0.8 * O_max;

	// detect hot spots from both directions
	hotspotsFromDirection(1, threshold);
	hotspotsFromDirection(-1, threshold);
}


void Offset::showHotVertices()
{
	Color hotColor(1., 0., 0., 1.);

	for (std::vector< uint >::iterator itr = hotVertices.begin(); itr != hotVertices.end(); itr++)
	{
		activeObject()->setVertexColor(*itr, hotColor);
	}
}


void Offset::showHotSegments()
{

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
