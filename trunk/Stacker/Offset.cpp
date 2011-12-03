#include "Offset.h"
#include "HiddenViewer.h"
#include "ColorMap.h"
#include <numeric>
#include "SimpleDraw.h"

#define ZERO_TOLERANCE 0.01
#define BIG_NUMBER 9999

Offset::Offset( HiddenViewer *viewer )
{
	activeViewer = viewer;

	filterSize = 1;
}

void Offset::computeEnvelope( int direction, std::vector< std::vector<double> > &envelope, std::vector< std::vector<double> > &depth )
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
	envelope.resize(h);
	depth.resize(h);

	for(int y = 0; y < h; y++)
	{
		envelope[y].resize(w);
		depth[y].resize(w);

		for(int x = 0;x < w; x++)
		{
			double zU = depthBuffer[(y*w) + x];

			depth[y][x] = zU;

			if (zU == 1.0)
				envelope[y][x] = BIG_NUMBER;
			else
				envelope[y][x] = zCamera - direction * ( zU * zFar + (1-zU) * zNear );
		}
	}

	delete[] depthBuffer;
}

void Offset::computeOffset()
{
	if (!activeObject()) return;

	// Compute the height of the shape
	objectH = (activeObject()->bbmax - activeObject()->bbmin).z();

	// Save original camera settings
	activeViewer->camera()->addKeyFrameToPath(0);

	// Compute the offset function
	computeEnvelope(1, upperEnvelope, upperDepth);
	computeEnvelope(-1, lowerEnvelope, lowerDepth);
	offset = upperEnvelope; 
	int h = upperEnvelope.size();
	int w = upperEnvelope[0].size();
	std::vector<double> row_max;	

	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			// Two envelopes are horizontally flipped
			if (upperEnvelope[y][x]== BIG_NUMBER | lowerEnvelope[y][(w-1)-x] == BIG_NUMBER)
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


void Offset::hotspotsFromDirection( int direction )
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 2 );

	// Draw Faces Unique
	activeViewer->setMode(HV_FACEUNIQUE);
	activeViewer->updateGL(); 

	GLubyte* colormap = (GLubyte*)activeViewer->readBuffer(GL_RGBA, GL_UNSIGNED_BYTE);

	// The size of current viewer
	int w = activeViewer->width();
	int h = activeViewer->height();	

	// The depth buffer
	std::vector< std::vector<double> > &depth = (direction == 1)? upperDepth : lowerDepth;

	// Detect hot spots
	//QImage unique_image(w, h, QImage::Format_ARGB32);
	uint sid, fid_local;
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{

			double offsetVal;
			if (direction == 1)
				offsetVal = getValue(offset, x, y);
			else
				offsetVal = getValue(offset, (w-1)-x, y);

			// If this is not hot, skip
			// If this is on the edge, skip
			if (offsetVal == BIG_NUMBER || offsetVal/O_max < hotRangeThreshold) continue;
			

			// Get the face index back
			uint indx = ((y*w)+x)*4;
			uint r = (uint)colormap[indx+0];
			uint g = (uint)colormap[indx+1];
			uint b = (uint)colormap[indx+2];
			uint a = (uint)colormap[indx+3];

			uint f_id = ((255-a)<<24) + (r<<16) + (g<<8) + b - 1;

			// Segment index of this face
			if (f_id >= activeObject()->nbFaces()) continue;
			activeObject()->global2local_fid(f_id, sid, fid_local);
			hotFaces[sid].insert(fid_local);

			// 3d position of this hot pixel
			double depthVal = getValue(depth, x, y);
			if (depthVal == BIG_NUMBER) continue;

			Vec hotP= activeViewer->camera()->unprojectedCoordinatesOf(Vec(x, (h-1)-y, depthVal));
			hotPoints[sid].insert(Vec3d(hotP.x, hotP.y, hotP.z));

			//unique_image.setPixel(x, y, QColor::fromRgb(r,g,b).rgba());
		}
	}

	delete colormap;	
	//unique_image.save("unique_image.png");
}

void Offset::detectHotspots( int useFilterSize, double hotRange )
{
	// Check if the envelopes and offset are updated
	if (activeViewer->size().width() != offset.size())
		computeOffset();

	filterSize = useFilterSize;
	hotRangeThreshold = hotRange;

	// Initialization
	hotPoints.clear();
	hotFaces.clear();

	// detect hot spots from both directions
	hotspotsFromDirection(1);
	hotspotsFromDirection(-1);
}

void Offset::showHotSegments()
{
	// Clear past states
	for(uint i = 0; i < activeObject()->nbSegments(); i++)
	{
		QSurfaceMesh * seg = activeObject()->getSegment(i);

		seg->debug_points.clear();
		seg->setColorVertices(Color(1,1,1,1)); // white
	}

	// Show hot faces
	for (std::map< uint, std::set< uint > >::iterator i=hotFaces.begin();i!=hotFaces.end();i++)
	{
		uint sid = i->first;
		QSurfaceMesh* segment = activeObject()->getSegment(sid);

		segment->setColorVertices(Color(1, 0, 0, 1)); // red

		for (std::set< uint >::iterator fit = i->second.begin(); fit != i->second.end(); fit++)
		{
			segment->debug_points.push_back(segment->faceCenter((Surface_mesh::Face)*fit));
		}
	}

	// Show hot points
	for (std::map< uint, std::set< Vec3d > >::iterator i=hotPoints.begin();i!=hotPoints.end();i++)
	{
		uint sid = i->first;
		QSurfaceMesh* segment = activeObject()->getSegment(sid);

		for (std::set< Vec3d >::iterator pit = i->second.begin(); pit != i->second.end(); pit++)
		{
			segment->debug_points.push_back(*pit);
		}
	}
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
			offset_img.setPixel(x, y, QColor::fromRgb(rgb[0],rgb[1],rgb[2]).rgba());			
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

double Offset::getValue( std::vector< std::vector < double > >& image, uint x, uint y )
{
	// The window size of smoothing
	uint r = this->filterSize;

	int w = activeViewer->width();
	int h = activeViewer->height();	

	uint min_x = RANGED(0, x-r ,w-1);
	uint max_x = RANGED(0, x+r, w-1);
	uint min_y = RANGED(0, y-r ,h-1);
	uint max_y = RANGED(0, y+r, h-1);

	// Collect all the values in the neighborhood
	std::vector<double> vals;
	for (int i=min_y;i<=max_y;i++)
		for (int j=min_x;j<=max_x;j++)
			vals.push_back(image[i][j]);

	if (vals.empty())
		return BIG_NUMBER;

	// Check if (x, y) is on an edge by comparing the min and max
	double result;
	if (MaxElement(vals) - MinElement(vals) > ZERO_TOLERANCE)
		result = BIG_NUMBER;
	else
		result = Sum(vals) / ((2*r+1)*(2*r+1));

	return result;
}

std::set<uint> Offset::getHotSegment()
{
	std::set< uint > hs;

	for (std::map< uint, std::set< uint > >::iterator i=hotFaces.begin();i!=hotFaces.end();i++)
		hs.insert(i->first);

	return hs;
}

