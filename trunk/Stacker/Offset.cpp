#include "Offset.h"
#include "HiddenViewer.h"
#include "ColorMap.h"
#include "SimpleDraw.h"

#include <numeric>
#include <stack>

#define ZERO_TOLERANCE 0.01
#define BIG_NUMBER 9999

Offset::Offset( HiddenViewer *viewer )
{
	activeViewer = viewer;

	filterSize = 1;
}

QSegMesh* Offset::activeObject()
{
	return activeViewer->activeObject();
}



void Offset::computeEnvelope( int direction, std::vector< std::vector<double> > &envelope, std::vector< std::vector<double> > &depth )
{
	// Clear
	envelope.clear();
	depth.clear();

	// Set camera
	activeViewer->camera()->setType(Camera::ORTHOGRAPHIC);
	activeViewer->camera()->setPosition(Vec(0,0, direction * activeObject()->radius));
	activeViewer->camera()->lookAt(Vec());	
	activeViewer->camera()->setUpVector(Vec(1,0,0));
	activeViewer->camera()->fitBoundingBox(Vec(activeObject()->bbmin), Vec(activeObject()->bbmax));

	// Save this new camera settings
	activeViewer->camera()->deletePath(direction + 2);
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
	activeViewer->camera()->deletePath(0);
	activeViewer->camera()->addKeyFrameToPath(0);

	// Compute the offset function
	computeEnvelope(1, upperEnvelope, upperDepth);
	computeEnvelope(-1, lowerEnvelope, lowerDepth);
	offset = upperEnvelope; 
	int h = upperEnvelope.size();
	int w = upperEnvelope[0].size();

	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			// Two envelopes are horizontally flipped
			if (upperEnvelope[y][x]== BIG_NUMBER | lowerEnvelope[y][(w-1)-x] == BIG_NUMBER)
				offset[y][x] = 0.0; 
			else
				offset[y][x] = upperEnvelope[y][x] - lowerEnvelope[y][(w-1)-x];
		}
	}

	O_max = getMaxValue(offset);

	// Update the stackability in QSegMesh
	activeObject()->O_max = O_max;
	activeObject()->stackability = 1 - O_max/objectH;

	activeViewer->updateGL();
}

double Offset::getStackability()
{
	return 1 - O_max/objectH;
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

			// 3d position of this hot pixel
			double depthVal = getValue(depth, x, y);
			if (depthVal == BIG_NUMBER) 
				continue;			
			Vec hotP= activeViewer->camera()->unprojectedCoordinatesOf(Vec(x, (h-1)-y, depthVal));
			
			// Segment index of this face
			if (f_id >= activeObject()->nbFaces()) 
				continue;
			activeObject()->global2local_fid(f_id, sid, fid_local);
			hotSpots[sid].insert(Vec3d(hotP.x, hotP.y, hotP.z));
		}
	}

	delete colormap;	
}

void Offset::detectHotspots( int useFilterSize, double hotRange )
{
	filterSize = useFilterSize;
	hotRangeThreshold = hotRange;

	// Recompute envelopes and offset
	computeOffset();

	// Initialization
	hotSpots.clear();

	// Detect hot regions
	hotRegions.clear();
	hotRegions = getRegions(offset, std::bind2nd(std::greater<double>, O_max * hotRange);

	// Detect hot spots from both directions
	hotspotsFromDirection(1);
	hotspotsFromDirection(-1);
}

void Offset::showHotSpots()
{
	// Clear past states
	for(uint i = 0; i < activeObject()->nbSegments(); i++)
	{
		QSurfaceMesh * seg = activeObject()->getSegment(i);

		seg->debug_points.clear();
		seg->setColorVertices(Color(1,1,1,1)); // white
	}

	// Show hot segments and hot spots
	for (std::map< uint, std::set< Vec3d > >::iterator i=hotSpots.begin();i!=hotSpots.end();i++)
	{
		uint sid = i->first;
		QSurfaceMesh* segment = activeObject()->getSegment(sid);

		segment->setColorVertices(Color(1, 0, 0, 1)); // red

		for (std::set< Vec3d >::iterator pit = i->second.begin(); pit != i->second.end(); pit++)
		{
			segment->debug_points.push_back(*pit);
		}
	}
}

std::set<uint> Offset::getHotSegment()
{
	std::set< uint > hs;

	for (std::map< uint, std::set< Vec3d > >::iterator i=hotSpots.begin();i!=hotSpots.end();i++)
		hs.insert(i->first);

	return hs;
}



double Offset::getValue( std::vector< std::vector < double > >& image, uint x, uint y )
{
	// The window size of smoothing
	uint r = this->filterSize;

	int w = image[0].size();
	int h = image.size();	

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

double Offset::getMaxValue( std::vector< std::vector < double > >& image )
{
	int h = image.size();

	std::vector< double > row_max;
	for (int y = 0; y < h; y++)
		row_max.push_back(MaxElement(image[y]));

	return MaxElement(row_max);
}

double Offset::getMinValue( std::vector< std::vector < double > >& image )
{
	int h = image.size();

	std::vector< double > row_min;
	for (int y = 0; y < h; y++)
		row_min.push_back(MinElement(image[y]));

	return MinElement(row_min);
}

template< typename PREDICATE >
std::vector< Vec2ui > 
	Offset::getRegion( std::vector< std::vector < double > >& image, 
			std::vector< std::vector < bool > >& mask, 	Vec2ui seed, PREDICATE predicate )
{
	std::vector< Vec2ui > region;

	int w = image[0].size();
	int h = image.size();		
	
	std::stack activePnts;
	activePnts.push(seed);

	while (!activePnts.empty())
	{
		// Add the top point to region
		Vec2ui currP = activePnts.pop();
		mask[currP.y][currP.x] = true;
		region.push_back(currP);

		// Push all the neighbors to the stack
		uint min_x = RANGED(0, currP.x-1 ,w-1);
		uint max_x = RANGED(0, currP.x+1, w-1);
		uint min_y = RANGED(0, currP.y-1 ,h-1);
		uint max_y = RANGED(0, currP.y+1, h-1);

		for (uint y = min_y; y <= max_y; y++)
			for (uint x = min_x; x <= max_x; x++)
			{
				if (!mask[y][x] && predicate(image[y][x]))
					activePnts.push( Vec2ui<x, y> );
			}
	}

	return region;
}


template< typename PREDICATE >
std::vector< std::vector< Vec2ui > >
	Offset::getRegions( std::vector< std::vector < double > >& image, PREDICATE predicate )
{
	std::vector< std::vector< Vec2ui > > regions;

	int w = image[0].size();
	int h = image.size();

	std::vector< std::vector< bool > > mask(h, std::vector< bool >(w, false));

	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			if (!mask[y][x] && predicate(image[y][x]))
			{
				regions.push_back(getRegion(image, mask, Vec2ui(x, y), predicate));
			}
		}
	}

	return regions;
}
