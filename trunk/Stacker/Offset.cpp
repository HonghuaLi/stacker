#include "Offset.h"
#include "HiddenViewer.h"
#include "ColorMap.h"
#include "SimpleDraw.h"
#include <QFile>

#include <numeric>
#include <stack>

#define ZERO_TOLERANCE 0.01
#define BIG_NUMBER 10
#define DEPTH_EDGE_THRESHOLD 0.1


// OpenGL 2D coordinates system has origin at the left bottom conner, while Qt at left top conner
// OpenGL coordinates are mainly used in this class

// Camera path
// <-1, 1> + 2 = <1, 3> : The top and bottom setting for the entire shape
// <-1, 1> + 3 = <2, 4> : The top and bottom setting for the zoomed in region

Offset::Offset( HiddenViewer *viewer )
{
	activeViewer = viewer;

	HOT_RANGE = 0.95;
}

QSegMesh* Offset::activeObject()
{
	return activeViewer->activeObject();
}

void Offset::computeEnvelope(int direction)
{
	// Switcher
	std::vector< std::vector<double> > &envelope = (1 == direction)? upperEnvelope : lowerEnvelope;
	std::vector< std::vector<double> > &depth = (1 == direction)? upperDepth : lowerDepth;
	envelope.clear();
	depth.clear();

	// Read the buffer
	GLfloat* depthBuffer = (GLfloat*)activeViewer->readBuffer(GL_DEPTH_COMPONENT, GL_FLOAT);

	// Format the data
	int w = activeViewer->width();
	int h = activeViewer->height();
	double zCamera = (activeViewer->camera()->position()).z;
	double zNear = activeViewer->camera()->zNear();
	double zFar = activeViewer->camera()->zFar();
	envelope.resize(h);
	depth.resize(h);

	#pragma omp parallel for
	for(int y = 0; y < h; y++)
	{
		envelope[y].resize(w);
		depth[y].resize(w);

		for(int x = 0;x < w; x++)
		{
			double zU = depthBuffer[(y*w) + x];

			depth[y][x] = zU;

			if (zU == 1.0)
				envelope[y][x] = (direction == 1) ? -BIG_NUMBER : BIG_NUMBER;
			else
				envelope[y][x] = zCamera - direction * ( zU * zFar + (1-zU) * zNear );
		}
	}

	delete[] depthBuffer;
}

void Offset::computeEnvelopeOfShape( int direction )
{
	// Set camera
	activeViewer->camera()->setType(Camera::ORTHOGRAPHIC);
	activeViewer->camera()->setPosition(Vec(0,0, direction * activeObject()->radius));
	activeViewer->camera()->lookAt(Vec());	

	// Set \y as the upVector, then the projected 3D BB is still BB in 3D
	activeViewer->camera()->setUpVector(Vec(0,1,0));
	Vec delta(1, 1, 1);
	delta *= activeObject()->radius * 0.2;
	activeViewer->camera()->fitBoundingBox(Vec(activeObject()->bbmin) - delta, Vec(activeObject()->bbmax) + delta);

	// Save this new camera settings
	activeViewer->camera()->deletePath(direction + 2);
	activeViewer->camera()->addKeyFrameToPath(direction + 2);

	// Render
	activeViewer->setMode(HV_DEPTH);
	activeViewer->updateGL(); 

	// compute the envelope
	computeEnvelope(direction);
}

void Offset::computeEnvelopeOfRegion( int direction , Vec3d bbmin, Vec3d bbmax )
{
	// Set camera
	activeViewer->camera()->setType(Camera::ORTHOGRAPHIC);
	Vec3d bbCenter = (bbmin + bbmax) / 2;
	Vec3d pos = bbCenter;
	pos[2] = (1==direction)? bbmax.z() : bbmin.z();
	activeViewer->camera()->setPosition(Vec(pos));
	activeViewer->camera()->lookAt(Vec(bbCenter));	
	activeViewer->camera()->setUpVector(Vec(0,1,0));
	activeViewer->camera()->fitBoundingBox(Vec(bbmin), Vec(bbmax));

	// Save this new camera settings
	activeViewer->camera()->deletePath(direction + 3);
	activeViewer->camera()->addKeyFrameToPath(direction + 3);

	// Render
	activeViewer->setMode(HV_DEPTH);
	activeViewer->updateGL(); 

	// Compute
	computeEnvelope(direction);
}

void Offset::computeOffset()
{
	offset = upperEnvelope; 

	int h = upperEnvelope.size();
	int w = upperEnvelope[0].size();

	#pragma omp parallel for
	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++)
		{
			// Two envelopes are horizontally flipped
			if (upperEnvelope[y][x]== BIG_NUMBER | lowerEnvelope[y][(w-1)-x] == -BIG_NUMBER)
				offset[y][x] = 0.0; 
			else
				offset[y][x] = upperEnvelope[y][x] - lowerEnvelope[y][(w-1)-x];
		}
	}
}

double Offset::computeOffsetOfShape()
{
	if (!activeObject()) return 0;

	// Compute the height of the shape
	activeObject()->computeBoundingBox();
	objectH = (activeObject()->bbmax - activeObject()->bbmin).z();

	// Save original camera settings
	activeViewer->camera()->deletePath(0);
	activeViewer->camera()->addKeyFrameToPath(0);

	// Compute the offset function
	computeEnvelopeOfShape(1);
	computeEnvelopeOfShape(-1);
	computeOffset();

	// Update the stackability in QSegMesh
	O_max = getMaxValue(offset);
	activeObject()->O_max = O_max;
	activeObject()->stackability = 1 - O_max/objectH;

	// Save offset as image
	//saveAsImage(offset, O_max, "offset function.png");

	return activeObject()->stackability;
}

void Offset::computeOffsetOfRegion( std::vector< Vec2i >& region )
{
	// BB of hot 2D region
	Vec2i bbmin_region, bbmax_region;
	BBofRegion(region, bbmin_region, bbmax_region);

	// Project BB of shape to 2D
	Vec3d bbmin = activeObject()->bbmin;
	Vec3d bbmax = activeObject()->bbmax;
	Vec2i bbmin_shape = projectedCoordinatesOf(bbmin, 3);
	Vec2i bbmax_shape = projectedCoordinatesOf(bbmax, 3);


	// Shrink the BB of shape along x and y direction to contain the \region only
	Vec2i range_2D = bbmax_shape - bbmin_shape;
	Vec2i diff_min_2D = bbmin_region - bbmin_shape;
	Vec2i diff_max_2D = bbmax_region - bbmin_shape;

	Vec3d rang_3D = bbmax - bbmin;

	for (int i=0;i<2;i++)
	{
		double alpha = double(diff_min_2D[i]) / range_2D[i];
		double beta = double(diff_max_2D[i]) / range_2D[i];

		bbmax[i] = bbmin[i] + rang_3D[i] * beta;
		bbmin[i] = bbmin[i] + rang_3D[i] * alpha;
	}

	// Envelopes
	computeEnvelopeOfRegion(1, bbmin, bbmax );
	computeEnvelopeOfRegion(-1,  bbmin, bbmax );

	// Offset
	computeOffset();

	// Save offset as image
	//saveAsImage(offset, O_max, "offset function of region.png");
}

double Offset::getStackability()
{
	return 1 - O_max/objectH;
}



Offset::HotSpot Offset::detectHotspotInRegion(int direction, std::vector<Vec2i> &hotRegion)
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 3 );

	// Draw Faces Unique
	activeViewer->setMode(HV_FACEUNIQUE);
	activeViewer->updateGL(); 

	GLubyte* colormap = (GLubyte*)activeViewer->readBuffer(GL_RGBA, GL_UNSIGNED_BYTE);

	// The size of current viewer
	int w = activeViewer->width();
	int h = activeViewer->height();	

	// Switch between directions
	bool isUpper = (direction == 1);
	std::vector< std::vector<double> > &depth = isUpper? upperDepth : lowerDepth;

	// Detect hot spots
	uint sid, fid, fid_local;
	uint x, y;

	QMap< QString,  int > subHotRegionSize;
	QMap< QString,  std::vector< Vec3d > > subHotSamples;

//	QImage debugImg(w, h, QImage::Format_ARGB32);
	
	for (int j=0;j<hotRegion.size();j++)
	{
		// 2D coordinates in OpenGL format
		Vec2i &hotPixel = hotRegion[j];
		x = (direction == 1) ? hotPixel.x() : (w-1) - hotPixel.x();
		y = hotPixel.y();

		// 3d position of this hot sample
		// Flip \y to work in Qt format
		double depthVal = depth[y][x];
		Vec hotP= activeViewer->camera()->unprojectedCoordinatesOf(Vec(x, (h-1)-y, depthVal));

		// Get the face index and segment index back
		uint indx = ((y*w)+x)*4;
		uint r = (uint)colormap[indx+0];
		uint g = (uint)colormap[indx+1];
		uint b = (uint)colormap[indx+2];
		uint a = (uint)colormap[indx+3];

		fid = ((255-a)<<24) + (r<<16) + (g<<8) + b - 1;

		if (fid >= activeObject()->nbFaces()) 
			continue;

		activeObject()->global2local_fid(fid, sid, fid_local);

		// Store informations
		QString segmentID = activeObject()->getSegment(sid)->objectName();
		Vec3d hotPoint(hotP.x, hotP.y, hotP.z);
		subHotRegionSize[segmentID]++;
		subHotSamples[segmentID].push_back(hotPoint);
		hotPoints[segmentID].push_back(hotPoint);

		// debuging
//		debugImg.setPixel(x, y, QColor(r, g, b).rgb());
	}

//	debugImg.save("face unique.png");
	// Create hot spot
	HotSpot HS;
	if (subHotRegionSize.isEmpty())
	{
		HS.side = 0;
	}
	else
	{
		int largestRegionSize = 0;
		foreach(QString id, subHotRegionSize.keys())
		{
			if (subHotRegionSize[id] > largestRegionSize)
			{
				largestRegionSize = subHotRegionSize[id];
				HS.segmentID = id;
			}
		}

		HS.side = direction;
		HS.hotSamples = subHotSamples[HS.segmentID];
		HS.isRing = activeObject()->controller->getPrimitive(HS.segmentID)->isRotationalSymmetry;
	}

	return HS;
}

void Offset::detectHotspots( )
{
	// Initialization
	clear();

	// Recompute the offset function of the entire shape
	computeOffsetOfShape();

	// Detect hot regions
	hotRegions = getRegions(offset, std::bind2nd(std::greater<double>(), O_max * HOT_RANGE));
	//visualizeRegions(hotRegions, "hot regions of shape.png");

	// The max offset of hot regions
	maxOffsetInHotRegions.clear();
	for (int i=0;i<hotRegions.size();i++){
		maxOffsetInHotRegions.push_back(maxValueInRegion(offset, hotRegions[i]));
	}

	// The max of \UpperEnvelope and min of \LowerEnvelope
	double maxUE = getMaxValue(upperEnvelope);
	double minLE = getMinValue(lowerEnvelope);

	// Zoom into each hot region
	int h = activeViewer->height();
	int w = activeViewer->width();
	for (int i=0;i<hotRegions.size();i++)
	{
		computeOffsetOfRegion(hotRegions[i]);

		// Detect zoomed in hot region
		std::vector< std::vector<Vec2i> > zoomedHRs
			=  getRegions(offset, std::bind2nd(std::greater<double>(), O_max * HOT_RANGE));
		if (zoomedHRs.size() == 0){
			std::cout << "There is no hot region in the zoomed-in view.\n";
			continue;
		}

		// If there are multiple regions, pick up the one closest to the center
		//visualizeRegions(zoomedHRs, "zoomed in hot regions.png");
		Vec2i center(w/2, h/2);
		int minDis = w;
		int closestID = -1;
		for (int j=0;j<zoomedHRs.size();j++)
		{
			Vec2i c = centerOfRegion(zoomedHRs[j]);
			int dx = c.x() - center.x();
			int dy = c.y() - center.y();
			int dis = dx * dx + dy * dy;
			if (dis < minDis)
			{
				minDis = dis;
				closestID = j;
			}
		}

		std::vector<Vec2i> &zoomedHR = zoomedHRs[closestID];

		// Detect hot spots from both directions
		HotSpot UHS = detectHotspotInRegion(1, zoomedHR);
		if (0 == UHS.side) continue;
		HotSpot LHS = detectHotspotInRegion(-1, zoomedHR);
		if (0 == LHS.side) continue;

		UHS.hotRegionID = i;
		LHS.hotRegionID = i;

		std::vector< double > valuesU = getValuesInRegion(upperEnvelope, zoomedHR, false);
		UHS.defineHeight = MaxElement(valuesU) > (maxUE - ZERO_TOLERANCE);
		std::vector< double > valuesL = getValuesInRegion(lowerEnvelope, zoomedHR, true);
		LHS.defineHeight = MinElement(valuesL) < (minLE + ZERO_TOLERANCE);


		upperHotSpots.push_back(UHS);
		lowerHotSpots.push_back(LHS);
	}

	if(upperHotSpots.size() + lowerHotSpots.size() == 0)
		return;

	// Show results in the std output
	std::cout << "Hot spots: " << std::endl;
	for (int i=0;i<upperHotSpots.size();i++)
	{
		upperHotSpots[i].print();
		lowerHotSpots[i].print();
	}
	std::cout << std::endl;

	// Hot segments
	hotSegments.clear();
	for (int i=0;i<upperHotSpots.size();i++)
	{
		hotSegments.insert(upperHotSpots[i].segmentID);
		hotSegments.insert(lowerHotSpots[i].segmentID);
	}
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
	for (std::map< QString, std::vector< Vec3d > >::iterator i=hotPoints.begin();i!=hotPoints.end();i++)
	{
		QString sid = i->first;
		QSurfaceMesh* segment = activeObject()->getSegment(sid);

		segment->setColorVertices(Color(1, 0, 0, 1)); // red

		for (std::vector< Vec3d >::iterator pit = i->second.begin(); pit != i->second.end(); pit++)
		{
			segment->debug_points.push_back(*pit);
		}
	}
}

std::set<QString> Offset::getHotSegment()
{
	std::set< QString > hs;

	for (std::map< QString, std::vector< Vec3d > >::iterator i=hotPoints.begin();i!=hotPoints.end();i++)
		hs.insert(i->first);

	return hs;
}


template< typename T >
std::vector< std::vector < T > > Offset::createImage( int w, int h, T intial )
{
	return std::vector< std::vector < T > > ( h, std::vector<T>( w, intial ) );
}

void Offset::saveAsImage( std::vector< std::vector < double > >& image, double maxV, QString fileName )
{
	int h = image.size();
	int w = image[0].size();
	QImage Output(w, h, QImage::Format_ARGB32);
	
	// \p y is flipped, since OpenGL has origin at the left bottom conner, while Qt at left top conner
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			Output.setPixel(x, (h-1)-y, jetColor( Max(0., image[y][x] / maxV), 0., 1.));			
		}
	}

	Output.save(fileName);
}

void Offset::saveAsData( std::vector< std::vector < double > >& image, double maxV, QString fileName )
{
	int h = image.size();
	int w = image[0].size();

	QFile file(fileName); 
	file.open(QIODevice::WriteOnly | QIODevice::Text);

	// \p y is flipped, since OpenGL has origin at the left bottom conner, while Qt at left top conner
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			file.write(qPrintable(QString::number(image[y][x] / maxV) + "\t"));
		}
		file.write("\n");
	}

	file.close();
}

void Offset::saveHotSpots( QString filename, int direction, double percent)
{
	std::vector< HotSpot > curHotSpot = upperHotSpots;
	if(direction < 0) curHotSpot = lowerHotSpots;

	QFile file(filename); 
	file.open(QIODevice::WriteOnly | QIODevice::Text);

	foreach(HotSpot hp, curHotSpot){
		for(uint i = 0; i < Max(1, percent * hp.hotSamples.size()); i++)
		{
			Point p = hp.hotSamples[i];
			QString line = QString("%1 %2 %3\n").arg(p.x()).arg(p.y()).arg(p.z());
			file.write(qPrintable(line));

		}

		file.write("\n");
	}

	file.close();
}

void Offset::saveAsImage( std::vector< std::vector < bool > >& image, QString fileName )
{
	int h = image.size();
	int w = image[0].size();
	QImage Output(w, h, QImage::Format_ARGB32);

	QRgb red = QColor::fromRgb(255, 0, 0).rgba();
	QRgb blue = QColor::fromRgb(0, 0, 255).rgba();

	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			QRgb color = image[y][x] ? red : blue;
			Output.setPixel(x, (h-1)-y, color);			
		}
	}

	Output.save(fileName);
}


double Offset::getValue( std::vector< std::vector < double > >& image, int x, int y, int r )
{
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

double Offset::maxValueInRegion( std::vector< std::vector < double > >& image,  std::vector< Vec2i >& region )
{
	double result = 0;
	for (int i=0;i<region.size();i++)
	{
		int x = region[i].x();
		int y = region[i].y();

		if (image[y][x] > result)
			result = image[y][x];
	}

	return result;
}

template< typename PREDICATE >
std::vector< Vec2i > 
	Offset::getRegion( std::vector< std::vector < double > >& image, 
			std::vector< std::vector < bool > >& mask, 	Vec2i seed, PREDICATE predicate )
{
	std::vector< Vec2i > region;

	int w = image[0].size();
	int h = image.size();		
	
	std::stack<Vec2i> activePnts;
	activePnts.push(seed);
	mask[seed.y()][seed.x()] = true;

	while (!activePnts.empty())
	{
		// Add the top point to region
		Vec2i currP = activePnts.top();
		region.push_back(currP);
		activePnts.pop();

		// Push all the neighbors to the stack
		int min_x = RANGED(0, currP.x()-1 ,w-1);
		int max_x = RANGED(0, currP.x()+1, w-1);
		int min_y = RANGED(0, currP.y()-1 ,h-1);
		int max_y = RANGED(0, currP.y()+1, h-1);

		for (int y = min_y; y <= max_y; y++)
			for (int x = min_x; x <= max_x; x++)
			{
				if (!mask[y][x] && predicate(image[y][x]))
				{
					activePnts.push( Vec2i(x, y) );
					mask[y][x] = true;
				}
			}
	}

	return region;
}


template< typename PREDICATE >
std::vector< std::vector< Vec2i > >
	Offset::getRegions( std::vector< std::vector < double > >& image, PREDICATE predicate )
{
	std::vector< std::vector< Vec2i > > regions;

	int w = image[0].size();
	int h = image.size();

	std::vector< std::vector< bool > > mask = createImage(w, h, false);

	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			if (!mask[y][x] && predicate(image[y][x]))
			{
				//saveAsImage(mask, "mask1.png");
				regions.push_back(getRegion(image, mask, Vec2i(x, y), predicate));
				//saveAsImage(mask, "mask2.png");
			}

			mask[y][x] = true;
		}
	}

	return regions;
}

void Offset::clear()
{
	lowerEnvelope.clear();
	upperEnvelope.clear();
	offset.clear();

	lowerDepth.clear();
	upperDepth.clear();

	hotRegions.clear();
	hotPoints.clear();
	upperHotSpots.clear();
	lowerHotSpots.clear();
	hotSegments.clear();
}

std::vector< double > Offset::getValuesInRegion( std::vector< std::vector < double > >& image, 
												 std::vector< Vec2i >& region, bool xFlipped /*= false*/ )
{
	std::vector< double > values;

	int w = image[0].size();
	uint x, y;
	for (int i = 0; i < region.size(); i++)
	{
		x = region[i].x();
		y = region[i].y();
		if(xFlipped) x = (w-1) - x;

		values.push_back(image[y][x]);
	}

	return values;
}

void Offset::BBofRegion( std::vector< Vec2i >& region, Vec2i &bbmin, Vec2i &bbmax )
{
	uint minX, maxX, minY, maxY;
	minX = maxX = region[0].x();
	minY = maxY = region[0].y();
	for (int i=1; i<region.size(); i++)
	{
		uint x = region[i].x();
		uint y = region[i].y();

		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
	}

	bbmin = Vec2i(minX, minY);
	bbmax = Vec2i(maxX, maxY);
}


std::vector< Vec2i > Offset::deltaVectorsToKRing( int deltaX, int deltaY, int K )
{
	std::vector< Vec2i > Vecs;

	int leftX = - deltaX * K;
	int rightX = -leftX;
	int topY = - deltaY * K;
	int bottomY = -topY;

	// top and bottom lines
	for (int x = leftX; x <= rightX; x += deltaX)
	{
		Vecs.push_back( Vec2i(x, topY) );
		Vecs.push_back( Vec2i(x, bottomY) );
	}

	// left and right lines
	for (int y = topY + deltaY; y <= bottomY - deltaY; y+= deltaY)
	{
		Vecs.push_back( Vec2i(leftX, y) );
		Vecs.push_back( Vec2i(rightX, y) );
	}

	return Vecs;
}


std::vector< Vec2i > Offset::shiftRegionInBB( std::vector< Vec2i >& region, Vec2i delta, Vec2i bbmin, Vec2i bbmax )
{
	std::vector< Vec2i > toRegion;

	for (int i = 0; i < region.size(); i++)
	{
		Vec2i p = region[i] + delta;
		if ( RANGE( p.x(), bbmin.x(), bbmax.x() ) && RANGE( p.y(), bbmin.y(), bbmax.y() ) )
			toRegion.push_back( p );
		else
		{// Out of BB
			toRegion.clear();
			break;
		}			
	}

	return toRegion;
}

Vec2i Offset::sizeofRegion( std::vector< Vec2i >& region )
{
	uint minX, maxX, minY, maxY;
	minX = maxX = region[0].x();
	minY = maxY = region[0].y();
	for (int i=1; i<region.size(); i++)
	{
		uint x = region[i].x();
		uint y = region[i].y();

		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
	}

	return Vec2i( maxX - minX + 1, maxY - minY + 1 );
}

Vec3d Offset::unprojectedCoordinatesOf( uint x, uint y, int direction )
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 2 );
	activeViewer->updateGL();

	int w = activeViewer->height();
	int h = activeViewer->width();

	std::vector< std::vector<double> > &depth = (direction == 1)? upperDepth : lowerDepth;
	if (direction == -1)	x = (w-1) - x;

	Vec P = activeViewer->camera()->unprojectedCoordinatesOf(Vec(x, (h-1)-y, depth[y][x]));

	return Vec3d(P[0], P[1], P[2]);
}

Vec2i Offset::projectedCoordinatesOf( Vec3d point, int pathID )
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( pathID );

	// Make sure to call /updateGL() to update the projectionMatrix!!!
	activeViewer->updateGL();

	// \p is expressed in the Qt coordinates, (0, 0) being at the top left conner
	Vec p = activeViewer->camera()->projectedCoordinatesOf( Vec (point[0], point[1], point[2]) );

	// Convert to OpenGL coordinates
	int h = activeViewer->height();
	return Vec2i(p[0], (h-1)-p[1]);
}

void Offset::setRegionColor( std::vector< std::vector < double > >& image, std::vector< Vec2i >& region, double color )
{
	uint w = image[0].size();
	uint h = image.size();

	for (int i = 0; i < region.size(); i++)
	{
		uint x = RANGED(0, region[i].x(), w-1);
		uint y = RANGED(0, region[i].y(), h-1);

		image[y][x] = color;
	}
}

void Offset::setPixelColor( std::vector< std::vector < double > >& image, Vec2i pos, double color )
{
	uint w = image[0].size();
	uint h = image.size();

	uint x = RANGED(0, pos.x(), w-1);
	uint y = RANGED(0, pos.y(), h-1);

	image[y][x] = color;
}

QRgb Offset::jetColor( double val, double min, double max )
{
	uchar rgb[3];	

	ColorMap::jetColorMap(rgb, val, min, max);
	
	return QColor::fromRgb(rgb[0],rgb[1],rgb[2]).rgba();
}

void Offset::visualizeRegions( std::vector< std::vector<Vec2i> >& regions, QString filename )
{
	int w = activeViewer->width();
	int h = activeViewer->height();
	std::vector< std::vector< double > > debugImg = createImage(w, h, 0.0);
	double step = 1.0 / regions.size();
	for (int i=0;i<regions.size();i++)
	{
		setRegionColor(debugImg, regions[i], step * (i+1));
	}
	saveAsImage(debugImg, 1.0, filename);
}


std::vector< Vec3d > Offset::getHorizontalMoves( HotSpot& HS )
{
	std::vector< Vec3d > Ts;

	// The hot region	
	std::vector< Vec2i > &hotRegion = hotRegions[HS.hotRegionID];

	// Recompute the OPPOSITE envelope using hot segments only
	computeEnvelopeOfShape(-HS.side);

	// Switchers
	bool isUpper = (HS.side == 1);
	bool x_flipped = isUpper;
	std::vector< std::vector<double> > &op_envelope = isUpper ? lowerEnvelope : upperEnvelope;

	// Project BB to 2D buffer
	//std::vector< std::vector< double > > BBImg = createImage( offset[0].size(), offset.size(), 0. );
	Vec2i bbmin = projectedCoordinatesOf(pre_bbmin, 3);
	Vec2i bbmax = projectedCoordinatesOf(pre_bbmax, 3);
	//setPixelColor(BBImg, bbmin, 1.0);
	//setPixelColor(BBImg, bbmax, 0.5);
	//saveAsImage(BBImg, 1.0, "BB projection.png");

	// Opposite envelope at current position
	std::vector< double > curr_op_env = getValuesInRegion(op_envelope, hotRegion, x_flipped);
	double threshold = isUpper? MaxElement(curr_op_env) : MinElement( curr_op_env );

	//============ debug code
	//std::vector< std::vector< double > > debugImg = createImage( offset[0].size(), offset.size(), 0. );
	//setRegionColor(debugImg, hotRegion, 1.0);
	//============ end of debug code

	// The size of the hot region
	Vec2i size = sizeofRegion(hotRegion);
	int step = Max(size.x(), size.y());

	// Search for optional locations in 1-K rings
	int K = 5;
	for (uint k = 1; k < K; k++)
	{
		std::vector< Vec2i > deltas = deltaVectorsToKRing(step, step, k);
		for (int j = 0; j < deltas.size(); j++ )
		{
			std::vector< Vec2i > shiftedRegion = shiftRegionInBB(hotRegion, deltas[j], bbmin, bbmax);
			if (shiftedRegion.empty())  continue; // Out of BB

			////============ debug code
			//setRegionColor(debugImg, shiftedRegion, 1.0 - k * 0.1 );
			////============ end of debug code

			std::vector< double > new_op_env = getValuesInRegion(op_envelope, shiftedRegion, x_flipped);
			double newValue = isUpper? MinElement( new_op_env ) : MaxElement( new_op_env );
			bool isBetter = isUpper? (newValue > threshold) : (newValue < threshold);

			if (isBetter)
			{
				// Get the 3D translation
				Vec2i a = hotRegion[0];
				Vec2i b = shiftedRegion[0];
				Vec3d proj_a = unprojectedCoordinatesOf(a.x(), a.y(), HS.side);
				Vec3d proj_b = unprojectedCoordinatesOf(b.x(), b.y(), HS.side);
				Vec3d T = proj_b - proj_a;
				// Force the translation to be horizontal
				T[2] = 0;

				Ts.push_back(T);
			}
		}
	}

//	saveAsImage(debugImg, 1.0, "K Ring Neighbors.png");

	return Ts;
}

void Offset::applyHeuristicsOnHotspot( HotSpot &HS, HotSpot&opHS )
{

	Controller *ctrl = activeObject()->controller;
	Primitive* prim = ctrl->getPrimitive(HS.segmentID);
	Primitive* op_prim = ctrl->getPrimitive(opHS.segmentID);

	// Find the representative hot samples
	Point hotPoint = HS.hotPoint();
	Point op_hotPoint = opHS.hotPoint();

	// Save the initial hot shape state
	ShapeState initialHotShapeState = ctrl->getShapeState();

	// Move hot spot side away or closer to each other
	std::vector< Vec3d > Ts = getHorizontalMoves(HS);
	
	if (!HS.defineHeight)
	{// Move up/down directly
		Vec3d T(0, 0, 0.1 * objectH);
		if (1 == HS.side) T *= -1;
		Ts.push_back( T );
	}

//	Ts.push_back(Vec3d(-0.2, 0.4, 0));
	// Actually modify the shape to generate hot solutions
	for (int i=0;i<Ts.size();i++)
//	int i = Ts.size() - 1;
	{
		// Clear the frozen flags
		ctrl->setPrimitivesFrozen(false);		
		
		// Fix the opposite hot spot
		op_prim->addFixedPoint(op_hotPoint); 

		// Move the current hot spot
		prim->movePoint(hotPoint, Ts[i]);
		prim->addFixedPoint(hotPoint + Ts[i]);

			
		// fix the hot segments pair
		ctrl->regroupPair(prim->id, op_prim->id);

		// Propagation the deformation
		prim->isFrozen = true;
		op_prim->isFrozen = true;
		ctrl->weakPropagate();

		// Check if this is a candidate solution

		if ( satisfyBBConstraint() )
		{
			double stackability = computeOffsetOfShape();
			if (stackability > preStackability + 0.1)
			{
				ShapeState state = ctrl->getShapeState();
				if (isUnique(state, 4))
				{
					state.stackability = stackability;
					candidateSolutions.enqueue(state);
				}				
			}
		}


		// Restore the initial hot shape state
		ctrl->setShapeState(initialHotShapeState);
	}
}

void Offset::applyHeuristicsOnHotRing( HotSpot& HS )
{
	Controller *ctrl = activeObject()->controller;
	Primitive* prim = ctrl->getPrimitive(HS.segmentID);

	// The hot region	
	std::vector< Vec2i > &hotRegion = hotRegions[HS.hotRegionID];

	Vec2i center = centerOfRegion(hotRegion);
	Vec2i p = hotRegion[0];

	// Translations
	std::vector< Vec3d > Ts;
	double s = 0.15;
	Ts.push_back(Vec3d(1,0,0)*s);
	Ts.push_back(Vec3d(-1,0,0)*s);
	//Vec3d proj_c = unprojectedCoordinatesOf(center.x(), center.y(), HS.side);
	//Vec3d proj_p = unprojectedCoordinatesOf(p.x(), p.y(),  HS.side);
	//double step = 0.5 * activeObject()->radius;
	//Vec3d T = proj_p - proj_c;
	//T[2] = 0; 
	//T .normalize();
	//Ts.push_back(T * step);
	//Ts.push_back(-T * step);

	// Save the initial hot shape state
	ShapeState initialHotShapeState = ctrl->getShapeState();


	Vec3d hotPoint = HS.hotSamples[0];
	for (int i=0;i<Ts.size();i++)
	{
		// Clear the frozen flags
		ctrl->setPrimitivesFrozen(false);		

		// Move the current hot spot
		prim->movePoint(hotPoint, Ts[i]);

		// Propagation the deformation
		prim->isFrozen = true;

		ctrl->weakPropagate();

		// Check if this is a candidate solution

//		if ( satisfyBBConstraint() )
		{
			double stackability = computeOffsetOfShape();
//			if (stackability > preStackability + 0.1)
			{
				ShapeState state = ctrl->getShapeState();
//				if (isUnique(state, 0))
				{
					state.stackability = stackability;
					candidateSolutions.enqueue(state);
				}				
			}
		}
		
		// Restore the initial hot shape state
		ctrl->setShapeState(initialHotShapeState);
	}


}


void Offset::applyHeuristics()
{
	Controller *ctrl = activeObject()->controller;
	
	// Set only hot segments visible
	//ctrl->setSegmentsVisible(false);
	//foreach (QString segmentID, hotSegments)
	//	activeObject()->getSegment(segmentID)->isVisible = true;


	// After getting rid of redundancy caused by symmetries, hopefully only one pair of hot spots remains
	// For now, only apply heuristics on the hot region that has maximum offset
	int selectedID = -1;
	for (int i=0;upperHotSpots.size();i++)
	{
		int rid = upperHotSpots[i].hotRegionID;
		if (maxOffsetInHotRegions[rid] == O_max)
		{
			selectedID = i;
			break;
		}
	}

	if (-1 == selectedID)
	{
		std::cout << "There is no hot region contians O_max.\n";
		return;
	}

	HotSpot& upperHS = upperHotSpots[selectedID];
	HotSpot& lowerHS = lowerHotSpots[selectedID];

	if (upperHS.isRing)
		applyHeuristicsOnHotRing(upperHS);
	else
		applyHeuristicsOnHotspot(upperHS, lowerHS);		


	if (lowerHS.isRing)
		applyHeuristicsOnHotRing(lowerHS);
	else
		applyHeuristicsOnHotspot(lowerHS, upperHS);	
}

void Offset::improveStackability()
{
	Controller *ctrl = activeObject()->controller;

	// improve the stackability of current shape in three steps
	//=========================================================================================
	// Step 1: Detect hot spots
	computeOffsetOfShape();
	preStackability = getStackability();

	HOT_RANGE = 0.95;
	detectHotspots();
	while (upperHotSpots.empty())
	{
		HOT_RANGE -= 0.05;
		detectHotspots();

		if (HOT_RANGE < 0.8)
		{
			std::cout << "Hot spots detection failed.\n";
			return;
		}
	}

	//=========================================================================================
	// Step 2: Apply heuristics on hot spots and propagate deformations
	// Several hot solutions might be generated, which are stored in *hotSolutions*
	applyHeuristics();
}

void Offset::improveStackabilityTo( double targetS )
{
	Controller *ctrl = activeObject()->controller;

	// Clear
	candidateSolutions.clear();
	usedCandidateSolutions.clear();
	solutions.clear();

	// The bounding box constraint is hard
	pre_bbmin = activeObject()->bbmin;
	pre_bbmax = activeObject()->bbmax;
	pre_bbmin[0] *= 1.1;
	pre_bbmin[2] *= 1.1;
	pre_bbmax[0] *= 1.1;
	pre_bbmax[2] *= 1.1;

	// Push the current shape as the initial candidate solution
	ShapeState state = ctrl->getShapeState();
	state.stackability = getStackability();
	candidateSolutions.enqueue(state);

//	while(!candidateSolutions.empty())
	{

		// Get the first candidate solution
		ShapeState candidateState = candidateSolutions.dequeue();
		usedCandidateSolutions.push_back(candidateState);

		if ( candidateState.stackability >= targetS)
		{
			// Yes. Got one solution.
			solutions.push_back(candidateState);
//			continue;
		}
		else
		{
			// No. Try to improve the current shape
			// This iteration might generate several candidate solutions
			ctrl->setShapeState(candidateState);
			improveStackability();
		}


		std::cout << "#Candidates = " << candidateSolutions.size() << std::endl;
		std::cout << "#Solutions = " << solutions.size() << std::endl;
	}

	// Debug: add candidate solutions to solutions
	foreach (ShapeState state, candidateSolutions)
		solutions.push_back(state);


	// Cluster the solutions to get rid of redundancies

	ctrl->setShapeState(state);
	activeObject()->computeBoundingBox();
}


void Offset::showSolution( int i )
{
	if (solutions.empty())
	{
		std::cout << "There is no solution.\n";
		return;
	}

	int id = i % solutions.size();
	Controller *ctrl = activeObject()->controller;
	ctrl->setShapeState(solutions[id]);

	std::cout << "Showing the " << id << "th solution out of " << solutions.size() <<".\n";
}

bool Offset::satisfyBBConstraint()
{
	bool result = true;

	Vec3d preBB = pre_bbmax - pre_bbmin;
	activeObject()->computeBoundingBox();
	Vec3d currBB = activeObject()->bbmax - activeObject()->bbmin;

	Vec3d diff = preBB - currBB;
	if ( diff[0] < 0 || diff[2] < 0 )
		result = false;

	if( abs(diff[1]) > 0.1 )
		result = false;

	// debug
	//std::cout << "-----------------------------------\n"
	//	<<"The preBB size: (" << preBB <<")\n";	
	//std::cout << "The currBB size:(" << currBB <<")\n";
	//std::cout << "BB-satisfying: " << result <<std::endl;


	return result;
}

bool Offset::isUnique( ShapeState state, double threshold )
{
	bool result = true;
	Controller *ctrl = activeObject()->controller;

	// \state should be dissimilar to all (used)candidate solutions
	foreach(ShapeState candState, usedCandidateSolutions)
	{
		double sim = ctrl->similarity(candState, state);
		if( sim < threshold)
		{
			result = false;
			break;
		}
//		std::cout << sim << '\t';
	}

	if (result)
	{
		foreach(ShapeState candState, candidateSolutions)
		{
			double sim = ctrl->similarity(candState, state);
			if( sim < threshold)
			{
				result = false;
				break;
			}
//			std::cout << sim << '\t';
		}
	}

	//std::cout << "Unique: " << result <<std::endl;

	return result;
}

Vec2i Offset::centerOfRegion( std::vector< Vec2i >& region )
{
	Vec2i center(0, 0);

	if (!region.empty())
	{
		for (int i=0;i<region.size();i++)
			center += region[i];

		center /= region.size();
	}

	return center;
}




