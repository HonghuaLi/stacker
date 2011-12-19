#include "Offset.h"
#include "HiddenViewer.h"
#include "ColorMap.h"
#include "SimpleDraw.h"

#include <numeric>
#include <stack>

#define ZERO_TOLERANCE 0.05
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
	Vec delta(1, 1, 1);
	delta *= activeObject()->radius * 0.2;
	activeViewer->camera()->fitBoundingBox(Vec(activeObject()->bbmin) - delta, Vec(activeObject()->bbmax) + delta);

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
				envelope[y][x] = (direction == 1) ? -BIG_NUMBER : BIG_NUMBER;
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
			if (upperEnvelope[y][x]== BIG_NUMBER | lowerEnvelope[y][(w-1)-x] == -BIG_NUMBER)
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

	// Save offset as image
	saveAsImage(offset, O_max, "offset function.png");
}

double Offset::getStackability()
{
	return 1 - O_max/objectH;
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

	// Switch between directions
	bool isUpper = (direction == 1);
	std::vector< std::vector<double> > &depth = isUpper? upperDepth : lowerDepth;
	std::vector< HotSpot > &hotSpots = isUpper? upperHotSpots : lowerHotSpots;
	hotSpots.clear();

	// Detect hot spots
	std::vector< std::vector<Vec2i> > newHotRegions;

	uint sid, fid, fid_local;
	uint x, y;
	for (int i=0;i<hotRegions.size();i++)	{

		std::map< QString,  std::vector< Vec2i > > subHotRegions;
		std::map< QString,  std::vector< Vec3d > > subHotSamples;

		for (int j=0;j<hotRegions[i].size();j++){

			Vec2i &hotPixel = hotRegions[i][j];
			x = (direction == 1) ? hotPixel.x() : (w-1) - hotPixel.x();
			y = hotPixel.y();

			// 3d position of this hot sample
			double depthVal = getValue(depth, x, y, filterSize);

			if (depthVal == BIG_NUMBER) 
				continue;

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
			hotPoints[sid].push_back(hotPoint);
			subHotRegions[segmentID].push_back(hotPixel);
			subHotSamples[segmentID].push_back(hotPoint);
		}

		// Split the hot region if necessary
		std::map< QString,  std::vector< Vec2i > >::iterator itr;
		for ( itr = subHotRegions.begin(); itr != subHotRegions.end(); itr++)
		{
			if (itr->second.size() < 10)
				continue; // a bad way to get rid of noise

			QString segmentID = itr->first;
			newHotRegions.push_back(itr->second);

			HotSpot HS;
			HS.hotRegionID = hotSpots.size();
			HS.segmentID = segmentID;
			HS.defineHeight = defineHeight( direction, itr->second );
			HS.hotSamples = subHotSamples[segmentID];
			hotSpots.push_back(HS);	
		}

	}

	// Update hotRegions
	hotRegions = newHotRegions;

	delete colormap;	
}

void Offset::detectHotspots( int useFilterSize, double hotRange )
{
	filterSize = useFilterSize;
	hotRangeThreshold = hotRange;

	// Initialization
	clear();

	// Recompute envelopes and offset
	computeOffset();

	// Detect hot regions
	hotRegions = getRegions(offset, std::bind2nd(std::greater<double>(), O_max * hotRange));
	visualizeHotRegions("hot regions 1.png");

	// Detect hot spots from both directions
	hotspotsFromDirection(1);
	hotspotsFromDirection(-1);
	if (upperHotSpots.size() < lowerHotSpots.size())
	{
		// Lower hot spots are splitted, so re-detect upper hot spots
		hotspotsFromDirection(1);
		std::cout << "Lower hot spots are splitted. \n";
	}
	visualizeHotRegions("hot regions 2.png");
	std::cout << "Hot spots: " << std::endl;
	for (int i=0;i<hotRegions.size();i++)
	{
		upperHotSpots[i].print();
		lowerHotSpots[i].print();
	}
	std::cout << std::endl;

	// Hot segments
	for (int i=0;i<hotRegions.size();i++)
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
	for (std::map< uint, std::vector< Vec3d > >::iterator i=hotPoints.begin();i!=hotPoints.end();i++)
	{
		uint sid = i->first;
		QSurfaceMesh* segment = activeObject()->getSegment(sid);

		segment->setColorVertices(Color(1, 0, 0, 1)); // red

		for (std::vector< Vec3d >::iterator pit = i->second.begin(); pit != i->second.end(); pit++)
		{
			segment->debug_points.push_back(*pit);
		}
	}
}

std::set<uint> Offset::getHotSegment()
{
	std::set< uint > hs;

	for (std::map< uint, std::vector< Vec3d > >::iterator i=hotPoints.begin();i!=hotPoints.end();i++)
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
	
	for(int y = 0; y < h; y++){
		for(int x = 0; x < w; x++)	{
			Output.setPixel(x, y, jetColor( Max(0., image[y][x] / maxV), 0., 1.));			
		}
	}

	Output.save(fileName);
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
			Output.setPixel(x, y, color);			
		}
	}

	Output.save(fileName);
}


double Offset::getValue( std::vector< std::vector < double > >& image, uint x, uint y, uint r )
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

bool Offset::defineHeight( int direction, std::vector< Vec2i >& region )
{
	std::vector< double > values;
	bool result;

	if ( direction == 1 )
	{
		values = getValuesInRegion( upperEnvelope, region, false);		
		result = MaxElement(values) > getMaxValue(upperEnvelope) - ZERO_TOLERANCE;
	}
	else
	{
		values = getValuesInRegion( lowerEnvelope, region, true);
		result = MinElement(values) < getMinValue(lowerEnvelope) + ZERO_TOLERANCE;
	}	

	return result;
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

		values.push_back(getValue(image, x, y, 0));
	}

	return values;
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

	std::vector< std::vector<double> > &depth = (direction == 1)? upperDepth : lowerDepth;
	if (direction == -1){
		x = depth[0].size() - 1 - x;
	}
	double depthVal = RANGED(0, getValue(depth, x, y, filterSize), 1);

	Vec P = activeViewer->camera()->unprojectedCoordinatesOf(Vec(x, (depth.size()-1)-y, depthVal));

	return Vec3d(P[0], P[1], P[2]);
}

Vec2i Offset::projectedCoordinatesOf( Vec3d point, int direction )
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 2 );

	Vec p = activeViewer->camera()->projectedCoordinatesOf( Vec (point[0], point[1], point[2]) );

	return Vec2i(p[0], (offset.size()-1) - p[1]);
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

	image[y][x] = color;
}

QRgb Offset::jetColor( double val, double min, double max )
{
	uchar rgb[3];	

	ColorMap::jetColorMap(rgb, val, min, max);
	
	return QColor::fromRgb(rgb[0],rgb[1],rgb[2]).rgba();
}

void Offset::visualizeHotRegions( QString filename )
{
	std::vector< std::vector< double > > debugImg = createImage(offset[0].size(), offset.size(), 0.0);
	double step = 1.0 / hotRegions.size();
	for (int i=0;i<hotRegions.size();i++)
	{
		setRegionColor(debugImg, hotRegions[i], step * (i+1));
	}
	saveAsImage(debugImg, 1.0, filename);
}

void Offset::applyHeuristics()
{
	// Get rid of redundancies
	Controller *ctrl = activeObject()->controller;
	std::set<QString> Ids;
	for (int i = 0; i < hotRegions.size(); i++)
	{
		Ids.insert(upperHotSpots[i].segmentID);
		Ids.insert(lowerHotSpots[i].segmentID);
	}
	Ids = ctrl->getRidOfRedundancy(Ids);


	// Heuristics are applied for each pair of hot spots	
	applyHeuristicsOnHotspot(0, 1);		
	applyHeuristicsOnHotspot(0, -1);

	
}

void Offset::applyHeuristicsOnHotspot( uint hid, int side )
{
	Controller *ctrl = activeObject()->controller;

	HotSpot &HS = (side == 1)? upperHotSpots[hid] : lowerHotSpots[hid];
	Primitive* prim = ctrl->getPrimitive(HS.segmentID);
	uint cid = prim->detectHotCurve(HS.hotSamples);

	// Remove the hot spot
	if (prim->excludePoints(HS.hotSamples))
		return;

	// Move hot spot side away or up/down
	if (HS.defineHeight)
	{
		//
		std::vector< Vec3d > Ts = getHorizontalMoves(hid, side);
		//if (T[0] != BIG_NUMBER)
		//{
		//	prim->moveCurveCenter(cid, T * 1.5);
		//}
	}
	else
	{
		Vec3d T(0, 0, 0.1);
		prim->translateCurve(cid, T, -1);
	}
}


std::vector< Vec3d > Offset::getHorizontalMoves( uint hid, int side )
{
	// The hot region	
	std::vector< Vec2i > &hotRegion = hotRegions[hid];
	
	// The size of buffer
	int w = offset[0].size();
	int h = offset.size();

	// Switchers
	bool isUpper = (side == 1);
	bool x_flipped = isUpper;
	std::vector< std::vector<double> > &opp_envelope = isUpper ? lowerEnvelope : upperEnvelope;

	// Project BB to 2D buffer
	std::vector< std::vector< double > > BBImg = createImage( offset[0].size(), offset.size(), 0. );
	Vec2i bbmin = projectedCoordinatesOf(activeObject()->bbmin, 1);
	Vec2i bbmax = projectedCoordinatesOf(activeObject()->bbmax, 1);
	setPixelColor(BBImg, bbmin, 1.0);
	setPixelColor(BBImg, bbmax, 0.5);
	saveAsImage(BBImg, 1.0, "BB projection.png");

	// Current position measures
	std::vector< double > curr_env = getValuesInRegion(opp_envelope, hotRegion, x_flipped);
	double threshold = isUpper? MaxElement(curr_env) : MinElement( curr_env );

	//============ debug code
	std::vector< std::vector< double > > debugImg = createImage( offset[0].size(), offset.size(), 0. );
	setRegionColor(debugImg, hotRegion, 1.0);
	//============ end of debug code

	// The size of the hot region
	Vec2i size = sizeofRegion(hotRegion);
	uint rw = size.x();
	uint rh = size.y();
	int step = Max(rw, rh);

	// Search for optional locations in 1-K rings
	std::vector< Vec3d > Ts;
	int K = 10;
	for (uint k = 1; k < K; k++)
	{
		std::vector< Vec2i > deltas = deltaVectorsToKRing(step, step, k);
		for (int j = 0; j < deltas.size(); j++ )
		{
			std::vector< Vec2i > shiftedRegion = shiftRegionInBB(hotRegion, deltas[j], bbmin, bbmax);
			if (shiftedRegion.empty())  continue; // Out of BB

			//============ debug code
			setRegionColor(debugImg, shiftedRegion, 1.0 - k * 0.1 );
			//============ end of debug code

			std::vector< double > new_env = getValuesInRegion(opp_envelope, shiftedRegion, x_flipped);
			double newValue = isUpper? MinElement( new_env ) : MaxElement( new_env );
			bool isBetter = isUpper? (newValue > threshold) : (newValue < threshold);

			if (isBetter)
			{
				// Get the 3D translation
				Vec2i a = hotRegion[0];
				Vec2i b = shiftedRegion[0];
				Vec3d proj_a = unprojectedCoordinatesOf(a.x(), a.y(), -1);
				Vec3d proj_b = unprojectedCoordinatesOf(b.x(), b.y(), -1);
				Vec3d T = proj_b - proj_a;
				// Force the translation to be horizontal
				T[2] = 0;

				Ts.push_back(T);
			}
		}
	}

	saveAsImage(debugImg, 1.0, "K Ring Neighbors.png");

	return Ts;
}

void Offset::improveStackabilityTo( double targetS )
{
	Controller *ctrl = activeObject()->controller;

	// Push the current shape as the initial candidate solution
	candidateSolutions.push(ctrl->getShapeState());

	while(!candidateSolutions.empty())
	{
		// Get the first candidate solution
		ShapeState currShape = candidateSolutions.front();
		candidateSolutions.pop();
		ctrl->setShapeState(currShape);

		// Check if this candidate is a solution
		computeOffset();
		if (getStackability() >= targetS)
		{
			// Yes. Got one solution.
			solutions.push_back(currShape);
			continue;
		}
		else
		{
			// No. Try to improve the current shape
			// This iteration might generate several candidate solutions
			improveStackability();
		}
	}
}

void Offset::improveStackability()
{
	Controller *ctrl = activeObject()->controller;

	// improve the stackability of current shape in three steps
	//=========================================================================================
	// Step 1: Detect hot spots
	detectHotspots();


	//=========================================================================================
	// Step 2: Apply heuristics on hot spots
	// Several hot solutions might be generated, which are stored in *hotSolutions*
	// Only hot segments are visible and available
	ctrl->setSegmentsVisible(false);
	ctrl->setPrimitivesAvailable(false);
	foreach(QString sid, hotSegments)
	{
		activeObject()->getSegment(sid)->isVisible = true;
		ctrl->getPrimitive(sid)->isAvailable = true;
	}

	hotSolutions.clear();
	applyHeuristics();


	//=========================================================================================
	// Step 3: Propagate hot solutions to remaining cold parts to generate *candidateSolutions*
	for (int i=0;i<hotSolutions.size();i++)
	{
		QMap< QString, void* > &currShape = hotSolutions[i];
		ctrl->setShapeState(currShape);
		
		if (ctrl->propagate(this))
		{// The propagation succeeds, we obtain one candidate solution
			candidateSolutions.push(ctrl->getShapeState());
		}
	}
}

