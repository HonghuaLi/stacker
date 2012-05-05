#include "Offset.h"
#include "HiddenViewer.h"
#include "Utility/ColorMap.h"
#include "Utility/SimpleDraw.h"
#include <QFile>
#include <numeric>
#include "Numeric.h"


#define BIG_NUMBER 10
#define DEPTH_EDGE_THRESHOLD 0.1


// Constructor
Offset::Offset( HiddenViewer *viewer )
{
	activeViewer = viewer;
}

// Shortener
QSegMesh* Offset::activeObject()
{
	return activeViewer->activeObject();
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

// OpenGL 2D coordinates system has origin at the left bottom conner, while Qt at left top conner
// OpenGL coordinates are mainly used in this class

// Camera paths
// <-1, 1> + 2 = <1, 3> : The top and bottom setting for the entire shape
// <-1, 1> + 3 = <2, 4> : The top and bottom setting for the zoomed in region

// == Envelope
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
	computeEnvelopeOfShape(direction, Vec3d(0,0,direction));
}

void Offset::computeEnvelopeOfShape( int direction, Vec3d pos, Vec3d upVector /*= Vec3d(0,1,0)*/, Vec3d horizontalShift /*= Vec3d(0,0,0)*/ )
{
	// Set camera
	activeViewer->camera()->setType(Camera::ORTHOGRAPHIC);
	activeViewer->camera()->setPosition(Vec(pos));
	activeViewer->camera()->lookAt(Vec());	

	// Set \y as the upVector, then the projected 3D BB is still BB in 3D
	activeViewer->camera()->setUpVector(Vec(upVector));
	Vec delta(1, 1, 1);
	delta *= activeObject()->radius * 0.25;
	activeViewer->camera()->setSceneRadius(activeObject()->radius * 3);
	activeViewer->camera()->fitBoundingBox(Vec(activeObject()->bbmin) - delta, Vec(activeObject()->bbmax) + delta);
	activeViewer->camera()->setPosition(activeViewer->camera()->position() + Vec(horizontalShift));

	// Save this new camera settings
	activeViewer->camera()->deletePath(direction+2);
	activeViewer->camera()->addKeyFrameToPath(direction+2);


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


// == Offset
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
			if (upperEnvelope[y][x]== -BIG_NUMBER | lowerEnvelope[y][(w-1)-x] == BIG_NUMBER)
				offset[y][x] = 0.0; 
			else
				offset[y][x] = upperEnvelope[y][x] - lowerEnvelope[y][(w-1)-x];
		}
	}
}

void Offset::computeOffsetOfShape( STACKING_TYPE type /*= STRAIGHT_LINE*/, int rotDensity /*= 1*/ )
{
	if (!activeObject()) return;

	// Compute the height of the shape
	activeObject()->computeBoundingBox();
	objectH = (activeObject()->bbmax - activeObject()->bbmin).z();

	// Save original camera settings
	activeViewer->camera()->deletePath(0);
	activeViewer->camera()->addKeyFrameToPath(0);

	// The upper envelope
	computeEnvelopeOfShape(1, Vec3d(0,0,1));

	// Rotation can be simulated by changing the position and upVector of the camera

	
	// Camera postions: posAngles are on y-z plane, starting from z in clockwise
	// Camera UpVector: upVectorAngles on the new x-y plane, string from y in clockwise
	double angleStep = 2 * M_PI / rotDensity;
	std::vector< double > angles, singleAngle;
	singleAngle.push_back(0);
	for (int i=0;i<rotDensity;i++)
		angles.push_back(i * angleStep);

	std::vector< double > PosAngles, UVAngles;
	bool shifting = false;
	switch (type)
	{
	case STRAIGHT_LINE:
		PosAngles = singleAngle;
		UVAngles = singleAngle;
		shifting = false;
		break;
	case ROT_AROUND_AXIS:
		PosAngles = singleAngle;
		UVAngles = angles;
		shifting = false;
		break;
	case ROT_FREE_FORM:
		PosAngles = angles;
		UVAngles = angles;
		shifting = true;
		break;
	}

	//std::cout << "Stacking type: " << type << "\t Shifting = " << shifting << std::endl;

	// Searching
	double minO_max = 2 * objectH;
	double bestPosAngle = 0;
	double bestUVAngle = 0;	
	Vec3d bestShift;

	Vec3d X(1,0,0);
	for (int i=0;i<PosAngles.size();i++)
	{
		// Camera position
		double alpha = PosAngles[i];
		Vec3d cameraPos(0, -sin(alpha), -cos(alpha));
		cameraPos.normalize();

		for(int j=0;j<UVAngles.size();j++)
		{
			// Up vector
			double beta = UVAngles[j];
			Vec3d newY = cross(X, cameraPos);
			Vec3d UV = X * sin(beta) + newY * cos(beta);

			// Horizontal shifting 
			int numSteps = shifting? 3 : 0;
			Vec3d bb = activeObject()->bbmax - activeObject()->bbmin;
			double xStep = bb[0] / 20;
			double yStep = bb[1] / 20;
			for (int j = -numSteps; j <= numSteps; j++)	{
				double xShift = j * xStep;
				for (int k = -numSteps; k <= numSteps; k++)
				{
					Vec3d horizontalShift(xShift, k * yStep, 0);

					// The rotated and shifted lower envelope
					computeEnvelopeOfShape(-1, cameraPos, UV, horizontalShift);

					// Compute the offset function
					computeOffset();
					O_max = getMaxValue(offset);

					if (O_max < minO_max)
					{
						minO_max = O_max;
						bestPosAngle = alpha;
						bestUVAngle = beta;
						bestShift = horizontalShift;
					}
				}
			}
		}
	
	}

	// Update the stackability in QSegMesh
	O_max = minO_max;
	activeObject()->val["O_max"]		= O_max;
	activeObject()->val["stackability"] = 1 - O_max/objectH;
	activeObject()->val["theta"]		= DEGREES(bestPosAngle);
	activeObject()->val["phi"]			= DEGREES(bestUVAngle);

	activeObject()->val["tranX"]		= bestShift[0];
	activeObject()->val["tranY"]		= -bestShift[1];
	activeObject()->val["tranZ"]		= 0;

	// Save offset as image
	//saveAsImage(offset, O_max, "offset function.png");
	activeObject()->data2D["offset"] = offset;
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

double Offset::getStackability( bool recompute /*= false*/ )
{
	if (recompute) computeOffsetOfShape();

	return activeObject()->val["stackability"];
}


// == Hot spots
HotSpot Offset::detectHotspotInRegion(int direction, std::vector<Vec2i> &hotRegion)
{
	// Restore the camera according to the direction
	activeViewer->camera()->playPath( direction + 3 );

	// Draw Faces Unique
	activeViewer->setMode(HV_FACEUNIQUE);
	activeViewer->updateGL(); 
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

	QMap< QString, int > subHotRegionSize;
	QMap< QString, QVector< Vec2i > > subHotPixels;
	QMap< QString, QVector< Vec3d > > subHotSamples;

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

		// Store information for subHotRegion
		QString segmentID = activeObject()->getSegment(sid)->objectName();
		Vec3d hotPoint(hotP.x, hotP.y, hotP.z);
		subHotRegionSize[segmentID]++;
		subHotPixels[segmentID].push_back(hotPixel);
		subHotSamples[segmentID].push_back(hotPoint);

		// All hot points in 3D
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

		Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

		HS.side = direction;
		HS.hotPixels = subHotPixels[HS.segmentID];
		HS.hotSamples = subHotSamples[HS.segmentID];
	}

	return HS;
}

void Offset::detectHotspots( )
{
	// Initialization
	clear();
	int h = activeViewer->height();
	int w = activeViewer->width();

	// Recompute the offset function of the entire shape
	computeOffsetOfShape();

	// Detect hot regions
	hotRegions.clear();
	double hot_cap = 1.0;
	while (hotRegions.empty()){
		hot_cap -= 0.05; // increase the cap
		hotRegions = getRegionsGreaterThan(offset, O_max * hot_cap);
	}
//	visualizeRegions(w, h, hotRegions, "hot regions of shape.png");

	// The max offset of hot regions
	maxOffsetInHotRegions.clear();
	for (int i=0;i<hotRegions.size();i++){
		maxOffsetInHotRegions.push_back(maxValueInRegion(offset, hotRegions[i]));
	}

	// The max of \UpperEnvelope and min of \LowerEnvelope
	double maxUE = getMaxValue(upperEnvelope);
	double minLE = getMinValue(lowerEnvelope);

	// Zoom into each hot region
	for (int i=0;i<hotRegions.size();i++)
	{
		computeOffsetOfRegion(hotRegions[i]);

		// Detect zoomed (in) hot region 
		std::vector< std::vector<Vec2i> > zoomedHRs
			=  getRegionsGreaterThan(offset, O_max * hot_cap);
		if (zoomedHRs.empty()){
			std::cout << "There is no hot region in the zoomed-in view.\n";
			continue;
		}

		// If there are multiple regions, pick up the one closest to the center
		//visualizeRegions(zoomedHRs, "zoomed in hot regions.png");
		Vec2i center(w/2, h/2);
		int minDis = w;
		int closestID = 0;
		for (int j=0;j<zoomedHRs.size();j++)
		{
			Vec2i c = centerOfRegion(zoomedHRs[j]);
			int dx = c.x() - center.x();
			int dy = c.y() - center.y();
			int dis = dx * dx + dy * dy;
			if (dis < minDis){
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

		UHS.decideType();
		LHS.decideType();
		UHS.computeRepresentative(ctrl());
		LHS.computeRepresentative(ctrl());

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

std::vector<HotSpot> Offset::getHotspots( int side )
{
	if (side == 1)
		return upperHotSpots;
	else
		return lowerHotSpots;
}


HotSpot& Offset::getHotspot( int side, int id )
{
	if (id > upperHotSpots.size() - 1) 
		return HotSpot();

	if (side == 1)
		upperHotSpots[id];
	else
		lowerHotSpots[id];
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

		// Hot segment in red
		//segment->setColorVertices(Color(1, 0, 0, 1)); 

		// Hot spots as red points
		for (std::vector< Vec3d >::iterator pit = i->second.begin(); pit != i->second.end(); pit++)
		{
			segment->debug_points.push_back(*pit);
		}
	}
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



// ==(un)Projection
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

Controller* Offset::ctrl()
{
	if (activeObject())
		return (Controller*)activeObject()->ptr["controller"];
	else
		return NULL;
}

