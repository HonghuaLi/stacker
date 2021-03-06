#include "GCylinder.h"
#include "GraphicsLibrary/Skeleton/SkeletonExtract.h"
#include "Utility/SimpleDraw.h"
#include "Numeric.h"


GCylinder::GCylinder( QSurfaceMesh* segment, QString newId, bool doFit) : Primitive(segment, newId)
{
	cage = NULL;

	cageScale = 1.25;
	cageSides = 16;

	// For visualization
	deltaScale = 1.25;

	// useful for fitting process
	if(!m_mesh->vertex_array.size()){
		m_mesh->assignFaceArray();
		m_mesh->assignVertexArray();
	}

	deformer = SKINNING;

	if(doFit)
	{
		fit();
		buildUp();
	}

	fixedPoints.clear();

    primType = GCYLINDER;
}

GCylinder::GCylinder( QSurfaceMesh* segment, QString newId) : Primitive(segment, newId)
{
	cage = NULL;
	cageScale = 0;
	cageSides = 0;
	deltaScale = 0;
    primType = GCYLINDER;
}

void GCylinder::fit()
{
	// Extract and save skeleton
	SkeletonExtract skelExt( m_mesh );
	Skeleton * skel = new Skeleton();
	skelExt.SaveToSkeleton( skel );

	// Select part of skeleton
	skel->selectLongestPath();

	// Compute generalized cylinder given spine points
	int numSteps = skel->sortedSelectedNodes.size();
	std::vector<Point> reSampledSpinePoints;
	foreach(ResampledPoint sample, skel->resampleSmoothSelectedPath(numSteps, 3)) 
		reSampledSpinePoints.push_back(sample.pos);

	// Add one more spine point at each end
	int N = reSampledSpinePoints.size();
	std::vector<Point> spinePoints;
	spinePoints.push_back(reSampledSpinePoints[0] * 2 - reSampledSpinePoints[1]);
	spinePoints.insert(spinePoints.end(), reSampledSpinePoints.begin(), reSampledSpinePoints.end());
	spinePoints.push_back(reSampledSpinePoints[N-2] * 2 - reSampledSpinePoints[N-1]);

	buildGC(spinePoints);
}

void GCylinder::buildGC( std::vector<Point> spinePoints, bool computeRadius )
{
	gc = new GeneralizedCylinder( spinePoints, m_mesh, computeRadius );

	printf(" GC with %d cross-sections. ", gc->crossSection.size());
}

void GCylinder::computeMeshCoordinates()
{
	// The coordinates depend on the underlying deformer
	// So create the deformer accordingly first
	// The coordinates can be required from the deformer

	if(deformer == GREEN_COORDIANTES)
		gcd = new GCDeformation(m_mesh, cage);

	if(deformer == SKINNING)
		skinner = new Skinning(m_mesh, gc);
}

void GCylinder::deformMesh()
{
	// Deform the underlying geometry using deformer

	if(deformer == GREEN_COORDIANTES) 
		gcd->deform();

	if(deformer == SKINNING) 
		skinner->deform();

	m_mesh->computeBoundingBox();
}

void GCylinder::draw()
{
	if(!isDraw) return;

	glDisable(GL_LIGHTING);

	glEnable(GL_BLEND); 
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Cross-sections
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		glLineWidth(2.0);
		glColor4d(0, 0.5, 1, 0.5); // blue

		if(isSelected) glColor4d(1, 1, 0, 1.0); // yellow

		if(c.index == this->selectedCurveId){
			glLineWidth(6.0);
			glColor4d(0, 1, 0, 1.0); // green
		}

		std::vector<Point> pnts = c.toSegments(30, gc->frames.U[c.index].s, deltaScale);
		pnts.push_back(pnts.front());
		glBegin(GL_LINE_STRIP);
		foreach(Vec3d p, pnts) glVertex3dv(p);
		glEnd();
	}

	// Along height side, dashed
	glLineStipple(1, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glLineWidth(2.0);
	glColor4d(0, 0.5, 1, 0.5);

	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.count(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * gc->crossSection[i].radius * deltaScale));
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.count(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * -gc->crossSection[i].radius * deltaScale));
	glEnd();

	glDisable(GL_LINE_STIPPLE);
	glEnable(GL_LIGHTING);

	if(cage) 
	{
		cage->drawDebug();
		cage->simpleDraw();
	}
}

void GCylinder::drawNames( int name, bool isDrawCurves)
{
	if(isDrawCurves)
	{
		int curveId = 0;

		foreach(GeneralizedCylinder::Circle c, gc->crossSection)
		{
			glPushName(curveId++);
			glBegin(GL_POLYGON);
			std::vector<Point> points = c.toSegments(20, gc->frames.U[c.index].s, deltaScale);
			foreach(Point p, points) glVertex3dv(p);
			glEnd();
			glPopName();
		}
	}
	else
	{
		glPushName(name);
		if(cage) cage->simpleDraw();
		glPopName();
	}
}

void GCylinder::buildCage()
{
	cage = new QSurfaceMesh;
		
	uint vindex = 0;
	std::map<uint, Surface_mesh::Vertex> v;
	std::vector<Surface_mesh::Vertex> verts(3), verts2(3);

	// Start vertex
	v[vindex++] = cage->add_vertex(gc->crossSection.front().center);
		
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		std::vector<Point> points = c.toSegments(cageSides, gc->frames.U[c.index].s, cageScale);
		for(int i = 0; i < cageSides; i++)
			v[vindex++] = cage->add_vertex(points[i]);
	}

	// End vertex
	v[vindex++] = cage->add_vertex(gc->crossSection.back().center);

	// Add faces:
	int findex = 0;

	// Start cap
	for(int i = 1; i <= cageSides; i++)
		cage->add_triangle( v[i], v[0], v[(i % cageSides) + 1] );

	// Sides
	for(uint c = 0; c < gc->crossSection.size() - 1; c++)
	{
		int offset = (c * cageSides) + 1;

		for(int i = 0; i < cageSides; i++){
			int v1 = NEXT(i, cageSides) + offset;
			int v2 = NEXT(i + 1, cageSides) + offset;
			int v3 = v2 + cageSides;
			int v4 = v1 + cageSides;

			verts[0] = v[v1]; verts[1] = v[v2]; verts[2] = v[v3];
			verts2[0] = v[v1]; verts2[1] = v[v3]; verts2[2] = v[v4];

			// Add the two faces
			cage->add_face(verts);
			cage->add_face(verts2);
		}
	}

	// End cap
	int end = cage->n_vertices() - 1;
	for(int i = 0; i < cageSides; i++)
	{
		cage->add_triangle(v[end], v[(end-1) - NEXT(i + 2, cageSides)], 
			v[(end-1) - NEXT(i + 1, cageSides)]);
	}
	
	cage->setColorVertices(0.8,0.8,1,0.3); // transparent cage
	cage->update_face_normals();
	cage->update_vertex_normals();

	// Save volume
	originalVolume = cage->volume();
}

void GCylinder::updateCage()
{
	Surface_mesh::Vertex_property<Point> cagePoints = cage->vertex_property<Point>("v:point");
	std::vector<Point> points;

	// First point
	cagePoints[Surface_mesh::Vertex(0)] = gc->crossSection.front().center;

	// Middle points
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		points = c.toSegments(cageSides, gc->frames.U[c.index].s, cageScale);

		for(int i = 0; i < cageSides; i++)
		{
			uint vi = (1 + c.index * cageSides) + i;
			cagePoints[Surface_mesh::Vertex(vi)] = points[i];
		}
	}

	// Last point
	cagePoints[Surface_mesh::Vertex(cage->n_vertices() - 1)] = gc->crossSection.back().center;
}

std::vector <Point> GCylinder::points()
{
	return gc->frames.point;
}



std::vector <double> GCylinder::scales()
{
	return curveScales;
}

double GCylinder::volume()
{
	return cage->volume();
}

Point GCylinder::getSelectedCurveCenter()
{
	if(RANGE(selectedCurveId, 0, gc->crossSection.size())) 
		return gc->frames.point[selectedCurveId];
	else
		return centerPoint();
}

void GCylinder::moveCurveCenter( int cid, Vec3d T )
{
	int N =  gc->crossSection.size();
	if(!RANGE(cid, 0, N-1))
	{
		// Manual
		if (symmPlanes.size() == 1)
		{
			T[0] = 0;
			T[1] = 0;
		}		
		
		if (RANGE(selectedCurveId, 0, N-1))
			curveTranslation[selectedCurveId] += T;
		else
			translate(T);
	}
	else
		curveTranslation[cid] += T;

	// Update the GC, cage, and mesh
	update();
}

double GCylinder::computeWeight( double x, bool useGaussian /* = false*/ ) 
{
	double weight = 1;
	if (!useGaussian)
	{
		weight = pow(1 - x, 10);
	}
	else
	{
		double sigma = GC_GAUSSIAN_SIGMA;
		double mu = 0;
		weight = gaussianFunction(x, mu, sigma);
	}

	return weight;
}

// Legacy code
void GCylinder::moveCurveCenterRanged(int cid, Vec3d T, int fixed_end_id1, int fixed_end_id2)
{
	int N = gc->crossSection.size();
	bool useGaussian = false;

	// Only for manual mode
	if (fixed_end_id2 < 0) fixed_end_id2 = N;

	if (cid < 0)	
	{
		useGaussian = true;

		// Fix the further end
		cid = selectedCurveId;
		if (cid - fixed_end_id1 < fixed_end_id2 - cid)
			fixed_end_id2 = N-1;
		else
			fixed_end_id1 = 0;

		// Fix two ends if at middle
		if( abs((double)cid/N - 0.5) < 0.1 )
		{
			fixed_end_id1 = 0;
			fixed_end_id2 = N - 1;
		}
	}


	// Range check
	if(fixed_end_id1 >= fixed_end_id2) return;

	// First half: (fixed_end_id1, \cid)
	double range = cid - fixed_end_id1; // Only if \fixed_end_id1 >= 0
	for (int i = fixed_end_id1 + 1; i < cid; i++)
	{
		double weight = 1.0;
		
		// This range is not free
		if (fixed_end_id1 != -1)
		{
			double dist = double(cid - i) / range;
			weight = computeWeight(dist, useGaussian);
		}

		gc->frames.point[i] += T * weight;
	}

	// The \cid
	gc->frames.point[cid] += T;

	// The second half: (\cid, fixed_end_id2)
	range = fixed_end_id2 - cid; // Only if \fixed_end_id1 >= 0
	for(int i = cid + 1; i < fixed_end_id2; i++)
	{
		double weight = 1.0;

		// This range is not free
		if (fixed_end_id2 < N)
		{
			double dist = double(i - cid) / range;
			weight = computeWeight(dist, useGaussian);
		}

		gc->frames.point[i] += T * weight;
	}

	// Update the GC, cage, and mesh
	update();
}

void GCylinder::scaleCurve( int cid, double s )
{
	if (cid == -1) cid = selectedCurveId;
	if (cid == -1) return;

	int N = gc->frames.count();

	// Scale \cid
	curveScales[cid] *= s;

	// Update the GC, cage, and mesh
	update();
}

QSurfaceMesh GCylinder::getGeometry()
{
	return *cage;
}

void* GCylinder::getState()
{
	std::vector<double> state;
	for(int i = 0; i < curveScales.size(); i++)
	{		
		for(int j = 0; j < 3; j++)	state.push_back(basicGC.crossSection[i].center[j]); // P		
		for(int j = 0; j < 3; j++)	state.push_back(curveTranslation[i][j]); // T		
		state.push_back(curveScales[i]); // S
	}
	return new std::vector<double>(state);
}

void GCylinder::setState( void* toState)
{
	std::vector<double> & state = *(std::vector<double> *)toState;

	if(state.size() != curveScales.size() * 7) return;

	for(int k = 0, i = 0; i < curveScales.size() * 7; k++)
	{		
		for(int j = 0; j < 3; j++)	basicGC.crossSection[k].center[j] = state[i++]; // P		
		for(int j = 0; j < 3; j++)	curveTranslation[k][j] = state[i++]; // T		
		curveScales[k] = state[i++]; // S
	}

	// Update the GC, cage, and mesh
	update();
}

void GCylinder::serialize( QTextStream &out)
{
	int N = gc->crossSection.size();
	out << N << ' ';

	// Each cross sections
	for(int i = 0; i < N; i++)
	{		
		for(int j = 0; j < 3; j++)	out << basicGC.crossSection[i].center[j] << ' '; // P		
		for(int j = 0; j < 3; j++)	out << curveTranslation[i][j] << ' '; // T		
		out << curveScales[i] << ' '; // S
	}
}


void GCylinder::unserialize( QTextStream &in)
{
	int N;
	in >> N;

	// Each cross sections
	for(int i = 0; i < N; i++)
	{		
		for(int j = 0; j < 3; j++)	in >> basicGC.crossSection[i].center[j]; // P		
		for(int j = 0; j < 3; j++)	in >> curveTranslation[i][j]; // T		
		in >> curveScales[i]; // S
	}

	update();
}



std::vector<double> GCylinder::getCoordinate( Point v )
{
	std::vector<double> coords;

	if (deformer == GREEN_COORDIANTES)
	{
		GCDeformation::GreenCoordiante c = gcd->computeCoordinates(v);

		coords.resize(c.coord_v.size() + c.coord_n.size());

		coords.insert(coords.begin(), c.coord_n.begin(), c.coord_n.end());
		coords.insert(coords.begin(), c.coord_v.begin(), c.coord_v.end());
	}
	else if (deformer == SKINNING)
	{
		coords = skinner->getCoordinate(v);
	}

	return coords;
}

Point GCylinder::fromCoordinate( std::vector<double> &coords )
{
	Point v;

	if (deformer == GREEN_COORDIANTES)
	{
		GCDeformation::GreenCoordiante c;

		// Coordinates from cage vertices
		uint NV = cage->n_vertices();
		for(uint i = 0; i < NV; i++)
			c.coord_v.push_back(coords[i]);

		// Coordinates from cage normal
		for(uint i = 0; i < cage->n_faces(); i++)
			c.coord_n.push_back(coords[i + NV]);

		gcd->initDeform();
		v = gcd->deformedPoint(c);
	}
	else if (deformer == SKINNING)
	{
		v = skinner->fromCoordinates(coords);
	}

	return v;
}

std::vector <Vec3d> GCylinder::majorAxis()
{
	std::vector <Vec3d> result;

	result.push_back(gc->crossSection.front().normal());
	result.push_back(gc->crossSection.back().normal());

	return result;
}

std::vector < std::vector <Vec3d> > GCylinder::getCurves()
{
	std::vector < std::vector <Vec3d> > result;

	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
		result.push_back(c.toSegments(cageSides, gc->frames.U[c.index].s, cageScale));

	return result;
}

void GCylinder::reshape( std::vector<Point>& pnts, std::vector<double>& scales )
{
	int N = basicGC.crossSection.size();
	for(int i = 0; i < N; i++)
	{
		basicGC.crossSection[i].center = pnts[i];
		curveTranslation[i] = pnts[N+i];
	}

	this->curveScales = scales;

	// Update the GC, cage, and mesh
	update();
}

int GCylinder::detectHotCurve( QVector<Point> &hotSamples )
{
	if (hotSamples.isEmpty()) return -1;

	Point samplesCenter(0,0,0);
	foreach(Point p, hotSamples) samplesCenter += p;
	samplesCenter /= hotSamples.size();

	return detectHotCurve(samplesCenter);
}

int GCylinder::detectHotCurve( Point hotSample )
{
	// Measure the distance to the curve centers
	uint closestID = 0;
	double minDist = DBL_MAX;
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		double dist = (c.center - hotSample).norm();
		if(dist < minDist)
		{
			minDist = dist;
			closestID = c.index;
		}
	}

	return closestID;
}

void GCylinder::translate( Vec3d &T )
{
	for(uint i = 0; i < gc->crossSection.size(); i++)
		basicGC.crossSection[i].center += T;

	// Update the GC, cage, and mesh
	update();
}

Point GCylinder::closestPoint( Point p )
{
	return cage->closestPointVertices(p);
}

bool GCylinder::containsPoint( Point p )
{
	// deprecated..
	// ToDo (is this function useful?)
	return false;
}

void GCylinder::setSymmetryPlanes( int nb_fold )
{
	// For special cases
	symmPlanes.resize(nb_fold);
}

void GCylinder::deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T )
{
	// theta = <p, j, p + T>
	Vec3d v1 = p - joint;
	Vec3d v2 = (p + T) - joint;

	double theta = acos(RANGED(-1.0, dot(v1.normalized(), v2.normalized()), 1.0));
	Vec3d axis = cross(v1, v2).normalized();

	// 1) Rotate with respect to joint (theta)
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		Vec3d newNormal = RotateAround(c.n, joint, axis, theta);
		Vec3d newCenter = RotateAround(c.center, joint, axis, theta);

		gc->crossSection[c.index].n = newNormal;
		gc->frames.point[c.index] = newCenter;
	}

	// 2) Scale along direction (joint -> p + T)
	double scaleAlong = 1;
	if (v1.norm() > 1e-10)
		scaleAlong = v2.norm() / v1.norm();

	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
		gc->frames.point[c.index] = joint + ((gc->frames.point[c.index] - joint) * scaleAlong);

	// Update the GC, cage, and mesh
	update();
}

void GCylinder::movePoint( Point p, Vec3d T )
{
	if (fixedPoints.isEmpty() && symmPlanes.isEmpty())
		translate(T);
	else
	{
		int cid = detectHotCurve(p);
		moveCurveCenter(cid, T);
	}
}

void GCylinder::save( std::ofstream &outF )
{
	outF << this->cageScale << '\t';
	outF << this->cageSides << '\t';
	outF << this->deltaScale << '\t';

	// The skeleton joints
	outF << gc->frames.count() << '\t';
	foreach(Point p, gc->frames.point)
	{
		outF << p << '\t';
	}

	// The radii
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		outF << c.radius << '\t';
	}
}

void GCylinder::load( std::ifstream &inF, Vec3d translation, double scaleFactor )
{
	inF >> this->cageScale;
	inF >> this->cageSides;
	inF >> this->deltaScale;

	// Load skeleton joints
	int N;
	inF >> N;
	std::vector<Point> spinePoints;
	for(int i = 0; i < N; i++)
	{
		Point p(0,0,0);
		inF >> p;
		p += translation; 
		p *= scaleFactor;
		spinePoints.push_back(p);
	}

	// Create GC using skeleton
	buildGC(spinePoints, false);

	// Load radius
	for(int i = 0; i < N; i++)
	{
		double radius;
		inF >> radius;
		radius *= scaleFactor;
		gc->crossSection[i].radius = radius;
	}

	buildUp();
}

void GCylinder::moveLineJoint( Point A, Point B, Vec3d deltaA, Vec3d deltaB )
{
	// Move the two points one by one
	movePoint(A, deltaA);

	// After moving \A, \B might change
	// == To do.
	movePoint(B, deltaB);
}

double GCylinder::curveRadius( int cid )
{
	double radius;
	if (cid < 0 || cid > gc->crossSection.size())
		radius = -1.0;
	else
		radius = gc->crossSection[cid].radius;

	return radius;
}

Point GCylinder::curveCenter( int cid )
{
	Point center;
	if (cid < 0 || cid > gc->crossSection.size())
		center = Point(DOUBLE_INFINITY,0,0);
	else
		center = gc->crossSection[cid].center;

	return center;
}

void GCylinder::buildUp()
{
	buildCage();
	computeMeshCoordinates();

	// Save the \basicGC and additional scales and translations
	basicGC = *gc;

	int N = gc->crossSection.size();
	curveScales.resize(N, 1.0);
	curveTranslation.resize(N, Vec3d(0.0));
}

void GCylinder::updateGC()
{
	// Gaussian parameters
	double sigma = GC_GAUSSIAN_SIGMA;
	double mu = 0;
	Vec3d zeroV(0.0);

	// Recompute the position of all spine points
	int N = gc->frames.count();
	for(int i = 0; i < N; i++)	
	{
		Vec3d final_translate = zeroV;

		// Sum up scales from all others
		for(int j = 0; j < N; j++)
		{
			if (curveTranslation[j] == zeroV) continue; // no contribution

			double dist = abs(double(j - i)) / double(N-1);
			double weight = gaussianFunction(dist, mu, sigma);
			final_translate += curveTranslation[j] * weight;
		}

		gc->frames.point[i] = final_translate + basicGC.crossSection[i].center;
	}

	// Rearrange frames
	gc->frames.compute();
	gc->realignCrossSections();

	// Update radius for each cross section
	for(int i = 0; i < N; i++)	
	{
		double final_scale = 1;

		// Sum up scales from all others
		for(int j = 0; j < N; j++)
		{

			if ( curveScales[j] == 1) continue; // no contribution

			double dist = abs(double(j - i)) / double(N-1);
			double weight = gaussianFunction(dist, mu, sigma);

			double deltaS = curveScales[j] - 1;
			final_scale *= 1 + ( deltaS *  weight);
		}

		gc->crossSection[i].radius = final_scale * basicGC.crossSection[i].radius;
	}

	// Fix the orientation of cross sections at the ends
	Point c0 = gc->crossSection[0].center;
	Point c1 = gc->crossSection[1].center;
	gc->crossSection[0].center = c1 - (c1-c0).norm() * gc->crossSection[1].n;
	gc->crossSection[0].n = gc->crossSection[1].n;

	Point cn_1 = gc->crossSection[N-1].center;
	Point cn_2 = gc->crossSection[N-2].center;
	gc->crossSection[N-1].center = cn_2 + (cn_1-cn_2).norm() * gc->crossSection[N-2].n;
	gc->crossSection[N-1].n = gc->crossSection[N-2].n;
}

void GCylinder::update()
{
	updateGC();
	updateCage();
	deformMesh();
}

Point GCylinder::closestProjection( Point p )
{
	return skinner->closestProjection(p);
}

bool GCylinder::atEnd( int dimensions, Point p )
{
	if (deformer == SKINNING)
	{
		return skinner->atEnd(p);
	}

	return false;
}


