#include "GCylinder.h"
#include "SkeletonExtract.h"
#include "SimpleDraw.h"

int skeletonJoints = 6;

GCylinder::GCylinder( QSurfaceMesh* segment, QString newId, bool doFit) : Primitive(segment, newId)
{
	cage = NULL;

	cageScale = 1.3;
	cageSides = 8;

	// For visualization
	deltaScale = 1.3;

	// useful for fitting process
	if(!m_mesh->vertex_array.size()){
		m_mesh->assignFaceArray();
		m_mesh->assignVertexArray();
	}

	if(doFit)
	{
		fit();
		buildCage();

		originalVolume = volume();
	}

	fixedPoints.clear();

	isFitted = doFit;

	primType = GC;
}

GCylinder::GCylinder( QSurfaceMesh* segment, QString newId) : Primitive(segment, newId)
{
	cage = NULL;
	cageScale = 0;
	cageSides = 0;
	skeletonJoints = 0;
	deltaScale = 0;
	isFitted = false;
	primType = GC;
}

void GCylinder::fit()
{
	// Extract and save skeleton
	SkeletonExtract skelExt( m_mesh );
	skel = new Skeleton();
	skelExt.SaveToSkeleton( skel );

	// Select part of skeleton
	skel->selectLongestPath();

	// Compute generalized cylinder given spine points
	int numSteps = Max(skel->sortedSelectedNodes.size(), skeletonJoints);
	std::vector<Point> reSampledSpinePoints;
	foreach(ResampledPoint sample, skel->resampleSmoothSelectedPath(numSteps, 3)) 
		reSampledSpinePoints.push_back(sample.pos);

	createGC(reSampledSpinePoints);

	Vec3d p = gc->frames.point.front();
	Vec3d q = gc->frames.point.back();
}

void GCylinder::createGC( std::vector<Point> spinePoints, bool computeRadius )
{
	gc = new GeneralizedCylinder( spinePoints, m_mesh, computeRadius );
	
	printf(" GC with %d cross-sections. ", gc->crossSection.size());
	
	this->originalSpine = spinePoints;
}

void GCylinder::computeMeshCoordiantes()
{
	gcd = new GCDeformation(m_mesh, cage);
}

void GCylinder::deformMesh()
{
	updateCage();
	gcd->deform();
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

		if(c.index == this->selectedPartId){
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


	// Debug
	/*Vec3d p(mf1->position().x, mf1->position().y, mf1->position().z);
	Vec3d q(mf2->position().x, mf2->position().y, mf2->position().z);

	SimpleDraw::IdentifyPoint(p);
	SimpleDraw::IdentifyPoint2(q);*/

	//gc->draw();
	//skel->draw(true);
	if(cage) 
	{
		//cage->simpleDrawWireframe();
		cage->drawDebug();
		cage->simpleDraw();
	}
}

void GCylinder::drawNames( int name, bool isDrawParts)
{
	if(isDrawParts)
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

void GCylinder::update()
{
	// old code
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

	computeMeshCoordiantes();
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

std::vector <Vec3d> GCylinder::points()
{
	return cage->clonePoints();
}

double GCylinder::volume()
{
	return cage->volume();
}

Vec3d GCylinder::selectedPartPos()
{
	Vec3d result(0,0,0);

	foreach(GeneralizedCylinder::Circle c, gc->crossSection){
		if(c.index == selectedPartId){
			result = c.center;
			break;
		}
	}

	return result;
}

void GCylinder::moveCurveCenter( int cid, Vec3d delta )
{
	// Using full range
	moveCurveCenterRanged(cid, delta);
}

void GCylinder::moveCurveCenterRanged(int cid, Vec3d delta, int start, int finish)
{
	if(cid < 0)	cid = selectedPartId;

	int N = gc->frames.count();

	// Range parameter
	if(finish < 0) 	finish = N;
	else N = finish - start;

	// Range check
	if(start >= finish) 
		return;

	// Move curves, with weight
	for(int i = start; i < finish; i++)
	{
		double sigma = 1.0 / sqrt(2 * M_PI);
		double mu = 0;

		double decay = (double(N) / gc->frames.count()) * 0.9;
		double dist = abs(double(cid - i)) / (N * decay);

		double weight = gaussianFunction(dist, mu, sigma);

		gc->frames.point[i] += delta * weight;
	}

	// Re-compute frames and align the cross-sections
	gc->frames.compute();
	gc->realignCrossSections();
	deformMesh();
}

void GCylinder::scaleCurve( int cid, double s )
{
	if(cid < 0)	cid = selectedPartId;

	int N = gc->frames.count();

	std::cout << "Curve scaling: s = " << s << " weights= "; 
	for(int i = 0; i < N; i++)	
	{
		// Gaussian parameters
		double sigma = 0.1;
		double mu = 0;

		double dist = abs(double(cid - i)) / N;

		double weight = 1 + (s * gaussianFunction(dist, mu, sigma));
			
		std::cout << weight << ", ";
		gc->crossSection[i].radius *= weight;
	}
	std::cout << std::endl;


	// Re-compute frames and align the cross-sections
	deformMesh();
}

QSurfaceMesh GCylinder::getGeometry()
{
	return *cage;
}

void* GCylinder::getGeometryState()
{
	return (void*) new std::vector<GeneralizedCylinder::Circle>(gc->crossSection);
}

void GCylinder::setGeometryState( void* toState)
{
	gc->crossSection = *( (std::vector<GeneralizedCylinder::Circle>*) toState );
	updateCage();
}

std::vector<double> GCylinder::getCoordinate( Point v )
{
	GCDeformation::GreenCoordiante c = gcd->computeCoordinates(v);
	
	std::vector<double> coords(c.coord_v.size() + c.coord_n.size());

	coords.insert(coords.begin(), c.coord_n.begin(), c.coord_n.end());
	coords.insert(coords.begin(), c.coord_v.begin(), c.coord_v.end());

	//uint vi = m_mesh->closestVertex(v);

	//Point closestPoint = m_mesh->getVertexPos(vi);
	//Vec3d delta = v - closestPoint;

	//std::vector<double> coords;

	//coords.push_back(vi);
	//coords.push_back(delta[0]);
	//coords.push_back(delta[1]);
	//coords.push_back(delta[2]);

	return coords;
}

Point GCylinder::fromCoordinate( std::vector<double> coords )
{
	uint NV = cage->n_vertices();

	GCDeformation::GreenCoordiante c;

	// Coordinates from cage vertices
	for(uint i = 0; i < NV; i++)
		c.coord_v.push_back(coords[i]);

	// Coordinates from cage normal
	for(uint i = 0; i < cage->n_faces(); i++)
		c.coord_n.push_back(coords[i + NV]);

	gcd->initDeform();
	return gcd->deformedPoint(c);

	//uint vi = coords[0];

	//Vec3d delta(coords[1], coords[2], coords[3]);

	//return m_mesh->getVertexPos(vi) + delta;
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

void GCylinder::reshapeFromPoints( std::vector<Vec3d>& pnts )
{
	// extract circles
	for(uint i = 0; i < gc->crossSection.size(); i++)
	{
		int offset = 1 + (cageSides * i);

		std::vector<Point> curvePoints;

		for(uint s = 0; s < cageSides; s++)
			curvePoints.push_back(pnts[offset + s]);
		
		// Computer center
		Point c(0,0,0);
		foreach(Point p, curvePoints)	c += p;
		c /= curvePoints.size();
		
		gc->frames.point[i] = c;

		// Compute radius
		gc->crossSection[i].radius = (curvePoints.front() - c).norm() / cageScale;
	}

	// Re-compute frames and align the cross-sections
	gc->frames.compute();
	gc->realignCrossSections();
	deformMesh();
}

void GCylinder::translateCurve( uint cid, Vec3d T, uint sid_respect )
{
	selectedPartId = cid;
	moveCurveCenter(cid, T);
}

uint GCylinder::detectHotCurve( std::vector< Vec3d > &hotSamples )
{
	Point samplesCenter(0,0,0);

	if(symmPlanes.empty())
	{
		foreach(Point p, hotSamples) samplesCenter += p;
		samplesCenter /= hotSamples.size();
	}
	else
		samplesCenter = hotSamples.front();

	uint closestIndex = 0;
	double minDist = DBL_MAX;

	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		double dist = (c.center - samplesCenter).norm();

		if(dist < minDist)
		{
			minDist = dist;
			closestIndex = c.index;
		}
	}

	selectedPartId = closestIndex;

	return selectedPartId;
}

bool GCylinder::excludePoints( std::vector< Vec3d >& pnts )
{
	// deprecated..
	// Shrink GC along its skeleton
	return true;
}


void GCylinder::translate( Vec3d &T )
{
	for(uint i = 0; i < gc->frames.count(); i++)
		gc->frames.point[i] += T;

	gc->frames.compute();
	gc->realignCrossSections();
	deformMesh();
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

void GCylinder::setSelectedPartId( Vec3d normal )
{
	// useless for GC
}

void GCylinder::setSymmetryPlanes( int nb_fold )
{
	Vec3d normal = gc->crossSection.front().n;
	Vec3d center = gc->crossSection.front().center;

	symmPlanes.push_back(Plane(normal, center));
}

void GCylinder::deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T )
{

	// theta = <p, j, p + T>
	Vec3d v1 = p - joint;
	Vec3d v2 = (p + T) - joint;

	double theta = acos(dot(v1.normalized(), v2.normalized()));
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

	gc->frames.compute();
	gc->realignCrossSections();
	deformMesh();

}

void GCylinder::movePoint( Point p, Vec3d T )
{
	// There is symmetry
	if (isRotationalSymmetry)
	{
		GeneralizedCylinder::Circle* c = &gc->crossSection[Primitive::detectHotCurve(p)];

		//// Project everything onto the plane
		//Plane plane(c->normal(), c->center);
		//Point q = plane.projectionOf(p + T);
		//p = plane.projectionOf(p);
		//T = q - p;

		//bool isScaleUp = true;

		//double r1 = (p - c->center).norm();
		//double r2 = (q - c->center).norm();

		//if(q - p < 0) isScaleUp = false;

		//double s = r2 / r1;
		
		double s = (T[0] > 0)? 0.4 : -0.4;
		this->scaleCurve(c->index, s );
	}
	// If there are no fixed points
	else if (fixedPoints.empty())
	{
		translate(T);
	}
	// Only one fixed point
	else if (fixedPoints.size() < 2)
	{
		deformRespectToJoint(fixedPoints.front(), p, T);

	}
	else
	// move corresponding curve piece only
	{
		int N = gc->crossSection.size();

		int start = 0, finish = N;
		GeneralizedCylinder::Circle* c = &gc->crossSection[Primitive::detectHotCurve(p)];

		std::vector<bool> fixedCrossSection(N, false);

		foreach(Point fp, fixedPoints)
		{
			int csi = gc->crossSection[Primitive::detectHotCurve(fp)].index;
			fixedCrossSection[csi] = true;
		}

		// If point to be moved is near fixed area, do nothing
		if(fixedCrossSection[c->index])
			return;

		// Find range
		for(uint i = 0; i < N; i++)
		{
			if(i < c->index && fixedCrossSection[i])
				start = Max(start, i);

			if(i > c->index && fixedCrossSection[i])
				finish = Min(finish, i);
		}

		// Move range
		moveCurveCenterRanged(c->index, T, start, finish);
	}
}

void GCylinder::save( std::ofstream &outF )
{
	outF << this->isFitted << "\t";
	outF << this->cageScale << "\t";
	outF << this->cageSides << "\t";
	outF << this->deltaScale << "\t";
	outF << this->originalSpine.size() << "\t";

	foreach(Point p, originalSpine)
	{
		outF << p << "\t";
	}

	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		outF << c.radius << "\t";
	}
}


void GCylinder::load( std::ifstream &inF, double scaleFactor )
{
	inF >> this->isFitted;
	inF >> this->cageScale;
	inF >> this->cageSides;
	inF >> this->deltaScale;

	inF >> skeletonJoints;

	for(int i = 0; i < skeletonJoints; i++)
	{
		Point p(0,0,0);
		inF >> p;
		p *= scaleFactor;
		originalSpine.push_back(p);
	}

	createGC(originalSpine, false);

	for(int i = 0; i < skeletonJoints; i++)
	{
		double radius;
		inF >> radius;
		radius *= scaleFactor;
		gc->crossSection[i].radius = radius;
	}

	buildCage();

	originalVolume = volume();
}

Point GCylinder::getSelectedCurveCenter()
{
	return gc->frames.point[selectedPartId];
}
