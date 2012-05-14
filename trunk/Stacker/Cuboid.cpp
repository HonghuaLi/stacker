#include "Cuboid.h"
#include "Utility/SimpleDraw.h"
#include "Numeric.h"

#include <Eigen/Geometry>
using namespace Eigen;

#include "MathLibrary/Bounding/OBB_PCA.h"
#include "MathLibrary/Bounding/OBB_Volume.h"

#include "MathLibrary/Coordiantes/MeanValueCoordinates.h"

Cuboid::Cuboid( QSurfaceMesh* segment, QString newId ) : Primitive(segment, newId)
{
	selectedCurveId = -1;
	isDrawAxis = false;
	isFrozen = false;

	primType = CUBOID;
}

Cuboid::Cuboid( QSurfaceMesh* segment, QString newId, bool useAABB, int fit_method  ) : Primitive(segment, newId)
{
	fit(useAABB, fit_method);

	selectedCurveId = -1;
	isDrawAxis = false;
	isFrozen = false;

	primType = CUBOID;
	isUsedAABB = useAABB;
}

void Cuboid::fit( bool useAABB, int obb_method )
{	
	if (useAABB)
	{
		m_mesh->computeBoundingBox();
		currBox.Center = (m_mesh->bbmax + m_mesh->bbmin) / 2;

		currBox.Axis[0] = Vec3d(1,0,0);
		currBox.Axis[1] = Vec3d(0,1,0);
		currBox.Axis[2] = Vec3d(0,0,1);

		currBox.Extent = (m_mesh->bbmax - m_mesh->bbmin) / 2;
	}
	else
	{
		Box3 fittedBox;

		switch(obb_method)
		{
		case 0:
			{
				MinOBB3 obb(m_mesh);
				fittedBox = obb.mMinBox;
				break;
			}
		case 1:
			{
				OBB_PCA obb;
				obb.build_from_mesh(m_mesh);

				// set box parameters
				fittedBox.Center = obb.center();
				fittedBox.Axis = obb.axis();
				fittedBox.Extent = obb.extents();

				break;
			}
		case 2:
			{
				OBB_Volume obb(m_mesh);

				// set box parameters
				fittedBox.Center = obb.center();
				fittedBox.Axis = obb.axis();
				fittedBox.Extent = obb.extents();

				break;
			}
		}

		currBox = fittedBox;
	}

	currBox.Extent += Vec3d(0.01); // needed for coordinates
	currBox.faceScaling.resize(6, 1.0);

	originalBox = currBox;

	id = m_mesh->objectName();

	originalVolume = volume();

	computeMeshCoordinates();
}

void Cuboid::computeMeshCoordinates()
{
	// Compute the OBB coordinates for all vertices
	coordinates.clear();
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	QSurfaceMesh cubeMesh = getGeometry();
	cubeMesh.fillTrianglesList();

	coordinates.resize(m_mesh->n_vertices(), std::vector<double>(cubeMesh.n_vertices()));

	#pragma omp parallel for
	for(int i = 0; i < m_mesh->n_vertices(); i++)
	{
		coordinates[i] = MeanValueCooridnates::weights(points[Surface_mesh::Vertex(i)], &cubeMesh);
	}
}

Vec3d Cuboid::getCoordinatesInUniformBox( Box3 &box, Vec3d &p )
{
	Vec3d local_p = p - box.Center;

	return Vec3d( dot(local_p, box.Axis[0]) / box.Extent[0],
		dot(local_p, box.Axis[1]) / box.Extent[1],
		dot(local_p, box.Axis[2]) / box.Extent[2]);
}

Vec3d Cuboid::getPositionInUniformBox( const Box3 &box, const Vec3d &coord )
{
	Vec3d local_p = box.Extent[0] * coord[0] * box.Axis[0]
	+ box.Extent[1] * coord[1] * box.Axis[1]
	+ box.Extent[2] * coord[2] * box.Axis[2];

	return local_p + box.Center;
}

Vec3d Cuboid::getPositionInBox( std::vector<Vec3d> & pnts, int vidx )
{
	Vec3d p(0,0,0);

	for(int i = 0; i < pnts.size(); i++)
		p += (pnts[i] * coordinates[vidx][i]);

	return p;
}

void Cuboid::deformMesh()
{
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");

	std::vector<Vec3d> pnts = getBoxCorners(currBox);

	#pragma omp parallel for
	for(int i = 0; i < m_mesh->n_vertices(); i++)
	{
		Surface_mesh::Vertex vit(i);
		int vidx = vit.idx();
		points[vit] = getPositionInBox(pnts, vidx);
	}

	m_mesh->computeBoundingBox();
}

std::vector<Point> Cuboid::getUniformBoxCorners( Box3 &box )
{
	std::vector<Vec3d> pnts(8);

	// Create right-hand system
	if ( dot(cross(box.Axis[0], box.Axis[1]), box.Axis[2]) < 0 ) 
		box.Axis[2]  = -box.Axis[2];

	std::vector<Vec3d> Axis;
	for (int i=0;i<3;i++)
		Axis.push_back( 2 * box.Extent[i] * box.Axis[i]);

	pnts[0] = box.Center - 0.5*Axis[0] - 0.5*Axis[1] + 0.5*Axis[2];
	pnts[1] = pnts[0] + Axis[0];
	pnts[2] = pnts[1] - Axis[2];
	pnts[3] = pnts[2] - Axis[0];

	pnts[4] = pnts[0] + Axis[1];
	pnts[5] = pnts[1] + Axis[1];
	pnts[6] = pnts[2] + Axis[1];
	pnts[7] = pnts[3] + Axis[1];

	return pnts;
}

std::vector<Point> Cuboid::getBoxCorners( Box3 &box )
{
	std::vector<Point> corner = getUniformBoxCorners(box);
	std::vector<Point> faceCenters = getUniformBoxFaceCenters(box);

	for (int i = 0; i < 6; i++)	
	{
		// Scale at most two faces, which have to be opposite to each other
		if(box.faceScaling[i] == 1.0) continue;
		
		// Scale corner point
		for (int j = 0; j < 4; j++)	{
			Vec3d delta = (corner[ cubeIds[i][j] ] - faceCenters[i]) * box.faceScaling[i];
			corner[ cubeIds[i][j] ] = faceCenters[i] + delta; 
		}
	}

	return corner;
}

std::vector< std::vector<Vec3d> > Cuboid::getBoxFaces(Box3 &fromBox)
{
	std::vector< std::vector<Vec3d> > faces(6);
	std::vector<Vec3d> pnts = getBoxCorners(fromBox);

	for (int i = 0; i < 6; i++)	{
		for (int j = 0; j < 4; j++)	{
			faces[i].push_back( pnts[ cubeIds[i][j] ] );
		}
	}

	return faces;
}

void Cuboid::draw()
{
	if (!isDraw) return;

	if(isDraw)
	{
		if(isSelected)
			drawCube(3, Vec4d(1,1,0,0.8));
		else
			drawCube(2, Vec4d(0,0,1,0.4));
	}

	// Draw axis
	if(isDrawAxis){
		glColor4f(1,0,0,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[0], 0.1f);
		glColor4f(0,1,0,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[1], 0.1f);
		glColor4f(0,0,1,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[2], 0.1f);
	}

	for (int i=0;i<symmPlanes.size();i++)
	{
		symmPlanes[i].draw(0.1);
	}
}

void Cuboid::drawCube(double lineWidth, Vec4d color, bool isOpaque)
{
	std::vector< std::vector<Vec3d> > faces = getBoxFaces(currBox);

	if(selectedCurveId >= 0 && this->selectedCurveId < faces.size())
		SimpleDraw::DrawSquare(faces[this->selectedCurveId], false, lineWidth * 2, Vec4d(0,1,0,1));

	for(int i = 0; i < faces.size(); i++)
		SimpleDraw::DrawSquare(faces[i], isOpaque, lineWidth, color);
}

void Cuboid::drawNames(int name, bool isDrawCurves)
{
	if(isDrawCurves)
	{
		int faceId = 0;

		std::vector< std::vector<Vec3d> > faces = getBoxFaces(currBox);

		for(int i = 0; i < faces.size(); i++)
		{
			glPushName(faceId++);
			SimpleDraw::DrawSquare(faces[i]);
			glPopName();
		}

	}
	else
	{
		glPushName(name);
		drawCube(1,Vec4d(1,1,1,1), true);
		glPopName();
	}
}

void Cuboid::translate( Vec3d &T )
{
	currBox.Center += T;
}

double Cuboid::volume()
{
	return getGeometry().volume();
}

std::vector<Point> Cuboid::points()
{
	return getUniformBoxFaceCenters(currBox);
}

std::vector<double> Cuboid::scales()
{
	return currBox.faceScaling;
}

Vec3d Cuboid::getSelectedCurveCenter()
{
	std::vector<Point> faceCenters = getUniformBoxFaceCenters(currBox);
	return faceCenters[selectedCurveId];
}

int Cuboid::detectHotCurve( Point hotSample )
{
	QVector<Point> hotSamples;
	hotSamples.push_back(hotSample);

	return detectHotCurve(hotSamples);
}

int Cuboid::detectHotCurve( QVector<Point> &hotSamples )
{
	if (hotSamples.isEmpty()) return -1;

	QSurfaceMesh currBoxMesh = getGeometry();
	QSurfaceMesh currUniformBoxMesh = getBoxGeometry(currBox, true);

	// Project hot samples to 3 axes of the cuboid
	std::vector< std::vector< double > > projections(3);
	for (int i = 0; i < hotSamples.size(); i++)
	{
		std::vector<double> weights = MeanValueCooridnates::weights(hotSamples[i], &currBoxMesh);
		Vec3d mappedHotPoint = MeanValueCooridnates::point(weights, &currUniformBoxMesh);

        Vec3d vec = mappedHotPoint - currBox.Center;
		for (int j = 0; j < 3; j++)
		{
            Vec3d axis = currBox.Axis[j];
			projections[j].push_back( dot( axis, vec ) );
		}
	}

	// The range and mean of projections
	std::vector< double > range, mean;
	for (int j = 0; j < 3; j++)
	{
		double maxProj = MaxElement( projections[j] );
		double minProj = MinElement( projections[j] );

		range.push_back( maxProj - minProj );
		mean.push_back( (maxProj + minProj) / 2 );
	}

	// The axis along which the projections have the minimal range is selected first
	// Use the mean of projections to decide which curve is hot
	uint axis = std::min_element( range.begin(), range.end() ) - range.begin();
	int cid = 2 * axis;
	if (mean[axis] < 0) cid += 1;

	return cid;
}


Vec3d Cuboid::faceCenterOfUniformBox( Box3 &box, uint fid )
{
	uint id = fid / 2;

	Vec3d faceCenter = box.Center;
	if ( fid % 2 == 0)
		faceCenter += box.Extent[id] * box.Axis[id];
	else
		faceCenter -= box.Extent[id] * box.Axis[id];

	return faceCenter;
}

QSurfaceMesh Cuboid::getGeometry()
{
	return getBoxGeometry(currBox);
}

QSurfaceMesh Cuboid::getBoxGeometry( Box3 &box, bool isUniform )
{
	std::vector< std::vector<Vec3d> > faces(6);
	std::vector<Vec3d> pnts;

	// Decide which corners to use
	if(isUniform) pnts = getUniformBoxCorners(box);
	else pnts = getBoxCorners(box);

	QSurfaceMesh mesh;

	for(uint i = 0; i < pnts.size(); i++)
		mesh.add_vertex(pnts[i]);

	for (int i = 0; i < 6; i++)	{

		Surface_mesh::Vertex v0(cubeIds[i][0]);
		Surface_mesh::Vertex v1(cubeIds[i][1]);
		Surface_mesh::Vertex v2(cubeIds[i][2]);
		Surface_mesh::Vertex v3(cubeIds[i][3]);

		mesh.add_triangle(v0, v1, v2);
		mesh.add_triangle(v2, v3, v0);
	}

	return mesh;
}

std::vector<double> Cuboid::getCoordinate( Point v )
{
	std::vector<double> coords(3);

	// Map points to uniform box coordinates
	QSurfaceMesh currBoxMesh = getGeometry();
	QSurfaceMesh currUniformBoxMesh = getBoxGeometry(currBox, true);

	v = MeanValueCooridnates::point(
		MeanValueCooridnates::weights(v, &currBoxMesh), &currUniformBoxMesh);

	Vec3d pos = getCoordinatesInUniformBox(currBox, v);

	coords[0] = pos.x();
	coords[1] = pos.y();
	coords[2] = pos.z();

	return coords;
}

Point Cuboid::fromCoordinate( std::vector<double> &coords )
{
	return getPositionInUniformBox(currBox, Vec3d(coords[0], coords[1], coords[2]));
}

//      k
//      |\
//      |-\ theta
//      |  \
//      p   q
void Cuboid::moveCurveCenter( int fid, Vec3d T )
{
	if (fid == -1)
		fid = selectedCurveId;

	double tol = currBox.Extent[0] / 10000;

	if (fid != -1)
	{
		uint opp_fid = ( fid % 2 == 0 ) ? fid+1 : fid-1;

		Vec3d k = faceCenterOfUniformBox(currBox, opp_fid);
		Vec3d p = faceCenterOfUniformBox(currBox, fid);
		Vec3d q = p + T;

		// Curve selected
		if(symmPlanes.isEmpty())
		{
			deformRespectToJoint(k, p, T);

			// Correct for singular cases
			Vec3d kk = faceCenterOfUniformBox(currBox, opp_fid);
			translate(k - kk);
		}
		else
		{
			if(symmPlanes.size() == 1)
			{
				// Single plane
				Plane plane = symmPlanes.front();
				Vec3d projT = plane.projectionOf(T) - plane.center;

				if(plane.IsOn(p))
				{
					deformRespectToJoint(k, p, projT);
				}
				else
				{
					translate(projT);

					// compute scale
					double s = abs( plane.GetPointDistance(q)/plane.GetPointDistance(p) );
					currBox.Extent[fid/2] *= s;
				}
			}
			else
			{
				// Two planes
				Plane planeA = symmPlanes.front();
				Plane planeB = symmPlanes.back();

				double dA = abs(planeA.GetPointDistance(p));
				double dB = abs(planeB.GetPointDistance(p));

				Vec3d projTA = planeA.projectionOf(T) - planeA.center;
				Vec3d projTB = planeB.projectionOf(T) - planeB.center;

				if(dA < tol && dB < tol)
				{
					// On both planes A and B
					Plane planeC( currBox.Axis[fid/2], currBox.Center );
					double s = abs( planeC.GetPointDistance(q)/planeC.GetPointDistance(p) );
					currBox.Extent[fid/2] *= s;
				}
				else if (dA < tol)
				{
					// On plane A
					double s = abs( planeB.GetPointDistance(q)/planeB.GetPointDistance(p) );
					currBox.Extent[fid/2] *= s;
				}
				else
				{
					// On plane B
					double s = abs( planeA.GetPointDistance(q)/planeA.GetPointDistance(p));
					currBox.Extent[fid/2] *= s;
				}
			}
		}

	}
	else
	{
		if(symmPlanes.size() == 1)
		{
			// Single plane
			Plane plane = symmPlanes.front();
			T = plane.projectionOf(T) - plane.center;
		}

		if(symmPlanes.size() == 2)
		{
			// Two planes
			Plane planeA = symmPlanes.front();
			Plane planeB = symmPlanes.back();

			T = planeA.projectionOf(T) - planeA.center;
			T = planeB.projectionOf(T) - planeB.center;
		}

		translate(T);
	}

	// Deform the mesh
	deformMesh();
}

//    joint
//      |\
//      |-\ theta
//      |  \
//      p  p+T

void Cuboid::deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T )
{
	// Map points to uniform box coordinates
	QSurfaceMesh currBoxMesh = getGeometry();
	QSurfaceMesh currUniformBoxMesh = getBoxGeometry(currBox, true);

	joint = MeanValueCooridnates::point(MeanValueCooridnates::weights(joint, &currBoxMesh), &currUniformBoxMesh);
	p = MeanValueCooridnates::point(MeanValueCooridnates::weights(p, &currBoxMesh), &currUniformBoxMesh);

	Vec3d q = p + T;
	Vec3d v1 = p - joint;
	Vec3d v2 = q - joint;

	Vec3d oldJointCoords = getCoordinatesInUniformBox(currBox, joint);
	Vec3d pCoords = getCoordinatesInUniformBox(currBox, p);

	if (v1.norm() == 0 || v2.norm() == 0)
		return; // Undefined behavior

	double scale = v2.norm() / v1.norm();

	// Rotation matrix
	Vec3d rotAxis = cross( v1, v2 );
	double dotProd = dot(v1.normalized(),v2.normalized());
	double theta = DEGREES( acos(RANGED(-1.0, dotProd, 1.0)) );
	if ( dot( rotAxis, cross( v1.normalized(),v2.normalized() ) ) < 0 )
		theta *= -1;

	Eigen::Matrix3d R = rotationMatrixAroundAxis(rotAxis, theta);

	// Rotate the box
	std::vector<Vec3d> conners = getUniformBoxCorners(currBox);

	currBox.Center = rotatePointByMatrix( R,  currBox.Center - joint ) + joint;

	Vec3d p0 = rotatePointByMatrix( R, conners[0] - joint ) + joint;
	Vec3d p1 = rotatePointByMatrix( R, conners[1] - joint ) + joint;
	Vec3d p4 = rotatePointByMatrix( R, conners[4] - joint ) + joint;
	Vec3d p3 = rotatePointByMatrix( R, conners[3] - joint ) + joint;
	currBox.Axis[0] = p1 - p0;
	currBox.Axis[1] = p4 - p0;
	currBox.Axis[2] = p0 - p3;
	currBox.normalizeAxis();

	// Scale the box only along one axis
	uint axisID = 0;
	double maxDot = 0;
	v2.normalize();
	for (int i = 0; i < 3; i++)
	{
		Vec3d axis = currBox.Axis[i];
		double abs_dot = abs( dot( axis, v2 ) );
		if (abs_dot > maxDot)
		{
			axisID = i;
			maxDot = abs_dot;
		}
	}

	// Blend coefficents
	double alpha = - oldJointCoords[axisID];
	double beta = pCoords[axisID];
	double sum = alpha + beta;
	alpha /= sum;
	beta /= sum;

	Vec3d newJointCoords = getCoordinatesInUniformBox(currBox, joint);
	Vec3d qCoords = getCoordinatesInUniformBox(currBox, q);

	Vec3d projJointCoords(0, 0, 0);
	Vec3d projQCoords(0, 0, 0);
	projJointCoords[axisID] = newJointCoords[axisID];
	projQCoords[axisID] = qCoords[axisID];
	Vec3d projJoint = getPositionInUniformBox(currBox, projJointCoords);
	Vec3d projQ = getPositionInUniformBox(currBox, projQCoords);
	
	currBox.Center = projJoint * beta + projQ * alpha;
	currBox.Extent[axisID] *= scale;
}

std::vector <Vec3d> Cuboid::majorAxis()
{
	std::vector<Vec3d> result;
	result.push_back(currBox.Axis[0]);
	result.push_back(currBox.Axis[1]);
	result.push_back(currBox.Axis[2]);
	return result;
}


void* Cuboid::getState()
{
	Box3 *box = new Box3(currBox);
	return (void*)box;
}

void Cuboid::setState( void* toState)
{
	currBox = *( (Box3*) toState );
}

std::vector < std::vector <Vec3d> > Cuboid::getCurves()
{
	return getBoxFaces(currBox);
}

// creat *nb_fold* symmetry according to the *selectedPartId*
void Cuboid::setSymmetryPlanes( int nb_fold )
{
	std::vector<Plane> result;

	Point center = currBox.Center;

	if(selectedCurveId < 0) selectedCurveId = 0;

	if (nb_fold == 1)
	{
		Vec3d normal = currBox.Axis[selectedCurveId/2];
		symmPlanes.push_back(Plane(normal, center));
	} 
	else
	{
		int id = selectedCurveId/2;
		Vec3d normal1 = currBox.Axis[(id+1)%3];
		Vec3d normal2 = currBox.Axis[(id+2)%3];

		symmPlanes.push_back(Plane(normal1, center));
		symmPlanes.push_back(Plane(normal2, center));
	}
}

void Cuboid::reshape( std::vector<Point>& pnts, std::vector<double>& scales )
{
	if(pnts.size() != 6 || scales.size() != 6) return;

	// Reshape from face centers and scales
	currBox.Center = (pnts[0] + pnts[1]) / 2;

	for (int i=0;i<3;i++)
	{
		currBox.Axis[i] = pnts[2*i] - pnts[2*i+1];
		currBox.Extent[i] = currBox.Axis[i].norm()/2;
	}

	currBox.normalizeAxis();

	currBox.faceScaling = scales;

	deformMesh();
}

bool Cuboid::containsPoint( Point p )
{
	QSurfaceMesh curBoxMesh = getGeometry();

	std::vector<double> weights = MeanValueCooridnates::weights(p, &curBoxMesh);

	foreach(double w, weights) 
		if(w < 0) return false;

	return true;
}

Point Cuboid::closestPoint( Point p )
{
	return getGeometry().closestVertex(p);
}

void Cuboid::movePoint( Point p, Vec3d T )
{
	// Move the control point p according to some properties, such as symmetry, joint, etc..

	// There are two symmetry planes
	if (!symmPlanes.empty())
	{
		Point newP = p + T;
		Vec3d coordP = getCoordinatesInUniformBox(currBox, p);
		Vec3d coordNewP = getCoordinatesInUniformBox(currBox, newP);

		Vec3d scales;
		for (int i = 0; i < 3; i++)
			scales[i] = (coordP[i] == 0.0)? 1 : coordNewP[i]/coordP[i];
		
		for (int i=0;i<3;i++ )
		{
			Vec3d axis = currBox.Axis[i];
			for (int j=0;j<symmPlanes.size();j++){
				if ( abs( dot(axis, symmPlanes[j].n) > 0.99 ) )
				{
					currBox.Extent[i] *= scales[i];
					break;
				}
			}
		}
		
	}
	// If there are no fixed points
	else if (fixedPoints.isEmpty())
	{
		// Translation
		translate(T);
	}
	else if (fixedPoints.size() == 1)
	{
		// Only two fix points are allowed
		deformRespectToJoint(fixedPoints[0], p, T);
	}
	else
		return;

	deformMesh();
}

void Cuboid::scaleCurve( int cid, double s )
{
	if(cid < 0)	cid = selectedCurveId;

	currBox.faceScaling[cid] *= s;

	// Reflect the scale
	foreach (Plane plane, symmPlanes)
	{
		Vec3d axis = currBox.Axis[cid/2];
		if( abs(dot(axis, plane.n) > 0.9) )
		{
			int op_cid = (cid % 2) ? cid - 1: cid + 1;
			currBox.faceScaling[op_cid] *= s;
			break;
		}
	}
	
	deformMesh();
}

void Cuboid::save( std::ofstream &outF )
{
	outF << this->currBox.Center << "\t" 
		<< this->currBox.Axis[0] << "\t" 
		<< this->currBox.Axis[1] << "\t" 
		<< this->currBox.Axis[2] << "\t"
		<< this->currBox.Extent[0] << "\t"
		<< this->currBox.Extent[1] << "\t"
		<< this->currBox.Extent[2] << "\t"
		<< this->isUsedAABB << "\t";

	foreach(double s, this->currBox.faceScaling)
		outF << s << "\t";
}


void Cuboid::load( std::ifstream &inF, Vec3d translation, double scaleFactor )
{
	this->currBox.faceScaling = std::vector<double>(6, 1.0);

	inF >> this->currBox.Center
		>> this->currBox.Axis[0] 
		>> this->currBox.Axis[1] 
		>> this->currBox.Axis[2] 
		>> this->currBox.Extent[0] 
		>> this->currBox.Extent[1]
		>> this->currBox.Extent[2]
		>> this->isUsedAABB;

	for(int i = 0; i < 6; i++)
		inF >> this->currBox.faceScaling[i];

	// Translating
	currBox.Center += translation;

	// Scaling
	std::vector< Point > p(3);
	for (int i=0;i<3;i++)
	{
		p[i] = currBox.Center + currBox.Axis[i] * currBox.Extent[i];
		p[i] *= scaleFactor;
	}
	currBox.Center *= scaleFactor;

	// Reconstruct the cuboid
	for (int i=0;i<3;i++ )
	{
		Vec3d vec = p[i] - currBox.Center;
		currBox.Axis[i] = vec.normalized();
		currBox.Extent[i] = vec.norm();
	}

	originalBox = currBox;
	id = m_mesh->objectName();

	originalVolume = volume();

	computeMeshCoordinates();
}


void Cuboid::moveLineJoint( Point A, Point B, Vec3d deltaA, Vec3d deltaB )
{


	if (fixedPoints.isEmpty())
	{
		// Translate \A to \newA
		currBox.Center += deltaA;

		Point newA = A + deltaA;
		Point newB = B + deltaA;
		Vec3d delta_newB = deltaB - deltaA;
		// Deform respect to joint
		deformRespectToJoint(newA, newB, delta_newB);
	}
	else
	{
		Point joint(0.0);
		foreach(Point p, fixedPoints) joint += p;
		joint /= fixedPoints.size();

		Point C = (A + B) / 2;
		Point newC = C + (deltaA + deltaB) / 2;

		// Project to the main axis
		Vec3d axis = currBox.ClosestAxis(C - joint);
		C = joint + dot(axis, C - joint) * axis;
		newC = joint + dot(axis, newC - joint) * axis;		

		deformRespectToJoint(joint, C, newC - C);
	}
	
	deformMesh();
}

double Cuboid::curveRadius( int cid )
{
	if (cid < 0 || cid > 5)
		return -1.0;

	int axis_id = cid / 2;
	Vec3d R = currBox.Extent;
	R[axis_id] = 0.0;

	return R.norm();
}

Point Cuboid::curveCenter( int cid )
{
	Point center;

	if (cid < 0 || cid > 5)
		center = Point(DOUBLE_INFINITY,0,0);
	else
	{
		int axis_id = cid / 2;

		if (cid % 2)
			center = currBox.Center - currBox.Axis[axis_id];
		else
			center = currBox.Center + currBox.Axis[axis_id];
	}

	return center;
}

std::vector<Point> Cuboid::getUniformBoxFaceCenters( Box3 &box )
{
	std::vector<Point> face_centers;
	for (int i = 0; i< 3; i++)
	{
		face_centers.push_back(box.Center + box.Axis[i]*box.Extent[i]);
		face_centers.push_back(box.Center - box.Axis[i]*box.Extent[i]);
	}

	return face_centers;
}

Point Cuboid::closestProjection( Point p )
{
	// If \p is inside the cuboid, return \p
	// Otherwise return the closest point on the faces.

	Vec3d xyz = getCoordinatesInUniformBox(currBox, p);

	for (int i = 0; i < 3; i++) 
		xyz[i] = RANGED(-1, xyz[i], 1);

	return getPositionInUniformBox(currBox, xyz);
}

bool Cuboid::atEnd( int dimensions, Point p )
{
	Vec3d coords = getCoordinatesInUniformBox(currBox, p);

	// Find the longest direction
	int longest_id = 0;
	Vec3d &ext = currBox.Extent;
	if (ext[1] > ext[0]) longest_id = 1;
	if (ext[2] > ext[longest_id]) longest_id = 2;


	if ( abs(coords[longest_id]) < 0.8 )
		return false;
	else
		return true;
}



