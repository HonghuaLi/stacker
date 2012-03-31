#include "Cuboid.h"
#include "SimpleDraw.h"

#include <Eigen/Geometry>
using namespace Eigen;

#include "OBB.h"
#include "OBB2.h"

Cuboid::Cuboid( QSurfaceMesh* segment, QString newId ) : Primitive(segment, newId)
{
	selectedPartId = -1;
	isDrawAxis = false;
	isFrozen = false;

	primType = CUBOID;
}

Cuboid::Cuboid( QSurfaceMesh* segment, QString newId, bool useAABB, int fit_method  ) : Primitive(segment, newId)
{
	fit(useAABB, fit_method);

	selectedPartId = -1;
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
		MinOBB3::Box3 fittedBox;

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
				OBB obb;
				obb.build_from_mesh(m_mesh);

				// set box parameters
				fittedBox.Center = obb.center();
				fittedBox.Axis = obb.axis();
				fittedBox.Extent = obb.extents();

				break;
			}
		case 2:
			{
				OBB2 obb(m_mesh);

				// set box parameters
				fittedBox.Center = obb.center();
				fittedBox.Axis = obb.axis();
				fittedBox.Extent = obb.extents();

				break;
			}
		}

		currBox = fittedBox;
	}

	originalBox = currBox;
	id = m_mesh->objectName();

	originalVolume = volume();

	computeMeshCoordiantes();
}

void Cuboid::computeMeshCoordiantes()
{
	// Compute the OBB coordinates for all vertices
	coordinates.clear();
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	for (vit = m_mesh->vertices_begin(); vit != vend; ++vit)
	{
		Vec3d coord = getCoordinatesInBox(originalBox, points[vit]);
		coordinates.push_back(coord);
	}
}

void Cuboid::deformMesh()
{
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	for (vit = m_mesh->vertices_begin(); vit != vend; ++vit)
	{
		int vidx = ((Surface_mesh::Vertex)vit).idx();
		points[vit] = getPositionInBox(currBox, coordinates[vidx]);
	}
}

Vec3d Cuboid::getCoordinatesInBox( MinOBB3::Box3 &box, Vec3d &p )
{
	Vec3d local_p = p - box.Center;

	return Vec3d( dot(local_p, box.Axis[0]) / box.Extent[0],
		dot(local_p, box.Axis[1]) / box.Extent[1],
		dot(local_p, box.Axis[2]) / box.Extent[2]);
}

Vec3d Cuboid::getPositionInBox( MinOBB3::Box3 &box, Vec3d &coord )
{
	Vec3d local_p = box.Extent[0] * coord[0] * box.Axis[0]
	+ box.Extent[1] * coord[1] * box.Axis[1]
	+ box.Extent[2] * coord[2] * box.Axis[2];

	return local_p + box.Center;
}

std::vector<Vec3d> Cuboid::getBoxConners( MinOBB3::Box3 &box )
{
	std::vector<Vec3d> pnts(8);

	// Create right-hand system
	if ( dot(cross(box.Axis[0], box.Axis[1]), box.Axis[2]) < 0 ) 
	{
		box.Axis[2]  = -box.Axis[2];
	}

	std::vector<Vec3d> Axis;
	for (int i=0;i<3;i++)
	{
		Axis.push_back( 2 * box.Extent[i] * box.Axis[i]);
	}

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

std::vector< std::vector<Vec3d> > Cuboid::getBoxFaces(MinOBB3::Box3 &fromBox)
{
	std::vector< std::vector<Vec3d> > faces(6);
	std::vector<Vec3d> pnts = getBoxConners(fromBox);

	uint pid[6][4] = {1, 2, 6, 5,
					  0, 4, 7, 3,
					  4, 5, 6, 7,
					  0, 3, 2, 1,
					  0, 1, 5, 4,
					  2, 3, 7, 6};

	for (int i = 0; i < 6; i++)	{
		for (int j = 0; j < 4; j++)	{
			faces[i].push_back( pnts[ pid[i][j] ] );
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
			drawCube(3, Vec4d(1,1,0,0.2));
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

	if(selectedPartId >= 0)
	{
		SimpleDraw::DrawSquare(faces[this->selectedPartId], false, lineWidth * 2, Vec4d(0,1,0,1));
		//SimpleDraw::IdentifyPoint(selectedPartPos(), 0,1,0,20);
	}

	for(int i = 0; i < faces.size(); i++)
		SimpleDraw::DrawSquare(faces[i], isOpaque, lineWidth, color);
}

void Cuboid::drawNames(int name, bool isDrawParts)
{
	if(isDrawParts)
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



void Cuboid::scaleAlongAxis( Vec3d &scales )
{
	currBox.Extent[0] *= scales[0];
	currBox.Extent[1] *= scales[1];
	currBox.Extent[2] *= scales[2];
}

void Cuboid::translate( Vec3d &T )
{
	currBox.Center += T;
}




void Cuboid::rotateAroundAxes( Vec3d &angles )
{
	Eigen::Matrix3d Rx = rotationMatrixAroundAxis(currBox.Axis[0], angles[0]);
	Eigen::Matrix3d Ry = rotationMatrixAroundAxis(currBox.Axis[1], angles[1]);
	Eigen::Matrix3d Rz = rotationMatrixAroundAxis(currBox.Axis[2], angles[2]);
	Eigen::Matrix3d R = Rx * Ry * Rz;

	Eigen::Vector3d p0 = R * V2E(currBox.Axis[0]);
	Eigen::Vector3d p1 = R * V2E(currBox.Axis[1]);
	Eigen::Vector3d p2 = R * V2E(currBox.Axis[2]);

	currBox.Axis[0] = E2V(p0);
	currBox.Axis[1] = E2V(p1);
	currBox.Axis[2] = E2V(p2);
}



void Cuboid::recoverMesh()
{
	currBox = originalBox;
	deformMesh();
}

double Cuboid::volume()
{
	return 8 * currBox.Extent.x() * currBox.Extent.y() * currBox.Extent.z();
}

std::vector <Vec3d> Cuboid::points()
{
	return getBoxConners(currBox);
}

Vec3d Cuboid::selectedPartPos()
{
	if(selectedPartId < 0) return Vec3d();

	Vec3d partPos(0,0,0);

	std::vector< std::vector<Vec3d> > faces = getBoxFaces(currBox);
	std::vector<Vec3d> face = faces[selectedPartId];

	for(int i = 0; i < face.size(); i++)
		partPos += face[i];

	return partPos / face.size();
}

uint Cuboid::detectHotCurve( std::vector< Vec3d > &hotSamples )
{
	if ( dot( currBox.Axis[2], cross( currBox.Axis[0],  currBox.Axis[1] ) ) < 0 )
		std::cout << "The coordinate frame is not right handed!" << std::endl;

	std::vector< std::vector< double > > projections(3);

	Vec3d &center = currBox.Center;

	for (int i = 0; i < hotSamples.size(); i++)
	{
		Vec3d &vec = hotSamples[i] - center;

		for (int j = 0; j < 3; j++)
		{
			Vec3d &axis = currBox.Axis[j];
			projections[j].push_back( dot( axis, vec ) );
		}

	}

	std::vector< double > range, mean;
	for (int j = 0; j < 3; j++)
	{
		double maxProj = MaxElement( projections[j] );
		double minProj = MinElement( projections[j] );

		range.push_back( maxProj - minProj );
		mean.push_back( (maxProj + minProj) / 2 );
	}


	uint axis = std::min_element( range.begin(), range.end() ) - range.begin();

	selectedPartId = 2 * axis;
	if (mean[axis] < 0) selectedPartId += 1;

	return selectedPartId;
}

void Cuboid::translateCurve( uint cid, Vec3d T, uint sid_respect /*= -1*/ )
{
	selectedPartId = cid;
	moveCurveCenter(cid, T);
}

//      k
//      |\
//      |-\ theta
//      |  \
//      j   q


void Cuboid::moveCurveCenter( int fid, Vec3d T )
{
	if (fid == -1)
		fid = selectedPartId;

	// Control points
	uint opp_fid = ( fid % 2 == 0 ) ? fid+1 : fid-1;

	Vec3d k = faceCenterOfBox(currBox, opp_fid);
	Vec3d j = faceCenterOfBox(currBox, fid);
	Vec3d q = j + T;

	deformRespectToJoint(k, j, T);

	// Deform the mesh
	deformMesh();
}

Vec3d Cuboid::faceCenterOfBox( MinOBB3::Box3 &box, uint fid )
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
	std::vector< std::vector<Vec3d> > faces(6);
	std::vector<Vec3d> pnts = getBoxConners(currBox);

	uint pid[6][4] = {1, 2, 6, 5,
		0, 4, 7, 3,
		4, 5, 6, 7,
		0, 3, 2, 1,
		0, 1, 5, 4,
		2, 3, 7, 6};

	QSurfaceMesh mesh;

	for(uint i = 0; i < pnts.size(); i++)
		mesh.add_vertex(pnts[i]);

	for (int i = 0; i < 6; i++)	{

		Surface_mesh::Vertex v0(pid[i][0]);
		Surface_mesh::Vertex v1(pid[i][1]);
		Surface_mesh::Vertex v2(pid[i][2]);
		Surface_mesh::Vertex v3(pid[i][3]);

		mesh.add_triangle(v0, v1, v2);
		mesh.add_triangle(v2, v3, v0);
	}

	return mesh;
}

std::vector<double> Cuboid::getCoordinate( Point v )
{
	std::vector<double> coords(3);

	Vec3d pos = getCoordinatesInBox(currBox, v);

	coords[0] = pos.x();
	coords[1] = pos.y();
	coords[2] = pos.z();

	return coords;
}

Point Cuboid::fromCoordinate( std::vector<double> coords )
{
	return getPositionInBox(currBox, Vec3d(coords[0], coords[1], coords[2]));
}

bool Cuboid::excludePoints( std::vector< Vec3d >& pnts )
{
	// Project pnts to axes
	std::vector< std::vector< double > > coords(3);
	for (int i = 0; i < pnts.size(); i++)
	{		
		Vec3d pos = getCoordinatesInBox(currBox, pnts[i]);

		for (int j = 0; j < 3; j++){
			coords[j].push_back( pos[j] );
		}
	}

	// Select the best axis to scale
	Vec3d z(0, 0, 1);
	int selectedID = -1;
	double bestScale = .0;
	double newPos = 0;
	for (int j = 0; j < 3; j++)
	{
		double epsilon = 0.1 * currBox.Extent[j];
		double left = MinElement(coords[j]) - epsilon;
		double right = MaxElement(coords[j]) + epsilon;

		// The pnts are not on the margin
		if (left * right < 0) continue;

		// Only scale along axis that is far away from z (the stacking direction)
		Vec3d &axis = currBox.Axis[j];
		if (abs( dot( axis, z ) ) > 0.5) continue;

		// if the scale is smaller
		double scale;
		if ( left > 0 )
			scale = (1+left) / 2;
		else
			scale = (1-right) / 2;

		if (scale > bestScale)
		{
			bestScale = scale;
			selectedID = j;
			newPos = (left>0)? left : right;
		}

	}

	// Scale along the best axis
	bool result = false;
	if (bestScale >.0)
	{
		double delta = (newPos > 0)? (-1 + newPos) / 2 : (newPos + 1) / 2;
		delta *= currBox.Extent[selectedID];
		currBox.Center += delta;
		currBox.Extent[selectedID] *= bestScale;
		result = true;
	}

	deformMesh();
	return result;
}

//    joint
//      |\
//      |-\ theta
//      |  \
//      p  p+T

void Cuboid::deformRespectToJoint( Vec3d joint, Vec3d p, Vec3d T )
{
	Vec3d q = p + T;
	Vec3d v1 = p - joint;
	Vec3d v2 = q - joint;

	Vec3d oldJointCoords = getCoordinatesInBox(currBox, joint);
	Vec3d pCoords = getCoordinatesInBox(currBox, p);

	if (v1.norm() == 0 || v2.norm() == 0)
		return; // Undefined behavior

	double scale = v2.norm() / v1.norm();

	// Rotation matrix
	Vec3d rotAxis = cross( v1, v2 );
	double dotProd = dot(v1.normalized(),v2.normalized());
	double theta = DEGREES( acos(dotProd) );
	if ( dot( rotAxis, cross( v1.normalized(),v2.normalized() ) ) < 0 )
		theta *= -1;

	Eigen::Matrix3d R = rotationMatrixAroundAxis(rotAxis, theta);

	// Rotate the box
	std::vector<Vec3d> conners = getBoxConners(currBox);

	currBox.Center = rotatePointByMatrix( R,  currBox.Center - joint ) + joint;

	Vec3d p0 = rotatePointByMatrix( R, conners[0] - joint ) + joint;
	Vec3d p1 = rotatePointByMatrix( R, conners[1] - joint ) + joint;
	Vec3d p4 = rotatePointByMatrix( R, conners[4] - joint ) + joint;
	Vec3d p3 = rotatePointByMatrix( R, conners[3] - joint ) + joint;
	currBox.Axis[0] = p1 - p0;
	currBox.Axis[1] = p4 - p0;
	currBox.Axis[2] = p0 - p3;
	currBox.normalizeAxis();


	//if(rotAxis.norm() < 10e-3) 
	//	int a = 1;
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

	double alpha = - oldJointCoords[axisID];
	double beta = pCoords[axisID];
	double sum = alpha + beta;
	alpha /= sum;
	beta /= sum;

	Vec3d newJointCoords = getCoordinatesInBox(currBox, joint);
	Vec3d qCoords = getCoordinatesInBox(currBox, q);

	Vec3d projJointCoords(0, 0, 0);
	Vec3d projQCoords(0, 0, 0);
	projJointCoords[axisID] = newJointCoords[axisID];
	projQCoords[axisID] = qCoords[axisID];
	Vec3d projJoint = getPositionInBox(currBox, projJointCoords);
	Vec3d projQ = getPositionInBox(currBox, projQCoords);
	
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


void* Cuboid::getGeometryState()
{
	MinOBB3::Box3 *box = new MinOBB3::Box3(currBox);
	return (void*)box;
}

void Cuboid::setGeometryState( void* state)
{
	currBox = *( (MinOBB3::Box3*) state );
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

	if (nb_fold == 1)
	{
		Vec3d normal = currBox.Axis[selectedPartId/2];
		symmPlanes.push_back(Plane(normal, center));
	} 
	else
	{
		int id = selectedPartId/2;
		Vec3d normal1 = currBox.Axis[(id+1)%3];
		Vec3d normal2 = currBox.Axis[(id+2)%3];

		symmPlanes.push_back(Plane(normal1, center));
		symmPlanes.push_back(Plane(normal2, center));
	}
}

void Cuboid::setSelectedPartId( Vec3d normal )
{
	double bestDot = 0;
	int id = -1;
	for (int i=0;i<3;i++)
	{
		double newDot = dot(normal, currBox.Axis[i]);

		if ( abs(newDot) > abs(bestDot))
		{
			id = i;
			bestDot = newDot;
		}
	}

	if (bestDot > 0)
		selectedPartId = id*2;
	else
		selectedPartId = id*2 + 1;

}

void Cuboid::reshapeFromPoints( std::vector<Vec3d>& corners )
{
	if (corners.size() != 8) return;

	Vec3d center(0,0,0);

	for (int i=0;i<8;i++)
		center += corners[i];

	currBox.Center = center / 8;

	currBox.Axis[0] = corners[1] - corners[0];
	currBox.Axis[1] = corners[4] - corners[0];
	currBox.Axis[2] = corners[0] - corners[3];

	for (int i=0;i<3;i++)
		currBox.Extent[i] = currBox.Axis[i].norm()/2;

	currBox.normalizeAxis();

	deformMesh();
}

bool Cuboid::containsPoint( Point p )
{
	bool result = false;

	Vec3d coor = getCoordinatesInBox(currBox, p);

	if ( RANGE(coor[0], -1, 1) && RANGE(coor[1], -1, 1) && RANGE(coor[2], -1, 1)) 
		result = true;

	return result;
}

Point Cuboid::closestPoint( Point p )
{
	return currBox.ClosestPoint(p);
}

void Cuboid::movePoint( Point p, Vec3d T )
{
	// Move the control point p according to some properties, such as symmetry, joint

	// There are two symmetry planes
	if (!symmPlanes.empty())
	{
		Point newP = p + T;
		Vec3d coordP = getCoordinatesInBox(currBox, p);
		Vec3d coordNewP = getCoordinatesInBox(currBox, newP);

		// Scaling along the normal of the symmetry planes
		Vec3d scales(coordNewP[0]/coordP[0], coordNewP[1]/coordP[1], coordNewP[2]/coordP[2]);
		
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
	else if (fixedPoints.empty())
	{
		// Translation
		currBox.Center += T;
	}
	else
	{
		// Only two fix points are allowed
		deformRespectToJoint(fixedPoints[0], p, T);
	}

	deformMesh();
}

void Cuboid::scaleCurve( int cid, double s )
{

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
}


void Cuboid::load( std::ifstream &inF, double scaleFactor )
{
	inF >> this->currBox.Center
		>> this->currBox.Axis[0] 
		>> this->currBox.Axis[1] 
		>> this->currBox.Axis[2] 
		>> this->currBox.Extent[0] 
		>> this->currBox.Extent[1]
		>> this->currBox.Extent[2]
		>> this->isUsedAABB;

	// Scaling
	Point center = currBox.Center;
	std::vector< Point > p(3);
	for (int i=0;i<3;i++)
	{
		p[i] = center + currBox.Axis[i] * currBox.Extent[i];
		p[i] *= scaleFactor;
	}
	center *= scaleFactor;

	currBox.Center = center;
	for (int i=0;i<3;i++ )
	{
		Vec3d vec = p[i] - center;
		currBox.Axis[i] = vec.normalized();
		currBox.Extent[i] = vec.norm();
	}


	originalBox = currBox;
	id = m_mesh->objectName();

	originalVolume = volume();

	computeMeshCoordiantes();
}

Point Cuboid::getSelectedCurveCenter()
{
	int id = selectedPartId % 2;

	return currBox.Center + currBox.Axis[id] * currBox.Extent[id];
}
