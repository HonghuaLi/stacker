#include "Cuboid.h"
#include "SimpleDraw.h"

#include <Eigen/Geometry>
using namespace Eigen;

Cuboid::Cuboid( QSurfaceMesh* segment, QString newId ) : Primitive(segment, newId)
{
	fit();

	selectedPartId = -1;
	isDrawAxis = false;
	isAvailable = true;
	isFrozen = false;
}

void Cuboid::fit()
{	
	MinOBB3 obb(m_mesh);
	originalBox = currBox = obb.mMinBox;
	id = m_mesh->objectName();

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
	// Draw center point
	//SimpleDraw::IdentifyPoint(currBox.Center);

	if(isSelected)
		drawCube(5, Vec4d(1,1,0,1));
	else
		drawCube(2, Vec4d(0,0,1,1));

	// Draw axis
	if(isDrawAxis){
		glColor4f(1,0,0,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[0], 0.1f);
		glColor4f(0,1,0,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[1], 0.1f);
		glColor4f(0,0,1,1);	SimpleDraw::DrawArrowDirected(currBox.Center, currBox.Axis[2], 0.1f);
	}

	//for (int i=0;i<symmPlanes.size();i++)
	//{
	//	symmPlanes[i].draw();
	//}
}

void Cuboid::drawCube(double lineWidth, Vec4d color, bool isOpaque)
{
	std::vector< std::vector<Vec3d> > faces = getBoxFaces(currBox);

	if(selectedPartId >= 0)
	{
		SimpleDraw::DrawSquare(faces[this->selectedPartId], false, 6, Vec4d(0,1,0,1));
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

Eigen::Vector3d Cuboid::V2E( Vec3d &vec )
{
	return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

Vec3d Cuboid::E2V( Eigen::Vector3d &vec )
{
	return Vec3d(vec[0], vec[1], vec[2]);
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

// Theta is measured in degree
Eigen::Matrix3d Cuboid::rotationMatrixAroundAxis( Vec3d u, double theta )
{
	u.normalize();

	double x = u[0], y = u[1], z = u[2];

	Eigen::Matrix3d I, cpm, tp, R;

	I = Eigen::Matrix3d::Identity(3,3);

	tp <<	x*x, x*y, x*z,
			x*y, y*y, y*z,
			x*z, y*z, z*z;

	cpm <<  0, -z,  y,
			z,  0, -x,
		   -y,  x,  0;

	theta = RADIANS(theta);
	R = cos(theta)*I + sin(theta)*cpm + (1-cos(theta))*tp;

	return R;
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

void Cuboid::deform( PrimitiveParam* params, bool isPermanent /*= false*/ )
{
	CuboidParam* cp = (CuboidParam*) params;

	// Deform the OBB
	translate(cp->getT());
	rotateAroundAxes(cp->getR());
	scaleAlongAxis(cp->getS());

	// Apply the deformation 
	deformMesh();

	// Apply deformation forever...
	if(isPermanent)
		originalBox = currBox;
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
	if ( dot( originalBox.Axis[2], cross( originalBox.Axis[0],  originalBox.Axis[1] ) ) < 0 )
		std::cout << "The coordinate frame is not right handed!" << std::endl;

	std::vector< std::vector< double > > projections(3);

	Vec3d &center = originalBox.Center;

	for (int i = 0; i < hotSamples.size(); i++)
	{
		Vec3d &vec = hotSamples[i] - center;

		for (int j = 0; j < 3; j++)
		{
			Vec3d &axis = originalBox.Axis[j];
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

Vec3d Cuboid::rotatePointByMatrix( Eigen::Matrix3d &R, Vec3d p )
{
	Eigen::Vector3d rp = R * V2E(p);
	return E2V(rp);
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

	if (v1.norm() == 0 || v2.norm() == 0)
		return; // Undefined behavior

	double scale = v2.norm() / v1.norm();

	// Rotation matrix
	Vec3d rotAxis = cross( v1, v2 );
	double theta = DEGREES( acos(RANGED(-1, dot(v1.normalized(),v2.normalized()), 1)) );
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

	// Scale the box only along one axis
	v2.normalize();
	uint selected_axis = -1;
	double largest_dot = 0;
	bool positive;
	for (int i = 0; i < 3; i++)
	{
		double dot_prod = dot( currBox.Axis[i], v2 );
		double abs_dot = abs( dot_prod );
		if (abs_dot > largest_dot)
		{
			selected_axis = i;
			largest_dot = abs_dot;
			positive = ( dot_prod > 0 );
		}
	}

	double dis = (scale - 1) * currBox.Extent[selected_axis];
	Vec3d t = currBox.Axis[selected_axis] * dis;
	if (!positive) t = -t;
	currBox.Center += t;

	currBox.Extent[selected_axis] *= scale;
}

std::vector <Vec3d> Cuboid::majorAxis()
{
	std::vector<Vec3d> result;
	result.push_back(originalBox.Axis[0]);
	result.push_back(originalBox.Axis[1]);
	result.push_back(originalBox.Axis[2]);
	return result;
}

void* Cuboid::getState()
{
	MinOBB3::Box3 *box = new MinOBB3::Box3(currBox);
	return (void*)box;
}

void Cuboid::setState( void *state)
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

		// Scaling along axis that define the normal of the symmetry plane
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
		deformRespectToJoint(fixedPoints[0], p, T);
	}

	deformMesh();
}

void Cuboid::scaleCurve( int cid, double s )
{

}
