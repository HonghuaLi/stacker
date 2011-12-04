#include "Cuboid.h"
#include "SimpleDraw.h"

#include <Eigen/Geometry>
using namespace Eigen;

#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)

Cuboid::Cuboid(QSurfaceMesh* mesh)
	: Primitive(mesh)
{
	fit();

	selectedPartId = -1;
}

void Cuboid::fit()
{	
	MinOBB3 obb(m_mesh);
	originalBox = currBox = obb.mMinBox;

	// Compute the OBB coordinates for all vertices
	coordinates.clear();
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	for (vit = m_mesh->vertices_begin(); vit != vend; ++vit)
	{
		Vector3 coord = getCoordinatesInBox(originalBox, points[vit]);
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

Vector3 Cuboid::getCoordinatesInBox( MinOBB3::Box3 &box, Vector3 &p )
{
	Vector3 local_p = p - box.Center;

	return Vector3( dot(local_p, box.Axis[0]) / box.Extent[0],
		dot(local_p, box.Axis[1]) / box.Extent[1],
		dot(local_p, box.Axis[2]) / box.Extent[2]);
}

Vector3 Cuboid::getPositionInBox( MinOBB3::Box3 &box, Vector3 &coord )
{
	Vector3 local_p = box.Extent[0] * coord[0] * box.Axis[0]
	+ box.Extent[1] * coord[1] * box.Axis[1]
	+ box.Extent[2] * coord[2] * box.Axis[2];

	return local_p + box.Center;
}

std::vector<Vector3> Cuboid::getBoxConners( MinOBB3::Box3 box )
{
	std::vector<Vector3> pnts(8);

	// Create right-hand system
	if ( dot(cross(box.Axis[0], box.Axis[1]), box.Axis[2]) < 0 ) 
	{
		box.Axis[2]  = -box.Axis[2];
	}

	std::vector<Vector3> Axis;
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

std::vector< std::vector<Vector3> > Cuboid::getBoxFaces(MinOBB3::Box3 fromBox)
{
	std::vector< std::vector<Vector3> > faces;
	std::vector<Vector3> pnts = getBoxConners(fromBox);
	std::vector<Vector3> f0 (4), f1 (4), f2 (4), f3 (4), f4 (4), f5 (4);

	f0[0] = pnts[1]; f0[1] = pnts[0]; f0[2] = pnts[3]; f0[3] = pnts[2];faces.push_back(f0);
	f1[0] = pnts[4]; f1[1] = pnts[5]; f1[2] = pnts[6]; f1[3] = pnts[7];faces.push_back(f1);
	f2[0] = pnts[0]; f2[1] = pnts[1]; f2[2] = pnts[5]; f2[3] = pnts[4];faces.push_back(f2);
	f3[0] = pnts[2]; f3[1] = pnts[3]; f3[2] = pnts[7]; f3[3] = pnts[6];faces.push_back(f3);
	f4[0] = pnts[1]; f4[1] = pnts[2]; f4[2] = pnts[6]; f4[3] = pnts[5];faces.push_back(f4);
	f5[0] = pnts[0]; f5[1] = pnts[4]; f5[2] = pnts[7]; f5[3] = pnts[3];faces.push_back(f5);
	 
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
}

void Cuboid::drawCube(double lineWidth, Vec4d color, bool isOpaque)
{
	std::vector< std::vector<Vector3> > faces = getBoxFaces(currBox);

	if(selectedPartId >= 0)
	{
		SimpleDraw::DrawSquare(faces[this->selectedPartId], false, 6, Vec4d(1,0,0,1));
		SimpleDraw::IdentifyPoint(selectedPartPos(), 0,1,0,20);
	}

	for(int i = 0; i < faces.size(); i++)
		SimpleDraw::DrawSquare(faces[i], isOpaque, lineWidth, color);
}

void Cuboid::drawNames(bool isDrawParts)
{
	if(isDrawParts)
	{
		int faceId = 0;

		std::vector< std::vector<Vector3> > faces = getBoxFaces(currBox);

		for(int i = 0; i < faces.size(); i++)
		{
			glPushName(faceId++);
			SimpleDraw::DrawSquare(faces[i]);
			glPopName();
		}

	}
	else
	{
		glPushName(this->id);
		drawCube(1,Vec4d(1,1,1,1), true);
		glPopName();
	}
}

Eigen::Vector3d Cuboid::V2E( Vector3 &vec )
{
	return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

Vector3 Cuboid::E2V( Eigen::Vector3d &vec )
{
	return Vector3(vec[0], vec[1], vec[2]);
}

void Cuboid::scaleAlongAxis( Vector3 &scales )
{
	currBox.Extent[0] *= scales[0];
	currBox.Extent[1] *= scales[1];
	currBox.Extent[2] *= scales[2];
}

void Cuboid::translate( Vector3 &T )
{
	currBox.Center += T;
}


Eigen::Matrix3d Cuboid::rotationMatrixAroundAxis( int axisId, double theta )
{
	theta = M_PI * theta / 180;
	Vector3 u = currBox.Axis[axisId];
	double x = u[0], y = u[1], z = u[2];

	Eigen::Matrix3d I, cpm, tp, R;

	I = Eigen::Matrix3d::Identity(3,3);

	tp <<	x*x, x*y, x*z,
			x*y, y*y, y*z,
			x*z, y*z, z*z;

	cpm <<  0, -z,  y,
			z,  0, -x,
		   -y,  x,  0;

	R = cos(theta)*I + sin(theta)*cpm + (1-cos(theta))*tp;

	return R;
}


void Cuboid::rotateAroundAxes( Vector3 &angles )
{
	Eigen::Matrix3d Rx = rotationMatrixAroundAxis(0, angles[0]);
	Eigen::Matrix3d Ry = rotationMatrixAroundAxis(1, angles[1]);
	Eigen::Matrix3d Rz = rotationMatrixAroundAxis(2, angles[2]);
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

	std::vector< std::vector<Vector3> > faces = getBoxFaces(currBox);
	std::vector<Vector3> face = faces[selectedPartId];

	for(int i = 0; i < face.size(); i++)
		partPos += face[i];

	return partPos / face.size();
}

PrimitiveParam * Cuboid::reshapePart( Vec3d q )
{
	CuboidParam * params = new CuboidParam;

	if(selectedPartId < 0) return NULL;

	debugPoints.clear();
	debugLines.clear();
	debugPoly.clear();

	std::vector< std::vector<Vector3> > faces = getBoxFaces(originalBox);
	std::vector< std::vector<Vector3> > newFaces = faces;

	// even / odd
	int j = selectedPartId;
	int k = j + 1;
	if(j % 2 != 0)	k = j - 1;

	// Get old faces vertices
	std::vector<Vector3> oldCurrFace = faces[j];
	std::vector<Vector3> oldOppositeFace = faces[k];
	std::vector<Vector3> tempFace(4);
	
	// Compute center of faces
	std::vector<Vec3d> faceCenter(faces.size());

	for(uint i = 0; i < faces.size(); i++){
		for(uint j = 0; j < faces[i].size(); j++)
			faceCenter[i] += faces[i][j];

		faceCenter[i] /= faces[i].size();
	}

	debugLines.push_back(std::make_pair(faceCenter[j], q));
	debugLines.push_back(std::make_pair(faceCenter[j], faceCenter[k]));

	// Compute rotation made
	Vec3d v1 = faceCenter[j] - faceCenter[k];
	Vec3d v2 = q - faceCenter[k];
	Vec3d axis = cross(v1,v2).normalized();

	Vec3d delta = faceCenter[k] - q;
	double omega = acos(RANGED(-1, dot(v1.normalized(),v2.normalized()), 1));

	// Rotate new face
	for(int i = 0; i < 4; i++)
	{
		Vec3d v = ((faces[j][i] - faceCenter[j]));
		v = v * cos(omega) + cross(axis, v) * sin(omega) + axis * dot(axis, v) * (1 - cos(omega));

		newFaces[j][i] = v + q;
		newFaces[k][i] = newFaces[j][i] + delta;

		tempFace[i] = newFaces[k][i] - (delta.normalized() * v1.norm());
	}

	// Compute parameters when reshaped

	int z = ((j / 2) + 1) % 3;
	int x = (z + 1) % 3;
	int y = (x + 1) % 3;

	// Translation:
	Vec3d t(0,0,0);
	t = (q - faceCenter[j]) / 2.0;
	
	params->forceParam(0, t.x()); params->forceParam(1, t.y()); params->forceParam(2, t.z());
	std::cout << "\nTrans:" << t << std::endl;

	// Scaling:
	Vec3d s(1,1,1);
	s[((j / 2) + 1) % 3] = delta.norm() / v1.norm();

	debugPoly.push_back(newFaces[j]);
	debugPoly.push_back(newFaces[k]);
	debugPoly.push_back(tempFace);

	params->forceParam(6, s.x()); params->forceParam(7, s.y());	params->forceParam(8, s.z());
	std::cout << "\nScale:" << s << std::endl;

	// Rotation:
	Vec3d v3 = (tempFace[1] - tempFace[0]);
	v1.normalize();
	v2.normalize();
	v3.normalize();

	/*Matrix3d m1, m2;

	Vec3d xA, yA, zA;
	xA = originalBox.Axis[0];
	yA = originalBox.Axis[1];
	zA = originalBox.Axis[2];

	m1 <<	xA.x(), yA.x(), zA.x(),
			xA.y(), yA.y(), zA.y(),
			xA.z(), yA.z(), zA.z();

	Vector3d xB, yB, zB;
	xB = m1 * V2E( v3  );
	zB = m1 * V2E( v2 );
	yB = zB.cross(xB);

	m2 <<	xB.x(), yB.x(), zB.x(),
			xB.y(), yB.y(), zB.y(),
			xB.z(), yB.z(), zB.z();

	Matrix3d R = m2;*/

	Matrix3d A, B;

	Vec3d xA, yA, zA;
	xA = originalBox.Axis[x];
	yA = originalBox.Axis[y];
	zA = originalBox.Axis[z];

	A << xA.x(), yA.x(), zA.x(),
		 xA.y(), yA.y(), zA.y(),
		 xA.z(), yA.z(), zA.z();

	Vec3d xB, yB, zB;
	xB = v3;
	zB = v2;
	yB = cross(zB,xB);

	B << xB.x(), yB.x(), zB.x(),
		 xB.y(), yB.y(), zB.y(),
		 xB.z(), yB.z(), zB.z();


	Matrix4d tm = umeyama(A, B, false);
	Affine3d T(tm);

	Matrix3d R = T.rotation();

	double theta = 0, psi = 0, phi = 0;

	if(abs(R(2,0)) != 1)
	{
		theta = -asin(R(2,0));
		psi = atan2( R(2,1) / cos(theta),  R(2,2) / cos(theta));
		phi = atan2( R(1,0) / cos(theta),  R(0,0) / cos(theta));
	}
	else
	{
		phi = 0;

		if(R(2,0) == -1)
		{
			theta = M_PI / 2.0;
			psi = phi + atan2(R(0,1), R(0,2));
		}
		else
		{
			theta = -M_PI / 2.0;
			psi = -theta + atan2(-R(0,1), -R(0,2));
		}
	}

	Vec3d rAngles(DEGREES(theta), DEGREES(psi), DEGREES(phi));

	params->forceParam(x + 3, rAngles[0]); 
	params->forceParam(y + 3, rAngles[1]);	
	params->forceParam(z + 3, rAngles[2]);
	std::cout << "\nRot:" << rAngles << std::endl << "\n" ;

	// Rotation:
	/*Vec3d r(0,0,0);
	v1.normalize();	v2.normalize();
	Vec3d pv1 (0, 0, 1);
	Vec3d pxv2 (0, dot(v2,originalBox.Axis[y]), dot(v2,originalBox.Axis[z]));
	Vec3d pyv2 (dot(v2,originalBox.Axis[x]), 0, dot(v2,originalBox.Axis[z]));

	// Normalize
	pxv2.normalize();
	pyv2.normalize();

	double sign_x = dot(cross(pv1, pxv2), Vec3d(1, 0, 0)) < 0;
	double sign_y = dot(cross(pv1, pyv2), Vec3d(0, 1, 0)) < 0;
	
	// Range it -1, 1
	sign_x = 2 * (sign_x - 0.5);
	sign_y = 2 * (sign_y - 0.5);

	double angle_x = DEGREES(acos(RANGED(-1, dot(pv1, pxv2), 1)));
	double angle_y = DEGREES(acos(RANGED(-1, dot(pv1, pyv2), 1)));
	if (angle_x < 0)	angle_x += 180;
	if (angle_y < 0)	angle_y += 180;

	r[x] = sign_x * angle_x;
	r[y] = sign_y * angle_y;

	params->forceParam(3, r[0]); params->forceParam(4, r[1]);	params->forceParam(5, r[2]);
	std::cout << "\nRot:" << r << std::endl << "\n" ;*/

	return params;
}
