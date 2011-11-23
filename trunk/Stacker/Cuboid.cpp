#include "Cuboid.h"
#include "SimpleDraw.h"


Cuboid::Cuboid(QSurfaceMesh* mesh)
	: Primitive(mesh)
{
	fit();
}

void Cuboid::fit()
{	
	MinOBB3 obb(m_mesh);
	preBox = currBox = obb.mMinBox;
}

void Cuboid::deformMesh()
{
	Surface_mesh::Vertex_property<Point> points = m_mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = m_mesh->vertices_end();

	Vector3 coord;
	for (vit = m_mesh->vertices_begin(); vit != vend; ++vit)
	{
		if (!m_mesh->is_deleted(vit))
		{
			coord = getCoordinatesInBox(preBox, points[vit]);
			points[vit] = getPositionInBox(currBox, coord);
		}
	}

//	preBox = currBox;
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

void Cuboid::draw()
{
	// Draw center point
	SimpleDraw::IdentifyPoint(currBox.Center);

	if(isSelected)
		drawCube(5, Vec4d(1,1,0,1));
	else
		drawCube(2, Vec4d(0,0,1,1));
}

void Cuboid::drawCube(double lineWidth, Vec4d color, bool isOpaque)
{
	std::vector<Vector3> pnts = getBoxConners(currBox);

	SimpleDraw::DrawSquare(pnts[1], pnts[0], pnts[3], pnts[2], isOpaque, lineWidth, color);
	SimpleDraw::DrawSquare(pnts[4], pnts[5], pnts[6], pnts[7], isOpaque, lineWidth, color);

	SimpleDraw::DrawSquare(pnts[0], pnts[1], pnts[5], pnts[4], isOpaque, lineWidth, color);
	SimpleDraw::DrawSquare(pnts[2], pnts[3], pnts[7], pnts[6], isOpaque, lineWidth, color);

	SimpleDraw::DrawSquare(pnts[1], pnts[2], pnts[6], pnts[5], isOpaque, lineWidth, color);
	SimpleDraw::DrawSquare(pnts[0], pnts[4], pnts[7], pnts[3], isOpaque, lineWidth, color);
}

void Cuboid::drawNames()
{
	glPushName(this->id);
	drawCube(1,Vec4d(1,1,1,1), true);
	glPopName();
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
	theta = 3.1415926 * theta / 180;
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

void Cuboid::transform( Vector3 &T, Vector3 &scales, Vector3 &angles )
{
	translate(T);
	scaleAlongAxis(scales);
	rotateAroundAxes(angles);
}

void Cuboid::undo()
{
	MinOBB3::Box3 box = preBox;
	preBox = currBox;
	currBox = box;
	deformMesh();
	preBox = box;
}
