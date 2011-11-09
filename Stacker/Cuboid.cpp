#include "Cuboid.h"


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

void Cuboid::draw()
{
	std::vector<Vector3> pnts = getBoxConners(currBox);

	SimpleDraw::IdentifyPoint(currBox.Center);
	SimpleDraw::IdentifyLine(pnts[0], pnts[1]);
	SimpleDraw::IdentifyLine(pnts[1], pnts[2]);
	SimpleDraw::IdentifyLine(pnts[2], pnts[3]);
	SimpleDraw::IdentifyLine(pnts[3], pnts[0]);

	SimpleDraw::IdentifyLine(pnts[0], pnts[4]);
	SimpleDraw::IdentifyLine(pnts[1], pnts[5]);
	SimpleDraw::IdentifyLine(pnts[2], pnts[6]);
	SimpleDraw::IdentifyLine(pnts[3], pnts[7]);

	SimpleDraw::IdentifyLine(pnts[4], pnts[5]);
	SimpleDraw::IdentifyLine(pnts[5], pnts[6]);
	SimpleDraw::IdentifyLine(pnts[6], pnts[7]);
	SimpleDraw::IdentifyLine(pnts[7], pnts[4]);
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

	preBox = currBox;
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

void Cuboid::scaleAlongAxis( int axisId, double scale )
{
	currBox.Extent[axisId] *= scale;
}

void Cuboid::translate( Vector3 T )
{
	currBox.Center += T;
}
