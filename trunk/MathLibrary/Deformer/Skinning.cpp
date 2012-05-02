#include "Skinning.h"
#include "GraphicsLibrary/Basic/Line.h"
#include "DualQuat.h"

Skinning::Skinning( QSurfaceMesh * src_mesh, GeneralizedCylinder * using_gc )
{
	this->mesh = src_mesh;
	this->gc = using_gc;
	this->origGC = *gc;

	computeMeshCoordinates();
}


Skinning::SkinningCoord Skinning::computeCoordinates( Point v )
{
	// The coordinates of \v is computed based the original GC \origGC

	// Go over the skeleton for the closest segment
	double minDis = DBL_MAX, minT = 0;
	int minIdx = -1;
	for(int i = 0; i < (int)origGC.crossSection.size() - 1; i++)
	{
		GeneralizedCylinder::Circle & c1 = origGC.crossSection[i];
		GeneralizedCylinder::Circle & c2 = origGC.crossSection[i+1];

		Line seg(c1.center, c2.center);
		double t = seg.timeAt(v);

		// Check belongs to segment
		if(t >= 0.0 && t <= 1.0){

			double dist = seg.distanceToUnbounded(v);
			double radAt = ((c1.radius * (1-t)) + (c2.radius * t)) * 1.2;

			// Check inside GC
			if( dist < radAt && dist < minDis )
			{
				minDis = dist;
				minIdx = i;
				minT = t;
			}
		}
	}

	if(minIdx == -1){
		// If at end or outside GC
		// Find the closest cross section
		minDis = DBL_MAX;
		for(int j = 0; j < (int)origGC.crossSection.size(); j++)
		{
			double dist = (origGC.crossSection[j].center - v).norm();

			if(dist < minDis){
				minDis = dist;
				minIdx = j;
			}
		}

		GeneralizedCylinder::Circle & c = origGC.crossSection[minIdx];
		Line seg(c.center, c.center + c.normal());
		double t = seg.timeAt(v);
		Point proj = c.center + c.normal()*t;

		return SkinningCoord(minIdx, minIdx, t, v - proj);
	}
	else
	{
		Point proj = origGC.crossSection[minIdx].center*(1-minT) + origGC.crossSection[minIdx+1].center*minT;
		return SkinningCoord(minIdx, minIdx + 1, minT, v - proj);
	}
}

void Skinning::computeMeshCoordinates()
{
	Surface_mesh::Vertex_property<Point> points = mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	coordinates.clear();
	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
	{
		Vec3d v = points[vit];
		coordinates.push_back(computeCoordinates(v));
	}
}

Point Skinning::fromCoordinates( SkinningCoord coords )
{
	int i1 = coords.n1;
	int i2 = coords.n2;
	double w = coords.time;

	GeneralizedCylinder::Circle & orig_c1 = origGC.crossSection[i1];
	GeneralizedCylinder::Circle & orig_c2 = origGC.crossSection[i2];
	GeneralizedCylinder::Circle & c1 = gc->crossSection[i1];
	GeneralizedCylinder::Circle & c2 = gc->crossSection[i2];

	// Projection on the skeleton
	Point proj;
	if (i1 == i2)
		proj = orig_c1.center + orig_c1.normal() * w;
	else
	{
		Line seg(orig_c1.center, orig_c2.center);
		proj = seg.pointAt(w);
	}

	// Scaling
	double s;
	if(i1 == i2)
		s = c1.radius / orig_c1.radius;
	else
	{
		// Interpolate radius
		double orig_r = orig_c1.radius*(1.0-w) + orig_c2.radius*w;
		double r = c1.radius*(1.0-w) + c2.radius*w;	 
		s = r / orig_r;
	}

	// Rotation and translation 
	// Using dual quaternion blending
	Matrix3d R1 = rotationOfCurve(i1);
	Matrix3d R2 = rotationOfCurve(i2);
	Vector3d T1 = V2E(c1.center) - R1 * V2E(orig_c1.center);
	Vector3d T2 = V2E(c2.center) - R2 * V2E(orig_c2.center);

	DualQuat dq1, dq2, dqb;
	dq1.SetTransform(R1, T1);
	dq2.SetTransform(R2, T2);
	dqb = dq1*(1-w)  +  dq2*w;

	Matrix3d R; Vector3d T;
	dqb.GetTransform(R, T); 

	// The result
	Vector3d V = V2E((proj + coords.d * s));
	V = R * V + T;
	return E2V(V);
}

std::vector<double> Skinning::getCoordinate( Point p )
{
	std::vector<double> coords;

	SkinningCoord skinning_coord = computeCoordinates(p);

	coords.push_back(skinning_coord.n1);
	coords.push_back(skinning_coord.n2);
	coords.push_back(skinning_coord.time);

	coords.push_back(skinning_coord.d[0]);
	coords.push_back(skinning_coord.d[1]);
	coords.push_back(skinning_coord.d[2]);

	return coords;
}

Point Skinning::fromCoordinates( std::vector<double> coords )
{
	if(coords.size() != 6) return Point(0.0);

	SkinningCoord skinning_coords;
	skinning_coords.n1 = int(coords[0]);
	skinning_coords.n2 = int(coords[1]);
	skinning_coords.time = coords[2];
	skinning_coords.d = Vec3d(coords[3], coords[4], coords[5]);

	return fromCoordinates(skinning_coords);
}

Matrix3d Skinning::rotationOfCurve( int cid )
{
	Matrix3d rm = Matrix3d::Identity();

	if (cid < 0 || cid >= gc->crossSection.size())
		return rm;

	GeneralizedCylinder::Circle & orig_c = origGC.crossSection[cid];
	GeneralizedCylinder::Circle & c = gc->crossSection[cid];

	Vec3d n1(orig_c.normal()), n2(c.normal());
	Vector3d axis = V2E(cross(n1, n2).normalized());

	if(axis.norm() > 0)
	{
		double angle = acos(dot(n1, n2));
		rm = AngleAxisd(angle, axis).toRotationMatrix();
	}

	return rm;
}

void Skinning::deform()
{
	Surface_mesh::Vertex_property<Point> points = mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
	{		
		int vi = Surface_mesh::Vertex(vit).idx();
		points[vit] = fromCoordinates(coordinates[vi]);
	}
}
