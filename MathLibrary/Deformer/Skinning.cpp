#include "Skinning.h"
#include "GraphicsLibrary/Basic/Line.h"
#include "DualQuat.h"
#include <Eigen/Geometry>
using namespace Eigen;

Skinning::Skinning( QSurfaceMesh * src_mesh, GeneralizedCylinder * using_gc )
{
	this->mesh = src_mesh;
	this->gc = using_gc;
	this->origGC = *gc;

	computeWeights();
}

void Skinning::computeWeights()
{
	Surface_mesh::Vertex_property<Point> points = mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
	{
		Vec3d v = points[vit];

		double minDis = DBL_MAX, minT = 0;
		int minIdx = -1;

		// Go over skeleton
		for(int j = 0; j < (int)origGC.crossSection.size() - 1; j++)
		{
			GeneralizedCylinder::Circle & c1 = origGC.crossSection[j];
			GeneralizedCylinder::Circle & c2 = origGC.crossSection[j+1];

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
					minIdx = j;
					minT = t;
				}
			}
		}

		// If at end or outside GC
		if(minIdx == -1){
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
			double tt = Line(c.center, c.center + c.normal()).timeAt(v);
			Point proj = c.center + c.normal()*tt;
			weights.push_back(SkinningWeight(minIdx, minIdx, 0.0, proj, v - proj));
		}
		else
		{
			Point proj = origGC.crossSection[minIdx].center*(1-minT) + origGC.crossSection[minIdx+1].center*minT;

			weights.push_back(SkinningWeight(minIdx, minIdx + 1, minT, proj, v - proj));
		}
	}
}

void Skinning::deform()
{
	int N = gc->crossSection.size();

	// Compute rotation and translation on each joint
	std::vector<Matrix3d> SPR(N, Matrix3d::Identity());
	std::vector<Vector3d> SPT(N);

	for(int i = 0; i < N; i++)
	{
		GeneralizedCylinder::Circle & orig_c = origGC.crossSection[i];
		GeneralizedCylinder::Circle & c = gc->crossSection[i];

		Vec3d n1(orig_c.normal()), n2(c.normal());
		Vector3d axis = V2E(cross(n1, n2).normalized());

		// Rotation
		if(axis.norm() > 0){
			double angle = acos(dot(n1, n2));
			SPR[i] = AngleAxisd(angle, axis).toRotationMatrix();
		}

		// Translation
		SPT[i] = V2E(c.center) - SPR[i] * V2E(orig_c.center);
	}

	// Blending
	Surface_mesh::Vertex_property<Point> points = mesh->vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
	{
		// Get weight and related skeleton segment
		int vi = Surface_mesh::Vertex(vit).idx();
		int i1 = weights[vi].n1;
		int i2 = weights[vi].n2;
		double w = weights[vi].weight;


		// Scaling
		double s = 1.0;
		GeneralizedCylinder::Circle & org_c1 = origGC.crossSection[i1];
		GeneralizedCylinder::Circle & org_c2 = origGC.crossSection[i2];
		GeneralizedCylinder::Circle & c1 = gc->crossSection[i1];
		GeneralizedCylinder::Circle & c2 = gc->crossSection[i2];

		if(w == 0)
			s = c1.radius / org_c1.radius; // Kevin used squared distance?
		else{
			// Interpolate radius
			double orig_r = org_c1.radius*(1-w) + org_c2.radius*w;
			double r = c1.radius*(1-w) + c2.radius*w;
			s = r / orig_r;
		}

		// Apply the skeleton scaling to original mesh point
		Vector3d V = V2E((weights[vi].c + weights[vi].d * s));
		

		// Rotation and translation using dual quaternion blending
		DualQuat dq1, dq2, dqb;
		dq1.SetTransform(SPR[i1], SPT[i1]);
		dq2.SetTransform(SPR[i2], SPT[i2]);
		dqb = dq1*(1-w)  +  dq2*w;

		Matrix3d R; Vector3d t;
		dqb.GetTransform(R, t); 
		
		V = R * V + t; // Apply

		points[vit] = E2V(V);
	}
}
