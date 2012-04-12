#pragma once

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"
#include "GraphicsLibrary/Skeleton/GeneralizedCylinder.h"

class Skinning{
public:
	Skinning(QSurfaceMesh * src_mesh, GeneralizedCylinder * using_gc);

	QSurfaceMesh * mesh;
	GeneralizedCylinder * gc;
	GeneralizedCylinder origGC;

	void deform();

private:
	void computeWeights();

	struct SkinningWeight{
		int n1, n2;	// skeleton node IDs
		double weight;
		Point c; //  projection on skeleton
		Vec3d d; //  delta from skeleton

		SkinningWeight(int s1, int s2, double w, Point proj, Vec3d delta) : 
			n1(s1),n2(s2),weight(w),c(proj),d(delta) {}
	};

	std::vector< SkinningWeight > weights;
};
