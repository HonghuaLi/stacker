#include "FFD.h"

FFD::FFD( QSurfaceMesh * src_mesh, FFD_FitType fit_type )
{
	this->mesh = src_mesh;

	if(!mesh) return;

	// Mesh dimensions
	width = mesh->bbmax.x() - mesh->bbmin.x();
	length = mesh->bbmax.y() - mesh->bbmin.y();
	height = mesh->bbmax.z() - mesh->bbmin.z();
}

StdVector<Vec3d> FFD::bbFit()
{
	StdVector<Vec3d> control_points;

	Vec3d center = mesh->center;

	int N = 3;

	double dx = width / (N-1);
	double dy = length / (N-1);
	double dz = height / (N-1);

	Vec3d start_corner(-width/2, -length/2, -height/2);

	for(int z = 0; z < N; z++){
		for(int y = 0; y < N; y++){
			for(int x = 0; x < N; x++){
				Vec3d p = start_corner + Vec3d(dx * x, dy * y, dz * z);
				control_points.push_back(p);
			}	
		}	
	}

	return control_points;
}
