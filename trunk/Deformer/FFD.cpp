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

void FFD::bbFit( Vec3i res )
{
	Vec3d center = mesh->center;

	int Nx = Max(2, res.x());
	int Ny = Max(2, res.y());
	int Nz = Max(2, res.z());

	this->resolution = Vec3i(Nx, Ny, Nz);

	double dx = width / (Nx-1);
	double dy = length / (Ny-1);
	double dz = height / (Nz-1);

	Vec3d start_corner(-width/2, -length/2, -height/2);

	// indexing
	int i = 0;

	// Nx x Ny x Nz
	pointsGridIdx = StdVector<StdVector<StdVector < int > > > 
		(Nx, StdVector< StdVector < int > >(Ny, StdVector < int >(Nz))); 

	for(int z = 0; z < Nz; z++){
		for(int y = 0; y < Ny; y++){
			for(int x = 0; x < Nx; x++){
				// Grid indexing
				pointsGridIdx[x][y][z] = i;

				// Control point position
				Vec3d p = start_corner + Vec3d(dx * x, dy * y, dz * z);

				// Add it
				points.push_back(new QControlPoint(p, i++, Vec3i(x,y,z)));
			}
		}	
	}
}

