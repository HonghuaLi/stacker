#pragma once

#include "QSurfaceMesh.h"
#include "kdtree.h"
#include "Voxel.h"
#include "BoundingBox.h"

#define glv glVertex3dv
#define gln glNormal3d

class Voxeler
{
private:
	QSurfaceMesh * mesh;
	std::vector< Voxel > voxels;
	KDTree kd;

	double voxelSize;

	Voxel minVox;
	Voxel maxVox;

	// Special voxels
	KDTree outerVoxels, innerVoxels;

public:
	Voxeler( QSurfaceMesh * src_mesh, double voxel_size);

	FaceBounds findFaceBounds( Surface_mesh::Face f );
	bool isVoxelIntersects( const Voxel & v, Surface_mesh::Face f );
	void computeBounds();

	// Find inside and outside of mesh surface
	std::vector< Voxel > fillOther();
	void fillInsideOut(KDTree & inside, KDTree & outside);
	void fillOuter(KDTree & outside);

	// Intersection
	std::vector<Voxel> Intersects(Voxeler * other);

	// Visualization:
	void draw();
	void setupDraw();
	static void drawVoxels( const std::vector< Voxel > & voxels, double voxel_size = 1.0);

	uint d1, d2;
};
