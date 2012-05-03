#pragma once

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"

class Decimater{
private:
	Scalar target_edge_length;
	Surface_mesh * mesh;
	Surface_mesh::Vertex_property<Point> points;
	Surface_mesh::Vertex_property<Normal> vnormal;

public:
	Decimater(Surface_mesh* mesh, double percent)
	{
		this->mesh = mesh;
		this->target_edge_length = target_edge_length;

		points = mesh->vertex_property<Point>("v:point");
		vnormal = mesh->vertex_property<Point>("v:normal");
	}

private:
	void doSimplify()
	{
		// 
	}

public:
	static void simplify(Surface_mesh *mesh, double percent)
	{
		Decimater d(mesh, percent);
		d.doSimplify();
	}
};
