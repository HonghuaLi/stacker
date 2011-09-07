#include "QSurfaceMesh.h"
#include "IO.h"

QSurfaceMesh::QSurfaceMesh() : Surface_mesh()
{
	vbo = NULL;

	triangles.clear();
	edges.clear();

	isReady = false;
}

void QSurfaceMesh::compute_bounding_box()
{
	Surface_mesh::Vertex_property<Point> points = vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	// compute bounding box
	bbmin = Point( FLT_MAX,  FLT_MAX,  FLT_MAX);
	bbmax = Point(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (vit = vertices_begin(); vit != vend; ++vit)
	{
		bbmin.minimize(points[vit]);
		bbmax.maximize(points[vit]);
	}

	center = (bbmin + bbmax) * 0.5f;
	radius = (bbmax - bbmin).norm() * 0.5f;
}

void QSurfaceMesh::set_color_vertices(double r, double g, double b, double a)
{
	Vertex_property<Color>  vcolors  = vertex_property<Color>("v:color");
	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	for (vit = vertices_begin(); vit != vend; ++vit)
		if (!is_deleted(vit))
			vcolors[vit] = Color(r,g,b,a);
}

void QSurfaceMesh::draw()
{
	if(!isReady) return;

	if(isDirty || !vbo)	update();

	vbo->render_smooth();
}

void QSurfaceMesh::drawFaceNames()
{
	if(isDirty)	update();
}

void QSurfaceMesh::update()
{
	update_face_normals();
	update_vertex_normals();

	// Create new VBO if you can't update
	if(vbo)
	{
		vbo->update();
	}
	else
	{
		// get required vertex and face properties
		Vertex_property<Point>  points   = vertex_property<Point>("v:point");
		Vertex_property<Point>  vnormals = vertex_property<Point>("v:normal");
		Face_property<Point>    fnormals = face_property<Point>("f:normal");
		Vertex_property<Color>  vcolors  = vertex_property<Color>("v:color");
		Vertex_property<float>  vtex     = vertex_property<float>("v:tex1D", 0.0);

		// Color

		// get face indices
		Surface_mesh::Face_iterator fit, fend = faces_end();
		Surface_mesh::Vertex_around_face_circulator fvit, fvend;
		Surface_mesh::Vertex v0, v1, v2;
		for (fit = faces_begin(); fit != fend; ++fit)
		{
			fvit = fvend = vertices(fit);
			v0 = fvit;
			v2 = ++fvit;

			do{
				v1 = v2;
				v2 = fvit;
				triangles.push_back(v0.idx());
				triangles.push_back(v1.idx());
				triangles.push_back(v2.idx());
			} while (++fvit != fvend);
		}

		// setup vertex arrays
		/*glVertexPointer(points.data());
		glNormalPointer(vnormals.data());
		glColorPointer(vcolors.data());
		glTexCoordPointer(vtex.data());*/

		vbo = new VBO<Point,Normal_,Color>(this->n_vertices(), points.data(), vnormals.data(), vcolors.data(), triangles);
	}

	isReady = true;
	isDirty = false;
}
