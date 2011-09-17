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

void QSurfaceMesh::drawFacesUnique()
{
	glDisable(GL_LIGHTING);

	Face_property<uint> findex = face_property<uint>("f:index");
	Vertex_property<Point> points = vertex_property<Point>("v:point");
	Face_iterator fit, fend = faces_end();
	Vertex_around_face_circulator fvit, fvend;
	Vertex v0, v1, v2;

	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_TRIANGLES);

	for(fit = faces_begin(); fit != fend; ++fit)
	{
		uint f_id = findex[fit] + 1;

		GLubyte a = (f_id & 0xFF000000) >> 24;
		GLubyte r = (f_id & 0x00FF0000) >> 16;
		GLubyte g = (f_id & 0x0000FF00) >> 8;
		GLubyte b = (f_id & 0x000000FF) >> 0;

		// Magical color!
		glColor4ub(r,g,b,255 - a);

		fvit = fvend = vertices(fit);
		v0 = fvit;
		v2 = ++fvit;

		do{
			v1 = v2;
			v2 = fvit;

			glVertex3dv(points[v0]);
			glVertex3dv(points[v1]);
			glVertex3dv(points[v2]);

		} while (++fvit != fvend);

	}

	glEnd();

	glEnable(GL_LIGHTING);
}

void QSurfaceMesh::moveCenterToOrigin()
{
	compute_bounding_box();

	Surface_mesh::Vertex_property<Point> points = vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	for (vit = vertices_begin(); vit != vend; ++vit)
	{
		if (!is_deleted(vit))
		{
			points[vit] -= center;
		}
	}

	compute_bounding_box();
}


void QSurfaceMesh::assignVertexIndices()
{
	Vertex_property<uint> vindex = vertex_property<uint>("v:index");
	Vertex_iterator vit, vend = vertices_end();

	uint i = 0;
	for(vit = vertices_begin(); vit != vend; ++vit) 
	{
		vertex_array.push_back(vit);
		vindex[vit] = i++;
	}
}

void QSurfaceMesh::assignFaceIndices()
{
	Face_property<uint> findex = face_property<uint>("f:index");
	Face_iterator fit, fend = faces_end();

	uint i = 0;
	for(fit = faces_begin(); fit != fend; ++fit) 
	{
		face_array.push_back(fit);
		findex[fit] = i++;
	}
}

std::vector<uint> QSurfaceMesh::vertexIndicesAroundFace( uint f_id )
{
	std::vector<uint> vindices;

	Vertex_property<uint> vindex = vertex_property<uint>("v:index");
	Vertex_around_face_circulator fvit, fvend;

	fvit = fvend = vertices(face_array[f_id]);

	do{
		vindices.push_back(vindex[fvit]);
	} while (++fvit != fvend);


	return vindices;
}

Point QSurfaceMesh::getVertex( uint v_id )
{
	Vertex_property<Point> points = vertex_property<Point>("v:point");
	return points[vertex_array[v_id]];
}

void QSurfaceMesh::setVertexColor( uint v_id, const Color& newColor )
{
	Vertex_property<Color> vcolor = vertex_property<Color>("v:color");
	vcolor[vertex_array[v_id]] = newColor;
}