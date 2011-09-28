#include "QSurfaceMesh.h"
#include "IO.h"

#include "SimpleDraw.h"

QSurfaceMesh::QSurfaceMesh() : Surface_mesh()
{
	triangles.clear();
	edges.clear();

	isReady = false;
	isDirty = false;
	isDrawBB = true;

	averageEdgeLength = -1;
}

QSurfaceMesh::QSurfaceMesh( const QSurfaceMesh& from ) : Surface_mesh(from)
{
	averageEdgeLength = from.averageEdgeLength;

	this->isReady = from.isReady;
	this->bbmin = from.bbmin;
	this->bbmax = from.bbmax;
	this->radius = from.radius;
	this->triangles = from.triangles;
	this->edges = from.edges;
	this->isReady = from.isReady;
	this->isDirty = from.isDirty;
	this->isDrawBB = from.isDrawBB;
}

QSurfaceMesh& QSurfaceMesh::operator=( const QSurfaceMesh& rhs )
{
	Surface_mesh::operator=(rhs);

	this->isReady = rhs.isReady;
	this->bbmin = rhs.bbmin;
	this->bbmax = rhs.bbmax;
	this->radius = rhs.radius;
	this->triangles = rhs.triangles;
	this->edges = rhs.edges;
	this->isReady = rhs.isReady;
	this->isDirty = rhs.isDirty;
	this->isDrawBB = rhs.isDrawBB;

	return *this;
}

void QSurfaceMesh::computeBoundingBox()
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

void QSurfaceMesh::setColorVertices( double r, double g, double b, double a )
{
	Vertex_property<Color>  vcolors  = vertex_property<Color>("v:color");
	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	for (vit = vertices_begin(); vit != vend; ++vit)
		if (!is_deleted(vit))
			vcolors[vit] = Color(r,g,b,a);
}

void QSurfaceMesh::drawDebug()
{
	// Render mesh indicators
	// Bounding Box
	if(isDrawBB)
	{
		double w = bbmax.x() - bbmin.x();
		double l = bbmax.y() - bbmin.y();
		double h = bbmax.z() - bbmin.z();
		SimpleDraw::DrawBox(center, w *0.5, l *0.5, h *0.5);
	}

	// Curvature:
	if(get_vertex_property<Vec3d>("v:d1"))
	{
		Vertex_property<Point> points = vertex_property<Point>("v:point");
		Vertex_property<Vec3d> d1 = vertex_property<Vec3d>("v:d1"),	d2 = vertex_property<Vec3d>("v:d2");
		Vertex_iterator vit, vend = vertices_end();
		
		std::vector<Vec3d> starts, directions1, directions2;

		for(vit = vertices_begin(); vit != vend; ++vit)
		{
			starts.push_back(points[vit]);
			directions1.push_back(d1[vit]);
			directions2.push_back(d2[vit]);
		}

		SimpleDraw::DrawLineTick(starts, directions1, getAverageEdgeLength() * 0.5, false, 1,0,0,0.25);
		SimpleDraw::DrawLineTick(starts, directions2, getAverageEdgeLength() * 0.5, false, 0,0,1,0.25);
	}

	// Debug points
	foreach(Point p, debug_points)	SimpleDraw::IdentifyPoint(p, 1,0,0);
	foreach(Point p, debug_points2)	SimpleDraw::IdentifyPoint(p, 0,1,0);
	foreach(Point p, debug_points3)	SimpleDraw::IdentifyPoint(p, 0,0,1);

	// Debug lines
	foreach(std::vector<Point> line, debug_lines) SimpleDraw::IdentifyConnectedPoints(line, 1.0,0,0);
	foreach(std::vector<Point> line, debug_lines2) SimpleDraw::IdentifyConnectedPoints(line, 0,1.0,0);
	foreach(std::vector<Point> line, debug_lines3) SimpleDraw::IdentifyConnectedPoints(line, 0,0,1.0);
}

void QSurfaceMesh::simpleDraw()
{
	// Render mesh regularly (inefficient)
	Vertex_property<Point>  points   = vertex_property<Point>("v:point");
	Vertex_property<Normal>  vnormals = vertex_property<Point>("v:normal");
	Vertex_property<Color>  vcolors  = vertex_property<Color>("v:color");
	Face_iterator fit, fend = faces_end();
	Vertex_iterator vit, vend = vertices_end();
	Vertex_around_face_circulator fvit;
	Vertex v0, v1, v2;

	// Draw faces
	glBegin(GL_TRIANGLES);
	for(fit = faces_begin(); fit != fend; ++fit){
		fvit = vertices(fit);

		v0 = fvit; v1 = ++fvit; v2 = ++fvit;

		glColor4dv(vcolors[v0]);glNormal3dv(vnormals[v0]);glVertex3dv(points[v0]);
		glColor4dv(vcolors[v1]);glNormal3dv(vnormals[v1]);glVertex3dv(points[v1]);
		glColor4dv(vcolors[v2]);glNormal3dv(vnormals[v2]);glVertex3dv(points[v2]);
	}
	glEnd();
}

void QSurfaceMesh::drawFaceNames()
{
	// TODO:
}

void QSurfaceMesh::drawFacesUnique()
{
	glDisable(GL_LIGHTING);

	Vertex_property<Point> points = vertex_property<Point>("v:point");
	Face_iterator fit, fend = faces_end();
	Vertex_around_face_circulator fvit, fvend;
	Vertex v0, v1, v2;

	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_TRIANGLES);

	for(fit = faces_begin(); fit != fend; ++fit)
	{
		uint f_id = ((Face)fit).idx() + 1;

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
	computeBoundingBox();

	Surface_mesh::Vertex_property<Point> points = vertex_property<Point>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	for (vit = vertices_begin(); vit != vend; ++vit)
	{
		if (!is_deleted(vit))
		{
			points[vit] -= center;
		}
	}

	computeBoundingBox();
}

void QSurfaceMesh::assignVertexArray()
{
	Vertex_iterator vit, vend = vertices_end();

	for(vit = vertices_begin(); vit != vend; ++vit) 
		vertex_array.push_back(vit);
}

void QSurfaceMesh::assignFaceArray()
{
	Face_iterator fit, fend = faces_end();

	for(fit = faces_begin(); fit != fend; ++fit) 
		face_array.push_back(fit);
}

void QSurfaceMesh::fillTrianglesList()
{
	// get face indices
	Face_iterator fit, fend = faces_end();
	Vertex_around_face_circulator fvit, fvend;
	Vertex v0, v1, v2;

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
}

std::vector<uint> QSurfaceMesh::vertexIndicesAroundFace( uint f_id )
{
	std::vector<uint> vindices;

	Vertex_around_face_circulator fvit, fvend;
	fvit = fvend = vertices(face_array[f_id]);

	do{
		vindices.push_back( ((Vertex)fvit).idx() );
	} while (++fvit != fvend);

	return vindices;
}

Point QSurfaceMesh::getVertexPos( uint v_id )
{
	Vertex_property<Point> points = vertex_property<Point>("v:point");
	return points[vertex_array[v_id]];
}

Point QSurfaceMesh::getVertexPos( const Vertex & v )
{
	Vertex_property<Point> points = vertex_property<Point>("v:point");
	return points[v];
}

void QSurfaceMesh::setVertexColor( uint v_id, const Color& newColor )
{
	Vertex_property<Color> vcolor = vertex_property<Color>("v:color");
	vcolor[vertex_array[v_id]] = newColor;

	this->isDirty = true;
}

double QSurfaceMesh::getAverageEdgeLength()
{
	// Efficiency
	if(averageEdgeLength >= 0)
		return averageEdgeLength;

	Vertex_property<Point>  points  = vertex_property<Point>("v:point");

	Edge_iterator eit, eend = edges_end();

	double sumEdgeLen = 0;

	for(eit = edges_begin(); eit != eend; ++eit)
	{
		sumEdgeLen += (points[vertex(eit, 1)] - points[vertex(eit, 0)]).norm();
	}

	averageEdgeLength = sumEdgeLen / n_edges();

	return averageEdgeLength;
}

void QSurfaceMesh::resetVistedVertices(std::vector <Vertex>& all)
{
	Vertex_property<int> visited_map = vertex_property<int>("v:visit_map");

	std::vector<Vertex>::iterator it = all.begin(), ite = all.end();
	for(;it != ite; it++)
		visited_map[*it] = -1;
}

void QSurfaceMesh::resetVistedVertices(uint toState)
{
	Vertex_property<int> visited_map = vertex_property<int>("v:visit_map");

	Surface_mesh::Vertex_iterator vit, vend = vertices_end();

	for (vit = vertices_begin(); vit != vend; ++vit)
		visited_map[vit] = toState;
}

void QSurfaceMesh::collectEnoughRings(Vertex v, const size_t min_nb, std::vector <Vertex>& all)
{
	std::vector<Vertex> current_ring, next_ring;
	Vertex_property<int> visited_map = vertex_property<int>("v:visit_map");

	//initialize
	visited_map[v] = 0;
	current_ring.push_back(v);
	all.push_back(v);

	int i = 1;

	while ( (all.size() < min_nb) &&  (current_ring.size() != 0) )
	{
		// collect ith ring
		std::vector<Vertex>::iterator it = current_ring.begin(), ite = current_ring.end();

		for(;it != ite; it++)
		{
			// push neighbors of 
			Halfedge_around_vertex_circulator hedgeb = halfedges(*it), hedgee = hedgeb;

			do{
				Vertex vj = to_vertex(hedgeb);

				if (visited_map[vj] == -1)  
				{
					visited_map[vj] = i;
					next_ring.push_back(vj);
					all.push_back(vj);
				}
				
				++hedgeb;
			} while(hedgeb != hedgee);
		}

		//next round must be launched from p_next_ring...
		current_ring = next_ring;
		next_ring.clear();

		i++;
	}

	//clean up
	resetVistedVertices(all);
}

std::vector<Vec3d> QSurfaceMesh::pointsFace( Face f )
{
	Vertex_property<Point>  points  = vertex_property<Point>("v:point");

	Vertex_around_face_circulator fvit, fvend;
	fvit = fvend = vertices(f);

	std::vector<Vec3d> v;

	do{
		v.push_back(points[fvit]);
	} while (++fvit != fvend);

	return v;
}

double QSurfaceMesh::faceArea( Face f )
{
	std::vector<Vec3d> v = pointsFace(f);

	Vec3d t = cross((v[1] - v[0]), (v[2] - v[0]));
	return 0.5 * t.norm();
}

Vec3d QSurfaceMesh::getBaryFace( Face f, double U, double V )
{
	std::vector<Vec3d> v = pointsFace(f);

	if(U == 1.0) return v[1];
	if(V == 1.0) return v[2];

	double b1 = U;
	double b2 = V;
	double b3 = 1.0 - (U + V);

	Vec3d p;

	p.x() = (b1 * v[0].x()) + (b2 * v[1].x()) + (b3 * v[2].x());
	p.y() = (b1 * v[0].y()) + (b2 * v[1].y()) + (b3 * v[2].y());
	p.z() = (b1 * v[0].z()) + (b2 * v[1].z()) + (b3 * v[2].z());

	return p;
}

Vec3d QSurfaceMesh::fn( Face f )
{
	Face_property<Normal> normals = face_property<Normal>("f:normal");
	return normals[f];
}

Vec3d QSurfaceMesh::faceCenter( Face f )
{
	std::vector<Vec3d> v = pointsFace(f);
	return Vec3d ((v[0].x() + v[1].x() + v[2].x()) / 3.0, 
		(v[0].y() + v[1].y() + v[2].y()) / 3.0, 
		(v[0].z() + v[1].z() + v[2].z()) / 3.0);
}
