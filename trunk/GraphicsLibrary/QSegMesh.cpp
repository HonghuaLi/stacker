#include "QSegMesh.h"
#include <fstream>
#include <set>
#include <map>


QSegMesh::QSegMesh()
{
	isReady = false;
	segment.clear();
}

QSegMesh::QSegMesh( const QSegMesh& from )
{
	this->isReady = from.isReady;

	this->bbmin = from.bbmin;
	this->bbmax = from.bbmax;
	this->center = from.center;
	this->radius = from.radius;

	for (int i=0;i<from.segment.size();i++)
	{
		QSurfaceMesh *seg_mesh = new QSurfaceMesh(*from.segment[i]);
		segment.push_back(seg_mesh);
	}
}

QSegMesh& QSegMesh::operator=( const QSegMesh& rhs )
{
	this->isReady = rhs.isReady;

	this->bbmin = rhs.bbmin;
	this->bbmax = rhs.bbmax;
	this->center = rhs.center;
	this->radius = rhs.radius;

	for (int i=0;i<rhs.segment.size();i++)
	{
		QSurfaceMesh *seg_mesh = new QSurfaceMesh(*rhs.segment[i]);
		segment.push_back(seg_mesh);
	}

	return *this;
}





void QSegMesh::read( QString fileName )
{
	// Load entire mesh geometry
	QSurfaceMesh shape;
	shape.read(qPrintable(fileName));

	// Load segmentation file
	QString segFilename = fileName.replace(fileName.lastIndexOf('.')+1, 3, "seg");
	std::ifstream inF(qPrintable(segFilename), std::ios::in);		
	int nbSeg;
	inF >> nbSeg;

	std::vector<int> faceSeg(shape.n_faces());
	int fid, sid;
	for (int i=0;i<shape.n_faces()&&inF;i++)
	{
		inF >> fid >> sid;	
		faceSeg[fid] = sid;
	}
	inF.close();

	// Create segments
	for (int i=0;i<nbSeg;i++)
	{
		segment.push_back(new QSurfaceMesh());
	}


	// Create unique vertex set for each segment
	std::vector<std::set<int>> segVertices(nbSeg);
	Surface_mesh::Face_iterator fit, fend = shape.faces_end();
	Surface_mesh::Vertex_around_face_circulator fvit;	
	
	for (fit = shape.faces_begin(); fit!=fend; ++fit)
	{
		Surface_mesh::Face f = fit;
		int fid = f.idx();
		int sid = faceSeg[fid];

		fvit = shape.vertices(fit);	
		Surface_mesh::Vertex v0, v1, v2;
		v0 = fvit; v1 = ++fvit; v2 = ++fvit;
		segVertices[sid].insert(v0.idx());
		segVertices[sid].insert(v1.idx());
		segVertices[sid].insert(v2.idx());
	}

	// Add Vertices to each segment	
	std::vector<std::map<int, int>> segVerMap(nbSeg);	
	Surface_mesh::Vertex_property<Point>  points   = shape.vertex_property<Point>("v:point");

	for (int i=0;i<nbSeg;i++)
	{
		std::set<int>::iterator vit, vend = segVertices[i].end();
		int j = 0;
		for (vit=segVertices[i].begin(); vit!=vend; vit++, j++)
		{
			segment[i]->add_vertex(shape.getVertexPos(*vit));			
			segVerMap[i].insert(std::make_pair(*vit, j));  // Create a new index for each vertex
		}
	}

	// Add Faces to each segment
	std::vector<Surface_mesh::Vertex>  vertices(3);
	for (fit = shape.faces_begin(); fit!=fend; ++fit)
	{
		Surface_mesh::Face f = fit;
		int fid = f.idx();
		int sid = faceSeg[fid];

		Surface_mesh::Vertex v0, v1, v2;
		fvit = shape.vertices(fit);	
		v0 = fvit; v1 = ++fvit; v2 = ++fvit;		
		
		vertices[0] = Surface_mesh::Vertex(segVerMap[sid][v0.idx()]);
		vertices[1] = Surface_mesh::Vertex(segVerMap[sid][v1.idx()]);
		vertices[2] = Surface_mesh::Vertex(segVerMap[sid][v2.idx()]);

		segment[sid]->add_face(vertices);
	}


	// Build up
	build_up();
}

void QSegMesh::build_up()
{
	setColorVertices();
	moveCenterToOrigin();
}

void QSegMesh::update_face_normals()
{
	for (int i=0;i<nbSegments();i++)
	{
		segment[i]->update_face_normals();
	}
}

void QSegMesh::update_vertex_normals()
{
	for (int i=0;i<nbSegments();i++)
	{
		segment[i]->update_vertex_normals();
	}
}


void QSegMesh::computeBoundingBox()
{
	// compute bounding box
	bbmin = Point( FLT_MAX,  FLT_MAX,  FLT_MAX);
	bbmax = Point(-FLT_MAX, -FLT_MAX, -FLT_MAX);	
	
	for (int i=0;i<nbSegments();i++)
	{
		Surface_mesh::Vertex_property<Point> points = segment[i]->vertex_property<Point>("v:point");
		Surface_mesh::Vertex_iterator vit, vend = segment[i]->vertices_end();

		for (vit = segment[i]->vertices_begin(); vit != vend; ++vit)
		{
			bbmin.minimize(points[vit]);
			bbmax.maximize(points[vit]);
		}		
	}
	
	center = (bbmin + bbmax) * 0.5f;
	radius = (bbmax - bbmin).norm() * 0.5f;
}

void QSegMesh::moveCenterToOrigin()
{
	computeBoundingBox();

	for (int i=0;i<nbSegments();i++)
	{
		Surface_mesh::Vertex_property<Point> points = segment[i]->vertex_property<Point>("v:point");
		Surface_mesh::Vertex_iterator vit, vend = segment[i]->vertices_end();

		for (vit = segment[i]->vertices_begin(); vit != vend; ++vit)
		{
			if (!segment[i]->is_deleted(vit))
			{
				points[vit] -= center;
			}
		}
	}

	computeBoundingBox();
}

void QSegMesh::setColorVertices()
{
	for (int i=0;i<segment.size();i++)
	{
		segment[i]->setColorVertices();
	}
}

int QSegMesh::nbSegments()
{
	return segment.size();
}

QSurfaceMesh* QSegMesh::operator[]( int i )
{
	return segment[i];
}

QSurfaceMesh* QSegMesh::getSegment( int i )
{
	return segment[i];
}

void QSegMesh::simpleDraw()
{

}



void QSegMesh::drawFacesUnique()
{

}