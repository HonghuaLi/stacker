#include "GCylinder.h"
#include "SkeletonExtract.h"
#include "SimpleDraw.h"

GCylinder::GCylinder( QSurfaceMesh* mesh ) : Primitive(mesh)
{
	fit();

	cage = NULL;
	buildCage();
	gcd = new GCDeformation(m_mesh, cage);
}

void GCylinder::fit()
{
	if(!m_mesh->vertex_array.size())
	{
		m_mesh->assignFaceArray();
		m_mesh->assignVertexArray();
	}

	// Extract and save skeleton
	SkeletonExtract skelExt( m_mesh );
	skel = new Skeleton();
	skelExt.SaveToSkeleton( skel );

	// Select part of skeleton
	skel->selectLongestPath();

	// Compute generalized cylinder
	gc = new GeneralizedCylinder( skel, m_mesh );

	Vec3d p = gc->frames.point.front();
	Vec3d q = gc->frames.point.back();

	// Manual deformation
	mf1 = new qglviewer::ManipulatedFrame;
	mf2 = new qglviewer::ManipulatedFrame;
	mf1->setPosition(qglviewer::Vec(p.x(), p.y(), p.z()));
	mf2->setPosition(qglviewer::Vec(q.x(), q.y(), q.z()));
	connect(mf1, SIGNAL(manipulated()), this, SLOT(update()));
	connect(mf2, SIGNAL(manipulated()), this, SLOT(update()));
}

void GCylinder::deformMesh()
{
	updateCage();
	gcd->deform();
}

void GCylinder::draw()
{
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glColor3d(0, 0.5, 1);

	double delta = 1.1;

	// Cross-sections
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		std::vector<Point> pnts = c.toSegments(30, gc->frames.U[c.index].s, delta);
		pnts.push_back(pnts.front());
		glBegin(GL_LINE_STRIP);
		foreach(Vec3d p, pnts) glVertex3dv(p);
		glEnd();
	}

	// Along height side, dashed
	glLineStipple(1, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);

	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.count(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * gc->crossSection[i].radius * delta));
	glEnd();
	glBegin(GL_LINE_STRIP);
	for(uint i = 0; i < gc->frames.count(); i++)
		glVertex3dv(gc->frames.point[i] + (gc->frames.U[i].r * -gc->crossSection[i].radius * delta));
	glEnd();

	glDisable(GL_LINE_STIPPLE);
	glEnable(GL_LIGHTING);

	// Debug
	Vec3d p(mf1->position().x, mf1->position().y, mf1->position().z);
	Vec3d q(mf2->position().x, mf2->position().y, mf2->position().z);

	SimpleDraw::IdentifyPoint(p);
	SimpleDraw::IdentifyPoint2(q);

	//gc->draw();
	cage->simpleDraw();
	//skel->draw(true);
}

void GCylinder::update()
{
	int N = gc->frames.count();
	std::vector<Point> oldPos = gc->frames.point;
	double segLength = (oldPos.front() - oldPos[1]).norm();
	double totalLength = segLength * N;

	Vec3d p(mf1->position().x, mf1->position().y, mf1->position().z);

	bool forward = true;
	Vec3d delta = p - oldPos.front();

	printf("p %f %f %f \n", delta.x(), delta.y(), delta.z());

	for(uint i = 0; i < N; i++)
	{
		double weight = 1 - ((i * segLength) / totalLength);

		gc->frames.point[i] += delta * weight;
	}

	// Re-compute frames and align the cross-sections
	gc->frames.compute();
	gc->realignCrossSections();

	deformMesh();
}

void GCylinder::buildCage()
{
	cage = new QSurfaceMesh;
		
	int sides = 6;
	uint vindex = 0;
	std::map<uint, Surface_mesh::Vertex> v;
	std::vector<Surface_mesh::Vertex> verts(3), verts2(3);

	// Start vertex
	v[vindex++] = cage->add_vertex(gc->crossSection.front().center);
		
	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		std::vector<Point> points = c.toSegments(sides, gc->frames.U[c.index].s, 1.25);
		for(int i = 0; i < sides; i++)
			v[vindex++] = cage->add_vertex(points[i]);
	}

	// End vertex
	v[vindex++] = cage->add_vertex(gc->crossSection.back().center);

	// Add faces
	int findex = 0;

	// Start cap
	for(int i = 1; i <= sides; i++)
	{
		verts[0] = v[i]; verts[1] = v[0]; verts[2] = v[(i % sides) + 1];
		cage->add_face( verts );
	}

	// Sides
	for(uint c = 0; c < gc->crossSection.size() - 1; c++)
	{
		int offset = (c * sides) + 1;

		for(int i = 0; i < sides; i++){
			int v1 = NEXT(i, sides) + offset;
			int v2 = NEXT(i + 1, sides) + offset;
			int v3 = v2 + sides;
			int v4 = v1 + sides;

			verts[0] = v[v1]; verts[1] = v[v2]; verts[2] = v[v3];
			verts2[0] = v[v1]; verts2[1] = v[v3]; verts2[2] = v[v4];

			// Add the two faces
			cage->add_face(verts);
			cage->add_face(verts2);
		}
	}

	// End cap
	int end = cage->n_vertices() - 1;
	for(int i = 0; i < sides; i++){
		verts[0] = v[end];
		verts[1] = v[(end-1) - NEXT(i + 2, sides)]; 
		verts[2] = v[(end-1) - NEXT(i + 1, sides)];
		cage->add_face(verts);
	}
	
	cage->setColorVertices(1,1,1,0.5);
	cage->update_face_normals();
	cage->update_vertex_normals();
}

void GCylinder::updateCage()
{
	int sides = 5;

	Surface_mesh::Vertex_property<Point> cagePoints = cage->vertex_property<Point>("v:point");
	
	// First point
	cagePoints[Surface_mesh::Vertex(0)] = gc->crossSection.front().center;

	foreach(GeneralizedCylinder::Circle c, gc->crossSection)
	{
		if(c.index < gc->crossSection.size() - 1) // cage constraints
		{
			std::vector<Point> points = c.toSegments(sides, gc->frames.U[c.index].s, 1.25);

			for(int i = 0; i < sides; i++)
			{
				uint vi = (1 + c.index * sides) + i;
				cagePoints[Surface_mesh::Vertex(vi)] = points[i];
			}
		}
	}

	// Last point
	//cagePoints[Surface_mesh::Vertex(cage->n_vertices() - 1)] = gc->crossSection.back().center;
}
