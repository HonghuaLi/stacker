#include "Contoller.h"
#include "QSegMesh.h"
#include "Cuboid.h"
#include "GCylinder.h"

Controller::Controller( QSegMesh* mesh )
{
	m_mesh = mesh;
}

Controller::~Controller()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		delete primitives[i];
	}
}

void Controller::fitPrimitives()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		GCylinder* cub = new GCylinder(m_mesh->getSegment(i));
		primitives.push_back(cub);
	}
}


void Controller::fitOBBs()
{
	foreach (QSurfaceMesh* segment, m_mesh->getSegments())
	{
		Cuboid* cub = new Cuboid(segment);
		primitives.push_back(cub);
	}
}


void Controller::draw()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		primitives[i]->draw();
	}
}

void Controller::test1()
{
	// Deform the primitive
	Cuboid* cp = ( Cuboid* )primitives[0];
	cp->scaleAlongAxis( Vector3(0.9, 0.9, 0) );
	cp->deformMesh();
	m_mesh->computeBoundingBox();

}

void Controller::test2(Vector3 scale, Vector3 transl, Vector3 angles)
{
	
	// top
	Cuboid* cp0 = ( Cuboid* )primitives[0];
	cp0->transform( Vector3(0, 0, 0),scale, Vector3(0, 0, 0));

	// legs
	double tx = transl[0], ty = transl[1], tz = transl[2];
	double a = angles[0],  b = angles[1], c = angles[2];
	Cuboid* cp1 = ( Cuboid* )primitives[1];
	cp1->transform( Vector3(tx, ty, tz), Vector3(1, 1, 1), Vector3(a, b, c));
	Cuboid* cp2 = ( Cuboid* )primitives[2];
	cp2->transform( Vector3(tx, -ty, tz), Vector3(1, 1, 1), Vector3(a, b, c));
	Cuboid* cp3 = ( Cuboid* )primitives[3];
	cp3->transform( Vector3(-tx, -ty, tz), Vector3(1, 1, 1), Vector3(a, -b, c));
	Cuboid* cp4 = ( Cuboid* )primitives[4];
	cp4->transform( Vector3(-tx, +ty, tz), Vector3(1, 1, 1), Vector3(a, b, c));

	cp0->deformMesh();
	cp1->deformMesh();
	cp2->deformMesh();
	cp3->deformMesh();
	cp4->deformMesh();

	m_mesh->computeBoundingBox();

}

Primitive * Controller::getPrimitive( int id )
{
	return primitives[id];
}



void Controller::undo()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		(( Cuboid* )primitives[i])->undo();
	}

	m_mesh->computeBoundingBox();
}
