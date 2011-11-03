#include "Contoller.h"

Controller::Controller( QSegMesh* mesh )
{
	m_mesh = mesh;
	
	fitPrimitives();
}


void Controller::fitPrimitives()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		primitives.push_back(Cuboid(m_mesh->getSegment(i)));
	}
}



void Controller::draw()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		
	}
}
