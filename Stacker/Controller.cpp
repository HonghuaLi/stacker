#include "Contoller.h"

Controller::Controller( QSegMesh* mesh )
{
	m_mesh = mesh;
	
	fitPrimitives();
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
		Cuboid* cub = new Cuboid(m_mesh->getSegment(i));
		primitives.push_back((Primitive*)cub);
	}
}



void Controller::draw()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		primitives[i]->draw();
	}
}
