#include "Cuboid.h"


Cuboid::Cuboid(QSurfaceMesh* mesh)
	: Primitive(mesh)
{
	fit();
}


Cuboid::~Cuboid(void)
{
	delete m_mobb;
}


 void Cuboid::fit()
 {	
	m_mobb = new MinOBB3(m_mesh);
 }

void Cuboid::draw()
{
	if (!m_mobb) return;

	m_mobb->draw();
}

void Cuboid::defromMesh()
{

}
