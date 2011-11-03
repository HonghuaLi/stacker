#include "Cuboid.h"


Cuboid::Cuboid(QSurfaceMesh* mesh)
{
	m_mobb = new MinOBB3(mesh);
}


Cuboid::~Cuboid(void)
{
	delete m_mobb;
}

void Cuboid::draw()
{
	m_mobb->draw();
}
