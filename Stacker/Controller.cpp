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

		// Assign an ID
		primitives.back()->id = primitives.size();
	}
}

void Controller::fitOBBs()
{
	foreach (QSurfaceMesh* segment, m_mesh->getSegments())
	{
		Cuboid* cub = new Cuboid(segment);
		primitives.push_back(cub);

		// Assign an ID
		primitives.back()->id = primitives.size();
	}
}

void Controller::draw()
{
	for (uint i = 0; i < primitives.size(); i++)
	{
		primitives[i]->draw();

		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Cuboid * c1 = (Cuboid *) primitives[i];
			Cuboid * c2 = (Cuboid *) primitives[j];

			Vec3d p,q;

			c1->currBox.ClosestSegment(c2->currBox,p,q);
			SimpleDraw::IdentifyLine(p,q,1,1,0);
		}
	}
}

void Controller::drawNames()
{
	for (uint i = 0; i < primitives.size(); i++)
	{
		primitives[i]->drawNames();
	}
}

void Controller::select(int id)
{
	// Deselect all if given '-1'
	if(id < 0){
		for (uint i = 0; i < primitives.size(); i++)
			primitives[i]->isSelected = false;
		return;
	}

	// Select primitives by setting flag
	for (uint i = 0; i < primitives.size(); i++)
	{
		if(primitives[i]->id == id)
			primitives[i]->isSelected = true;
	}
}

void Controller::convertToGC( int primitiveId, bool isUsingSkeleton )
{
	int p_index = 0;

	// Locate index of primitive (ID could be non-integer ?)
	for (uint i = 0; i < primitives.size(); i++){
		if(primitiveId == primitives[i]->id){
			p_index = i;
			break;
		}
	}

	// Convert to generalized cylinder
	if(isUsingSkeleton)
	{
		Primitive * oldPrimitive = primitives[p_index];

		primitives[p_index] = new GCylinder(primitives[p_index]->getMesh());

		delete oldPrimitive;
	}
	else
	{

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

void Controller::deformShape( std::vector<cuboidDeformParam>& params, bool isPermanent )
{
	
	// leg
	Cuboid* cp0 = ( Cuboid* )primitives[0];
	cp0->deform( params[0], isPermanent );

	// top
	Cuboid* cp1 = ( Cuboid* )primitives[1];
	cp1->deform( params[1], isPermanent );
	
	/*Cuboid* cp2 = ( Cuboid* )primitives[2];
	cp2->deform( params[2], isPermanent );
	Cuboid* cp3 = ( Cuboid* )primitives[3];
	cp3->deform( params[3], isPermanent );
	Cuboid* cp4 = ( Cuboid* )primitives[4];
	cp4->deform( params[4], isPermanent );*/

	m_mesh->computeBoundingBox();

}

Primitive * Controller::getPrimitive( int id )
{
	return primitives[id];
}

uint Controller::numPrimitives()
{
	return primitives.size();
}

void Controller::recoverShape()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		(( Cuboid* )primitives[i])->recoverMesh();
	}

	m_mesh->computeBoundingBox();
}

int Controller::numHotPrimitives()
{
	int num = 0;
	for (int i=0;i<numPrimitives();i++)
	{
		if (primitives[i]->isHot)
			num++;
	}
	return num;
}

Controller::Stat Controller::getStat()
{
	Stat stat;

	// Compute volume of each controller + total volume of Bounding Box
	Point bbmin = Point( DBL_MAX, DBL_MAX, DBL_MAX);
	Point bbmax = Point( DBL_MIN, DBL_MIN, DBL_MIN);

	int pi = 0;
	stat.volumePrim = std::vector<double>(primitives.size());

	foreach (Primitive * prim, primitives)
	{
		foreach (Point p, prim->points())
		{
			bbmin.minimize(p);
			bbmax.maximize(p);
		}

		// Compute volume of each controller
		stat.volumePrim[pi++] = prim->volume();
	}

	// Compute total volume of controllers BB
	Point center = (bbmin + bbmax) / 2.0;

	stat.volumeBB = abs((bbmax.x() - center.x()) * 
						(bbmax.y() - center.y()) *
						(bbmax.z() - center.z())) * 8;

	// Compute proximity measure
	for(uint i = 0; i < primitives.size(); i++)
	{
		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Cuboid * pi = (Cuboid *)primitives[i];
			Cuboid * pj = (Cuboid *)primitives[j];
			
			Vec3d p, q;

			pi->currBox.ClosestSegment(pj->currBox, p, q);

			stat.proximity[std::make_pair(i, j)] = (p-q).norm();
		}
	}

	// Compute coplanarity measure
	for(uint i = 0; i < primitives.size(); i++)
	{
		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * pi = primitives[i];
			Primitive * pj = primitives[j];

			stat.coplanarity[std::make_pair(i, j)] = 0;
		}
	}

	return stat;
}

std::vector< double > Controller::difference( Controller::Stat s2 )
{
	Stat s1 = this->getStat();

	std::vector< double > E(4);	 // difference measures

	// SHAPE BB:
	E[0] = abs(s1.volumeBB - s2.volumeBB) / s2.volumeBB;

	// PART BB:
	double sumV = 0;
	for(int i = 0; i < primitives.size(); i++)
	{
		E[1] += abs(s1.volumePrim[i] - s2.volumePrim[i]);
		sumV += s2.volumePrim[i];
	}
	E[1] /= sumV;

	// PROXMIITY:
	double sumP = 0;
	for(uint i = 0; i < primitives.size(); i++)
	{
		for (uint j = i + 1; j < primitives.size(); j++)
		{
			E[2] += abs(s1.proximity[std::make_pair(i,j)] - s2.proximity[std::make_pair(i,j)]);
			sumP += s2.proximity[std::make_pair(i,j)];
		}
	}

	if(sumP == 0)
		sumP = 1;

	E[2] /= sumP;



//	E[3] = s1.coplanarity[std::make_pair(0, 1)] - s2.proximity[std::make_pair(0, 1)];

	return E;
}
