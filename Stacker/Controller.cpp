#include "Controller.h"
#include "QSegMesh.h"
#include "Cuboid.h"
#include "GCylinder.h"
#include "Primitive.h"
#include "Offset.h"
#include <QQueue>
#include "JointGroup.h"

Controller::Controller( QSegMesh* mesh )
{
	m_mesh = mesh;

	// Fit
	// fitPrimitives()
	fitOBBs();

	// Assign numerical IDs
	assignIds();

	// Save original stats
	originalStat = getStat();
}

Controller::~Controller()
{
	foreach(Primitive * prim, primitives)
		delete prim;
}

void Controller::assignIds()
{
	foreach(Primitive * prim, primitives)
	{
		primitiveIdNum[primitiveIdNum.size()] = prim->id;
	}
}

void Controller::fitPrimitives()
{
	for (int i=0;i<m_mesh->nbSegments();i++)
	{
		// ToDo: automatic fitting
	}
}

void Controller::fitOBBs()
{
	foreach (QSurfaceMesh* segment, m_mesh->getSegments())
	{
		QString segId = segment->objectName();

		Cuboid* cub = new Cuboid(segment, segId);

		primitives[segId] = cub;

		currStat.params[segId] = (PrimitiveParam*) new CuboidParam(segId);
	}
}

void Controller::draw()
{
	foreach(Primitive * prim, primitives)
	{
		prim->draw();
		prim->drawDebug();

		// Draw proximity debug 
		/*for (uint j = i + 1; j < primitives.size(); j++){
			Cuboid * c1 = (Cuboid *) primitives[i];
			Cuboid * c2 = (Cuboid *) primitives[j];
			Vec3d p,q;
			c1->currBox.ClosestSegment(c2->currBox,p,q);
			SimpleDraw::IdentifyLine(p,q,1,1,0);
		}*/
	}

	// DEBUG:
	foreach(Point p, debugPoints)
		SimpleDraw::IdentifyPoint(p);

	foreach(std::vector<Point> line, debugLines) 
		SimpleDraw::IdentifyConnectedPoints(line, 1.0,0,0);
}

void Controller::drawNames(bool isDrawParts)
{
	foreach(Primitive * prim, primitives)
	{
		prim->drawNames(getPrimitiveIdNum(prim->id), isDrawParts);
	}
}

void Controller::select(int id)
{
	if(id != -1)
		select(primitiveIdNum[id]);
	else
		select("deselectAll");
}

void Controller::select(QString id)
{
	// Deselect all if given '-1'
	if(id == "deselectAll")
	{
		foreach(Primitive * prim, primitives)
			prim->isSelected = false;
		return;
	}

	// Toggle selection of primitive
	primitives[id]->isSelected = !primitives[id]->isSelected;
}

bool Controller::selectPrimitivePart( int id )
{
	bool beenSelected = false;

	foreach(Primitive * prim, primitives)
	{
		if(prim->isSelected){
			prim->selectedPartId = id;
			beenSelected = true;
		}
	}

	return beenSelected;
}

Vec3d Controller::getPrimPartPos()
{
	foreach(Primitive * prim, primitives)
	{
		if(prim->isSelected)
			return prim->selectedPartPos();
	}

	return Vec3d();
}

void Controller::convertToGC( QString primitiveId, bool isUsingSkeleton )
{
	// Convert to generalized cylinder
	if(isUsingSkeleton)
	{
		Primitive * oldPrimitive = primitives[primitiveId];

		primitives[primitiveId] = new GCylinder(primitives[primitiveId]->getMesh(), primitiveId);

		delete oldPrimitive;
	}
	else
	{

	}
}


void Controller::deformShape( PrimitiveParamMap& primParams, bool isPermanent )
{
	foreach(PrimitiveParam* param, primParams.params)
	{
		QString p_id = param->id;
		Primitive * pri = primitives[p_id];
		pri->deform(param);

		// Update param for each primitive
		PrimitiveParam* tmp = currStat.params[p_id];
		currStat.params[p_id] = param->clone();
		delete tmp;
	}

	m_mesh->computeBoundingBox();
}

Primitive * Controller::getPrimitive( uint id )
{
	return primitives[primitiveIdNum[id]];
}

Primitive * Controller::getPrimitive( QString id )
{
	return primitives[id];
}

uint Controller::numPrimitives()
{
	return primitives.size();
}

void Controller::recoverShape()
{
	foreach(Primitive * prim, primitives)
		(( Cuboid* )prim)->recoverMesh();
		
	// recovery the stat
	currStat.params = originalStat.params;

	m_mesh->computeBoundingBox();
}

int Controller::numHotPrimitives()
{
	int num = 0;

	foreach(Primitive * prim, primitives)
		if (prim->isHot)
			num++;

	return num;
}

Controller::Stat& Controller::getStat()
{
	// Compute volume of each controller + total volume of Bounding Box
	Point bbmin = Point( DBL_MAX, DBL_MAX, DBL_MAX);
	Point bbmax = Point( DBL_MIN, DBL_MIN, DBL_MIN);

	int pi = 0;
	currStat.volumePrim = std::vector<double>(primitives.size());

	foreach (Primitive * prim, primitives)
	{
		foreach (Point p, prim->points())
		{
			bbmin.minimize(p);
			bbmax.maximize(p);
		}

		// Compute volume of each controller
		currStat.volumePrim[pi++] = prim->volume();
	}

	// Compute total volume of controllers BB
	Point center = (bbmin + bbmax) / 2.0;

	currStat.volumeBB = abs((bbmax.x() - center.x()) * 
						(bbmax.y() - center.y()) *
						(bbmax.z() - center.z())) * 8;

	QVector<QString> keys = primitives.keys().toVector();

	// Compute proximity measure
	for(uint i = 0; i < primitives.size(); i++)
	{
		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Cuboid * pi = (Cuboid *)primitives[keys[i]];
			Cuboid * pj = (Cuboid *)primitives[keys[j]];
			
			Vec3d p, q;

			pi->currBox.ClosestSegment(pj->currBox, p, q);

			currStat.proximity[std::make_pair(i, j)] = (p-q).norm();
		}
	}

	// Compute coplanarity measure
	for(uint i = 0; i < primitives.size(); i++)
	{
		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * pi = primitives[keys[i]];
			Primitive * pj = primitives[keys[j]];

			currStat.coplanarity[std::make_pair(i, j)] = 0;
		}
	}

	return currStat;
}

Controller::Stat& Controller::getOriginalStat()
{
	return this->originalStat;
}

Primitive * Controller::getSelectedPrimitive()
{
	foreach(Primitive * prim, primitives)
	{
		if (prim->isSelected)
			return prim;
	}

	return NULL;
}

void Controller::findJoints(double threshold)
{
	std::vector<Voxeler> voxels;

	foreach(Primitive * prim, primitives)
	{
		QSurfaceMesh mesh(prim->getGeometry());
		voxels.push_back( Voxeler(&mesh, threshold) );
	}

	QVector<QString> keys = primitives.keys().toVector();

	for(uint i = 0; i < primitives.size(); i++)
	{
		Primitive * a = primitives[keys[i]];

		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * b = primitives[keys[i]];

			std::vector<Voxel> intersection = voxels[i].Intersects(&voxels[j]);

			if(intersection.size()){
				int N = intersection.size();
				Voxel center;
				foreach(Voxel v, intersection){
					center.x += v.x;
					center.y += v.y; 
					center.z += v.z;
				}
				double scale = threshold / N;
				Point centerPoint(center.x * scale, center.y * scale, center.z * scale);

				JointGroup *newGroup = new JointGroup(this, JOINT);
				QVector<QString> segments;
				segments.push_back(keys[i]);
				segments.push_back(keys[j]);
				newGroup->process(segments, centerPoint);

				Primitive::Joint joint;
				joint.pos = centerPoint;
				joint.frozen = false;
				a->joints.push_back(joint);
				b->joints.push_back(joint);

				this->groups[newGroup->id] = newGroup;
			}
		}
	}
}

QVector<QString> Controller::stringIds( QVector<int> numericalIds )
{
	QVector<QString> result;

	foreach(int numId, numericalIds)
		result.push_back(primitiveIdNum[numId]);

	return result;
}

int Controller::getPrimitiveIdNum( QString stringId )
{
	QMapIterator<int, QString> i(primitiveIdNum);
	while (i.hasNext()) {
		i.next();
		if(i.value() == stringId) return i.key();
	}
	return -1;
}

std::vector<Primitive*> Controller::getPrimitives()
{
	std::vector<Primitive*> result;

	foreach(Primitive * prim, primitives)
		result.push_back(prim);

	return result;
}

ShapeState Controller::getShapeState()
{
	ShapeState state;

	foreach(Primitive * prim, primitives)
		state[prim->id] = prim->getState();

	return state;
}

void Controller::setShapeState( ShapeState &shapeState )
{
	foreach(Primitive * prim, primitives)
	{
		prim->setState(shapeState[prim->id]);
		prim->deformMesh();
	}	
}

std::set< QString > Controller::getRidOfRedundancy( std::set< QString > Ids )
{
	std::set<QString> result;

	// One representative per group
	foreach(Group * group, groups)
	{
		// Check if *group* has nodes in *Ids*
		bool has = false;
		QString rep;
		foreach(QString node, group->nodes)
		{
			if (std::find(Ids.begin(), Ids.end(), node) != Ids.end())
			{
				has = true;
				rep = node;
				break;
			}
		}

		// The representative
		result.insert(rep);

		// Remove redundant nodes in Ids
		std::set<QString>::iterator itr;
		foreach(QString node, group->nodes)
		{
			itr = Ids.find(node);
			if (itr != Ids.end())
				Ids.erase(itr);
		}
	}

	result.insert(Ids.begin(), Ids.end());

	return result;
}

// Propagations happen only to *available* segments
// From *frozen* segments to *unfrozen* ones
void Controller::propagate()
{
	// Stack frozen primitives
	QQueue<Primitive *> frozen;
	foreach(Primitive * p, primitives)
		if(p->isFrozen) frozen.enqueue(p);
	
	while(!frozen.isEmpty())
	{
		Primitive * prim = frozen.dequeue();

		QVector< Group * > grps = groupsOf(prim->id);

		foreach(Group * g, grps)
		{
			QVector<Primitive *> regrouped = g->regroup();

			foreach(Primitive * next, regrouped)
			{
				next->isFrozen = true;
				frozen.enqueue(next);
			}
		}
	}
}

QVector< Group * > Controller::groupsOf( QString id )
{
	QVector< Group * > result;

	foreach(Group * group, groups)
		if(group->has(id))	result.push_back(group);

	return result;
}

void Controller::setSegmentsVisible( bool isVisible /*= true*/ )
{
	foreach(Primitive* prim, primitives)
		prim->m_mesh->isVisible = isVisible;
}

void Controller::setPrimitivesFrozen( bool isFrozen /*= false*/ )
{
	foreach(Primitive* prim, primitives)
	{
		prim->isFrozen = isFrozen;
		
		// Clean up the fixed points
		prim->fixedPoints.clear();
	}
}

void Controller::setPrimitivesAvailable( bool isAvailable /*= true*/ )
{
	foreach(Primitive* prim, primitives)
		prim->isAvailable = isAvailable;
}

void Controller::regroupPair( QString id1, QString id2 )
{
	Group *pairGrp = NULL;
	QVector<Group*> groups = groupsOf(id1);
	for (int i=0;i<groups.size();i++){
		if (groups[i]->has(id2))
		{
			pairGrp = groups[i];
			break;
		}
	}

	if (pairGrp)
		pairGrp->regroup();
}

