#include "Controller.h"
#include "QSegMesh.h"
#include "Cuboid.h"
#include "GCylinder.h"
#include "Primitive.h"
#include "Offset.h"
#include <QQueue>

#include "JointGroup.h"
double JOINT_THRESHOLD = 0.035;

#include "EditSuggestion.h"

Controller::Controller( QSegMesh* mesh, bool useAABB /*= true*/ )
{
	m_mesh = mesh;

	// Fit
	fitOBBs(useAABB);

	// Assign numerical IDs
	assignIds();

	// Save original stats
	originalStat = getStat();

	primTypeNames.push_back("CUBOID");
	primTypeNames.push_back("GC");
	primTypeNames.push_back("WIRE");
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

void Controller::fitOBBs( bool useAABB /*= true*/ )
{
	foreach (QSurfaceMesh* segment, m_mesh->getSegments())
	{
		QString segId = segment->objectName();

		Cuboid* cub = new Cuboid(segment, segId, useAABB, 0);

		primitives[segId] = cub;

		currStat.params[segId] = (PrimitiveParam*) new CuboidParam(segId);
	}
}

void Controller::draw()
{
	foreach(Primitive * prim, primitives)
	{
		prim->drawDebug();
		prim->draw();

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

void Controller::convertToGC( QString primitiveId, bool isUsingSkeleton, int cuboidAxis )
{
	Primitive * oldPrimitive = primitives[primitiveId];

	// Convert to generalized cylinder
	primitives[primitiveId] = new GCylinder(primitives[primitiveId]->getMesh(), primitiveId, isUsingSkeleton);
	
	if(!isUsingSkeleton)
	{
		GCylinder * gc = (GCylinder *)primitives[primitiveId];

		Cuboid * cuboid = (Cuboid*)oldPrimitive;

		double extent = cuboid->originalBox.Extent[cuboidAxis];
		Vec3d axis = cuboid->originalBox.Axis[cuboidAxis];
		Vec3d center = cuboid->centerPoint();

		Line line(center + (axis * extent), center + (-axis * extent));

		std::vector<Point> spinePoints = line.uniformSample(skeletonJoints);

		gc->createGC(spinePoints);
		gc->buildCage();
	}

	delete oldPrimitive;
}

void Controller::convertToCuboid( QString primitiveId, bool useAABB, int fit_method)
{
	Primitive * oldPrimitive = primitives[primitiveId];

	primitives[primitiveId] = new Cuboid(primitives[primitiveId]->getMesh(), primitiveId, useAABB, fit_method);

	// bug?
	//delete oldPrimitive;
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


void Controller::findPairwiseJoints( QString a, QString b, int nbJoints )
{
	Primitive * primA = getPrimitive(a);
	QSurfaceMesh meshA(primA->getGeometry());
	Voxeler voxelerA(&meshA, JOINT_THRESHOLD);

	Primitive * primB = getPrimitive(b);
	QSurfaceMesh meshB(primB->getGeometry());
	Voxeler voxelerB(&meshB, JOINT_THRESHOLD);

	std::vector<Voxel> intersectionVoxels = voxelerA.Intersects(&voxelerB);
	bool result;
	if(intersectionVoxels.empty())
		result = false;
	else
	{
		QVector<Vec3d> intersectionPoints;
		foreach(Voxel v, intersectionVoxels)
			intersectionPoints.push_back(Vec3d(v.x, v.y, v.z));

		QVector<QString> segments;
		segments.push_back(a);
		segments.push_back(b);
		QVector<Vec3d> centers = centerOfClusters(intersectionPoints, nbJoints);
		foreach(Vec3d center, centers)
		{
			Vec3d joint = center * JOINT_THRESHOLD;
			JointGroup *newGroup = new JointGroup(this, JOINT);
			newGroup->process(segments, joint);
			this->groups[newGroup->id] = newGroup;
		}

	}
}


void Controller::findJoints()
{
	std::vector<Voxeler> voxels;

	foreach(Primitive * prim, primitives)
	{
		QSurfaceMesh mesh(prim->getGeometry());
		voxels.push_back( Voxeler(&mesh, JOINT_THRESHOLD) );
	}

	QVector<QString> keys = primitives.keys().toVector();

	for(uint i = 0; i < primitives.size(); i++)
	{
		Primitive * a = primitives[keys[i]];

		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * b = primitives[keys[i]];

			std::vector<Voxel> intersection = voxels[i].Intersects(&voxels[j]);

			if(!intersection.empty()){
				int N = intersection.size();
				Voxel center;
				foreach(Voxel v, intersection){
					center.x += v.x;
					center.y += v.y; 
					center.z += v.z;
				}
				double scale = JOINT_THRESHOLD / N;
				Point centerPoint(center.x * scale, center.y * scale, center.z * scale);

				JointGroup *newGroup = new JointGroup(this, JOINT);
				QVector<QString> segments;
				segments.push_back(keys[i]);
				segments.push_back(keys[j]);
				newGroup->process(segments, centerPoint);

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
		state.primStates[prim->id] = prim->getState();

	return state;
}

void Controller::setShapeState( ShapeState &shapeState )
{
	foreach(Primitive * prim, primitives)
	{
		prim->setState(shapeState.primStates[prim->id]);
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

// From *frozen* segments to *unfrozen* ones
void Controller::weakPropagate(QVector<QString> seeds)
{
	QMap< QString, bool > debugFrozenFlags1 = getFrozenFlags();

	QQueue<QString> frozen;
	foreach(QString id, seeds)
		frozen.enqueue(id);

	while(!frozen.isEmpty())
	{
		QString seed = frozen.dequeue();
		QVector< Group * > grps = groupsOf(seed);

		foreach(Group * g, grps)
		{
			QVector<QString> regrouped = g->regroup();

			// Only frozen \next's are considered
			foreach(QString next, regrouped)
				frozen.enqueue(next);

			//QMap< QString, bool > debugFrozenFlags2 = getFrozenFlags();
		}
	}
}

void Controller::weakPropagate()
{
	QVector<QString> seeds;
	foreach(Primitive * p, primitives)
		if(p->isFrozen) seeds.push_back(p->id);

	weakPropagate(seeds);
}

// In case semi-frozen primitives exist, force one of them as frozen and apply weakPropagation
QVector<ShapeState> Controller::strongPropagate()
{
	QVector< ShapeState > results;

	// Initialize the queue of candidates
	QQueue< ShapeState > candidates;
	candidates.enqueue(getShapeState());

	while (!candidates.isEmpty())
	{
		// Pick up one candidate
		ShapeState currCandidate = candidates.dequeue();
		setShapeState(currCandidate);

		// Apply weak propagation
//		QMap< QString, bool > debugFrozenFlags1 = getFrozenFlags();
		if (currCandidate.seeds.isEmpty())
			weakPropagate();
		else
			weakPropagate(currCandidate.seeds);
//		QMap< QString, bool > debugFrozenFlags2 = getFrozenFlags();

		// Check whether the propagation is done
		QQueue<QString> semi_frozen;
		foreach(Primitive * p, primitives)
			if(!p->isFrozen && !p->fixedPoints.isEmpty()) 
				semi_frozen.enqueue(p->id);	

//		int debug = 0;

		if (semi_frozen.isEmpty())
		{
			// Weak propagation is done
			results.push_back(getShapeState());
		}
		else
		{
			// Weak propagation is stuck because of the semi-frozen primitives
			// Force one of them to be frozen and continue the weak propagation
			for (int i=0;i<semi_frozen.size();i++)
			{
				Primitive * prim = getPrimitive(semi_frozen[i]);
				prim->isFrozen = true;
				QMap< QString, bool > debugFrozenFlags3 = getFrozenFlags();
				ShapeState state = getShapeState();
				state.seeds.push_back(prim->id);
				candidates.enqueue(state);

				// Restore 
				prim->isFrozen = false;
				QMap< QString, bool > debugFrozenFlags4 = getFrozenFlags();

			}
		}
	}

	// set the shape as the first result
	setShapeState(results[0]);
 
	return results;
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

void Controller::test()
{
	Primitive * prim = primitives.values().first();
	
	prim->isSelected = true;

	std::vector< std::vector<Point> > curves = prim->getCurves();
	int N = curves.size();
	int NP = curves.front().size();

	Point p(0,0,0), q(0,0,0), r(0,0,0), T(0,0,0);

	/*  Test : deform respect joint*/
	std::vector<Point> A = curves.front(), B = curves.back();
	foreach(Point pnt, A) p += pnt; p /= N;
	foreach(Point pnt, B) q += pnt; q /= N;
	T = Vec3d(0.5,0.5,0);
	prim->deformRespectToJoint(p, q, T);


	/* Test: multiple fixed points  */
	/*foreach(Point pnt, curves[N * 0.25]) p += pnt; p /= NP;
	foreach(Point pnt, curves[N - (double(N) * 0.25)]) q += pnt; q /= NP;
	prim->fixedPoints.clear();
	prim->addFixedPoint(p);
	prim->addFixedPoint(q);
	T = Vec3d(0,0,0.5);
	foreach(Point pnt, curves[N / 2]) r += pnt; r /= NP;
	prim->movePoint(r, T);*/
}

QMap< QString, bool > Controller::getFrozenFlags()
{
	QMap< QString, bool > result;

	foreach(Primitive * p, primitives)
		result[p->id] = p->isFrozen;

	return result;
}

double Controller::similarity( ShapeState state1, ShapeState state2 )
{
	double result = 0;

	foreach(Primitive* prim, primitives)
	{
		QString id = prim->id;
		result += prim->similarity(state1.primStates[id], state2.primStates[id]);
	}

	return result;
}

void Controller::save( std::ofstream &outF )
{
	foreach(Primitive * prim, primitives)
	{
		QString id = prim->id;
		
		outF << qPrintable(primTypeNames[prim->primType]) << "\t" << qPrintable(id) << "\t";

		prim->save(outF);

		outF << std::endl;
	}
}

void Controller::load( std::ifstream &inF )
{
	primitives.clear();

	while(!inF.eof())
	{
		std::string strType, strId;
		
		// Get type and name
		inF >> strType >> strId;

		int primType = primTypeNames.indexOf(QString(strType.c_str()));
		QString primId(strId.c_str());

		if(primType < 0)
			continue;

		switch(primType)
		{
			case CUBOID: primitives[primId] = new Cuboid(m_mesh->getSegment(primId), primId); break;
			case GC: primitives[primId] = new GCylinder(m_mesh->getSegment(primId), primId, false); break;
		}

		primitives[primId]->load(inF);
	}
}

void Controller::removePrimitive( Primitive * prim )
{
	primitives.remove(prim->id);
}

void Controller::clearPrimitives()
{
	primitives.clear();
}

QVector< Vec3d > Controller::centerOfClusters( QVector< Vec3d> &data, int nbCluster )
{
	QVector< Vec3d > centers;

	switch(nbCluster)
	{
	case 1:
		{
			Vec3d center(0, 0, 0);
			foreach(Vec3d p, data)
				center += p;

			center /= data.size();
			centers.push_back(center);
		}
		break;
	case 2:
		{
			// Find the furthest two points
			Vec3d p1 = data[0];
			Vec3d p2;
			double maxDis = 0;
											foreach( Vec3d p, data)
		{
			double dis = (p-p1).norm();
			if (dis > maxDis)
			{
				p2 = p;
				maxDis = dis;
			}
		}

			// Assign all the data to two clusters
			QVector< QVector< Vec3d > >clusters(2);
			foreach( Vec3d p, data)
			{
				if ((p-p1).norm() < (p-p2).norm())
					clusters[0].push_back(p);
				else
					clusters[1].push_back(p);
			}

			// Computer the centers
			for (int i=0;i<2;i++)
			{
				Vec3d center(0, 0, 0);
				foreach(Vec3d p, clusters[i])
					center += p;

				center /= clusters[i].size();
				centers.push_back(center);
			}
		
		}
		break;
	}

	return centers;
}

double Controller::volume()
{
	double result = 0;

	foreach (Primitive * prim, primitives)
		result += prim->volume();

	return result;

}
