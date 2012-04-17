#include "Controller.h"
#include "GraphicsLibrary/Mesh/QSegMesh.h"
#include "Cuboid.h"
#include "GCylinder.h"
#include "Primitive.h"
#include "Offset.h"
#include <QQueue>

#include "EditSuggestion.h"

Controller::Controller( QSegMesh* mesh, bool useAABB /*= true*/ )
{
	m_mesh = mesh;

	// BB
	m_mesh->computeBoundingBox();
	original_bbmin = m_mesh->bbmin;
	original_bbmax = m_mesh->bbmax;

	// Fit
	fitOBBs(useAABB);

	// Assign numerical IDs
	assignIds();

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
	//foreach(Point p, debugPoints)
	//	SimpleDraw::IdentifyPoint(p);

	//foreach(std::vector<Point> line, debugLines) 
	//	SimpleDraw::IdentifyConnectedPoints(line, 1.0,0,0);
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

	if(!primitives.contains(id))
		return;

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
}

void Controller::convertToGC( QString primitiveId, bool isUsingSkeleton, int cuboidAxis )
{
	Primitive * oldPrimitive = primitives[primitiveId];

	if(oldPrimitive->primType != CUBOID)
	{
		convertToCuboid(primitiveId);
		oldPrimitive = primitives[primitiveId];
	}

	// Convert to generalized cylinder
	primitives[primitiveId] = new GCylinder(primitives[primitiveId]->getMesh(), primitiveId, isUsingSkeleton);
	
	if(!isUsingSkeleton)
	{
		GCylinder * gc = (GCylinder *)primitives[primitiveId];

		Cuboid * cuboid = (Cuboid*)oldPrimitive;

		double extent = cuboid->currBox.Extent[cuboidAxis];
		Vec3d axis = cuboid->currBox.Axis[cuboidAxis];
		Vec3d center = cuboid->centerPoint();

		Line line(center + (axis * extent), center + (-axis * extent));

		std::vector<Point> spinePoints = line.uniformSample(skeletonJoints);

		gc->createGC(spinePoints);
		gc->buildCage();
		gc->computeMeshCoordiantes();
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


int Controller::numHotPrimitives()
{
	int num = 0;

	foreach(Primitive * prim, primitives)
		if (prim->isHot)
			num++;

	return num;
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


QVector<Primitive*> Controller::getPrimitives( QVector<QString> ids )
{
	QVector<Primitive*> results;
	foreach(QString id, ids)
		results.push_back(getPrimitive(id));

	return results;
}


QVector<Primitive*> Controller::getPrimitives( QVector<int> ids )
{
	QVector<Primitive*> results;
	foreach(int id, ids)
		results.push_back(getPrimitive(id));

	return results;
}

QVector<Primitive*> Controller::getPrimitives()
{
	QVector<Primitive*> results;
	foreach(Primitive* p, primitives)
		results.push_back(p);

	return results;
}

ShapeState Controller::getShapeState()
{
	ShapeState state;

	foreach(Primitive * prim, primitives)
		state.primStates[prim->id] = prim->getState();

	return state;
}

void Controller::setShapeState(const ShapeState &shapeState )
{
	foreach(Primitive * prim, primitives)
	{
		prim->setState(shapeState.primStates[prim->id]);
		prim->deformMesh();
	}
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
			ShapeState newState = getShapeState();

			// Check if unique
			bool isUnique = true;
			foreach(ShapeState res, results){
				if(similarity(res, newState) < 0)
				{
					isUnique = false;
					break;
				}
			}

			// if unique, add to \results
			results.push_back(newState);
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
	//setShapeState(results[0]);
 
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

	double scaleFactor = 1/m_mesh->scaleFactor;

	std::cout << "Scaling = " << scaleFactor << std::endl;

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
            case GCYLINDER: primitives[primId] = new GCylinder(m_mesh->getSegment(primId), primId, false); break;
		}

		primitives[primId]->load(inF, scaleFactor);
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

double Controller::volume()
{
	double result = 0;

	foreach (Primitive * prim, primitives)
		result += prim->volume();

	return result;

}

double Controller::originalVolume()
{
	double result = 0;

	foreach (Primitive * prim, primitives)
		result += prim->originalVolume;

	return result;
}

double Controller::getDistortion()
{
	// Distortion terms
	std::vector< double > D; 	

	// Change of the total volume of primitives
	double orgV = originalVolume();
	double D1 = abs(volume() - orgV) /orgV;
	D.push_back(D1);

	// Change of BB along 3 main axes
	Vec3d bbmin, bbmax;
	m_mesh->computeBoundingBox();
	bbmin = m_mesh->bbmin;
	bbmax = m_mesh->bbmax;
	double D2 = 0;
	Vec3d orgSize = original_bbmax - original_bbmin;
	Vec3d currSize = bbmax - bbmin;
	Vec3d delta = currSize - orgSize;
	for (int i=0;i<3;i++)
		D2 += abs(delta[i]);

	// Total Energy
	return Sum(D);
}
