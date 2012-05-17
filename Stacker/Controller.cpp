#include "Controller.h"

#include <QQueue>
#include <QTime>

#include "Offset.h"
#include "Cuboid.h"
#include "Primitive.h"
#include "GCylinder.h"
#include "EditPath.h"
#include "GraphicsLibrary/Mesh/QSegMesh.h"

#include "Group.h"
#include "SymmetryGroup.h"
#include "ConcentricGroup.h"
#include "CoplanarGroup.h"
#include "PointJointGroup.h"
#include "LineJointGroup.h"
#include "JointDetector.h"

Controller::Controller( QSegMesh* mesh, bool useAABB /*= true*/, QString loadFromFile /* = ""*/ )
{
	m_mesh = mesh;

	// BB
	m_mesh->computeBoundingBox();
	original_bbmin = m_mesh->bbmin;
	original_bbmax = m_mesh->bbmax;

	primTypeNames.push_back("CUBOID");
	primTypeNames.push_back("GC");
	primTypeNames.push_back("WIRE");

	// GC along axis
	GC_SKELETON_JOINTS_NUM = 16;

	// Group types
	groupTypes.push_back("SYMMETRY");
	groupTypes.push_back("POINTJOINT");
	groupTypes.push_back("LINEJOINT");
	groupTypes.push_back("CONCENTRIC");
	groupTypes.push_back("COPLANNAR");
	groupTypes.push_back("SELF_SYMMETRY");
	groupTypes.push_back("SELF_ROT_SYMMETRY");

	if(loadFromFile.isEmpty())
	{
		// Fit
		fitOBBs(useAABB);
	}
	else
	{
		std::ifstream inF(qPrintable(loadFromFile), std::ios::in);
		load(inF);
		inF.close();
	}

	// Assign numerical IDs
	assignIds();
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

void Controller::draw(bool isDrawGroups, bool isDrawDebug)
{
	foreach(Primitive * prim, primitives)
	{
		prim->draw();
		if(isDrawDebug)	prim->drawDebug();
	}

	if(isDrawGroups)
	{
		foreach(Group * g, groups)
			g->draw();
	}

	//DEBUG:
	foreach(Point p, debugPoints) SimpleDraw::IdentifyPoint(p, 1.0, 0.8, 0);
	foreach(std::vector<Point> line, debugLines) SimpleDraw::IdentifyConnectedPoints(line, 1.0,0.8,0);
}

void Controller::drawNames(bool isDrawParts)
{
	foreach(Primitive * prim, primitives)
	{
		if(!isDrawParts)
			prim->drawNames(getPrimitiveIdNum(prim->id), isDrawParts);
		else if(prim->isSelected)
			prim->drawNames(getPrimitiveIdNum(prim->id), isDrawParts);
	}
}

void Controller::selectPrimitive(int id)
{
	if(id != -1)
		selectPrimitive(primitiveIdNum[id]);
	else
		selectPrimitive("deselectAll");
}

void Controller::selectPrimitive(QString id)
{
	// Deselect all if given '-1'
	if(id == "deselectAll")
	{
		foreach(Primitive * prim, primitives)
		{
			prim->isSelected = false;
			prim->selectedCurveId = -1;
		}

		return;
	}

	if(!primitives.contains(id))
		return;

	// Toggle selection of primitive
	primitives[id]->isSelected = !primitives[id]->isSelected;
}

bool Controller::selectPrimitiveCurve( int id )
{
	bool beenSelected = false;

	foreach(Primitive * prim, primitives)
	{
		if(prim->isSelected){
			prim->selectedCurveId = id;
			beenSelected = true;
		}
	}

	return beenSelected;
}

Point Controller::getSelectedCurveCenter()
{
	foreach(Primitive * prim, primitives)
	{
		if(prim->isSelected)
		{
			if(prim->selectedCurveId >= 0)
				return prim->getSelectedCurveCenter();
		}
	}

	return Point(0.0);
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

		double s = 1.05;
		Line line(center + (axis * extent * s), center + (-axis * extent * s));

		std::vector<Point> spinePoints = line.uniformSample(Max(5,GC_SKELETON_JOINTS_NUM));

		gc->buildGC(spinePoints);
		gc->buildUp();
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

int Controller::numPrimitives()
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

	state.stacking_shift = m_mesh->vec["stacking_shift"];
	state.stackability = m_mesh->val["stackability"];

	return state;
}

void Controller::setShapeState(const ShapeState &shapeState )
{
	foreach(Primitive * prim, primitives)
	{
		prim->setState(shapeState.primStates[prim->id]);
		prim->deformMesh();
	}

	m_mesh->vec["stacking_shift"] = shapeState.stacking_shift;
	m_mesh->val["stackability"] = shapeState.stackability;

	m_mesh->computeBoundingBox();
}

QVector< Group * > Controller::groupsOf( QString id )
{
	QVector< Group * > result;

	foreach(Group * group, groups)
		if(group->has(id))	result.push_back(group);

	return result;
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
	clearPrimitives();

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

		primitives[primId]->load(inF, m_mesh->translation, m_mesh->scaleFactor);
	}
}

void Controller::removePrimitive( Primitive * prim )
{
	primitives.remove(prim->id);
}

void Controller::clearPrimitives()
{
	foreach(QString id, primitives.keys())
		delete primitives[id];

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

double Controller::meshRadius()
{
	return m_mesh->radius;
}

void Controller::loadGroups( std::ifstream &inF )
{
	if (!inF) return;

	// Clear
	groups.clear();
	foreach (Primitive * p, getPrimitives()) p->symmPlanes.clear();

	// Read
	while (inF)
	{
		std::string str;
		inF >> str;
		int type = groupTypes.indexOf(str.c_str());
		if (type == -1) break;

		Group* newGroup = NULL;

		switch (type)
		{
		case SYMMETRY:
			newGroup = new SymmetryGroup(SYMMETRY);
			break;
		case POINTJOINT:
			newGroup = new PointJointGroup(POINTJOINT);
			break;
		case LINEJOINT:
			newGroup = new LineJointGroup(LINEJOINT);
			break;
		case SELF_SYMMETRY:
			{
				inF >> str;
				QString primId = QString(str.c_str());
				Primitive* prim = getPrimitive(primId);
				int nb_fold = 0; 
				inF >> nb_fold;
				for (int i=0;i<nb_fold;i++)
				{
					Plane p;
					p.center = prim->centerPoint();
					inF >> p.n;
					prim->symmPlanes.push_back(p);
				}
				break;
			}
		}

		if(newGroup)
		{
			int n;
			inF >> n;
			std::string str;
			QVector<Primitive*> segments;
			for (int i=0;i<n;i++)
			{
				inF >> str;
				segments.push_back(getPrimitive(str.c_str()));
			}
			newGroup->loadParameters(inF, m_mesh->translation, m_mesh->scaleFactor);
			newGroup->process(segments);

			groups[newGroup->id] = newGroup;
		}
	}
}

void Controller::saveGroups( std::ofstream &outF )
{
	foreach(Group* group, groups)
	{
		// type
		outF << qPrintable(groupTypes[group->type]) << '\t'; 

		// size
		outF << group->nodes.size() << "\t";

		// primitives
		foreach(Primitive* node, group->nodes)
			outF << qPrintable(node->id) << "\t";

		// parameters
		group->saveParameters(outF);

		// break line
		outF << '\n';
	}

	// Save properties for single segment (not groups though)
	foreach(Primitive * prim, getPrimitives())
	{
		int nb = prim->symmPlanes.size();

		if(nb > 0)
		{
			outF << qPrintable(groupTypes[SELF_SYMMETRY]) << "\t" << qPrintable(prim->id) << "\t" << nb << "\t";
			foreach(Plane p, prim->symmPlanes)
				outF << p.n << "\t";
			outF << std::endl;
		}
	}
}