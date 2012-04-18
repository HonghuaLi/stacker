#include "Propagator.h"

#include <QQueue>

#include "Group.h"
#include "Primitive.h"
#include "Controller.h"
#include "ShapeState.h"

Propagator::Propagator( Controller* ctrl ) : mCtrl( ctrl )
{

}


void Propagator::regroupPair( QString id1, QString id2 )
{
	Group *pairGrp = NULL;
	QVector<Group*> groups = mCtrl->groupsOf(id1);
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

// From *frozen* segments to *unfrozen* ones
void Propagator::weakPropagate(QVector<QString> seeds)
{
	QMap< QString, bool > debugFrozenFlags1 = mCtrl->getFrozenFlags();

	QQueue<QString> frozen;
	foreach(QString id, seeds)
		frozen.enqueue(id);

	while(!frozen.isEmpty())
	{
		QString seed = frozen.dequeue();
		QVector< Group * > grps = mCtrl->groupsOf(seed);

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

void Propagator::weakPropagate()
{
	QVector<QString> seeds;
	foreach(Primitive * p, mCtrl->getPrimitives())
		if(p->isFrozen) seeds.push_back(p->id);

	weakPropagate(seeds);
}

// In case semi-frozen primitives exist, force one of them as frozen and apply weakPropagation
QVector<ShapeState> Propagator::strongPropagate()
{
	QVector< ShapeState > results;

	// Initialize the queue of candidates
	QQueue< ShapeState > candidates;
	candidates.enqueue( mCtrl->getShapeState() );

	while (!candidates.isEmpty())
	{
		// Pick up one candidate
		ShapeState currCandidate = candidates.dequeue();
		mCtrl->setShapeState(currCandidate);

		// Apply weak propagation
		//		QMap< QString, bool > debugFrozenFlags1 = getFrozenFlags();
		if (currCandidate.seeds.isEmpty())
			weakPropagate();
		else
			weakPropagate(currCandidate.seeds);
		//		QMap< QString, bool > debugFrozenFlags2 = getFrozenFlags();

		// Check whether the propagation is done
		QQueue< QString > semi_frozen;
		foreach( Primitive * p, mCtrl->getPrimitives() )
			if(!p->isFrozen && !p->fixedPoints.isEmpty()) 
				semi_frozen.enqueue(p->id);	

		if (semi_frozen.isEmpty())
		{
			// Weak propagation is done
			ShapeState newState = mCtrl->getShapeState();

			// Check if unique
			bool isUnique = true;
			foreach(ShapeState res, results){
				if( mCtrl->similarity(res, newState) < 0 )
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
				Primitive * prim = mCtrl->getPrimitive(semi_frozen[i]);
				prim->isFrozen = true;
				QMap< QString, bool > debugFrozenFlags3 = mCtrl->getFrozenFlags();
				ShapeState state = mCtrl->getShapeState();
				state.seeds.push_back(prim->id);
				candidates.enqueue(state);

				// Restore 
				prim->isFrozen = false;
				QMap< QString, bool > debugFrozenFlags4 = mCtrl->getFrozenFlags();

			}
		}
	}

	// set the shape as the first result
	//setShapeState(results[0]);

	return results;
}