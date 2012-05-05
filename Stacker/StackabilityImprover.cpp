#include "StackabilityImprover.h"

#include "Offset.h"
#include "Primitive.h"
#include "Controller.h"
#include "Propagator.h"


StackabilityImprover::StackabilityImprover( Offset *offset )
{
	activeOffset = offset;
}

QVector<double> StackabilityImprover::getLocalScales( HotSpot& HS )
{
	QVector<double> scales;

	int K = 6;
	double stepSize = 0.05;

	double s = 0;
	for (int i = 0; i< K; i++)
	{
		s += stepSize;
		scales.push_back(1 + s);
		scales.push_back(1 - s);
	}

	return scales;
}

QVector<Vec3d> StackabilityImprover::getLocalMoves( HotSpot& HS )
{
	QVector< Vec3d > result;

	// pos
	Vec3d hotPoint = HS.rep.first();

	// step
	Vec3d step = (constraint_bbmax - constraint_bbmin) / 20;
	int K = 2;

	// Horizontal moves
	if (HS.type == POINT_HOTSPOT)
	{
		double min_x = - step[0] * K;
		double max_x = - min_x;
		double min_y = - step[1] * K;
		double max_y = - min_y;

		for (double x = min_x; x <= max_x; x += step[0]){
			for (double y = min_y; y <= max_y; y += step[1]){
				// for (double z = min_z; z <= max_z; z += step[2])
				double z = 0;
				{
					Vec3d delta(x,y,z);
					Vec3d pos = hotPoint + delta;

					// check whether \pos is in BB
					if (RANGE(pos[0], constraint_bbmin[0], constraint_bbmax[0])
						&& RANGE(pos[1], constraint_bbmin[1], constraint_bbmax[1])
						&& RANGE(pos[2], constraint_bbmin[2], constraint_bbmax[2]))
					{
						result.push_back(delta);
					}
				}			
			}
		}
	}
	else if ( HS.type == LINE_HOTSPOT)
	{
		// Move only perpendicular to the line
		Vec3d n = HS.rep.last() - HS.rep.first();
		n.normalize();
		Vec3d d1 = Vec3d(n.x(), - n.y(), 0);
		Vec3d d2 = Vec3d(-n.x(), n.y(), 0);
		d1.normalize();
		d2.normalize();

		for (int i = 0; i < K; i++)
		{
			result.push_back(d1 * i * step.x());
			result.push_back(d2 * i * step.x());
		}
	}

	// Vertical moves
	if (!HS.defineHeight)
	{
		double min_z = - step[2] * K;
		double max_z = - min_z;
		if (1 == HS.side)
			max_z = 0;
		else
			min_z = 0;

		for (double z = min_z; z <= max_z; z += step[2])
		{
			Vec3d delta = Vec3d(0, 0, z);
			Vec3d pos = hotPoint + delta;

			// check whether \pos is in BB
			if (RANGE(pos[0], constraint_bbmin[0], constraint_bbmax[0])
				&& RANGE(pos[1], constraint_bbmin[1], constraint_bbmax[1])
				&& RANGE(pos[2], constraint_bbmin[2], constraint_bbmax[2]))
			{
				result.push_back(delta);
			}
		}
	}

	return result;
}

void StackabilityImprover::recordSolution(Point handleCenter, Vec3d localMove, int side)
{
	double stackability = activeOffset->getStackability(true);

	ShapeState state = ctrl()->getShapeState();

	// \state has to be unique
	if (!isUnique(state, 0)) return;

	// Save state history
	state.history = currentCandidate.history;
	state.history.push_back(state);
	state.deltaStackability = stackability - origStackability;
	state.distortion = ctrl()->getDistortion();

	// Evaluate suggestion
	EditingSuggestion suggest;
	suggest.center = handleCenter;
	suggest.direction = localMove;
	suggest.deltaS = state.deltaStackability;
	suggest.deltaV = state.distortion;
	suggest.side = side;
	suggest.value = state.energy();

	state.trajectory = currentCandidate.trajectory;
	state.trajectory.push_back(suggest);

	// Store the state
	candidateSolutions.push(state);	
}

void StackabilityImprover::deformNearPointLineHotspot( int side )
{
	// The first pair of hot spots
	HotSpot& freeHS = activeOffset->getHotspot(side, 0);
	HotSpot& fixedHS = activeOffset->getHotspot(-side, 0);
	Primitive* free_prim = ctrl()->getPrimitive(freeHS.segmentID);
	Primitive* fixed_prim = ctrl()->getPrimitive(fixedHS.segmentID);
	QVector<Point> free_handle = freeHS.rep;
	QVector<Point> fixed_hanble = fixedHS.rep;

	// Move the hotspot locally
	Propagator propagator(ctrl());
	QVector<Vec3d> Ts = getLocalMoves(freeHS);

	//// debug
	//Ts.clear();
	//Ts.push_back(Vec3d(-0.25,0,0));

	foreach ( Vec3d T, Ts)
	{
		ctrl()->setPrimitivesFrozen(false);	// Clear flags
		setPositionalConstriants(fixedHS); // Fix one end

		// Move the other end
		if (freeHS.type == POINT_HOTSPOT)
		{
			free_prim->movePoint(free_handle.first(), T); 
			free_prim->addFixedPoint(free_handle.first() + T); 
		}else
		{
			free_prim->moveLineJoint(free_handle.first(), free_handle.last(), T, T);
			free_prim->addFixedPoint(free_handle.first() + T);
			free_prim->addFixedPoint(free_handle.last() + T);
		}

		// Fix the relation between hot segments then propagate the local modification
		propagator.regroupPair(free_prim->id, fixed_prim->id); 	
		propagator.execute(); 

//		if ( !satisfyBBConstraint() ) continue; // BB constraint is hard	

		// Record the shape state
		if (freeHS.type == POINT_HOTSPOT)
			recordSolution(free_handle.first(), T, freeHS.side);
		else
			recordSolution( (free_handle.first()+free_handle.last())/2, T, freeHS.side);

		// Restore the shape state of current candidate
		ctrl()->setShapeState(currentCandidate);
	}
}

void StackabilityImprover::deformNearRingHotspot( int side )
{
	// The first pair of hot spots
	HotSpot& freeHS = activeOffset->getHotspot(side, 0);
	HotSpot& fixedHS = activeOffset->getHotspot(-side, 0);
	Primitive* free_prim = ctrl()->getPrimitive(freeHS.segmentID);
	Primitive* fixed_prim = ctrl()->getPrimitive(fixedHS.segmentID);
	int free_cid = free_prim->detectHotCurve(freeHS.hotSamples);
	Point free_curve_center = free_prim->curveCenter(free_cid);

	// Scale the ring hot spot
	Propagator propagator(ctrl());
	QVector<double> scales = getLocalScales(freeHS);
	foreach (double scale, scales)
	{
		ctrl()->setPrimitivesFrozen(false);	// Clear flags
		setPositionalConstriants(fixedHS); // Fix one end

		// Scale the other end
		free_prim->scaleCurve(free_cid, scale);
		free_prim->addFixedCurve(free_cid);

		// Fix the relation between hot segments then propagate the local modification
		propagator.regroupPair(free_prim->id, fixed_prim->id); 
		propagator.execute(); 

		if ( !satisfyBBConstraint() ) continue; // BB constraint is strict	

		// Record the shape state
		Vec3d delta;
		Point hotSample = freeHS.hotSamples[0];
		if (scale > 1)
			delta = hotSample - free_curve_center;
		else
			delta = free_curve_center - hotSample;

		recordSolution(hotSample, delta.normalized()/10, freeHS.side);

		// Restore the shape state of current candidate
		ctrl()->setShapeState(currentCandidate);
	}
}

void StackabilityImprover::deformNearHotspot( int side )
{
	HOTSPOT_TYPE type;
	if (side == 1)
		type = activeOffset->upperHotSpots[0].type;
	else
		type = activeOffset->lowerHotSpots[0].type;

	switch (type)
	{
	case POINT_HOTSPOT:
	case LINE_HOTSPOT:
		deformNearPointLineHotspot(side);
		break;
	case RING_HOTSPOT:
		deformNearRingHotspot(side);
		break;
	}
}

void StackabilityImprover::localSearch()
{
	// Step 1: Detect hot spots
	activeOffset->computeOffsetOfShape();
	activeOffset->detectHotspots();

	// Step 2: Deform the object by locally modifying the hot spots
	deformNearHotspot(1); // upper
//	deformNearHotspot(-1);	//lower
}

bool StackabilityImprover::satisfyBBConstraint()
{
	bool result = true;

	Vec3d preBB = constraint_bbmax - constraint_bbmin;
	activeObject()->computeBoundingBox();
	Vec3d currBB = activeObject()->bbmax - activeObject()->bbmin;

	Vec3d diff = preBB - currBB;

	// Current BB is within expanded BB (with tolerence)
	if ( diff[0] < 0 || diff[1] < 0 || diff[2] < 0 )
		result = false;

	// debug
	//std::cout << "-----------------------------------\n"
	//	<<"The preBB size: (" << preBB <<")\n";	
	//std::cout << "The currBB size:(" << currBB <<")\n";
	//std::cout << "BB-satisfying: " << result <<std::endl;

	return result;
}

bool StackabilityImprover::isUnique( ShapeState state, double threshold )
{
	// dissimilar to \solutions
	PQShapeStateLessDistortion solutionsCopy = solutions;
	while(!solutionsCopy.empty())
	{
		ShapeState ss = solutionsCopy.top();

		if (ctrl()->similarity(ss, state) < threshold)
			return false;

		solutionsCopy.pop();
	}

	// dissimilar to used candidate solutions
	foreach(ShapeState ss, usedCandidateSolutions)
	{
		if (ctrl()->similarity(ss, state) < threshold)
			return false;
	}

	// dissimilar to candidate solutions
	PQShapeStateLessEnergy candSolutionsCopy = candidateSolutions;
	while(!candSolutionsCopy.empty())
	{
		ShapeState ss = candSolutionsCopy.top();

		if (ctrl()->similarity(ss, state) < threshold)
			return false;

		candSolutionsCopy.pop();
	}

	return true;
}

void StackabilityImprover::setPositionalConstriants( HotSpot& fixedHS )
{
	std::vector<HotSpot> fixedHotspots = activeOffset->getHotspots(fixedHS.side);

	foreach(HotSpot hs, fixedHotspots)
	{
		Primitive* prim = ctrl()->getPrimitive(hs.segmentID);

		// Make them fixed
		foreach(Point p, hs.rep)
			prim->addFixedPoint(p);

		// Set height constraint if \hs is height-defining
		// === To do
	}
}

void StackabilityImprover::showSolution( int i )
{
	if (solutions.empty())
	{
		std::cout << "There is no solution.\n";
		return;
	}

	int id = i % solutions.size();
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	PQShapeStateLessDistortion solutionsCopy = solutions;
	for (int i=0;i<id;i++)
		solutionsCopy.pop();

	ShapeState sln = solutionsCopy.top();
	ctrl->setShapeState(sln);

	std::cout << "Histrory length: " << sln.history.size() << std::endl; 
	//std::cout << "Showing the " << id << "th solution out of " << solutions.size() <<".\n";
}

void StackabilityImprover::normalizeSuggestions()
{
	if (suggestions.isEmpty())
		return;

	if (suggestions.size() == 1)
	{
		suggestions.first().value = 1;
		return;
	}


	// Normalize \deltaS and \deltaV respectively
	bool computeValue = false;
	if (computeValue)
	{
		double minS = DBL_MAX;
		double maxS = DBL_MIN;
		double minV = DBL_MAX;
		double maxV = DBL_MIN;

		foreach(EditingSuggestion sg, suggestions)
		{
			double s = sg.deltaS;
			double v = sg.deltaV;

			minS = Min(minS, s);
			maxS = Max(maxS, s);
			minV = Min(minV, v);
			maxV = Max(maxV, v);
		}

		double rangeS = maxS - minS;
		double rangeV = maxV - minV;

		bool zeroRangeS = abs(rangeS) < 1e-10;
		bool zeroRangeV = abs(rangeV) < 1e-10;
		for(int i = 0; i < suggestions.size(); i++)
		{
			double s = zeroRangeS? 1 : (suggestions[i].deltaS - minS) / rangeS;
			double v = zeroRangeV? 1 : (suggestions[i].deltaV - minV) / rangeV;

			double alpha = 0.7;
			suggestions[i].value = alpha * s - (1-alpha)*v;

		}
	}



	// Normalize \value
	double minValue = DBL_MAX;
	double maxValue = DBL_MIN;

	foreach(EditingSuggestion sg, suggestions)
	{
		double s = sg.value;

		minValue = Min(minValue, s);
		maxValue = Max(maxValue, s);

	}

	double range = maxValue - minValue;
	bool zeroRang = abs(range) < 1e-10;
	for(int i = 0; i < suggestions.size(); i++)
	{
		suggestions[i].value = zeroRang? 1 : (suggestions[i].value - minValue) / range;
	}

	////Output
	//std::cout << "There are " << suggestions.size() << " suggestions (from offset):\n";
	//for (int i=0;i<suggestions.size();i++)
	//{
	//	std::cout << " deltaS = " << suggestions[i].deltaS 
	//		<< "\tdeltaV = " << suggestions[i].deltaV
	//		<< "\tvalue = " << suggestions[i].value << std::endl;
	//}
}

void StackabilityImprover::showSuggestion( int i )
{
	if (candidateSolutions.empty())
	{
		std::cout << "There is no suggestion.\n";
		return;
	}

	// Get the i-th suggestion
	int id = i % candidateSolutions.size();
	PQShapeStateLessEnergy suggestSolutionsCopy = candidateSolutions;
	for (int i=0;i<id;i++)	suggestSolutionsCopy.pop();

	// Show the suggest
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];
	ctrl->setShapeState(suggestSolutionsCopy.top());
}

QSegMesh* StackabilityImprover::activeObject()
{
	return activeOffset->activeObject();
}

Controller* StackabilityImprover::ctrl()
{
	return (Controller*)activeObject()->ptr["controller"];
}

// === Main access
void StackabilityImprover::executeImprove(int level)
{
	// Suggesting: \level == 0
	if (level < 0) return;

	// Clear
	clear();

	// The original stackability
	origStackability = activeOffset->getStackability();

	// The bounding box constraint is hard
	constraint_bbmin = activeObject()->bbmin * BB_TOLERANCE;
	constraint_bbmax = activeObject()->bbmax * BB_TOLERANCE;

	// Push the current shape as the initial candidate solution
	ShapeState origState = ctrl()->getShapeState();
	origState.deltaStackability = 0;
	origState.distortion = 0;
	candidateSolutions.push(origState);

	while( !candidateSolutions.empty() && solutions.size() < NUM_EXPECTED_SOLUTION )
	{
		currentCandidate = candidateSolutions.top();
		candidateSolutions.pop();
		usedCandidateSolutions.push_back(currentCandidate);

		// Solution or not
		if (currentCandidate.deltaStackability + origStackability >= TARGET_STACKABILITY)
		{
			solutions.push(currentCandidate);
			std::cout << solutions.size() << " solutions have been found. \n";
			continue;
		}

		// Set up the current candidate
		ctrl()->setShapeState(currentCandidate);

		// Explore all local successors
		localSearch();

		std::cout << "#Candidates = " << candidateSolutions.size() << std::endl;

		// For suggesting or debugging
		if (level > 0)	level--;
		if (level == 0) break;
	}

	std::cout << "Searching completed.\n" << std::endl;

	// Suggestions
//	if (isSuggesting) normalizeSuggestions();
}

void StackabilityImprover::clear()
{
	usedCandidateSolutions.clear();

	candidateSolutions = PQShapeStateLessEnergy();
	solutions = PQShapeStateLessDistortion();
}
