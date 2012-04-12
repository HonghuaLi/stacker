#include "StackabilityImprover.h"
#include "Offset.h"

QVector<EditSuggestion> suggestions;

StackabilityImprover::StackabilityImprover( Offset *offset )
{
	activeOffset = offset;
	isSuggesting = false;
}

QSegMesh* StackabilityImprover::activeObject()
{
	return activeOffset->activeObject();
}

//*************************** Improve stackability *************************//
std::vector< Vec3d > StackabilityImprover::getHorizontalMoves( HotSpot& HS )
{
	std::vector< Vec3d > Ts;

	// The hot region	
	std::vector< Vec2i > &hotRegion = activeOffset->hotRegions[HS.hotRegionID];

	// Recompute the envelopes using hot segments only
	activeOffset->computeOffsetOfShape();

	// Switchers
	bool isUpper = (HS.side == 1);
	bool x_flipped = isUpper;
	std::vector< std::vector<double> > &op_envelope = isUpper ? activeOffset->lowerEnvelope : activeOffset->upperEnvelope;

	// Project BB to 2D buffer
	//std::vector< std::vector< double > > BBImg = createImage( offset[0].size(), offset.size(), 0. );
	Vec2i bbmin = activeOffset->projectedCoordinatesOf(org_bbmin, 3);
	Vec2i bbmax = activeOffset->projectedCoordinatesOf(org_bbmax, 3);
	//setPixelColor(BBImg, bbmin, 1.0);
	//setPixelColor(BBImg, bbmax, 0.5);
	//saveAsImage(BBImg, 1.0, "BB projection.png");

	// Opposite envelope at current position
	std::vector< double > curr_op_env = getValuesInRegion(op_envelope, hotRegion, x_flipped);
	double threshold = isUpper? MaxElement(curr_op_env) : MinElement( curr_op_env );

	//============ debug code
	std::vector< std::vector< double > > debugImg = createImage( activeOffset->offset[0].size(), activeOffset->offset.size(), 0. );
	setRegionColor(debugImg, hotRegion, 1.0);
	//============ end of debug code

	// The size of the hot region
	Vec2i size = sizeofRegion(hotRegion);
	int step = Max(size.x(), size.y());
	step *= 2;

	// Search for optional locations in 1-K rings
	int K = 4;
	for (uint k = 1; k < K; k++)
	{
		std::vector< Vec2i > deltas = deltaVectorsToKRing(step, step, k);
		for (int j = 0; j < deltas.size(); j++ )
		{
			std::vector< Vec2i > shiftedRegion = shiftRegionInBB(hotRegion, deltas[j], bbmin, bbmax);
			if (shiftedRegion.empty())  
			{
				std::cout << "Out of BB" << std::endl;
				continue; // 
			}

			//============ debug code
			setRegionColor(debugImg, shiftedRegion, 1.0 - k * 0.1 );
			//============ end of debug code

			std::vector< double > new_op_env = getValuesInRegion(op_envelope, shiftedRegion, x_flipped);
			double newValue = isUpper? MinElement( new_op_env ) : MaxElement( new_op_env );
			bool isBetter = isUpper? (newValue > threshold + ZERO_TOLERANCE) : (newValue < threshold - ZERO_TOLERANCE);

			if (isBetter)
			{
				// Get the 3D translation
				Vec2i a = hotRegion[0];
				Vec2i b = shiftedRegion[0];
				Vec3d proj_a = activeOffset->unprojectedCoordinatesOf(a.x(), a.y(), HS.side);
				Vec3d proj_b = activeOffset->unprojectedCoordinatesOf(b.x(), b.y(), HS.side);
				Vec3d T = proj_b - proj_a;
				// Force the translation to be horizontal
				T[2] = 0;

				Ts.push_back(T);
			}
		}
	}

	saveAsImage(debugImg, 1.0, "K Ring Neighbors.png");

	return Ts;
}

std::vector< Vec3d > StackabilityImprover::getLocalMoves( HotSpot& HS )
{
	std::vector< Vec3d > result;

	// pos
	Vec3d hotPoint = HS.hotPoint();

	// step
	Vec3d step = (org_bbmax - org_bbmin) / 20;
	int K = 6;

	// Horizontal moves
	if (HS.defineHeight)
	{
		double min_x = - step[0] * K;
		double max_x = - min_x;
		double min_y = - step[1] * K;
		double max_y = - min_y;

		for (double x = min_x; x <= max_x; x += step[0]){
			for (double y = min_y; y <= max_y; y += step[1]){
				//			for (double z = min_z; z <= max_z; z += step[2])
				double z = 0;
				{
					Vec3d delta(x,y,z);
					Vec3d pos = hotPoint + delta;

					// check whether \pos is in BB
					if (RANGE(pos[0], org_bbmin[0], org_bbmax[0])
						&& RANGE(pos[1], org_bbmin[1], org_bbmax[1])
						&& RANGE(pos[2], org_bbmin[2], org_bbmax[2]))
					{
						result.push_back(delta);
					}
				}			
			}
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
			if (RANGE(pos[0], org_bbmin[0], org_bbmax[0])
				&& RANGE(pos[1], org_bbmin[1], org_bbmax[1])
				&& RANGE(pos[2], org_bbmin[2], org_bbmax[2]))
			{
				result.push_back(delta);
			}
		}
	}
	return result;
}

void StackabilityImprover::applyHeuristicsOnHotspot( HotSpot& HS, HotSpot& opHS )
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	Primitive* prim = ctrl->getPrimitive(HS.segmentID);
	Primitive* op_prim = ctrl->getPrimitive(opHS.segmentID);

	// Find the representative hot samples
	Point hotPoint = HS.hotPoint();
	Point op_hotPoint = opHS.hotPoint();

	// Move hot spot side away or closer to each other
	//std::vector< Vec3d > Ts = getHorizontalMoves(HS);
	std::vector< Vec3d > Ts = getLocalMoves(HS);

	//Ts.clear();
	//Ts.push_back(Vec3d(0.4, 0, 0));

	// Actually modify the shape to generate hot solutions
	for (int i=0;i<Ts.size();i++)
	{
		// Clear the frozen flags
		ctrl->setPrimitivesFrozen(false);		

		// Fix the opposite hot spot
		op_prim->addFixedPoint(op_hotPoint); 

		// Move the current hot spot
		prim->movePoint(hotPoint, Ts[i]);

		prim->addFixedPoint(hotPoint + Ts[i]);

		// fix the hot segments pair
		ctrl->regroupPair(prim->id, op_prim->id);
		// Propagation the deformation
		prim->isFrozen = true;
		op_prim->isFrozen = true;
		ctrl->weakPropagate();

		// Check if this is a candidate solution		
		if ( satisfyBBConstraint() )
		{
			double stackability = activeOffset->computeOffsetOfShape();

			ShapeState state = ctrl->getShapeState();
			state.history = currentCandidate.history;
			state.history.push_back(state);
			state.deltaStackability = stackability - orgStackability;
			state.distortion = ctrl->getDistortion();

			EditSuggestion suggest;
			suggest.center = hotPoint;
			suggest.direction = Ts[i];
			suggest.deltaS = state.deltaStackability;
			suggest.deltaV = state.distortion;
			suggest.side = HS.side;
			suggest.value = state.energy();

			state.trajectory = currentCandidate.trajectory;
			state.trajectory.push_back(suggest);

			if (isSuggesting)
			{
				suggestions.push_back(suggest);
				suggestSolutions.push(state);
			}
			else
			{
				if (stackability > TARGET_STACKABILITY)
					solutions.push(state);
				else if (isUnique(state, 0))
					candidateSolutions.push(state);
			}		

		}

		// Restore the shape state of current candidate
		ctrl->setShapeState(currentCandidate);
	}
}

void StackabilityImprover::applyHeuristicsOnHotRing( HotSpot& HS )
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	Primitive* prim = ctrl->getPrimitive(HS.segmentID);

	// The hot region	
	std::vector< Vec2i > &hotRegion = activeOffset->hotRegions[HS.hotRegionID];

	Vec2i center = centerOfRegion(hotRegion);
	Vec2i p = hotRegion[0];

	// Translations
	std::vector< Vec3d > Ts;
	Ts.push_back(Vec3d(1,0,0));
	Ts.push_back(Vec3d(-1,0,0));

	// Save the initial hot shape state
	ShapeState initialHotShapeState = ctrl->getShapeState();


	Vec3d hotPoint = HS.hotSamples[0];
	for (int i=0;i<Ts.size();i++)
	{
		// Clear the frozen flags
		ctrl->setPrimitivesFrozen(false);		

		// Move the current hot spot
		prim->movePoint(hotPoint, Ts[i]);

		// Propagation the deformation
		prim->isFrozen = true;

		ctrl->weakPropagate();

		// Check if this is a candidate solution

		//		if ( satisfyBBConstraint() )
		{
			double stackability = activeOffset->computeOffsetOfShape();
			//			if (stackability > orgStackability + 0.1)
			{
				ShapeState state = ctrl->getShapeState();
				//				if (isUnique(state, 0))
				{
					state.distortion = 0;
					state.deltaStackability = stackability - orgStackability;

					EditSuggestion suggest;
					suggest.center = hotPoint;

					Vec3d t = hotPoint;
					t[2] = 0;
					if (Ts[i].x() < 0) t *= -1;
					t.normalize();
					t *= 0.15;

					suggest.direction = t;
					suggest.deltaS = state.deltaStackability;
					suggest.deltaV = state.distortion;
					suggest.side = HS.side;
					suggest.value = state.energy();

					state.trajectory = currentCandidate.trajectory;
					state.trajectory.push_back(suggest);

					if (isSuggesting)
					{
						suggestions.push_back(suggest);
						suggestSolutions.push(state);
					}
				}			

			}
		}

		// Restore the initial hot shape state
		ctrl->setShapeState(initialHotShapeState);
	}


}

void StackabilityImprover::applyHeuristics()
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	// Set only hot segments visible
	//ctrl->setSegmentsVisible(false);
	//foreach (QString segmentID, hotSegments)
	//	activeObject()->getSegment(segmentID)->isVisible = true;


	// After getting rid of redundancy caused by symmetries, hopefully only one pair of hot spots remains
	// For now, only apply heuristics on the hot region that has maximum offset
	int selectedID = -1;
	for (int i=0;activeOffset->upperHotSpots.size();i++)
	{
		int rid = activeOffset->upperHotSpots[i].hotRegionID;
		if (activeOffset->maxOffsetInHotRegions[rid] == activeOffset->O_max)
		{
			selectedID = i;
			break;
		}
	}

	if (-1 == selectedID)
	{
		std::cout << "There is no hot region containing O_max.\n";
		return;
	}

	HotSpot& upperHS = activeOffset->upperHotSpots[selectedID];
	HotSpot& lowerHS = activeOffset->lowerHotSpots[selectedID];

	if (upperHS.isRing)
		applyHeuristicsOnHotRing(upperHS);
	else
		applyHeuristicsOnHotspot(upperHS, lowerHS);		


	if (lowerHS.isRing)
		applyHeuristicsOnHotRing(lowerHS);
	else
		applyHeuristicsOnHotspot(lowerHS, upperHS);	
}

bool StackabilityImprover::satisfyBBConstraint()
{
	bool result = true;

	Vec3d preBB = org_bbmax - org_bbmin;
	activeObject()->computeBoundingBox();
	Vec3d currBB = activeObject()->bbmax - activeObject()->bbmin;

	Vec3d diff = preBB - currBB;
	if ( diff[0] < 0 || diff[1] < 0 )
		result = false;

	if( abs(diff[2]) > 0.1 )
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
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	// dissimilar to \solutions
	PQShapeShateLessDistortion solutionsCopy = solutions;
	while(!solutionsCopy.empty())
	{
		ShapeState ss = solutionsCopy.top();

		if (ctrl->similarity(ss, state) < threshold)
			return false;

		solutionsCopy.pop();
	}

	// dissimilar to used candidate solutions
	foreach(ShapeState ss, usedCandidateSolutions)
	{
		if (ctrl->similarity(ss, state) < threshold)
			return false;
	}

	// dissimilar to candidate solutions
	PQShapeShateLessEnergy candSolutionsCopy = candidateSolutions;
	while(!candSolutionsCopy.empty())
	{
		ShapeState ss = candSolutionsCopy.top();

		if (ctrl->similarity(ss, state) < threshold)
			return false;

		candSolutionsCopy.pop();
	}


	//std::cout << "Unique: " << result <<std::endl;

	return true;
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

	PQShapeShateLessDistortion solutionsCopy = solutions;
	for (int i=0;i<id;i++)
		solutionsCopy.pop();

	ShapeState sln = solutionsCopy.top();
	ctrl->setShapeState(sln);

	std::cout << "Histrory length: " << sln.history.size() << std::endl; 
	//std::cout << "Showing the " << id << "th solution out of " << solutions.size() <<".\n";
}


// Suggestions
QVector<EditSuggestion> StackabilityImprover::getSuggestions()
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	// Clear
	suggestions.clear();
	suggestSolutions = PQShapeShateLessEnergy();
	solutions = PQShapeShateLessDistortion();

	// Current candidate
	currentCandidate = ctrl->getShapeState();
	currentCandidate.deltaStackability = activeOffset->getStackability() - orgStackability;
	currentCandidate.distortion = ctrl->getDistortion();


	// The bounding box constraint is hard
	org_bbmin = activeObject()->bbmin;
	org_bbmax = activeObject()->bbmax;
	org_bbmin[0] *= BB_TOLERANCE;
	org_bbmin[1] *= BB_TOLERANCE;
	org_bbmax[0] *= BB_TOLERANCE;
	org_bbmax[1] *= BB_TOLERANCE;

	org_bbmin[2] *= 1.0001;
	org_bbmax[2] *= 1.0001;

	isSuggesting = true;
	improveStackability();
	isSuggesting = false;

	normalizeSuggestions();
	return suggestions;
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

		foreach(EditSuggestion sg, suggestions)
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

	foreach(EditSuggestion sg, suggestions)
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

	//Output
	std::cout << "There are " << suggestions.size() << " suggestions (from offset):\n";
	for (int i=0;i<suggestions.size();i++)
	{
		std::cout << " deltaS = " << suggestions[i].deltaS 
			<< "\tdeltaV = " << suggestions[i].deltaV
			<< "\tvalue = " << suggestions[i].value << std::endl;
	}
}

void StackabilityImprover::showSuggestion( int i )
{
	if (suggestSolutions.empty())
	{
		std::cout << "There is no suggestion.\n";
		return;
	}

	int id = i % suggestSolutions.size();

	PQShapeShateLessEnergy suggestSolutionsCopy = suggestSolutions;
	for (int i=0;i<id;i++)
		suggestSolutionsCopy.pop();

	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	ctrl->setShapeState(suggestSolutionsCopy.top());
}

// Main access
void StackabilityImprover::improveStackabilityToTarget()
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	// Clear
	candidateSolutions = PQShapeShateLessEnergy();
	solutions = PQShapeShateLessDistortion();
	usedCandidateSolutions.clear();

	// The bounding box constraint is hard
	org_bbmin = activeObject()->bbmin;
	org_bbmax = activeObject()->bbmax;
	org_bbmin[0] *= BB_TOLERANCE;
	org_bbmin[2] *= BB_TOLERANCE;
	org_bbmax[0] *= BB_TOLERANCE;
	org_bbmax[2] *= BB_TOLERANCE;

	// Push the current shape as the initial candidate solution
	ShapeState state = ctrl->getShapeState();
	state.deltaStackability = activeOffset->getStackability() - orgStackability;
	state.distortion = ctrl->getDistortion();
	state.history.push_back(state);
	candidateSolutions.push(state);

	while(!candidateSolutions.empty())
	{
		currentCandidate = candidateSolutions.top();
		candidateSolutions.pop();
		usedCandidateSolutions.push_back(currentCandidate);

		ctrl->setShapeState(currentCandidate);
		improveStackability();

		std::cout << "#Candidates = " << candidateSolutions.size() << std::endl;
		std::cout << "#Solutions = " << solutions.size() << std::endl;

		if (solutions.size() >= NUM_EXPECTED_SOLUTION)
		{
			std::cout << NUM_EXPECTED_SOLUTION << " solutions have been found." << std::endl;
			break;
		}

	}

	//// Debug: add candidate solutions to solutions
	//while (!candidateSolutions.empty())
	//{
	//	solutions.push( candidateSolutions.top() );
	//	candidateSolutions.pop();
	//}

	ctrl->setShapeState(state);
	activeObject()->computeBoundingBox();
}

void StackabilityImprover::improveStackability()
{
	Controller* ctrl = (Controller*)activeObject()->ptr["controller"];

	// improve the stackability of current shape in three steps
	//=========================================================================================
	// Step 1: Detect hot spots
	activeOffset->computeOffsetOfShape();
	orgStackability = activeOffset->getStackability();
	activeOffset->detectHotspots();

	//=========================================================================================
	// Step 2: Apply heuristics on hot spots and propagate deformations
	// Several hot solutions might be generated, which are stored in *hotSolutions*
	applyHeuristics();
}