// From Geometric Tools, LLC

#include "ConvexHull.h"



ConvexHull::ConvexHull (InputIterator begin, InputIterator end)
{
	bool CCW;
	std::vector<InputIterator> mExtreme = getExtremes(begin, end, &CCW);
	int i0 = mExtreme[0] - begin;
	int i1 = mExtreme[1] - begin;
	int i2 = mExtreme[2] - begin;
	int i3 = mExtreme[3] - begin;

	TriFace* tri0;
	TriFace* tri1;
	TriFace* tri2;
	TriFace* tri3;

	if (CCW){
		tri0 = new TriFace(i0, i1, i3);
		tri1 = new TriFace(i0, i2, i1);
		tri2 = new TriFace(i0, i3, i2);
		tri3 = new TriFace(i1, i2, i3);
		tri0->AttachTo(tri1, tri3, tri2);
		tri1->AttachTo(tri2, tri3, tri0);
		tri2->AttachTo(tri0, tri3, tri1);
		tri3->AttachTo(tri1, tri2, tri0);
	}
	else{
		tri0 = new TriFace(i0, i3, i1);
		tri1 = new TriFace(i0, i1, i2);
		tri2 = new TriFace(i0, i2, i3);
		tri3 = new TriFace(i1, i3, i2);
		tri0->AttachTo(tri2, tri3, tri1);
		tri1->AttachTo(tri0, tri3, tri2);
		tri2->AttachTo(tri1, tri3, tri0);
		tri3->AttachTo(tri0, tri2, tri1);
	}

	mHull.clear();
	mHull.insert(tri0);
	mHull.insert(tri1);
	mHull.insert(tri2);
	mHull.insert(tri3);

	for (InputIterator itr = begin; itr!=end; itr++)
	{
		if (!Update(itr)){
			DeleteHull();
			return;
		}
	}

	ExtractIndices();
}

bool ConvexHull::Update (InputIterator v)
{
	// Locate a TriFace visible to the input point (if possible).
	TriFace* visible = 0;
	TriFace* tri;
	typename std::set<TriFace*>::iterator iter;
	for (iter= mHull.begin(); iter != mHull.end(); ++iter)
	{
		tri = *iter;
		if (tri->GetSign(v) > 0)
		{
			visible = tri;
			break;
		}
	}

	if (!visible){
		// The point is inside the current hull; nothing to do.
		return true;
	}

	// Locate and remove the visible TriFaces.
	std::stack<TriFace*> visibleSet;
	std::map<int,TerminatorData> terminator;
	visibleSet.push(visible);
	visible->OnStack = true;
	int j, v0, v1;
	while (!visibleSet.empty())
	{
		tri = visibleSet.top();
		visibleSet.pop();
		tri->OnStack = false;
		for (j = 0; j < 3; ++j)
		{
			TriFace* adj = tri->Adj[j];
			if (adj)
			{
				// Detach TriFace and adjacent TriFace from each other.
				int nullIndex = tri->DetachFrom(j, adj);

				if (adj->GetSign(v) > 0)
				{
					if (!adj->OnStack)
					{
						// Adjacent TriFace is visible.
						visibleSet.push(adj);
						adj->OnStack = true;
					}
				}
				else
				{
					// Adjacent TriFace is invisible.
					v0 = tri->V[j];
					v1 = tri->V[(j+1)%3];
					terminator[v0] = TerminatorData(v0, v1, nullIndex, adj);
				}
			}
		}
		mHull.erase(tri);
		delete tri;
	}

	// Insert the new edges formed by the input point and the terminator
	// between visible and invisible TriFaces.
	int size = (int)terminator.size();
	std::map<int,TerminatorData>::iterator edge = terminator.begin();
	v0 = edge->second.V[0];
	v1 = edge->second.V[1];
	tri = new TriFace(i, v0, v1);
	mHull.insert(tri);

	// Save information for linking first/last inserted new TriFaces.
	int saveV0 = edge->second.V[0];
	TriFace* saveTri = tri;

	// Establish adjacency links across terminator edge.
	tri->Adj[1] = edge->second.T;
	edge->second.T->Adj[edge->second.NullIndex] = tri;
	for (j = 1; j < size; ++j)
	{
		edge = terminator.find(v1);
		v0 = v1;
		v1 = edge->second.V[1];
		TriFace* next = new TriFace(i, v0, v1);
		mHull.insert(next);

		// Establish adjacency links across terminator edge.
		next->Adj[1] = edge->second.T;
		edge->second.T->Adj[edge->second.NullIndex] = next;

		// Establish adjacency links with previously inserted TriFace.
		next->Adj[0] = tri;
		tri->Adj[2] = next;

		tri = next;
	}


	// Establish adjacency links between first/last TriFaces.
	saveTri->Adj[0] = tri;
	tri->Adj[2] = saveTri;
	return true;
}

void ConvexHull::ExtractIndices ()
{
	int mNumSimplices = (int)mHull.size();
	mIndices = new1<int>(3*mNumSimplices);

	typename std::set<TriFace*>::iterator iter = mHull.begin();
	typename std::set<TriFace*>::iterator end = mHull.end();
	for (int i = 0; iter != end; ++iter)
	{
		TriFace* tri = *iter;
		for (int j = 0; j < 3; ++j, ++i)
		{
			mIndices[i] = tri->V[j];
		}
		delete0(tri);
	}
	mHull.clear();
}

void ConvexHull::DeleteHull ()
{
	typename std::set<TriFace*>::iterator iter = mHull.begin();
	typename std::set<TriFace*>::iterator end = mHull.end();
	for (/**/; iter != end; ++iter)
	{
		TriFace* tri = *iter;
		delete tri;
	}
	mHull.clear();
}

std::vector<InputIterator> ConvexHull::getExtremes( InputIterator begin, InputIterator end, bool *CCW)
{
	// Compute the axis-aligned bounding box for the input points.  Keep track
	// of the indices into 'points' for the current min and max.
	Vector3 mMin, mMax;
	mMin = *begin;
	mMax = *begin;

	std::vector<InputIterator> itrMin, itrMax;
	for (int j = 0; j < 3; ++j)
	{
		itrMin.push_back(end);
		itrMax.push_back(end);
	}

	for (InputIterator itr = begin; itr!=end; itr++)
	{	
		for (int j = 0; j < 3; ++j)
		{
			Vector3 p = *itr;

			if (p[j] < mMin[j])
			{
				mMin[j] = p[j];
				itrMin[j] = itr;
			}
			else if (p[j] > mMax[j])
			{
				mMax[j] = p[j];
				itrMax[j] = itr;
			}
		}
	}

	// Determine the maximum range for the bounding box.
	std::vector<InputIterator> mExtreme(4, end);
	Real mMaxRange = mMax[0] - mMin[0];
	mExtreme[0] = itrMin[0];
	mExtreme[1] = itrMax[0];

	Real range = mMax[1] - mMin[1];
	if (range > mMaxRange)
	{
		mMaxRange = range;
		mExtreme[0] = itrMin[1];
		mExtreme[1] = itrMax[1];
	}
	range = mMax[2] - mMin[2];
	if (range > mMaxRange)
	{
		mMaxRange = range;
		mExtreme[0] = itrMin[2];
		mExtreme[1] = itrMax[2];
	}

	// The origin is either the point of minimum x-value, point of
	// minimum y-value, or point of minimum z-value.
	Vector3 mOrigin = *mExtreme[0];
	std::vector<Vector3> mDirection(3, Vector3());

	// Test whether the point set is (nearly) a line segment.
	mDirection[0] = *mExtreme[1] - mOrigin;
	mDirection[0].normalize();
	Real maxDistance = (Real)0;
	Real distance;
	for (InputIterator itr = begin; itr!=end; itr++)
	{
		Vector3 diff = *itr - mOrigin;
		Vector3 proj = diff - dot(mDirection[0], diff) * mDirection[0];
		distance = proj.norm();
		if (distance > maxDistance)
		{
			maxDistance = distance;
			mExtreme[2] = itr;
			mDirection[1] = proj;
		}
	}


	// Test whether the point set is (nearly) a planar polygon.
	mDirection[1].normalize();
	mDirection[2] = cross(mDirection[0], mDirection[1]);
	maxDistance = (Real)0;
	for (InputIterator itr = begin; itr!=end; itr++)
	{
		Vector3 diff = *itr - mOrigin;
		distance = dot(mDirection[2], diff);
		if (fabs(distance) > maxDistance)
		{
			maxDistance = fabs(distance);
			mExtreme[3] = itr;
			CCW = (distance > (Real)0);
		}
	}

	return mExtreme;
}

// ConvexHull::TriFace
ConvexHull::TriFace::TriFace (int v0, int v1, int v2)
: Sign(0),Time(-1),OnStack(false)
{
	V[0] = v0;
	V[1] = v1;
	V[2] = v2;
	Adj[0] = 0;
	Adj[1] = 0;
	Adj[2] = 0;
}


int ConvexHull::TriFace::GetSign( InputIterator v )
{
	Vector3 a = V[1] - V[0];
	Vector3 b = V[2] - V[1];
	Vector3 c = (*v) - V[0];
	return dot(cross(a,b), c) > 0;
}

void ConvexHull::TriFace::AttachTo (TriFace* adj0, TriFace* adj1,
	TriFace* adj2)
{
	// assert:  The input adjacent TriFaces are correctly ordered.
	Adj[0] = adj0;
	Adj[1] = adj1;
	Adj[2] = adj2;
}

int ConvexHull::TriFace::DetachFrom (int adjIndex, TriFace* adj)
{
	assertion(0 <= adjIndex && adjIndex < 3 && Adj[adjIndex] == adj,
		"Invalid inputs\n");

	Adj[adjIndex] = 0;
	for (int i = 0; i < 3; ++i)
	{
		if (adj->Adj[i] == this)
		{
			adj->Adj[i] = 0;
			return i;
		}
	}
	return -1;
}

// ConvexHull::TerminatorData
ConvexHull::TerminatorData::TerminatorData (int v0, int v1, int nullIndex, TriFace* tri) : NullIndex(nullIndex), T(tri)
{
	V[0] = v0;
	V[1] = v1;
}
