// From Geometric Tools, LLC
#pragma once

#include "SurfaceMesh/Vector.h"
#include <vector>
#include <set>
#include <map>
#include <stack>

typedef double								Real;
typedef Vector<Real, 3>						Vector3;
typedef std::vector<Vector3>::iterator		InputIterator;

class  ConvexHull
{
public:
    // The input to the constructor is a container of vertices whose convex hull
    // is required.  
    ConvexHull (InputIterator begin, InputIterator end);

private:

	std::vector<InputIterator> getExtremes( InputIterator begin, InputIterator end, bool *CCW);
	class TriFace
	{
	public:
		TriFace (int v0, int v1, int v2);

		int GetSign (InputIterator v);
		void AttachTo (TriFace* adj0, TriFace* adj1, TriFace* adj2);
		int DetachFrom (int adjIndex, TriFace* adj);

		int V[3];
		TriFace* Adj[3];
		int Sign;
		int Time;
		bool OnStack;
	};

    bool Update (InputIterator v);
    void ExtractIndices ();
    void DeleteHull ();

    // The current hull.
    std::set<TriFace*> mHull;

    class TerminatorData
    {
    public:
        TerminatorData (int v0 = -1, int v1 = -1, int nullIndex = -1,
            TriFace* tri = 0);

        int V[2];
        int NullIndex;
        TriFace* T;
    };
};
