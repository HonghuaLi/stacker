// From Geometric Tools, LLC
#pragma once

//Visulization
#include "Surface_mesh.h"
#include "SimpleDraw.h"

#include "SurfaceMesh/Vector.h"
#include <vector>
#include <set>
#include <map>
#include <stack>

typedef double								Real;
typedef Vector<Real, 3>						Vector3;

class  ConvexHull
{
public:
	ConvexHull(std::vector<Vector3> &pnts);
	ConvexHull(Surface_mesh * mesh);
    void computeCH();
	void draw();

private:
	class TriFace
	{
	public:
		TriFace (int v0, int v1, int v2);

		int GetSign( int id , std::vector<Vector3> &pnts);
		void AttachTo (TriFace* adj0, TriFace* adj1, TriFace* adj2);
		int DetachFrom (int adjIndex, TriFace* adj);

		int V[3];
		TriFace* Adj[3];
		int Sign;
		int Time;
		bool OnStack;
	};

    class TerminatorData
    {
    public:
        TerminatorData (int v0 = -1, int v1 = -1, int nullIndex = -1,
            TriFace* tri = 0);

        int V[2];
        int NullIndex;
        TriFace* T;
    };

private:
	bool getExtremes( std::vector<int> &mExtremes);
	bool Update (int id);
	void ExtractIndices ();
	void DeleteHull ();

private:
	std::vector<Vector3> mPnts;	// The input pnts
	std::set<TriFace*> mHull;		// The current hull.
	int mNumSimplices;
	std::vector<int> mIndices;
	bool isReady;
};
