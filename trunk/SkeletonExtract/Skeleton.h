#pragma once

#include "SkeletonNode.h"
#include "SkeletonEdge.h"

#include "Utility/Graph.h"

#include "Surface_mesh.h"

typedef Graph<uint, double> SkeletonGraph;

class Skeleton
{

public:
	Skeleton();
	~Skeleton();

	bool isReady;
	bool isVisible;
	bool isUserFriendly;

	// STRUCTURE
	std::vector<SkeletonNode> nodes;
	std::vector<SkeletonEdge> edges;
	std::map<int, std::vector<int> > corr;	// Node -> vertex indices
	std::map<int, int> v_corr;				// vertex index -> Node

	// DATA LOADING
	void loadFromFile(const char* fileName);
	void loadCorrespondenceFile(const char* fileName);
	void postSkeletonLoad();

	// COMPUTATIONS
	void calculateEdgesLengths();
	SkeletonGraph getGraph();

	// SELECTION
	std::vector<SkeletonEdge> originalSelectedEdges;
	std::vector<SkeletonEdge> selectedEdges;
	std::vector<bool> selectedNodes;
	std::vector<int> sortedSelectedNodes;
	std::vector<int> originalSelectedNodes;
	int selectNodeStart;
	int selectNodeEnd;
	int originalStart;
	int originalEnd;
	double minEdgeLength;

	void selectNode(int index);
	void selectNodeFromFace(int index);
	void selectEdge(int index, int node1_index);
	void selectEdges(int start, int end);
	void deselectAll();

	Vec3d selectedEdgesPlane();

	// NODES FUNCTIONS
	std::vector<SkeletonNode*> nodeNeighbours(int node_index);
	std::vector<SkeletonEdge*> nodeNeighbourEdges(int node_index);
	double nodeRadius(int node_index);
	Vec3d centerOfNode(SkeletonNode * n);

	// EDGE FUNCTIONS
	int getEdge(int n1, int n2);

	// GET POINTS
	std::vector<Vec3d> getSelectedSkeletonPoints();

	// FACE SELECTION
	std::vector<int> getSelectedFaces(int start = 0, int end = 0);
	std::vector<int> getSelectedFaces(bool growSelection);
	std::vector<int> lastSelectedFaces;

	// SMOOTH EDGES
	std::vector<SkeletonEdge> smoothEdges;
	std::vector<SkeletonNode> smoothNodes;
	void smoothSelectedEdges(int numSmoothingIterations = 3);
	void cropSelectedEdges(int start = 1, int end = 1);

	// MODIFY OPERATIONS
	std::pair< std::vector<int>, std::vector<int> > Split(int edgeIndex);

	// EMBEDDING
	Surface_mesh * embedMesh;

	// RENDERING FOR SELECTION
	void drawNodesNames();
	void drawMeshFacesNames();

	// VISUALIZATION
	std::vector<Color> colors;

	void draw(bool drawMeshPoints = true);

	// DEBUG:
	std::set<int> testNodes;
};

// IO name decorations
#include <fstream>
#define FileStream std::ifstream 
#define FileStreamOutput std::ofstream 
#define GetLine std::getline 

#define SkelEpsilon  ( std::numeric_limits<double>::epsilon() * 10 ) 
