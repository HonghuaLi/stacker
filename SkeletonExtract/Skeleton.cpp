#include "Skeleton.h"

#include "SimpleDraw.h"

Skeleton::Skeleton()
{
	isReady = false;
	isVisible = false;
	embedMesh = NULL;
	originalStart = originalEnd = -1;
}

Skeleton::~Skeleton()
{

}

void Skeleton::loadFromFile(const char* fileName)
{
	printf("\n\n==================\n");
	printf("Loading Skeleton file...(%s)\n", fileName);

	std::string inputLine;
	FileStream file (fileName);

	double x,y,z;
	int n1, n2;

	int degree;
	int numberOfNodes;
	int numberOfEdges;

	int nCount = 0;
	int nEdges = 0;

	nodes.clear();
	edges.clear();
	lastSelectedFaces.clear();

	if (file.is_open())
	{
		if(!file.eof() && GetLine (file, inputLine))
		{
			if(sscanf(inputLine.c_str(), "# D:%d NV:%d NE:%d", &degree, &numberOfNodes, &numberOfEdges) != 3)
			{
				printf("Error reading skeleton file (check header).");
				printf("\n%d - %d - %d\n", degree, numberOfNodes, numberOfEdges);
				return;
			}
		}

		while (!file.eof())
		{
			GetLine (file, inputLine);

			switch(inputLine[0])
			{
			case 'v':
				if(sscanf(inputLine.c_str(), "v %f %f %f", &x,&y,&z) == 3)
					nodes.push_back( SkeletonNode(x,y,z, nCount++) );
				break;

			case 'e':
				if(sscanf(inputLine.c_str(), "e %d %d", &n1, &n2) == 2)
					edges.push_back( SkeletonEdge(&nodes[n1 - 1], &nodes[n2 - 1], nEdges++) );
				break;
			}
		}

		file.close();

		printf("\nSkeleton file loaded: \n\n\t Nodes: %d \t Edges: %d\n\n", nCount, nEdges);

		postSkeletonLoad();
	}
	else
	{
		printf("\n ERROR: cannot open skeleton file.\n");
	}
}

void Skeleton::loadCorrespondenceFile(const char* fileName)
{
	std::string inputLine;
	FileStream file (fileName);

	printf("Loading correspondence file...");

	int nodeIndex = 0;
	int vIndex = 0;

	corr.clear();

	if (file.is_open())
	{
		while (!file.eof())
		{
			GetLine (file, inputLine);

			if(inputLine.length() > 0 && sscanf(inputLine.c_str(), "%d", &nodeIndex))
			{
				v_corr[vIndex] = nodeIndex - 1;
				corr[v_corr[vIndex]].push_back(vIndex);

				vIndex++;
			}
		}

		file.close();
		printf("done.\n");
	}
	else
	{
		printf("ERROR: cannot open correspondence file.\n");
	}
}

void Skeleton::postSkeletonLoad()
{
	isReady = true;
	isVisible = true;
	isUserFriendly = false;

	// empty selection
	deselectAll();

	corr.clear();
}

void Skeleton::calculateEdgesLengths()
{
	double len, minEdgeLength = DBL_MAX;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		len = edges[i].calculateLength();

		if(len < minEdgeLength)
			minEdgeLength = len;

		if(len < SkelEpsilon)
			printf("Bad bone! ");
	}
}

SkeletonGraph Skeleton::getGraph()
{
	SkeletonGraph g;

	for(int i = 0; i < (int)edges.size(); i++)
		g.AddEdge(edges[i].n1->index, edges[i].n2->index, edges[i].length, edges[i].index);

	return g;
}

void Skeleton::deselectAll()
{
	selectedEdges.clear();
	sortedSelectedNodes.clear();

	smoothNodes.clear();
	smoothEdges.clear();

	lastSelectedFaces.clear();

	selectedNodes = std::vector<bool>(nodes.size(), false);
	selectNodeStart = selectNodeEnd = -1; // clear = -1
}

void Skeleton::selectNode(int index)
{
	if((int)selectedNodes.size() > index && index > -1)
		selectedNodes[index] = true;

	if(selectNodeStart == -1)		
	{
		selectNodeStart = index;
		lastSelectedFaces.clear();

		std::vector<SkeletonEdge*> startNeighbours = nodeNeighbourEdges(selectNodeStart);
		Vec3d startPlaneNormal = startNeighbours.front()->direction().normalized();
	}

	if(selectNodeStart != index)	selectNodeEnd = index; 
}

void Skeleton::selectEdge(int index, int node1_index)
{
	if(index < 0) return;

	// Don't select a selected edge
	for(int i = 0; i < (int)selectedEdges.size(); i++)
		if(selectedEdges[i].index == index) return;

	selectedEdges.push_back(edges[index]);

	// order nodes based on selection direction
	if(edges[index].n1->index != node1_index)
	{
		// swap if necessary
		SkeletonNode * temp = edges[index].n1;
		edges[index].n1 = edges[index].n2;
		edges[index].n2 = temp;
	}
}

void Skeleton::selectEdges(int start, int end)
{
	printf("Selection:  Start (%d), End (%d) \n", selectNodeStart, selectNodeEnd);

	if(selectNodeStart != -1 && selectNodeEnd != -1 && selectNodeStart != selectNodeEnd)
	{
		SkeletonGraph g = getGraph();

		std::list<uint> path = g.DijkstraShortestPath(start, end);

		deselectAll();

		uint prevNode = *path.begin();

		for(std::list<uint>::iterator it = path.begin(); it != path.end(); it++)
		{
			selectNode(*it);
			selectEdge(getEdge(prevNode, *it), prevNode);

			sortedSelectedNodes.push_back(*it);

			prevNode = *it;
		}

		// If we never crop
		if(originalEnd == -1)
		{
			originalStart = selectNodeStart;
			originalEnd = selectNodeEnd;
		}
	}

	originalSelectedNodes = sortedSelectedNodes;
	originalSelectedEdges = selectedEdges;
}

void Skeleton::smoothSelectedEdges(int numSmoothingIterations)
{
	if(!selectedEdges.size())
		return;

	smoothNodes.clear();
	smoothEdges.clear();

	int nCount = 0;
	int eCount = 0;

	SkeletonNode *n1, *n2, *n3;
	n1 = n2 = n3 = NULL;

	smoothNodes.push_back(SkeletonNode(*selectedEdges[0].n1, nCount));
	n1 = &smoothNodes[nCount];

	for(int i = 0; i < (int)selectedEdges.size(); i++)
	{
		smoothNodes.push_back(SkeletonNode::Midpoint(*n1, *selectedEdges[i].n2, nCount + 1));
		smoothNodes.push_back(SkeletonNode(*selectedEdges[i].n2, nCount + 2));

		n2 = &smoothNodes[nCount+1];
		n3 = &smoothNodes[nCount+2];

		smoothEdges.push_back(SkeletonEdge(n1, n2, eCount));	
		smoothEdges.push_back(SkeletonEdge(n2, n3, eCount+1));	

		smoothEdges[eCount].calculateLength();
		smoothEdges[eCount+1].calculateLength();

		nCount += 2;
		eCount += 2;

		n1 = n3;
	}

	// Laplacian smoothing (best option?)
	for(int stage = 0; stage < numSmoothingIterations; stage++)
	{
		std::vector<double*> positions = std::vector<double*>(smoothNodes.size());

		for(int i = 1; i < (int)smoothNodes.size() - 1; i++)
		{
			positions[i] = new double[3];

			positions[i][0] = (smoothNodes[i-1].x() + smoothNodes[i+1].x()) / 2;
			positions[i][1] = (smoothNodes[i-1].y() + smoothNodes[i+1].y()) / 2;
			positions[i][2] = (smoothNodes[i-1].z() + smoothNodes[i+1].z()) / 2;
		}

		for(int i = 1; i < (int)smoothNodes.size() - 1; i++)
		{
			smoothNodes[i].set(positions[i]);
			delete [] positions[i];
		}
	}

	// clear old list of selected edges
	selectedEdges.clear();

	// fill in the new smooth edges
	selectedEdges = smoothEdges;
}

void Skeleton::cropSelectedEdges(int start, int end)
{
	originalStart = selectNodeStart;
	originalEnd = selectNodeEnd;

	if(sortedSelectedNodes.size() > 5)
	{
		int N = sortedSelectedNodes.size() - 1;
		selectEdges(sortedSelectedNodes[start], sortedSelectedNodes[N - end]);
	}
}		

std::vector<Vec3d> Skeleton::getSelectedSkeletonPoints()
{
	std::vector<Vec3d> skeletonPoints;

	for(int i = 0; i < (int)selectedEdges.size(); i++)
		skeletonPoints.push_back(*selectedEdges[i].n1);

	skeletonPoints.push_back(*(*selectedEdges.rbegin()).n2); // Last point

	return skeletonPoints;
}

int Skeleton::getEdge(int n1, int n2)
{
	int index = -1;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		if((edges[i].n1->index == n1 && edges[i].n2->index == n2) || 
			(edges[i].n1->index == n2 && edges[i].n2->index == n1))
		{
			return edges[i].index;
		}
	}

	return index;
}

Vec3d Skeleton::selectedEdgesPlane()
{
	if(selectedEdges.size() < 3)
		return Vec3d(0,0,0);
	else
		return cross(selectedEdges[0].direction() , selectedEdges[selectedEdges.size() - 1].direction());
}

std::vector<SkeletonNode *> Skeleton::nodeNeighbours(int node_index)
{
	std::vector<SkeletonNode *> neighbours;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		if(edges[i].n1->index == node_index)
			neighbours.push_back(&nodes[edges[i].n2->index]);
		else if(edges[i].n2->index == node_index)
			neighbours.push_back(&nodes[edges[i].n1->index]);
	}

	return neighbours;
}

std::vector<SkeletonEdge *> Skeleton::nodeNeighbourEdges(int node_index)
{
	std::vector<SkeletonEdge *> neighbours;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		if(edges[i].n1->index == node_index)
			neighbours.push_back(&edges[i]);
		else if(edges[i].n2->index == node_index)
			neighbours.push_back(&edges[i]);
	}

	return neighbours;
}

double Skeleton::nodeRadius(int node_index)
{
	Surface_mesh::Vertex_property<Point> points = embedMesh->vertex_property<Point>("v:point");

	double avgDist = 0;

	Vec3d node_center = nodes[node_index];

	for(int j = 0; j < (int)corr[node_index].size(); j++)
	{
		avgDist += (points[Surface_mesh::Vertex(corr[node_index][j])] - node_center).norm();
	}

	avgDist /= corr[node_index].size();

	return avgDist;
}

Vec3d Skeleton::centerOfNode(SkeletonNode * n)
{
	Surface_mesh::Vertex_property<Point> points = embedMesh->vertex_property<Point>("v:point");

	Vec3d center;

	for(int j = 0; j < (int)corr[n->index].size(); j++)
	{
		center += points[Surface_mesh::Vertex(corr[n->index][j])];
	}

	return center / corr[n->index].size();
}

std::pair< std::vector<int>, std::vector<int> > Skeleton::Split(int edgeIndex)
{
	SkeletonGraph g;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		if(edgeIndex != i)
			g.AddEdge(edges[i].n1->index, edges[i].n2->index, edges[i].length, edges[i].index);
	}

	SkeletonEdge * e = &edges[edgeIndex];

	std::map<int, int> partA, partB;

	// Find the two connected components
	for(int i = 0; i < (int)nodes.size(); i++)
	{
		if(g.isConnected( e->n1->index, nodes[i].index ))
			partA[i] = i;
		else
			partB[i] = i;
	}

	std::vector<int> pointsA, pointsB;


	for(std::map<int,int>::iterator it = partA.begin(); it != partA.end(); it++)
	{
		int i = (*it).first;
		for(int p = 0; p < (int)corr[i].size(); p++)
			pointsA.push_back(corr[i][p]);
	}

	for(std::map<int,int>::iterator it = partB.begin(); it != partB.end(); it++)
	{
		int i = (*it).first;
		for(int p = 0; p < (int)corr[i].size(); p++)
			pointsB.push_back(corr[i][p]);
	}

	return std::make_pair(pointsA, pointsB);
}

void Skeleton::drawNodesNames()
{
	for(int i = 0; i < (int)nodes.size(); i++)
	{
		glPushName(i);
		glBegin(GL_POINTS);
		glVertex3dv(nodes[i]);
		glEnd();
		glPopName();
	}
}

void Skeleton::draw(bool drawMeshPoints)
{
	if(!isVisible || !isReady)
		return;

	if(isUserFriendly)
	{
		//drawUserFriendly();
		return;
	}

	glClear(GL_DEPTH_BUFFER_BIT);
	glDisable(GL_LIGHTING);

	// Draw the nodes
	glPointSize(4.0f);
	glColor3f(1,0,0);
	glEnable(GL_POINT_SMOOTH);

	//======================
	// Draw Skeleton Edges

	float oldLineWidth = 0;
	glGetFloatv(GL_LINE_WIDTH, &oldLineWidth);
	glLineWidth(1.5f);

	glEnable(GL_BLEND); 
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_LINES);
	for(int i = 0; i < (int)edges.size(); i++)
	{
		int nIndex1 = edges[i].n1->index;
		int nIndex2 = edges[i].n2->index;

		glColor4d(1,1,1,0.2);

		bool isEdgeSelected = false;
		if(selectedNodes[nIndex1] && selectedNodes[nIndex2])
			isEdgeSelected = true;

		if(isEdgeSelected)	glColor3d(0.5, 0.0, 0.2);
		glVertex3d(edges[i].n1->x(), edges[i].n1->y(), edges[i].n1->z());

		if(isEdgeSelected)	glColor3f(1.0f, 0.2f, 0.2f);
		glVertex3d(edges[i].n2->x(), edges[i].n2->y(), edges[i].n2->z());
	}
	glEnd();

	glDisable(GL_BLEND);

	//======================
	// Draw Skeleton Nodes
	for(int i = 0; i < (int)nodes.size(); i++)
	{
		if(selectNodeStart == i)		glColor3d(0.4, 0.0, 0.5);
		else if(selectNodeEnd == i)		glColor3d(0.8, 0.3, 0.95);
		else if(selectedNodes[i])		glColor3d(0.9, 0.2, 0.2);
		else							glColor3d(0.7, 0.7, 0.7);

		glPointSize(7.0f);
		glBegin(GL_POINTS);
		glVertex3f(nodes[i].x(), nodes[i].y(), nodes[i].z());
		glEnd();

		// White Border
		glPointSize(10.0f);
		glColor3f(1,1,1);

		glBegin(GL_POINTS);
		glVertex3f(nodes[i].x(), nodes[i].y(), nodes[i].z());
		glEnd();
	}

	// Draw smooth skeletons if any
	glClear(GL_DEPTH_BUFFER_BIT);
	for(int i = 0; i < (int)smoothEdges.size(); i++)
	{
		glLineWidth(2.5f);
		glColor3f(0,0.6f,0);

		SkeletonNode * n1 = smoothEdges[i].n1;
		SkeletonNode * n2 = smoothEdges[i].n2;

		glBegin(GL_LINES);
		glVertex3f(n1->x(), n1->y(), n1->z());
		glVertex3f(n2->x(), n2->y(), n2->z());
		glEnd();

		glPointSize(8.0f);
		glBegin(GL_POINTS);
		glVertex3f(n1->x(), n1->y(), n1->z());
		glVertex3f(n2->x(), n2->y(), n2->z());
		glEnd();

		glPointSize(12.0f);
		glColor3f(0.8f, 0.9f, 0.8f);
		glBegin(GL_POINTS);
		glVertex3f(n1->x(), n1->y(), n1->z());
		glVertex3f(n2->x(), n2->y(), n2->z());
		glEnd();
	}

	glLineWidth(oldLineWidth);
	glColor3f(1,1,1);

	glEnable(GL_LIGHTING);
}
