#include "AnalyzeWires.h"

#include "Macros.h"
#include "Graph.h"
typedef Graph<uint, double> WireGraph;

#include "Curvature.h"

std::vector<Wire> AnalyzeWires::fromMesh( Surface_mesh * m, double dihedralThreshold )
{
	std::vector<Wire> wires;

	if(m == NULL)
		return wires;

	Surface_mesh::Face_property<Normal_> fnormals = m->face_property<Normal_>("f:normal");
	Surface_mesh::Edge_property<double> iswire = m->edge_property<double>("e:is_wire");
	Surface_mesh::Vertex_property<Point> v = m->vertex_property<Point>("v:point");
	Surface_mesh::Edge_iterator eit, eend = m->edges_end();

	Surface_mesh::Face f1, f2;
	Normal_ n1, n2;

	// go over edges of mesh
	for(eit = m->edges_begin(); eit != eend; ++eit)
	{
		f1 = m->face(m->halfedge(eit, 0));
		f2 = m->face(m->halfedge(eit, 1));

		n1 = fnormals[f1];
		n2 = fnormals[f2];

		double di_angle = acos( RANGED(-1, dot(n1, n2), 1) );

		iswire[eit] = Max(0.01, di_angle);
	}

	uint vi1, vi2;
	Surface_mesh::Vertex vertex1, vertex2;

	// first wire
	WireGraph allWiresGraph;
	std::map<uint, Surface_mesh::Vertex> vmap;

	for(eit = m->edges_begin(); eit != eend; ++eit)
	{
		if( iswire[eit] > dihedralThreshold )
		{
			vertex1 = m->vertex(eit, 0);
			vertex2 = m->vertex(eit, 1);

			// indices
			vi1 = vertex1.idx();
			vi2 = vertex2.idx();

			allWiresGraph.AddEdge(vi1, vi2, iswire[eit]);

			// save in a map, for later
			vmap[vi1] = vertex1;
			vmap[vi2] = vertex2;
		}
	}

	std::vector< WireGraph > separateWiresGraph = allWiresGraph.toConnectedParts();

	for(std::vector< WireGraph >::iterator wg = separateWiresGraph.begin(); wg != separateWiresGraph.end(); wg++)
	{
		// Add empty wire
		wires.push_back(Wire());
	
		// Get edges
		std::vector<WireGraph::Edge> edges = wg->GetEdges();

		// Add edges
		for(std::vector<WireGraph::Edge>::iterator e = edges.begin(); e != edges.end(); e++)
		{
			uint pi1 = e->index;
			uint pi2 = e->target;

			wires.back().addEdge(v[vmap[pi1]], v[vmap[pi2]], pi1, pi2, e->weight);
		}
	}

	return wires;
}

std::vector<Wire> AnalyzeWires::fromMesh2( Surface_mesh * m, double dihedralThreshold )
{
	std::vector<Wire> wires;

	Curvature c;

	c.computeDerivativesCurvatures(m);

	return wires;
}
