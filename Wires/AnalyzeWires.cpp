#include "AnalyzeWires.h"

#include "Macros.h"
#include "Graph.h"
typedef Graph<uint, double> WireGraph;

#include "Ridges.h"

std::vector<Wire> AnalyzeWires::fromMesh( QSurfaceMesh * m, double dihedralThreshold )
{
	std::vector<Wire> wires;

	if(m == NULL)
		return wires;

	QSurfaceMesh::Face_property<Normal_> fnormals = m->face_property<Normal_>("f:normal");
	QSurfaceMesh::Edge_property<double> iswire = m->edge_property<double>("e:is_wire");
	QSurfaceMesh::Vertex_property<Point> v = m->vertex_property<Point>("v:point");
	QSurfaceMesh::Edge_iterator eit, eend = m->edges_end();

	QSurfaceMesh::Face f1, f2;
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
	QSurfaceMesh::Vertex vertex1, vertex2;

	// first wire
	WireGraph allWiresGraph;
	std::map<uint, QSurfaceMesh::Vertex> vmap;

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

std::vector<Wire> AnalyzeWires::fromMesh2( QSurfaceMesh * src_mesh, double dihedralThreshold )
{
	std::vector<Wire> wires;

	QSurfaceMesh::Vertex_property<Point> points = src_mesh->vertex_property<Point>("v:point");
	QSurfaceMesh::Vertex_property<Normal_> normals = src_mesh->vertex_property<Normal_>("v:point");
	QSurfaceMesh::Vertex_iterator vit, vend = src_mesh->vertices_end();

	// Differential properties
	QSurfaceMesh::Vertex_property<Vec3d> d1 = src_mesh->vertex_property<Vec3d>("v:d1");
	QSurfaceMesh::Vertex_property<Vec3d> d2 = src_mesh->vertex_property<Vec3d>("v:d2");
	QSurfaceMesh::Vertex_property<double> k1 = src_mesh->vertex_property<double>("v:k1");
	QSurfaceMesh::Vertex_property<double> k2 = src_mesh->vertex_property<double>("v:k2");
	QSurfaceMesh::Vertex_property<double> b0 = src_mesh->vertex_property<double>("v:b0");
	QSurfaceMesh::Vertex_property<double> b3 = src_mesh->vertex_property<double>("v:b3");
	QSurfaceMesh::Vertex_property<double> P1 = src_mesh->vertex_property<double>("v:P1");
	QSurfaceMesh::Vertex_property<double> P2 = src_mesh->vertex_property<double>("v:P2");

	// Visitation map
	src_mesh->resetVistedVertices(-1);

	size_t d_fitting = 3;
	size_t d_monge = 3;
	size_t min_nb_points = (d_fitting + 1) * (d_fitting + 2) / 2;

	// Local differential properties of sampled surfaces via polynomial fitting
	for(vit = src_mesh->vertices_begin(); vit != vend; ++vit)
	{
		std::vector<Surface_mesh::Vertex> neighbours;
		src_mesh->collectEnoughRings(vit, min_nb_points, neighbours);

		Monge_via_jet_fitting::Monge_form monge_form;
		Monge_via_jet_fitting monge_fit;

		// Convert vertex handle to point
		std::vector<Point> neighbour_points;
		for(std::vector<Surface_mesh::Vertex>::iterator it = neighbours.begin(); it != neighbours.end(); it++)
			neighbour_points.push_back(points[*it]);

		monge_form = monge_fit(neighbour_points.begin(), neighbour_points.end(), d_fitting, d_monge);

		//switch min-max ppal curv/dir wrt the mesh orientation
		const Vector_3 normal_mesh = normals[vit];
		monge_form.comply_wrt_given_normal(normal_mesh);

		//Store monge data needed for ridge computations in property maps
		d1[vit] = monge_form.maximal_principal_direction();
		d2[vit] = monge_form.minimal_principal_direction();
		k1[vit] = monge_form.coefficients()[0];
		k2[vit] = monge_form.coefficients()[1];
		b0[vit] = monge_form.coefficients()[2];
		b3[vit] = monge_form.coefficients()[5];

		if ( d_monge >= 4) {
			//= 3*b1^2+(k1-k2)(c0-3k1^3)
			P1[vit] =
				3*monge_form.coefficients()[3]*monge_form.coefficients()[3]
			+(monge_form.coefficients()[0]-monge_form.coefficients()[1])
				*(monge_form.coefficients()[6]
			-3*monge_form.coefficients()[0]*monge_form.coefficients()[0]
			*monge_form.coefficients()[0]);
			//= 3*b2^2+(k2-k1)(c4-3k2^3)
			P2[vit] =
				3*monge_form.coefficients()[4]*monge_form.coefficients()[4]
			+(-monge_form.coefficients()[0]+monge_form.coefficients()[1])
				*(monge_form.coefficients()[10]
			-3*monge_form.coefficients()[1]*monge_form.coefficients()[1]
			*monge_form.coefficients()[1]);
		}
	}

	return wires;
}
