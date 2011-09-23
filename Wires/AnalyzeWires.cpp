#include "AnalyzeWires.h"

#include "Macros.h"
#include "Graph.h"
typedef Graph<uint, double> WireGraph;

#include "Ridges.h"

std::vector<Wire> AnalyzeWires::fromMesh( QSurfaceMesh * src_mesh, double sharp_threshold, double strength_threshold )
{
	std::vector<Wire> wires;

	QSurfaceMesh::Vertex_property<Point> points = src_mesh->vertex_property<Point>("v:point");
	QSurfaceMesh::Vertex_property<Normal_> normals = src_mesh->vertex_property<Normal_>("v:normal");
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

	//min_nb_points *= 6;

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

	// Ridges
	Ridge_approximation ridge_approximation(src_mesh);
	std::vector<Ridge_line*> ridge_lines;
	
	back_insert_iterator<std::vector<Ridge_line*> > max_ridges(ridge_lines);
	back_insert_iterator<std::vector<Ridge_line*> > min_ridges(ridge_lines);
	back_insert_iterator<std::vector<Ridge_line*> > crest_ridges(ridge_lines);

	// Find MAX_RIDGE, MIN_RIDGE, CREST_RIDGES
	ridge_approximation.compute_ridges(max_ridges, MAX_RIDGE);
	ridge_approximation.compute_ridges(min_ridges, MIN_RIDGE);
	ridge_approximation.compute_ridges(crest_ridges, CREST_RIDGE);

	std::vector<Ridge_line*>::iterator iter_lines = ridge_lines.begin(), iter_end = ridge_lines.end();

	src_mesh->debug_lines.clear();
	src_mesh->debug_lines2.clear();
	src_mesh->debug_lines3.clear();

	for (;iter_lines != iter_end; iter_lines++)
	{
		Ridge_line * ridge_line = (*iter_lines);

		// Filter?
		if(ridge_line->sharpness() >= sharp_threshold && ridge_line->strength() >= strength_threshold)
		{
			std::list<Ridge_halfhedge>::iterator iter = ridge_line->line()->begin(), ite = ridge_line->line()->end();

			std::vector<Point> cur_line;

			for (; iter != ite; iter++)
			{
				//he: p->q, r is the crossing point
				Point p = points[src_mesh->to_vertex(iter->first)];
				Point q = points[src_mesh->from_vertex(iter->first)];
				Point r = Ridge_approximation::barycenter(q, iter->second, p);

				// R is cool point
				cur_line.push_back(r);
			}

			Ridge_type t = ridge_line->line_type();

			switch(t)
			{
			case MAX_ELLIPTIC_RIDGE: case MAX_HYPERBOLIC_RIDGE:
				src_mesh->debug_lines.push_back(cur_line);
				break;

			case MIN_ELLIPTIC_RIDGE: case MIN_HYPERBOLIC_RIDGE:
				src_mesh->debug_lines2.push_back(cur_line);
				break;

			case MAX_CREST_RIDGE: case MIN_CREST_RIDGE:
				src_mesh->debug_lines3.push_back(cur_line);
				break;
			}
		}
	}

	return wires;
}
