#include "Ridges.h"

Ridge_approximation::Ridge_approximation( QSurfaceMesh * m ) : M(m)
{
	//init the is_visited_map and check that the mesh is a triangular one.
	Surface_mesh::Face_iterator fit, fend = M->faces_end();

	for(fit = M->faces_begin(); fit != fend; ++fit) 
		is_visited_map[fit] = false;

	//Min_sphere_d<Optimisation_d_traits_3<typename Surface_mesh::Traits> > 
	//	min_sphere(M->points_begin(), M->points_end());
	squared_model_size = M->radius * M->radius;

	//maybe better to use Min_sphere_of_spheres_d ?? but need to create spheres?

	tag_order = Ridge_order_3;

	// Copy property maps
	k1 = M->get_vertex_property<double>("v:k1");
	k2 = M->get_vertex_property<double>("v:k2");

	d1 = M->get_vertex_property<Vec3d>("v:d1");
	d2 = M->get_vertex_property<Vec3d>("v:d2");

	b0 = M->get_vertex_property<double>("v:b0");
	b3 = M->get_vertex_property<double>("v:b3");

	P1 = M->get_vertex_property<double>("v:P1");
	P2 = M->get_vertex_property<double>("v:P2");
}

OutputIterator Ridge_approximation::compute_ridges( OutputIterator ridge_lines_it, Ridge_interrogation_type r_type, Ridge_order ord)
{
	tag_order = ord;

	//re-init the is_visited_map
	Surface_mesh::Face_iterator itb, ite = M->faces_end();
	for(itb = M->faces_begin(); itb != ite; ++itb) 
		is_visited_map[itb] = false;

	for(itb = M->faces_begin(); itb != ite; ++itb)
	{
		Surface_mesh::Face f = itb;

		if (is_visited_map.find(f)->second) 
			continue;

		is_visited_map.find(f)->second = true;

		Halfedge h1, h2, curhe1, curhe2, curhe;

		//h1 h2 are the hedges crossed if any, r_type should be
		//MAX_RIDGE, MIN_RIDGE or CREST_RIDGE ; cur_ridge_type should be
		//MAX_ELLIPTIC_RIDGE, MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE,
		//MIN_ELLIPTIC_RIDGE, MIN_HYPERBOLIC_RIDGE, MIN_CREST_RIDGE or NO_RIDGE
		Ridge_type cur_ridge_type = Face_ridge_type(f, h1, h2, r_type);

		if ( cur_ridge_type == NO_RIDGE )
			continue;

		// A ridge_line is beginning and stored
		Ridge_line* cur_ridge_line = new Ridge_line();
		init_ridge_line(cur_ridge_line, h1, h2, cur_ridge_type);
		*ridge_lines_it++ = cur_ridge_line;

		//next triangle adjacent to h1 (push_front)
		if ( !(M->is_boundary(h1)) ) 
		{
			f = M->face(M->opposite_halfedge(h1));
			curhe = h1;

			while (cur_ridge_type == Face_ridge_type(f,curhe1,curhe2,r_type))
			{
				//follow the ridge from curhe
				if (is_visited_map.find(f)->second) break;
				is_visited_map.find(f)->second = true;
				if (M->opposite_halfedge(curhe) == curhe1) curhe = curhe2;
				else curhe = curhe1;//curhe stays at the ridge extremity
				addfront(cur_ridge_line, curhe, cur_ridge_type);
				if ( !(M->is_boundary(curhe)) ) 
					f = M->face(M->opposite_halfedge(curhe));
				else break;
			}
			
			//exit from the while if
			//1. border or already visited (this is a ridge loop)
			//2. not same type, then do not set visited cause a MAX_ELLIPTIC_RIDGE
			//	  follows a MAX_HYPERBOLIC_RIDGE
		}

		//next triangle adjacent to h2 (push_back)
		if ( !(M->is_boundary(h2)) ) 
		{
			f = M->face(M->opposite_halfedge(h2));
			curhe = h2;

			while (cur_ridge_type == Face_ridge_type(f,curhe1,curhe2,r_type))
			{
				//follow the ridge from curhe
				if (is_visited_map.find(f)->second) break;
				is_visited_map.find(f)->second = true;
				if (M->opposite_halfedge(curhe) == curhe1) curhe = curhe2;
				else curhe = curhe1;
				addback(cur_ridge_line, curhe, cur_ridge_type);

				if ( !(M->is_boundary(curhe)) ) 
					f = M->face(M->opposite_halfedge(curhe));
				else break;
			}
		} 
	}
	return ridge_lines_it;
}

Ridge_type Ridge_approximation::Face_ridge_type(const Surface_mesh::Face f, Halfedge& he1, Halfedge& he2, 
	Ridge_interrogation_type r_type)
{
	if(f.idx() == -1)
		return NO_RIDGE;

	//polyhedral data
	//we have v1--h1-->v2--h2-->v3--h3-->v1
	const Halfedge h1 = M->halfedge(f);
	const Vertex v2 = M->to_vertex(h1);
	const Halfedge h2 = M->next_halfedge(h1);
	const Vertex v3 = M->to_vertex(h2);
	const Halfedge h3 = M->next_halfedge(h2);
	const Vertex v1 = M->to_vertex(h3);

	//check for regular Face
	//i.e. if there is a coherent orientation of ppal dir at the Face vertices
	if ( dot(d1[v1],d1[v2]) * dot(d1[v1],d1[v3]) * dot(d1[v2],d1[v3]) < 0 )
	{
		return NO_RIDGE;
	}

	//determine potential crest color
	//MAX_CREST_RIDGE if |sum(k1)|>|sum(k2)| sum over Face vertices vi
	//MIN_CREST_RIDGE if |sum(k1)|<|sum(k2)|
	Ridge_type crest_color = NO_RIDGE;
	if (r_type == CREST_RIDGE) 
	{
		if ( abs(k1[v1]+k1[v2]+k1[v3]) > abs(k2[v1]+k2[v2]+k2[v3]) ) 
			crest_color = MAX_CREST_RIDGE; 
		if ( abs(k1[v1]+k1[v2]+k1[v3]) < abs(k2[v1]+k2[v2]+k2[v3]) ) 
			crest_color = MIN_CREST_RIDGE;
		if ( abs(k1[v1]+k1[v2]+k1[v3]) == abs(k2[v1]+k2[v2]+k2[v3]) ) 
			return NO_RIDGE;
	}

	//compute Xing on the 3 edges
	bool h1_is_crossed = false, h2_is_crossed = false, h3_is_crossed = false;
	if ( r_type == MAX_RIDGE || crest_color == MAX_CREST_RIDGE ) 
	{
		xing_on_edge(h1, h1_is_crossed, MAX_RIDGE);
		xing_on_edge(h2, h2_is_crossed, MAX_RIDGE);
		xing_on_edge(h3, h3_is_crossed, MAX_RIDGE);
	}
	if ( r_type == MIN_RIDGE || crest_color == MIN_CREST_RIDGE ) 
	{
		xing_on_edge(h1, h1_is_crossed, MIN_RIDGE);
		xing_on_edge(h2, h2_is_crossed, MIN_RIDGE);
		xing_on_edge(h3, h3_is_crossed, MIN_RIDGE);
	}

	//there are either 0 or 2 crossed edges
	if ( !h1_is_crossed && !h2_is_crossed && !h3_is_crossed ) 
		return NO_RIDGE; 
	if (h1_is_crossed && h2_is_crossed && !h3_is_crossed)
	{
		he1 = h1; 
		he2 = h2;
	}
	if (h1_is_crossed && !h2_is_crossed && h3_is_crossed)
	{
		he1 = h1; 
		he2 = h3;
	}
	if (!h1_is_crossed && h2_is_crossed && h3_is_crossed)
	{
		he1 = h2; 
		he2 = h3;
	}

	//check there is no other case (just one edge crossed)
	//CGAL_postcondition ( !( (h1_is_crossed && !h2_is_crossed && !h3_is_crossed)
	//	|| (!h1_is_crossed && h2_is_crossed && !h3_is_crossed)
	//	|| (!h1_is_crossed && !h2_is_crossed && h3_is_crossed)) );
	bool is_elliptic;  

	if(M->is_valid(he1))
	{
		//There is a ridge segment in the triangle, determine its type elliptic/hyperbolic
		if ( r_type == MAX_RIDGE || crest_color == MAX_CREST_RIDGE ) 
			is_elliptic = tag_as_elliptic_hyperbolic(MAX_RIDGE, he1, he2);
		else 
			is_elliptic = tag_as_elliptic_hyperbolic(MIN_RIDGE, he1, he2);
	}
	else
		return NO_RIDGE;

	if (r_type == MAX_RIDGE) 
	{if (is_elliptic) return MAX_ELLIPTIC_RIDGE;
	else return MAX_HYPERBOLIC_RIDGE; }
	if (crest_color == MAX_CREST_RIDGE && is_elliptic) return MAX_CREST_RIDGE;

	if (r_type == MIN_RIDGE) 
	{if (is_elliptic) return MIN_ELLIPTIC_RIDGE;
	else return MIN_HYPERBOLIC_RIDGE; }
	if (crest_color == MIN_CREST_RIDGE && is_elliptic) return MIN_CREST_RIDGE;

	return NO_RIDGE;
}

void Ridge_approximation::xing_on_edge(const Halfedge he, bool& is_crossed, Ridge_interrogation_type color)
{
	is_crossed = false;
	double sign = 0;
	double b_p, b_q; // extremalities at p and q for he: p->q

	Vertex vp = M->from_vertex(he);
	Vertex vq = M->to_vertex(he);

	//ppal dir
	Vector_3  d_p = d1[vp];
	Vector_3  d_q = d1[vq];

	if ( color == MAX_RIDGE ) 
	{
		b_p = b0[vp];
		b_q = b0[vq];
	}
	else 
	{     
		b_p = b3[vp];
		b_q = b3[vq];
	}

	if ( b_p == 0 && b_q == 0 ) return;
	if ( b_p == 0 && b_q !=0 ) sign = dot(d_p,d_q) * b_q;
	if ( b_p != 0 && b_q ==0 ) sign = dot(d_p,d_q) * b_p;
	if ( b_p != 0 && b_q !=0 ) sign = dot(d_p,d_q) * b_p * b_q;
	if ( sign < 0 ) is_crossed = true;
}

bool Ridge_approximation::tag_as_elliptic_hyperbolic(const Ridge_interrogation_type color, const Halfedge he1, const Halfedge he2)
{
	// hei: pi->qi
	const Vertex v_p1 = M->from_vertex(he1);
	const Vertex v_q1 = M->to_vertex(he1);
	const Vertex v_p2 = M->from_vertex(he2);
	const Vertex v_q2 = M->to_vertex(he2); 

	double coord1, coord2;
	if (color == MAX_RIDGE) 
	{
		coord1 = abs(b0[v_q1]) / ( abs(b0[v_p1]) + abs(b0[v_q1]) );
		coord2 = abs(b0[v_q2]) / ( abs(b0[v_p2]) + abs(b0[v_q2]) ); 
	}
	else 
	{
		coord1 = abs(b3[v_q1]) / ( abs(b3[v_p1]) + abs(b3[v_q1]) );
		coord2 = abs(b3[v_q2]) / ( abs(b3[v_p2]) + abs(b3[v_q2]) ); 
	}

	if ( tag_order == Ridge_order_3 ) 
	{
		Vector_3 r1 = barycenter(M->getVertexPos(v_p1), coord1, M->getVertexPos(v_q1));
		Vector_3 r2 = barycenter(M->getVertexPos(v_p2), coord2, M->getVertexPos(v_q2));

		//identify the 3 different vertices v_p1, v_q1 and v3 = v_p2 or v_q2
		Vertex v3;
		if (v_p2 == v_p1 || v_p2 == v_q1) v3 = v_q2;
		else v3 = v_p2;

		int b_sign = b_sign_pointing_to_ridge(v_p1, v_q1, v3, r1, r2, color); 

		if (color == MAX_RIDGE) 
			if (b_sign == 1) return true; 
			else return false;
		else if (b_sign == -1) return true; 
		else return false;
	}
	else {//tag_order == Ridge_order_4, check the sign of the meanvalue of the signs
		//      of Pi at the two crossing points
		double sign_P;
		if (color == MAX_RIDGE) 
			sign_P =  P1[v_p1]*coord1 + P1[v_q1]*(1-coord1) 
			+ P1[v_p2]*coord2 + P1[v_q2]*(1-coord2);
		else sign_P =  P2[v_p1]*coord1 + P2[v_q1]*(1-coord1) 
			+ P2[v_p2]*coord2 + P2[v_q2]*(1-coord2);

		if ( sign_P < 0 ) return true; else return false;
	}
}

int Ridge_approximation::b_sign_pointing_to_ridge(const Vertex v1, const Vertex v2, const Vertex v3,
	const Vector_3 r1, const Vector_3 r2,  const Ridge_interrogation_type color)
{
	Vector_3 r = r2 - r1, dv1, dv2, dv3;
	double bv1, bv2, bv3;
	if ( color == MAX_RIDGE ) {
		bv1 = b0[v1];
		bv2 = b0[v2];
		bv3 = b0[v3];
		dv1 = d1[v1];
		dv2 = d1[v2];
		dv3 = d1[v3];
	}
	else {
		bv1 = b3[v1];
		bv2 = b3[v2];
		bv3 = b3[v3];
		dv1 = d2[v1];
		dv2 = d2[v2];
		dv3 = d2[v3];    
	}
	if ( r != NULL_VECTOR ) r = r/sqrt(dot(r,r));

	Point p1 = M->getVertexPos(v1);
	Point p2 = M->getVertexPos(v2);
	Point p3 = M->getVertexPos(v3);

	double sign1, sign2, sign3;

	sign1 = bv1 * dot((r1 - p1 + dot((p1-r1),r)*r ),dv1);
	sign2 = bv2 * dot((r1 - p2 + dot((p2-r1),r)*r ),dv2);
	sign3 = bv3 * dot((r1 - p3 + dot((p3-r1),r)*r ),dv3);

	int compt = 0;
	if ( sign1 > 0 ) compt++; else if (sign1 < 0) compt--;
	if ( sign2 > 0 ) compt++; else if (sign2 < 0) compt--;
	if ( sign3 > 0 ) compt++; else if (sign3 < 0) compt--;

	if (compt > 0) return 1; else return -1;
}

void Ridge_approximation::init_ridge_line(Ridge_line* ridge_line, const Halfedge h1, const Halfedge h2, const Ridge_type r_type)
{
	ridge_line->line_type() = r_type;
	ridge_line->line()->push_back(Ridge_halfhedge(h1, bary_coord(h1,r_type)));
	addback(ridge_line, h2, r_type);
}

void Ridge_approximation::addback(Ridge_line* ridge_line, const Halfedge he,	const Ridge_type r_type)
{
	Halfedge he_cur = ( --(ridge_line->line()->end()) )->first;

	double coord_cur = ( --(ridge_line->line()->end()) )->second;	//bary_coord(he_cur);
	double coord = bary_coord(he,r_type);

	// he: p->q
	Vertex v_p = M->from_vertex(he), v_q = M->to_vertex(he);
	Vertex v_p_cur = M->from_vertex(he_cur), v_q_cur = M->to_vertex(he_cur); 

	Point p = M->getVertexPos(v_p), q = M->getVertexPos(v_q);
	Point p_cur = M->getVertexPos(v_p_cur), q_cur = M->getVertexPos(v_q_cur);

	Vector_3 segment = barycenter(p, coord, q) - barycenter(p_cur, coord_cur, q_cur);

	double k1x, k2x;	 //abs value of the ppal curvatures at the Xing point on he.
	double k_second = 0; // abs value of the second derivative of the curvature along the line of curvature

	k1x = abs(k1[v_p]) * coord + abs(k1[v_q]) * (1-coord) ;   
	k2x = abs(k2[v_p]) * coord + abs(k2[v_q]) * (1-coord) ;   

	if ( (ridge_line->line_type() == MAX_ELLIPTIC_RIDGE) || (ridge_line->line_type() == MAX_HYPERBOLIC_RIDGE) || (ridge_line->line_type() == MAX_CREST_RIDGE) ) 
	{
		ridge_line->strength() += k1x * sqrt(dot(segment,segment)); 
		if (tag_order == Ridge_order_4) 
		{ 
			if (k1x != k2x) 
				k_second = abs(( abs(P1[v_p]) * coord + abs(P1[v_q]) * (1-coord) )/(k1x-k2x));
			ridge_line->sharpness() += k_second * sqrt(dot(segment,segment)) * squared_model_size; 
		}
	}

	if ( (ridge_line->line_type() == MIN_ELLIPTIC_RIDGE) || (ridge_line->line_type() == MIN_HYPERBOLIC_RIDGE) || (ridge_line->line_type() == MIN_CREST_RIDGE) ) 
	{
		ridge_line->strength() += k2x * sqrt(dot(segment,segment)); 
		if (tag_order == Ridge_order_4) 
		{
			if (k1x != k2x) 
				k_second = abs(( abs(P2[v_p]) * coord + abs(P2[v_q]) * (1-coord) )/(k1x-k2x));
			ridge_line->sharpness() += k_second * sqrt(dot(segment,segment)) * squared_model_size;
		}
	} 

	ridge_line->line()->push_back( Ridge_halfhedge(he, coord));
}

void Ridge_approximation::addfront(Ridge_line* ridge_line,  const Halfedge he, const Ridge_type r_type)
{
	Halfedge he_cur = ( ridge_line->line()->begin() )->first;
	double coord_cur = ( ridge_line->line()->begin() )->second;
	double coord = bary_coord(he,r_type);

	// he: p->q
	Vertex v_p = M->from_vertex(he), v_q = M->to_vertex(he);
	Vertex v_p_cur = M->from_vertex(he_cur), v_q_cur = M->to_vertex(he_cur); 

	Point p = M->getVertexPos(v_p), q = M->getVertexPos(v_q);
	Point p_cur = M->getVertexPos(v_p_cur), q_cur = M->getVertexPos(v_q_cur);

	Vector_3 segment = barycenter(p, coord, q) - barycenter(p_cur, coord_cur, q_cur);

	double k1x, k2x;		// abs value of the ppal curvatures at the Xing point on he.
	double k_second = 0.0;	// abs value of the second derivative of the curvature along the line of curvature

	k1x = abs(k1[v_p]) * coord + abs(k1[v_q]) * (1-coord) ;
	k2x = abs(k2[v_p]) * coord + abs(k2[v_q]) * (1-coord) ;

	if ( (ridge_line->line_type() == MAX_ELLIPTIC_RIDGE) || (ridge_line->line_type() == MAX_HYPERBOLIC_RIDGE) || (ridge_line->line_type() == MAX_CREST_RIDGE) ) 
	{
		ridge_line->strength() += k1x * sqrt(dot(segment,segment)); 
		if (tag_order == Ridge_order_4) 
		{
			if (k1x != k2x) 
				k_second = abs(( abs(P1[v_p]) * coord + abs(P1[v_q]) * (1-coord) )/(k1x-k2x));
			ridge_line->sharpness() += k_second * sqrt(dot(segment,segment)) * squared_model_size; 
		}
	}
	if ( (ridge_line->line_type() == MIN_ELLIPTIC_RIDGE) || (ridge_line->line_type() == MIN_HYPERBOLIC_RIDGE) || (ridge_line->line_type() == MIN_CREST_RIDGE) ) 
	{
		ridge_line->strength() += k2x * sqrt(dot(segment,segment)); 
		if (tag_order == Ridge_order_4) 
		{
			if (k1x != k2x) 
				k_second = abs(( abs(P2[v_p]) * coord + abs(P2[v_q]) * (1-coord) )/(k1x-k2x));
			ridge_line->sharpness() += k_second * sqrt(dot(segment,segment)) * squared_model_size; 
		}
	} 
	ridge_line->line()->push_front( Ridge_halfhedge(he, coord));
}

Vector_3 Ridge_approximation::barycenter( Vector_3 p, double w, Vector_3 q )
{
	return (w*p) + ((1.0 - w)*q);
}

double Ridge_approximation::bary_coord(const Halfedge he, const Ridge_type r_type)
{
	double b_p = 0.0, b_q = 0.0; // extremalities at p and q for he: p->q

	if ( (r_type == MAX_ELLIPTIC_RIDGE) || (r_type == MAX_HYPERBOLIC_RIDGE) || (r_type == MAX_CREST_RIDGE) ) 
	{
		b_p = b0[M->from_vertex(he)];
		b_q = b0[M->to_vertex(he)];    
	}

	if ( (r_type == MIN_ELLIPTIC_RIDGE) || (r_type == MIN_HYPERBOLIC_RIDGE) || (r_type == MIN_CREST_RIDGE) ) 
	{
		b_p = b3[M->from_vertex(he)];
		b_q = b3[M->to_vertex(he)];    
	}

	//denominator cannot be 0 since there is no crossing when both extremalities are 0
	return abs(b_q) / ( abs(b_q) + abs(b_p) );
}

std::vector<Ridge_line*> Ridge_approximation::get_ridges(QSurfaceMesh * m)
{
	// Initialize ridge approximation
	Ridge_approximation r_approx(m);

	// Setup output
	std::vector<Ridge_line*> ridge_lines;
	back_insert_iterator<std::vector<Ridge_line*> > ii(ridge_lines);

	// Compute
	r_approx.compute_ridges(ii);

	return ridge_lines;
}
