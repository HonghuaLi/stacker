#pragma once

#include <list>
#include <map>
#include <iterator>
#include <vector>
#include "QSurfaceMesh.h"
#include "Monge_via_jet_fitting.h"

using namespace std;

enum Ridge_interrogation_type {MAX_RIDGE, MIN_RIDGE, CREST_RIDGE};

enum Ridge_type {NO_RIDGE = 0, 
	MAX_ELLIPTIC_RIDGE, MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE, 
	MIN_ELLIPTIC_RIDGE, MIN_HYPERBOLIC_RIDGE, MIN_CREST_RIDGE};

// Are ridges tagged as elliptic or hyperbolic using 3rd or 4th order differential quantities?
// with Ridge_order_3 P1 and P2 are not used and the sharpness is not defined.
enum Ridge_order {Ridge_order_3 = 3, Ridge_order_4 = 4};

// CGAL translations
typedef Point Vector_3;
typedef Surface_mesh::Vertex_property<double> VertexdoublePropertyMap;
typedef Surface_mesh::Vertex_property<Vector_3> VertexVectorPropertyMap;
typedef Surface_mesh::Halfedge Halfedge;
typedef Surface_mesh::Vertex Vertex;

// Ridges stuff
typedef std::pair<Halfedge, double>    Ridge_halfhedge;
#define NULL_VECTOR Vector_3()

//---------------------------------------------------------------------------
// Ridge_line : a connected sequence of edges of a
// Surface_Mesh crossed by a ridge (with a barycentric coordinate to compute the crossing point),
// with a Ridge_type and weights : strength and sharpness. Note
// sharpness is only available (more precisely only meaningful)
// if the Ridge_approximation has been computed with the Ridge_order Ridge_order_4.
// (else, if it is computed with Ridge_order_3 it keeps its initial value 0)
//--------------------------------------------------------------------------
class Ridge_line
{
public:
	Ridge_type line_type() const {return m_line_type;}
	Ridge_type& line_type() {return m_line_type;}

	const double strength() const {return m_strength;}
	double& strength() {return m_strength;}

	const double sharpness() const {return m_sharpness;}
	double& sharpness() {return m_sharpness;}

	const std::list<Ridge_halfhedge>* line() const { return &m_line;}
	std::list<Ridge_halfhedge>* line() { return &m_line;}

	//constructor
	Ridge_line() : m_strength(0), m_sharpness(0) {};

protected:
	Ridge_type m_line_type;

	std::list<Ridge_halfhedge> m_line;

	double m_strength; 	// = integral of ppal curvature along the line
	double m_sharpness; 	// = (integral of second derivative of curvature along the line) 
	// multiplied by the squared of the size of the model (which is 
	// the radius of the smallest enclosing ball)
};

typedef back_insert_iterator< std::vector<Ridge_line*> > OutputIterator;

//---------------------------------------------------------------------------
//Ridge_approximation
//--------------------------------------------------------------------------
class Ridge_approximation
{
public:  
	Ridge_approximation(QSurfaceMesh * m);

	//Find MAX_RIDGE, MIN_RIDGE or CREST_RIDGE ridges iterate on P Faces,
	//find a non-visited, regular (i.e. if there is a coherent
	//orientation of ppal dir at the Face vertices), 2Xing triangle,
	//follow non-visited, regular, 2Xing triangles in both sens to
	//create a Ridge line.  Each time an edge is added the strength and
	//sharpness(if Ridge_order_4) are updated.
	OutputIterator compute_ridges(OutputIterator ridge_lines_it, 
		Ridge_interrogation_type r_type = MAX_RIDGE, Ridge_order ord = Ridge_order_3);

protected:

	QSurfaceMesh * M;
	double squared_model_size;//squared radius of the smallest enclosing sphere of the Surface_mesh
	//used to make the sharpness scale independent and iso independent
	Ridge_order tag_order;

	//tag to visit faces
	struct Face_cmp{ //comparison is wrt Face addresses
		bool operator()(Surface_mesh::Face a,  Surface_mesh::Face b) const{ return a.idx() < b.idx(); }
	};
	typedef std::map<Surface_mesh::Face, bool, Face_cmp> Face2bool_map_type;
	Face2bool_map_type is_visited_map;

	//Property maps
	VertexdoublePropertyMap k1, k2, b0, b3, P1, P2;
	VertexVectorPropertyMap d1, d2;

	//is a Face crossed by a BLUE, RED or CREST_RIDGE ridge? if so, return
	//the crossed edges and more precise type from MAX_ELLIPTIC_RIDGE,
	//MAX_HYPERBOLIC_RIDGE, MAX_CREST_RIDGE, MIN_ELLIPTIC_RIDGE,
	//MIN_HYPERBOLIC_RIDGE, MIN_CREST_RIDGE or NO_RIDGE
	Ridge_type Face_ridge_type(const Surface_mesh::Face f, Halfedge& he1, Halfedge& he2, Ridge_interrogation_type r_type);
	
	//is an edge crossed by a BLUE/RED ridge? (color is MAX_RIDGE or
	//MIN_RIDGE ).  As we only test edges of regular triangles, the ppal
	//direction at endpoints d_p and d_q cannot be orthogonal. If both
	//extremalities vanish, we consider no crossing occurs. If only one
	//of them vanishes, we consider it as an positive infinitesimal and
	//apply the general rule. The general rule is that for both
	//non-vanishing extremalities, a crossing occurs if their sign
	//differ; Assuming the accute rule to orient the ppal directions,
	//there is a crossing iff d_p.d_q * b_p*b_q < 0
	void xing_on_edge(const Halfedge he, bool& is_crossed, Ridge_interrogation_type color);

	//given a ridge segment of a given color, in a triangle crossing he1
	//(v_p1 -> v_q1) and he2 (v_p2 -> v_q2) return true if it is
	//elliptic, false if it is hyperbolic.
	bool tag_as_elliptic_hyperbolic(const Ridge_interrogation_type color, const Halfedge he1, const Halfedge he2);

	// for the computation with tag_order == 3 only
	// for a ridge segment [r1,r2] in a triangle (v1,v2,v3), let r = r2 -
	// r1 and normalize, the projection of a point p on the line (r1,r2)
	// is pp=r1+tr, with t=(p-r1)*r then the vector v starting at p is
	// pointing to the ridge line (r1,r2) if (pp-p)*v >0. Return the sign
	// of b, for a ppal direction pointing to the ridge segment,
	// appearing at least at two vertices of the Face.
	//
	//  for color = MAX_RIDGE, sign = 1 if MAX_ELLIPTIC_RIDGE, -1 if
	//  MAX_HYPERBOLIC_RIDGE 
	//
	//  for color = MIN_RIDGE, sign = -1 if MIN_ELLIPTIC_RIDGE, 1 if
	//  MIN_HYPERBOLIC_RIDGE
	int b_sign_pointing_to_ridge(const Vertex v1, const Vertex v2, const Vertex v3, 
		const Vector_3 r1, const Vector_3 r2, 	const Ridge_interrogation_type color);

	//a ridge line begins with a segment in a triangle given by the 2 he
	//crossed
	void init_ridge_line(Ridge_line* ridge_line, const Halfedge h1, const Halfedge h2, const Ridge_type r_type);


	// When the line is extended with a he, the bary coord of the
	// crossing point is computed, the pair (he,coord) is added and the
	// weights are updated 
	void addback(Ridge_line* ridge_line, const Halfedge he, const Ridge_type r_type);
	void addfront(Ridge_line* ridge_line, const Halfedge he, const Ridge_type r_type);

public:
	// compute the barycentric coordinate of the xing point (blue or red)
	// for he: p->q (wrt the extremality values b0/3).  coord is st
	// xing_point = coord*p + (1-coord)*q
	double bary_coord(const Halfedge he, const Ridge_type r_type);
	static Vector_3 barycenter( Vector_3 p, double w, Vector_3 q );

	static std::vector<Ridge_line*> get_ridges(QSurfaceMesh * m);
};
