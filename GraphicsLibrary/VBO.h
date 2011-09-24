#pragma once

#include "Macros.h"

// Surface-mesh
#include "Vector.h"

typedef Vec3d PointType;
typedef Vec3d NormalType;
typedef Vec4d ColorType;
typedef uint Index;

class VBO
{
	unsigned int vertex_vbo_id;
	unsigned int Normalvbo_id;
	unsigned int color_vbo_id;
	unsigned int faces_id;

public:
	std::string objectId;
	int vCount;

	const PointType * vertices;
	const NormalType * normals;
	const ColorType * colors;
	StdVector<Index> indices;

	VBO(){ isVBOEnabled = false; isDirty = false; isReady = false; };
	VBO( unsigned int vert_count, const PointType * v, const NormalType * n, const ColorType * c, StdVector<Index> faces );

	void free_vbo(Index vbo);
	~VBO();

	void update();
	void update_vbo(Index *vbo, int vbo_size, const GLvoid *vbo_data);
	void update_ebo(Index *ebo, int ebo_size, const GLvoid *ebo_data);

	// Rendering Vertex Buffer Object (VBO)
	void render_regular(bool dynamic = false);
	void render_wireframe(bool dynamic = false);
	void render_vertices(bool dynamic = false);
	void render_as_points(bool dynamic = false);
	void render(bool dynamic = false);

	// State of VBO
	bool isDirty;
	void setDirty(bool state);
	bool isReady;
	bool isVBOEnabled;

	// Rendering flags
	bool isDrawRegular;
	bool isDrawWireframe;
	bool isDrawAsPoints;
	bool isFlatShade;
};
