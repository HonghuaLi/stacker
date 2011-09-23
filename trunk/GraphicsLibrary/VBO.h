#pragma once

// Legacy
#include "Point.h"
#include "Face.h"

// Surface-mesh
#include "Surface_mesh.h"

template<class PointType = Point3D, class NormalType = Normal, class ColorType = Color4, class FaceType = Face>
class VBO
{
	unsigned int vertex_vbo_id;
	unsigned int normal_vbo_id;
	unsigned int color_vbo_id;
	unsigned int faces_id;

public:
	int vCount;

	const PointType * vertices;
	const NormalType * normals;
	const ColorType * colors;
	Vector<Index> * indices;

	VBO(Vector<PointType> * v, Vector<NormalType> * n, Vector<ColorType> * c, StdList<FaceType> * f)
	{
		vertex_vbo_id = 0;
		normal_vbo_id = 0;
		color_vbo_id = 0;
		faces_id = 0;

		isReady = false;

		int num_of_indices = f->size() * 3;

		vCount = v->size();

		// Reserve space in memory
		vertices = &v->front();
		normals = &n->front();
		colors = &c->front();
		indices = new Vector<Index>(num_of_indices);

		int vIndex;

		// Fill in index array
		for( StdList<Face>::iterator face = f->begin(); face != f->end(); face++ )
			for(vIndex = 0; vIndex < 3; vIndex++)
				(*indices)[(3 * face->index) + vIndex] = face->vIndex[vIndex];

		// If VBO is supported, enable it
		if(GLEE_ARB_vertex_buffer_object)
		{
			isVBOEnabled = true;
			update();
		}
		else
			isVBOEnabled = false;

		isDirty = false;
	}

	VBO(unsigned int vert_count, const PointType * v, const NormalType * n, const ColorType * c, Vector<Index> f)
	{
		vertex_vbo_id = 0;
		normal_vbo_id = 0;
		color_vbo_id = 0;
		faces_id = 0;

		vCount = vert_count;

		vertices = v;
		normals = n;
		colors = c;

		indices = new Vector<Index>(f.size());
		*indices = f;

		// If VBO is supported, enable it
		if(GLEE_ARB_vertex_buffer_object)
		{
			isVBOEnabled = true;
			update();
		}
		else
			isVBOEnabled = false;

		isDirty = false;
	}

	~VBO()
	{
		if(this->isVBOEnabled)
		{
			if(vertex_vbo_id)	free_vbo(vertex_vbo_id);
			if(normal_vbo_id)	free_vbo(normal_vbo_id);
			if(color_vbo_id)	free_vbo(color_vbo_id);
			if(faces_id)		free_vbo(faces_id);

			vertex_vbo_id = 0;
			normal_vbo_id = 0;
			color_vbo_id = 0;
			faces_id = 0;

			delete indices;
		}
	}

	void update()
	{
		if(this->isVBOEnabled)
		{
			if(indices->size() > 0)
			{
				update_vbo(&vertex_vbo_id, vCount * sizeof(PointType), vertices);

				if(vertex_vbo_id) isReady = true;

				if(normals) update_vbo(&normal_vbo_id, vCount * sizeof(NormalType), normals);
				if(colors) update_vbo(&color_vbo_id, vCount * sizeof(ColorType), colors);

				update_ebo(&faces_id, indices->size() * sizeof(Index), &indices->front());	// ELEMENT_ARRAY case
			}
		}

		isDirty = false;
	}

	void update_vbo(Index *vbo, int vbo_size, const GLvoid *vbo_data) {
		if(*vbo == 0)
			glGenBuffersARB(1, vbo);

		glBindBufferARB(GL_ARRAY_BUFFER_ARB, *vbo);
		glBufferDataARB(GL_ARRAY_BUFFER_ARB, vbo_size, vbo_data, GL_DYNAMIC_DRAW_ARB);
	}

	void update_ebo(Index *ebo, int ebo_size, const GLvoid *ebo_data) {
		if(*ebo == 0)
			glGenBuffersARB(1, ebo);

		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, *ebo);
		glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB, ebo_size, ebo_data, GL_DYNAMIC_DRAW_ARB);
	}

	void free_vbo(Index vbo) 
	{
		glDeleteBuffersARB(1, &vbo);
	}

	// Rendering Vertex Buffer Object (VBO)
	void render_smooth(bool dynamic = false)
	{
		if(dynamic || isDirty)
			update();

		if(vertex_vbo_id == 0) return; // something went wrong..

		glEnable(GL_LIGHTING);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset( 1.0, 1.0 );

		glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);

		// Bind vertex positions
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertex_vbo_id);
		glVertexPointer(3, GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_VERTEX_ARRAY);

		// Bind normals
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, normal_vbo_id);
		glNormalPointer(GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_NORMAL_ARRAY);

		// Bind colors
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, color_vbo_id);
		glColorPointer(4, GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_COLOR_ARRAY);

		// Bind faces
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, faces_id);

		// Draw all faces
		glDrawElements(GL_TRIANGLES, indices->size(), GL_UNSIGNED_INT, NULL);

		glPopClientAttrib();

		glBindBuffer(GL_ARRAY_BUFFER_ARB, 0);

		glDisable(GL_POLYGON_OFFSET_FILL);
	}

	void render_wireframe(bool dynamic = false)
	{
		if(dynamic || isDirty)
		{
			update_vbo(&vertex_vbo_id, vCount * sizeof(PointType), vertices);
			isDirty = false;
		}

		if(vertex_vbo_id == 0) return;

		glLineWidth(1.0f); // default?

		glDisable(GL_LIGHTING);
		glEnable(GL_BLEND); 
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glColor4f(0, 0.6f, 0.2f, 0.5);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);

		glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertex_vbo_id);
		glVertexPointer(3, GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_VERTEX_ARRAY);

		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, faces_id);
		glDrawElements(GL_TRIANGLES, indices->size(), GL_UNSIGNED_INT, NULL);

		glPopClientAttrib();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDisable(GL_BLEND); 
		glEnable(GL_LIGHTING);
	}

	void render_vertices(bool dynamic = false)
	{
		if(dynamic || isDirty)
		{
			update_vbo(&vertex_vbo_id, vCount*sizeof(PointType), vertices);
			isDirty = false;
		}

		if(vertex_vbo_id == 0) return;

		glDisable(GL_LIGHTING);
		GLfloat pointSize;
		GLfloat color[4];
		glGetFloatv(GL_CURRENT_COLOR, color);
		glGetFloatv(GL_POINT_SIZE, &pointSize);

		glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertex_vbo_id);
		glVertexPointer(3, GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_VERTEX_ARRAY);

		glPointSize(4.0f);
		glColor3f(0.33f, 0.33f, 0.94f);	// lighter blue

		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, faces_id);
		glDrawElements(GL_POINTS, indices->size(), GL_UNSIGNED_INT, NULL);

		glPointSize(pointSize);
		glColor4fv(color);
		glEnable(GL_LIGHTING);
	}

	void render_as_points(bool dynamic = false)
	{
		if(dynamic || isDirty)
			update();

		if(vertex_vbo_id == 0) return;

		GLfloat pointSize;
		glGetFloatv(GL_POINT_SIZE, &pointSize);

		glPointSize(6.0);

		// Bind vertex positions
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertex_vbo_id);
		glVertexPointer(3, GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_VERTEX_ARRAY);

		// Bind normals
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, normal_vbo_id);
		glNormalPointer(GL_DOUBLE, 0, NULL);
		glEnableClientState(GL_NORMAL_ARRAY);

		// Bind colors
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, color_vbo_id);
		glColorPointer(4, GL_UNSIGNED_BYTE, 0, NULL);
		glEnableClientState(GL_COLOR_ARRAY);

		// Bind faces
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, faces_id);

		// Draw all faces
		glDrawElements(GL_POINTS, indices->size(), GL_UNSIGNED_INT, NULL);

		glPopClientAttrib();
		glBindBuffer(GL_ARRAY_BUFFER_ARB, 0);
		glDisable(GL_POLYGON_OFFSET_FILL);

		glPointSize(pointSize);
	}

	bool isDirty;
	inline void setDirty(bool state) {isDirty = state;}

	bool isReady;

	bool isVBOEnabled;
};
