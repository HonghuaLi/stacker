#include "VBO.h"

VBO::VBO( unsigned int vert_count, const PointType * v, const NormalType * n, const ColorType * c, StdVector<Index> faces )
{
	vertex_vbo_id = 0;
	Normalvbo_id = 0;
	color_vbo_id = 0;
	faces_id = 0;

	vCount = vert_count;

	vertices = v;
	normals = n;
	colors = c;

	indices = faces;

	// If VBO is supported, enable it
	if(GLEE_ARB_vertex_buffer_object)
		isVBOEnabled = true;
	else
		isVBOEnabled = false;

	isDirty = true;
	isReady = false;

	// Default rendering settings
	isDrawRegular = true;
	isDrawWireframe = false;
	isDrawAsPoints = false;
	isFlatShade = false;
}

VBO::~VBO()
{
	if(this->isVBOEnabled)
	{
		if(vertex_vbo_id)	free_vbo(vertex_vbo_id);
		if(Normalvbo_id)	free_vbo(Normalvbo_id);
		if(color_vbo_id)	free_vbo(color_vbo_id);
		if(faces_id)		free_vbo(faces_id);

		vertex_vbo_id = 0;
		Normalvbo_id = 0;
		color_vbo_id = 0;
		faces_id = 0;
	}
}

bool VBO::isVBOSupported()
{
	return GLEE_ARB_vertex_buffer_object;
}

void VBO::update()
{
	if(this->isVBOEnabled)
	{
		if(indices.size() > 0)
		{
			update_vbo(&vertex_vbo_id, vCount * sizeof(PointType), vertices);

			if(vertex_vbo_id) isReady = true;

			if(normals) update_vbo(&Normalvbo_id, vCount * sizeof(NormalType), normals);
			if(colors) update_vbo(&color_vbo_id, vCount * sizeof(ColorType), colors);

			update_ebo(&faces_id, indices.size() * sizeof(Index), &indices.front());	// ELEMENT_ARRAY case
		}
	}

	isDirty = false;
	isReady = true;
}

void VBO::update_vbo( Index *vbo, int vbo_size, const GLvoid *vbo_data )
{
	if(*vbo == 0)
		glGenBuffers(1, vbo);

	glBindBuffer(GL_ARRAY_BUFFER, *vbo);
	glBufferData(GL_ARRAY_BUFFER, vbo_size, vbo_data, GL_STREAM_DRAW);
}

void VBO::update_ebo( Index *ebo, int ebo_size, const GLvoid *ebo_data )
{
	if(*ebo == 0)
		glGenBuffers(1, ebo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ebo_size, ebo_data, GL_STREAM_DRAW);
}

void VBO::free_vbo( Index vbo )
{
	glDeleteBuffers(1, &vbo);
}

void VBO::render_regular( bool dynamic /*= false*/ )
{
	if(dynamic || isDirty)
		update();

	if(vertex_vbo_id == 0) return; // something went wrong..

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset( 1.0, 1.0 );

	// bind
	glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_id);
	glVertexPointer(3, GL_DOUBLE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, Normalvbo_id);
	glNormalPointer(GL_DOUBLE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, color_vbo_id);
	glColorPointer(4, GL_DOUBLE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_id);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	// draw
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, NULL);

	glDisable(GL_POLYGON_OFFSET_FILL);
}

void VBO::render_wireframe( bool dynamic /*= false*/ )
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

	glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_id);
	glVertexPointer(3, GL_DOUBLE, 0, NULL);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_id);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, NULL);

	glDisableClientState(GL_VERTEX_ARRAY);

	glPopClientAttrib();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_BLEND); 
	glEnable(GL_LIGHTING);
}

void VBO::render_vertices( bool dynamic /*= false*/ )
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

	glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_id);
	glVertexPointer(3, GL_DOUBLE, 0, NULL);
	glEnableClientState(GL_VERTEX_ARRAY);

	glPointSize(4.0f);
	glColor3f(0.33f, 0.33f, 0.94f);	// lighter blue

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_id);
	glDrawElements(GL_POINTS, indices.size(), GL_UNSIGNED_INT, NULL);

	glPointSize(pointSize);
	glColor4fv(color);
	glEnable(GL_LIGHTING);
}

void VBO::render_as_points( bool dynamic /*= false*/ )
{
	if(dynamic || isDirty)
		update();

	if(vertex_vbo_id == 0) return;

	GLfloat pointSize;
	glGetFloatv(GL_POINT_SIZE, &pointSize);

	glPointSize(6.0);

	// Bind vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo_id);
	glVertexPointer(3, GL_DOUBLE, 0, NULL);
	glEnableClientState(GL_VERTEX_ARRAY);

	// Bind normals
	glBindBuffer(GL_ARRAY_BUFFER, Normalvbo_id);
	glNormalPointer(GL_DOUBLE, 0, NULL);
	glEnableClientState(GL_NORMAL_ARRAY);

	// Bind colors
	glBindBuffer(GL_ARRAY_BUFFER, color_vbo_id);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, NULL);
	glEnableClientState(GL_COLOR_ARRAY);

	// Bind faces
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_id);

	// Draw all faces
	glDrawElements(GL_POINTS, indices.size(), GL_UNSIGNED_INT, NULL);

	glPopClientAttrib();
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glPointSize(pointSize);
}

void VBO::render( bool dynamic )
{
	if(!isVBOEnabled)
		return;

	if(dynamic || isDirty)
		update();

	if(isFlatShade) glShadeModel(GL_FLAT);

	// render
	if(isDrawRegular) render_regular();
	if(isDrawWireframe) render_wireframe();
	if(isDrawAsPoints) render_as_points();
}

void VBO::setDirty( bool state )
{
	isDirty = state;
}
