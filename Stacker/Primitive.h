#pragma once

#include "QSurfaceMesh.h"

class Primitive
{
public:
	Primitive(QSurfaceMesh* m_mesh);
	virtual ~Primitive(void);

	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit(){}

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh(){}

	// Visualize the primitive and potential actions
	virtual void draw(){}
	virtual void drawNames(){}

	int id;
	bool isSelected;
	QSurfaceMesh* getMesh(){ return m_mesh; }

protected:
	QSurfaceMesh*		m_mesh;			// The underlying geometry
	bool				isHot;			// Is this hot component?
	bool				isDirty;		// Has the underlying geometry been updated?
};

