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

	virtual std::vector <Vec3d> points() = 0;
	virtual double volume() = 0;

	int id;
	bool isSelected;
	QSurfaceMesh* getMesh(){ return m_mesh; }

	QSurfaceMesh*		m_mesh;			// The underlying geometry
	bool				isHot;			// Is this hot component?
	bool				isDirty;		// Has the underlying geometry been updated?
};