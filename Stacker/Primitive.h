#pragma once

#include "QSurfaceMesh.h"
#include "PrimativeParam.h"

class Primitive
{
public:
	Primitive(QSurfaceMesh* m_mesh);

	// Fit primitive to the underlying QSurfaceMesh
	virtual void fit() = 0;

	// Deform the underlying geometry according to the \pre_state and current state
	virtual void deformMesh() = 0;
	virtual void deform( PrimitiveParam* params, bool isPermanent = false) = 0;

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
