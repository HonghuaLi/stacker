#pragma once 

// Linear Solver ================
// WARNING: disabled SparseLib++ since we aren't using it in this project
/*#undef Vector

#define COMPLEX std::complex<double>

#include "compcol_double.h"
#include "mvblasd.h"

// Pre conditioners
#include "diagpre_double.h"
#include "icpre_double.h"
#include "ilupre_double.h"

// Solvers
#include "bicg.h"
#include "bicgstab.h"
#include "cg.h"
#include "cgs.h"

#define Vector std::vector*/
// End of Solver ================

#undef min
#undef max

#include "GraphicsLibrary/Mesh/QSurfaceMesh.h"

class Smoother{
private:
	static void treatBorders(Surface_mesh * m, std::vector<Surface_mesh::Vertex> & U);
	static void untreatBorders(Surface_mesh * m, std::vector<Surface_mesh::Vertex> & U);

public:
	static void LaplacianSmoothing(Surface_mesh * m, int numIteration, bool protectBorders = true);
	static Point LaplacianSmoothVertex(Surface_mesh * m, int vi);

	static void ScaleDependentSmoothing(Surface_mesh * m, int numIteration, double step_size = 0.5, bool protectBorders = true);
	static Point ScaleDependentSmoothVertex(Surface_mesh * m, Surface_mesh::Vertex v, double step_size = 0.5);

	static void MeanCurvatureFlow(QSurfaceMesh * m, double step, int numIteration, bool isVolumePreservation = true);
	static void MeanCurvatureFlowExplicit(QSurfaceMesh * m, int numIteration, double step = 0.5);
};
