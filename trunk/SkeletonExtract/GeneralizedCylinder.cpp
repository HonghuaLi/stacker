#include "GeneralizedCylinder.h"
#include "RMF.h"
#include "Intersection.h"
#include "ClosedPolygon.h"
#include "SimpleDraw.h"

GeneralizedCylinder::GeneralizedCylinder( Skeleton * skeleton, QSurfaceMesh * mesh )
{
	int numSteps = Max(skeleton->sortedSelectedNodes.size(), 10);
	std::vector<ResampledPoint> reSampledSpine = skeleton->resampleSmoothSelectedPath(numSteps, 3);

	std::vector<Point> reSampledSpinePoints;
	foreach(ResampledPoint sample, reSampledSpine) reSampledSpinePoints.push_back(sample.pos);

	// Build minimum rotation frames on this spine
	frames = RMF (reSampledSpinePoints);

	// Get faces structure
	skeleton->getSelectedFaces(true);
	std::map< uint, std::vector<Point> > face;
	foreach(uint fi, skeleton->lastSelectedFaces)
		face[fi] = mesh->facePoints(mesh->face_array[fi]);
	SkeletonGraph skel_graph = skeleton->getGraph();

	// Build cross-section
	for(uint i = 0; i < reSampledSpine.size(); i++)
	{
		// Find radius from the cross-section
		double radius = 0;
		std::vector<Point> cur_cs;
		
		ClosedPolygon polygon(frames.point[i]);

		foreach(uint fi, skeleton->lastSelectedFaces)
		{
			Vec3d p1, p2;

			if(ContourFacet(frames.U[i].t, frames.point[i], face[fi], p1, p2) > 0)
				polygon.insertLine(p1,p2);
		}

		polygon.close();

		// Sort based on distance and filter based on segment distance
		foreach(Point p, polygon.closedPoints)
			radius = Max(radius, (p - frames.point[i]).norm());
				
		crossSection.push_back(Circle(frames.point[i], radius, frames.U[i].t, i));
	}

	// Silly filter for cross-sections radius
	for(int j = 0; j < 2; j++)
	{
		for(uint i = 1; i < crossSection.size() - 1; i++)
		{
			if(crossSection[i].radius > crossSection[i+1].radius * 2)
				crossSection[i].radius = (crossSection[i-1].radius + crossSection[i+1].radius) / 2.0;
		}
	}

	src_mesh = mesh;
	isDrawFrames = false;

	printf("\nGC generated.\n");
}

void GeneralizedCylinder::draw()
{
	if(!frames.U.size()) return;

	glDisable(GL_DEPTH_TEST);

	foreach(Circle c, crossSection)
	{
		std::vector<Point> pnts = c.toSegments(20, frames.U[c.index].s);
		pnts.push_back(pnts.front());

		SimpleDraw::IdentifyConnectedPoints(pnts, 0,0,0);
	}

	foreach(Point p, debugPoints)
		SimpleDraw::IdentifyPoint(p);

	// Draw frames
	if(isDrawFrames)
	{
		std::vector<Point> dir1, dir2, dir3;
		for(uint i = 0; i < frames.U.size(); i++){
			dir1.push_back(frames.U[i].r);
			dir2.push_back(frames.U[i].s);
			dir3.push_back(frames.U[i].t);
		}
		SimpleDraw::DrawLineTick(frames.point, dir1, 0.1f, false, 1,0,0,1);
		SimpleDraw::DrawLineTick(frames.point, dir2, 0.1f, false, 0,1,0,1);
		SimpleDraw::DrawLineTick(frames.point, dir3, 0.1f, false, 0,0,1,1);
	}

	glEnable(GL_DEPTH_TEST);
}

std::vector<Point> GeneralizedCylinder::Circle::toSegments( int numSegments, const Vec3d& startVec, double delta )
{
	std::vector<Point> result;
	double theta = (2 * M_PI) / numSegments;
	qglviewer::Vec v(startVec.x(), startVec.y(), startVec.z());
	qglviewer::Quaternion q (qglviewer::Vec(n.x(), n.y(), n.z()), theta);
	qglviewer::Vec vi = v;

	for(int i = 0; i < numSegments; i++){
		result.push_back(center + (Point(vi.x, vi.y, vi.z) * radius * delta) );
		vi = q.rotate(vi).unit();
	}

	return result;
}
