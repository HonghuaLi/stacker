#include "GeneralizedCylinder.h"
#include "RMF.h"
#include "Intersection.h"
#include "ClosedPolygon.h"
#include "SimpleDraw.h"

GeneralizedCylinder::GeneralizedCylinder( Skeleton * skeleton, QSurfaceMesh * mesh )
{
	int numSteps = Max(skeleton->sortedSelectedNodes.size() * 0.75, 10);
	std::vector<ResampledPoint> reSampledSpine = skeleton->resampleSmoothSelectedPath(numSteps, 3);

	std::vector<Point> reSampledSpinePoints;
	foreach(ResampledPoint sample, reSampledSpine) reSampledSpinePoints.push_back(sample.pos);

	// Build minimum rotation frames on this spine
	frames = RMF(reSampledSpinePoints);

	// Get faces structure
	skeleton->getSelectedFaces(true);
	std::map< uint, std::vector<Point> > face;
	foreach(uint fi, skeleton->lastSelectedFaces)
		face[fi] = mesh->facePoints(mesh->face_array[fi]);
	SkeletonGraph skel_graph = skeleton->getGraph();

	// Build cross-section
	for(uint i = 0; i < reSampledSpine.size(); i++)
	{
		spine.push_back(frames.point[i]);

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

	printf("\nGC generated.\n");
}

void GeneralizedCylinder::draw()
{
	if(!spine.size()) return;

	glDisable(GL_DEPTH_TEST);

	foreach(Circle c, crossSection)
	{
		std::vector<Point> pnts = c.toSegments(20, frames.U[c.index].s);
		pnts.push_back(pnts.front());

		SimpleDraw::IdentifyConnectedPoints(pnts, 0,0,0);
		SimpleDraw::IdentifyPoint(spine[c.index]);
	}

	foreach(Point p, debugPoints)
		SimpleDraw::IdentifyPoint(p);

	glEnable(GL_DEPTH_TEST);
}

std::vector<Point> GeneralizedCylinder::Circle::toSegments( int numSegments, const Vec3d& startVec )
{
	std::vector<Point> result;
	double theta = (2 * M_PI) / numSegments;
	qglviewer::Vec v(startVec.x(), startVec.y(), startVec.z());

	for(int i = 0; i < numSegments; i++){
		qglviewer::Vec r = qglviewer::Quaternion (qglviewer::Vec(n.x(), n.y(), n.z()), theta * i).rotate(v).unit();
		result.push_back(center + (Point(r.x, r.y, r.z) * radius) );
	}

	return result;
}
