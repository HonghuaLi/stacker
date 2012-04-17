#include "JointDetector.h"


#include "Primitive.h"
#include "PointJointGroup.h"
#include "StackerGlobal.h"



QVector<Group*> JointDetector::detect( QMap<QString, Primitive*> primitives )
{
	QVector<Group*> Joints;
	std::vector<Voxeler> voxels;

	foreach(Primitive * prim, primitives)
	{
		QSurfaceMesh mesh(prim->getGeometry());
		voxels.push_back( Voxeler(&mesh, JOINT_THRESHOLD) );
	}

	QVector<QString> keys = primitives.keys().toVector();

	for(uint i = 0; i < primitives.size(); i++)
	{
		Primitive * a = primitives[keys[i]];

		for (uint j = i + 1; j < primitives.size(); j++)
		{
			Primitive * b = primitives[keys[i]];

			std::vector<Voxel> intersection = voxels[i].Intersects(&voxels[j]);

			// Analyze the intersection
		}
	}

	return Joints;
}

QVector<Group*> JointDetector::analyzeIntersection( std::vector<Voxel> &intersection )
{
	QVector<Group*> results;

	//if(!intersection.empty()){
	//	int N = intersection.size();
	//	Voxel center;
	//	foreach(Voxel v, intersection){
	//		center.x += v.x;
	//		center.y += v.y; 
	//		center.z += v.z;
	//	}
	//	double scale = JOINT_THRESHOLD / N;
	//	Point centerPoint(center.x * scale, center.y * scale, center.z * scale);

	//	PointJointGroup *joint = new PointJointGroup(POINTJOINT);
	//	joint->joint = centerPoint;

	//	QVector<Primitive*> segments;
	//	segments.push_back(primitives[keys[i]]);
	//	segments.push_back(primitives[keys[j]]);
	//	joint->process(segments);

	//	Joints.push_back(joint);
	//}


	return results;

}

//void JointDetector::detectPairwiseJoint()
//{
//	Primitive * primA = getPrimitive(a);
//	QSurfaceMesh meshA(primA->getGeometry());
//	Voxeler voxelerA(&meshA, JOINT_THRESHOLD);
//
//	Primitive * primB = getPrimitive(b);
//	QSurfaceMesh meshB(primB->getGeometry());
//	Voxeler voxelerB(&meshB, JOINT_THRESHOLD);
//
//	std::vector<Voxel> intersectionVoxels = voxelerA.Intersects(&voxelerB);
//	bool result;
//	if(intersectionVoxels.empty())
//		result = false;
//	else
//	{
//		QVector<Vec3d> intersectionPoints;
//		foreach(Voxel v, intersectionVoxels)
//			intersectionPoints.push_back(Vec3d(v.x, v.y, v.z));
//
//		QVector<QString> segments;
//		segments.push_back(a);
//		segments.push_back(b);
//		QVector<Vec3d> centers = centerOfClusters(intersectionPoints, nbJoints);
//		foreach(Vec3d center, centers)
//		{
//			Vec3d joint = center * JOINT_THRESHOLD;
//			JointGroup *newGroup = new JointGroup(this, JOINT);
//			newGroup->process(segments, joint);
//			this->groups[newGroup->id] = newGroup;
//		}
//
//	}
//}


//QVector< Vec3d > Controller::centerOfClusters( QVector< Vec3d> &data, int nbCluster )
//{
//	QVector< Vec3d > centers;
//
//	switch(nbCluster)
//	{
//	case 1:
//		{
//			Vec3d center(0, 0, 0);
//			foreach(Vec3d p, data)
//				center += p;
//
//			center /= data.size();
//			centers.push_back(center);
//		}
//		break;
//	case 2:
//		{
//			// Find the furthest two points
//			Vec3d p1 = data[0];
//			Vec3d p2;
//			double maxDis = 0;
//			foreach( Vec3d p, data)
//			{
//				double dis = (p-p1).norm();
//				if (dis > maxDis)
//				{
//					p2 = p;
//					maxDis = dis;
//				}
//			}
//
//			// Assign all the data to two clusters
//			QVector< QVector< Vec3d > >clusters(2);
//			foreach( Vec3d p, data)
//			{
//				if ((p-p1).norm() < (p-p2).norm())
//					clusters[0].push_back(p);
//				else
//					clusters[1].push_back(p);
//			}
//
//			// Computer the centers
//			for (int i=0;i<2;i++)
//			{
//				Vec3d center(0, 0, 0);
//				foreach(Vec3d p, clusters[i])
//					center += p;
//
//				center /= clusters[i].size();
//				centers.push_back(center);
//			}
//
//		}
//		break;
//	}
//
//	return centers;
//}