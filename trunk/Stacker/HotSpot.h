#pragma once

#include <QVector>
#include <Qstring>
#include "GraphicsLibrary/Mesh/QSegMesh.h"

enum HOTSPOT_TYPE { POINT_HOTSPOT, LINE_HOTSPOT, RING_HOTSPOT};

struct HotSpot
{
	int					side;		// upper or lower
	int					hotRegionID; 
	QString				segmentID;
	bool				defineHeight;
	QVector< Vec2i >	hotPixels;
	QVector< Point >	hotSamples;

	HOTSPOT_TYPE		type;
	QVector<Point>		rep;		//representative

	void decideType();
	void computeRepresentative();
	void print();	
};
