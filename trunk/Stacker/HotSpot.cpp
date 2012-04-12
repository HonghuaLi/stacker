#include "HotSpot.h"


void HotSpot::print()
{
	std::cout << "side="   << side 
		<< "\tsegmentID="	  << qPrintable(segmentID) 
		<< "\tdefineHeight=" << defineHeight << std::endl; 
}

Vec3d HotSpot::hotPoint()
{
	return hotSamples[hotSamples.size()/2];
}
