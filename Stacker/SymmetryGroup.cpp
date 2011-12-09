#include "SymmetryGroup.h"

void SymmetryGroup::process(std::vector<int> segments)
{
	addNodes(segments);

}

void SymmetryGroup::draw()
{
	// Center point of each node
	Group::draw();


}
