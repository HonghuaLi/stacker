#include "Stacker.h"


Stacker::Stacker( Scene *scene )
{
	activeScene = scene;
}

Stacker::~Stacker()
{

}

void Stacker::computeOffset()
{
	Offset offset(activeScene);
	offset.run();
}



