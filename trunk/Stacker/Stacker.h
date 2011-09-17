#pragma once

#include "Scene.h"
#include "Offset.h"

class Stacker
{
public:
	Stacker(Scene *scene);
	~Stacker();

public:
	void computeOffset();


private:
	Scene *activeScene;
};