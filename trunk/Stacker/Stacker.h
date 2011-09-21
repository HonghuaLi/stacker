#pragma once

#include "Scene.h"
#include "Offset.h"

class Stacker
{
public:
	Stacker();

	void computeOffset();

	void setScene(Scene * newScene);
	Scene * scene();

private:
	Scene *activeScene;
};