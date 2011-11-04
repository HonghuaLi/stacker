#pragma once

#include "ColorMap.h"
#include <vector>
#include <set>
#include "Scene.h"
#include "QSegMesh.h"


class Offset
{
public:
	Offset(Scene* scene);
	~Offset();

public:
	std::vector< std::vector<double> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction, std::vector< std::vector<double> > &offset, double O_max);
	void computeOffset();
	void run();
	void setDirty(bool dirty = true);
	double getMaxOffset();
	void saveOffsetAsImage(QString fileName);

private:
	Scene * activeScene;
	std::vector< std::vector<double> > upperEnvolope;
	std::vector< std::vector<double> > lowerEnvolope;
	std::vector< std::vector<double> > offset; 
	double O_max;
	bool isDirty;
	double objectH;
};