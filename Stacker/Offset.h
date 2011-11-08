#pragma once

#include "ColorMap.h"
#include <vector>
#include <set>
#include "QSegMesh.h"

class HiddenViewer;

class Offset
{
public:
	Offset(HiddenViewer* viewer);
	~Offset();

public:
	void computeOffset();
	void run();
	void setDirty(bool dirty = true);
	double getMaxOffset();
	double getStackability();

	void saveOffsetAsImage(QString fileName);
private:
	std::vector< std::vector<double> > computeEnvelope(int direction);
	std::set<uint> verticesOnEnvelope(int direction);
	void setOffsetColors(int direction, std::vector< std::vector<double> > &offset, double O_max);


private:
	HiddenViewer * activeViewer;
	std::vector< std::vector<double> > upperEnvolope;
	std::vector< std::vector<double> > lowerEnvolope;
	std::vector< std::vector<double> > offset; 
	double O_max;
	bool isDirty;
	double objectH;
};