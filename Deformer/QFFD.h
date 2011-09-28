#pragma once

#include "QSurfaceMesh.h"
#include "FFD.h"

extern FFD current_ffd;

// FFD with UI properties
class QFFD : public QObject
{
	Q_OBJECT

public:
	QFFD(QSurfaceMesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD);

	bool isReady;

	void draw();

	void drawNames();
	void postSelection(int idx);
	StdVector<uint> selectedPoints;

	QControlPoint & getQControlPoint( int index );
	FFD * ffd();
};
