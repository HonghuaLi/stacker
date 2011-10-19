#pragma once

#include "QSurfaceMesh.h"
#include "FFD.h"

extern FFD current_ffd;

// FFD with UI properties
class QFFD : public QObject
{
	Q_OBJECT

public:
	QFFD( QSurfaceMesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD, 
		Vec3i gridResolution = Vec3i(2,2,2));

	bool isReady;

	void draw();
	void drawConnections(QControlPoint * cp);

	void drawNames();
	void postSelection(int idx);
	StdVector<uint> selectedPoints;

	QControlPoint * getQControlPoint( int index );
	FFD * ffd();

public slots:
	void updateMesh();

signals:
	void meshDeformed();
};
