#pragma once

#include "ui_DeformerWidget.h"
#include "QFFD.h"

class DeformerPanel : public QWidget
{
	Q_OBJECT

private:
	Ui::DeformerWidget dw;

public:
	DeformerPanel();

	QFFD * activeDeformer;

public slots:
	void createFFD(QSurfaceMesh * mesh);

signals:
	void deformerCreated( QFFD * );
};
