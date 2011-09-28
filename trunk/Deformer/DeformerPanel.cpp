#include "DeformerPanel.h"

DeformerPanel::DeformerPanel()
{
	dw.setupUi(this);
}

void DeformerPanel::createFFD( QSurfaceMesh * mesh )
{
	this->activeDeformer = new QFFD(mesh);

	emit(deformerCreated(activeDeformer));
}
