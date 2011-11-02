#include "DeformerPanel.h"

DeformerPanel::DeformerPanel()
{
	dw.setupUi(this);

	// Connections
	connect(dw.createBoundingButton, SIGNAL(clicked()), SLOT(onCreateBoundingClicked()));
}

void DeformerPanel::setActiveScene( Scene * newScene)
{
	activeScene = newScene;
}

void DeformerPanel::onCreateBoundingClicked()
{
	if(!activeScene || !activeScene->activeObject())
		return;

	activeDeformer = new QFFD(activeScene->activeObject()->getSegment(0), BoundingBoxFFD, 
		Vec3i(dw.xRes->value(), dw.yRes->value(), dw.zRes->value()));

	connect(activeDeformer, SIGNAL(meshDeformed()), activeScene, SLOT(updateActiveObject()));

	emit( deformerCreated(activeDeformer) );
}
