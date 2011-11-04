#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview();
	panel.groupBox->layout()->addWidget(stacker_preview);

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(this, SIGNAL(activeSceneChanged()), stacker_preview, SLOT(updateGL()));
}

void StackerPanel::onOffsetButtonClicked()
{
	if (!activeScene && !activeScene->isEmpty())
	{
		activeScene->print("There is no object in the scene!");
		return;
	}

	// compute offset
	activeScene->m_offset->computeOffset();
	activeScene->m_offset->saveOffsetAsImage("offset_image.png");
}

void StackerPanel::setActiveScene( Scene * scene )
{
	if(activeScene != scene)
	{
		activeScene = scene;
		stacker_preview->setActiveScene(scene);
	}

	emit(activeSceneChanged());
}


void StackerPanel::updateActiveObject()
{
	stacker_preview->updateActiveObject();

	emit(activeSceneChanged());
}
