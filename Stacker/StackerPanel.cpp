#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Stacker computes offset and stacking related matters
	stacker = new Stacker();

	// Add a stacking preview widget
	stacker_preview = new StackerPreview();
	panel.groupBox->layout()->addWidget(stacker_preview);

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(this, SIGNAL(activeSceneChanged()), stacker_preview, SLOT(updateGL()));
}

void StackerPanel::onOffsetButtonClicked()
{
	if (activeScene && !activeScene->activeObjectId.size())
	{
		activeScene->print("There is no mesh opened!");
		return;
	}

	// compute offset
	stacker->setScene(activeScene);
	stacker->computeOffset();
}

void StackerPanel::setActiveScene( Scene * scene )
{
	if(activeScene != scene)
	{
		activeScene = scene;
		stacker->setScene(scene);
		stacker_preview->setActiveScene(scene);
	}

	emit(activeSceneChanged());
}

void StackerPanel::sceneUpdated()
{
	stacker_preview->setActiveScene(activeScene);
	stacker_preview->updateGL();

	emit(activeSceneChanged());
}
