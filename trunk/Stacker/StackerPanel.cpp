#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	panel.groupBox->layout()->addWidget(stacker_preview);

	// Offset function
	hidden_viewer = new HiddenViewer();
	hidden_viewer->setVisible(true);
	panel.groupBox->layout()->addWidget(hidden_viewer);
	activeOffset = new Offset(hidden_viewer);
	stacker_preview->setActiveOffset(activeOffset);

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
	activeOffset->computeOffset();
	activeOffset->saveOffsetAsImage("offset_image.png");
}

void StackerPanel::setActiveScene( Scene * scene )
{
	if(activeScene != scene)
	{
		activeScene = scene;
		stacker_preview->setActiveScene(scene);
		hidden_viewer->setActiveScene(scene);
	}

	emit(activeSceneChanged());
}


void StackerPanel::updateActiveObject()
{
	stacker_preview->updateActiveObject();
	activeOffset->computeOffset();

	emit(activeSceneChanged());
}
