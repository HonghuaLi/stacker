#include "StackerPanel.h"
#include "Contoller.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	panel.groupBox->layout()->addWidget(stacker_preview);

	// Offset function
	hidden_viewer = new HiddenViewer();
	panel.groupBox->layout()->addWidget(hidden_viewer);
	activeOffset = new Offset(hidden_viewer);

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(panel.controllerButton, SIGNAL(clicked()), SLOT(onControllerButtonClicked()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));

	connect(this, SIGNAL(objectModified()), SLOT(updateActiveObject()));
}

void StackerPanel::onOffsetButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		emit(printMessage("There is no valid object."));
		return;
	}

	// compute offset
	activeOffset->computeOffset();
	activeOffset->saveOffsetAsImage("offset_image.png");
}

void StackerPanel::onControllerButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		emit(printMessage("There is no valid object."));
		return;
	}

	activeObject()->controller = new Controller(activeObject());
	activeObject()->controller->fitOBBs();

	showMessage("Controller is build for " + activeObject()->objectName());
}


void StackerPanel::onImproveButtonClicked()
{
	if (!activeScene || activeScene->isEmpty())
	{
		showMessage("There is no valid object.");
		return;
	}

	if (!activeObject()->controller)
	{
		showMessage("There is no controller built.");
	}

	Controller* ctrl = activeObject()->controller;
	ctrl->test1();
	emit(objectModified());
}



void StackerPanel::setActiveScene( Scene * scene )
{
	if(activeScene != scene)
	{
		activeScene = scene;
		stacker_preview->setActiveScene(scene);
		hidden_viewer->setActiveScene(scene);
	}
}

void StackerPanel::updateActiveObject()
{
	activeOffset->computeOffset();	
	stacker_preview->updateActiveObject();
}

QSegMesh* StackerPanel::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
}

void StackerPanel::showMessage( QString message )
{
	emit(printMessage(message));
}


