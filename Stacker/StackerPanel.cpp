#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
}


void StackerPanel::onOffsetButtonClicked()
{
	if (!activeScene->activeObject)
	{
		activeScene->print("There is no mesh opened!");
		return;
	}

	Stacker stacker(activeScene);
	stacker.computeOffset();
}


void StackerPanel::setActiveScene( Scene * scene)
{
	activeScene = scene;
}
