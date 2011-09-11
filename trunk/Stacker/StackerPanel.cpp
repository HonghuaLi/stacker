#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	connect(panel.stackButton, SIGNAL(clicked()), SLOT(onStackButtonClicked()));
}

void StackerPanel::onStackButtonClicked()
{
	emit(StackButtonClicked());
}
