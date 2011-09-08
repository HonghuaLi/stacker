#include "StackerPanel.h"

StackerPanel::StackerPanel()
{
	sw.setupUi(this);

	connect(sw.stackButton, SIGNAL(clicked()), SLOT(stackButtonClicked()));
}

void StackerPanel::stackButtonClicked()
{
	emit(doStuffScene(QString("Value is : %1").arg(sw.spinBox->value())));
}
