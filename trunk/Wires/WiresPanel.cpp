#include "WiresPanel.h"
#include "AnalyzeWires.h"
#include "Macros.h"

WiresPanel::WiresPanel()
{
	ww.setupUi(this);

	connect(ww.analyzeWiresButton, SIGNAL(clicked()), SLOT(analyzeButtonClicked()));
}

void WiresPanel::analyzeButtonClicked()
{
	if(!activeScene || activeScene->numObjects() < 1)
		return;

	QVector<Wire> newWires = QVector<Wire>::fromStdVector(
		AnalyzeWires::fromMesh2(activeScene->activeObject, TO_RAD(ww.angleTolerance->value()))); 

	emit(wiresFound( newWires ));
}

void WiresPanel::setActiveScene( Scene * newScene)
{
	activeScene = newScene;
}
