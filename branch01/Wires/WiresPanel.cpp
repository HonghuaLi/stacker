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
	if(!activeScene || activeScene->isEmpty())
		return;

	QVector<Wire> newWires = QVector<Wire>::fromStdVector(
		AnalyzeWires::fromMesh(activeScene->activeObject()->getSegment(0), ww.sharpnessThreshold->value(),
		ww.strengthThreshold->value())); 

	emit(wiresFound( newWires ));
}

void WiresPanel::setActiveScene( Scene * newScene)
{
	activeScene = newScene;
}
