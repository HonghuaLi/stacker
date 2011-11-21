#include "StackerPanel.h"
#include "Contoller.h"
#include <QVBoxLayout>
#include "Vector.h"
#include <fstream>

StackerPanel::StackerPanel()
{
	panel.setupUi(this);

	// Offset function calculator
	hidden_viewer = new HiddenViewer();
	panel.groupBox->layout()->addWidget(hidden_viewer);
	activeOffset = new Offset(hidden_viewer);

	// Add a stacking preview widget
	stacker_preview = new StackerPreview(this);
	QVBoxLayout *previewLayout = new QVBoxLayout(panel.previewBox);
	previewLayout->addWidget(stacker_preview);	

	// Connections
	connect(panel.offsetButton, SIGNAL(clicked()), SLOT(onOffsetButtonClicked()));
	connect(panel.controllerButton, SIGNAL(clicked()), SLOT(onControllerButtonClicked()));
	connect(panel.improveButton, SIGNAL(clicked()), SLOT(onImproveButtonClicked()));
	connect(panel.hotspotsButton, SIGNAL(clicked()), SLOT(onHotspotsButtonClicked()));

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
	//ctrl->test2(Vec3d(0.8, 0.8, 1), Vec3d(-0.3, -0.4, 0.1), Vec3d(15,10,0));
	//emit(objectModified());
	//return;


	// Sampling in the deformation space
	// top: 2 scales
	// leg: 3 translations, 3 rotations
	int  N = 4;
	std::vector<double> S(N), T(N), R(N);
	double S_step = 0.5/N;
	double T_step = 0.5/N;
	double R_step = 40/N;
	for (int i=0; i<N; i++)
	{
		S[i] = 1 - i * S_step;
		T[i] = i * T_step;
		R[i] = i * R_step - 30;
	}

	std::vector< std::vector< double > > selectedParameters;

	for (int i=0; i<N; i++){
		for (int j=0; j<N; j++){
			for (int k=0; k<N; k++){
				for (int m=0; m<N; m++){
					for (int n=0; n<N; n++){
						for (int u=0; u<N; u++){
							for (int v=0; v<N; v++){
								for (int w=0; w<N; w++){
									
									// deformation
									double s1 = S[m], s2 = S[n];
									double tx = T[i], ty = T[j], tz = T[k];
									double a = R[u],  b = R[v], c = R[w];
									ctrl->test2(Vec3d(s1, s2, 1), Vec3d(tx, ty, tz), Vec3d(a,b,c));

									// update offset
									activeOffset->computeOffset();	
									double stackability = activeOffset->getStackability();

									if (stackability > 0.2)
									{
										double p[9] = {stackability, s1, s2, tx, ty, tz, a, b, c};
										std::vector< double > param(p, p+8);
										selectedParameters.push_back(param);
									}

									// undo
									ctrl->undo();
								}
							}
						}
					}
				}
			}
		}
	}

	std::ofstream outF("selectedParameters.txt", std::ios::out);
	for (int i=0; i<selectedParameters.size(); i++)
	{
		outF<< selectedParameters[i][0] << '\t'
			<< selectedParameters[i][1] << '\t'
			<< selectedParameters[i][2] << '\t'
			<< selectedParameters[i][3] << '\t'
			<< selectedParameters[i][4] << '\t'
			<< selectedParameters[i][5] << '\t'
			<< selectedParameters[i][6] << '\t'
			<< selectedParameters[i][7] << '\t'
			<< selectedParameters[i][8] << '\n';
	}

	showMessage("Searching is done.");
}



void StackerPanel::onHotspotsButtonClicked()
{
	activeOffset->detectHotspots();
	activeOffset->showHotVertices();
	emit(objectModified());
	showMessage("Hot spots are detected.");
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
	else 
		return NULL;
}

void StackerPanel::showMessage( QString message )
{
	emit(printMessage(message));
}


