#pragma once

#include "ui_StackerWidget.h"
#include "Scene.h"
#include "StackerPreview.h"
#include "HiddenViewer.h"
#include "Offset.h"

class StackerPanel : public QWidget
{
	Q_OBJECT

public:
	StackerPanel();

	// Active Object
	QSegMesh* activeObject();

	// Message
	void showMessage( QString message );

	Ui::StackerWidget panel;
	Scene * activeScene;
	StackerPreview * stacker_preview;
	HiddenViewer * hidden_viewer;
	Offset * activeOffset;

public slots:
	// Buttons
	void onOffsetButtonClicked();
	void onControllerButtonClicked();
	void onImproveButtonClicked();
	void onHotspotsButtonClicked();

	// Others
	void setActiveScene( Scene * );
	void updateActiveObject();

signals:
	void printMessage( QString );
	void objectModified();
};
