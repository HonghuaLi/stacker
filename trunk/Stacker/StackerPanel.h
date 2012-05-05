#pragma once

#include "ui_StackerWidget.h"
#include "GUI/Scene.h"
#include "Previewer.h"
#include "HiddenViewer.h"
#include "Controller.h"

// Forward declaration
class Offset;
class Improver;

class StackerPanel : public QWidget
{
	Q_OBJECT

public:
	StackerPanel();
	~StackerPanel();

	// Active Object
	QSegMesh* activeObject();
	void showMessage( QString message );

	// Improve and suggestion
	QMap<QString, ShapeState> treeNodes;
	void addChildren(QTreeWidgetItem* parent, QVector<ShapeState> &children);
	QString getItemId(QTreeWidgetItem* item);

	QVector<EditPath> suggestions;
	void setSuggestions();
	void draw();

	// Core components
	Ui::StackerWidget	panel;
	Scene			* activeScene;
	Previewer		* previewer;
	HiddenViewer	* hiddenViewer;
	Offset			* activeOffset;
	Improver		* improver;

public slots:
	// Scene management
	void setActiveScene( Scene * newScene);
	void setActiveObject();
	void updateActiveObject();

	// Improve and suggest
	void onImproveButtonClicked();
	void searchDirection();

	// Message
	void print(QString message);

	// Debug
	void onHotspotsButtonClicked();
	void outputForPaper();

signals:
	void printMessage( QString );
	void objectModified();
};
