#pragma once

#include "ui_StackerWidget.h"
#include "ShapeState.h"


class Scene;
class QSegMesh;
class Controller;
class Previewer;
class HiddenViewer;
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
	Controller* ctrl();
	void showMessage( QString message );

	// Improve and suggestion
	QMap<QString, ShapeState> treeNodes;
	void addChildren(QTreeWidgetItem* parent, QVector<ShapeState> &children);
	QTreeWidgetItem* selectedItem();

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
	void setSelectedShapeState();
	void resetSolutionTree();
	void onImproveButtonClicked();

	// Message
	void print(QString message);

	// Debug
	void onHotspotsButtonClicked();
	void outputForPaper();

signals:
	void printMessage( QString );
	void objectModified();
};
