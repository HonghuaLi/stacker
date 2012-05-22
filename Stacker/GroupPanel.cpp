#include "GroupPanel.h"

#include <fstream>

#include <QFileDialog>

#include "GUI/global.h"
#include "Primitive.h"
#include "Controller.h"
#include "JointDetector.h"
#include "Group.h"

GroupPanel::GroupPanel( QWidget * parent) : QWidget(parent)
{
	groupWidget.setupUi(this);

	// Joints
	jointDetector = new JointDetector();
	groupWidget.jointThreshold->setValue(jointDetector->JOINT_THRESHOLD);
	connect(groupWidget.jointThreshold, SIGNAL(valueChanged(double)),SLOT(setJointThreshold(double)) );
	connect(groupWidget.findJointsButton, SIGNAL(clicked()), SLOT(findJoints()));

	// Group tree
	connect(groupWidget.removeButton, SIGNAL(clicked()), SLOT(removeSelectedItem()));
	connect(groupWidget.clearButton, SIGNAL(clicked()), SLOT(clearGroups()));
	connect(groupWidget.showGroups, SIGNAL(stateChanged (int)), SLOT(toggleGroupDisplay(int)));

	// Save and load
	connect(groupWidget.saveGroupsButton, SIGNAL(clicked()), SLOT(saveGroups()));
	connect(groupWidget.loadGroupsButton, SIGNAL(clicked()), SLOT(loadGroups()));

}

void GroupPanel::setActiveScene( Scene * newScene )
{
	this->activeScene = newScene;
	updateWidget();
}

void GroupPanel::toggleGroupDisplay(int state)
{
	if(!activeScene || !activeScene->activeObject() || !(Controller *)activeScene->activeObject()->ptr["controller"])
		return;

	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];

	foreach(Group* group, ctrl->groups)
	{
		group->isDraw = state;
	}

	activeScene->updateGL();
}

void GroupPanel::updateWidget()
{
	groupWidget.groupTree->clear();

	if(!activeScene || !activeScene->activeObject() || !(Controller *)activeScene->activeObject()->ptr["controller"])
		return;

	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];

	foreach(Group* group, ctrl->groups)
	{
		QTreeWidgetItem *groupItem = new QTreeWidgetItem(groupWidget.groupTree);

		groupItem->setText(0, QString("g%1:%2").arg(ctrl->groupTypes[group->type]).arg(group->id));

		foreach(QString node, group->getNodes())
		{
			QTreeWidgetItem *item = new QTreeWidgetItem(groupItem);
			item->setText(0, QString("segment:%1").arg(node));
		}
	}

	activeScene->updateGL();
}

QString GroupPanel::getItemId(QTreeWidgetItem* item)
{
	QString txt = item->text(0);

	return txt.replace(0,txt.lastIndexOf(":") + 1,"");
}

void GroupPanel::removeSelectedItem()
{
	QList<QTreeWidgetItem*> selectedItems = groupWidget.groupTree->selectedItems();
	if(selectedItems.isEmpty()) return;
	
	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];

	foreach(QTreeWidgetItem* item, selectedItems)
	{
		QString itemId = getItemId(item);
		QString itemType = item->text(0).at(0);

		// For groups
		if(itemType.startsWith("g"))
		{
			ctrl->groups.erase( ctrl->groups.find(itemId) );
		}
	}

	updateWidget();

	emit( groupsModified() );
}

void GroupPanel::saveGroups()
{
	if(!activeScene || !activeScene->activeObject() || !(Controller *)activeScene->activeObject()->ptr["controller"])
		return;

	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];
	if (ctrl == NULL) 
	{
		std::cout << "There is no controller.\n";
		return;
	}

	QString fileName = QFileDialog::getSaveFileName(0, "Export Groups", DEFAULT_FILE_PATH, "Group File (*.grp)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	ctrl->saveGroups(outF);

	outF.close();
	std::cout << "Groups have been saved.\n";

	activeScene->updateGL();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();
}

void GroupPanel::loadGroups()
{
	if(!activeScene || !activeScene->activeObject() || !(Controller *)activeScene->activeObject()->ptr["controller"])
		return;

	Controller * ctrl = (Controller *)activeObject()->ptr["controller"];
	if (ctrl == NULL) 
	{
		std::cout << "There is no controller.\n";
		return;
	}

	QString fileName = QFileDialog::getOpenFileName(0, "Import Groups", DEFAULT_FILE_PATH, "Group File (*.grp)"); 
	if(fileName.isEmpty() || !QFileInfo(fileName).exists()) return;

	std::ifstream inF(qPrintable(fileName), std::ios::in);

	ctrl->loadGroups(inF);

	std::cout << "Groups have been loaded.\n";
	updateWidget();

	DEFAULT_FILE_PATH = QFileInfo(fileName).absolutePath();

	emit( groupsModified() );
}

void GroupPanel::clearGroups()
{
	if(!activeScene || !activeScene->activeObject()) return;

	Controller * ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];
	ctrl->groups.clear();

	// Clear self symmetry
	foreach(Primitive * prim, ctrl->getPrimitives())
		prim->symmPlanes.clear();

	std::cout << "Groups cleared.\n";
	updateWidget();

	emit( groupsModified() );
}

void GroupPanel::findJoints()
{
	if(!activeScene || !activeObject() || !activeObject()->ptr["controller"])	return;

	// Call joint detector to find joints
	Controller* ctrl = (Controller *)activeScene->activeObject()->ptr["controller"];
	JointDetector JD;
	QVector<Group*> jointGroups = JD.detect(ctrl->getPrimitives());

	int i = ctrl->groups.size();
	foreach(Group* g, jointGroups)
	{
		g->id = QString::number(i++);
		ctrl->groups[g->id] = g;
	}

	// update
	this->updateWidget();

	emit( groupsModified() );
}

void GroupPanel::setJointThreshold( double threshold )
{
	jointDetector->JOINT_THRESHOLD = threshold;
}

QSegMesh* GroupPanel::activeObject()
{
	if (activeScene)
		return activeScene->activeObject();
	else 
		return NULL;
}
