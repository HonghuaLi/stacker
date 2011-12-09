#include "GroupPanel.h"

GroupPanel::GroupPanel( QWidget * parent) : QWidget(parent)
{
	groupWidget.setupUi(this);

	connect(groupWidget.removeButton, SIGNAL(clicked()), SLOT(removeSelectedItem()));

	// Strings to show in tree, ordered as in group type enum
	groupTypes.push_back("SYMMETRY");
	groupTypes.push_back("CONCENTRIC");
	groupTypes.push_back("COPLANNAR");
}

void GroupPanel::setActiveScene( Scene * newScene )
{
	this->activeScene = newScene;
	updateWidget();
}

void GroupPanel::updateWidget()
{
	groupWidget.groupTree->clear();

	if(!activeScene || !activeScene->activeObject() || !activeScene->activeObject()->controller)
		return;

	Controller * ctrl = activeScene->activeObject()->controller;

	for(std::map<int, Group*>::iterator it = ctrl->groups.begin(); it != ctrl->groups.end(); it++)
	{
		Group * group = it->second;

		QTreeWidgetItem *groupItem = new QTreeWidgetItem(groupWidget.groupTree);

		groupItem->setText(0, QString("g %1 : %2").arg(groupTypes[group->type]).arg(group->id));

		foreach(int node, group->nodes)
		{
			QTreeWidgetItem *item = new QTreeWidgetItem(groupItem);
			item->setText(0, QString("segment : %1").arg(node));
		}
	}
}

int GroupPanel::getItemId(QTreeWidgetItem* item)
{
	QString txt = item->text(0);

	return txt.replace(0,txt.lastIndexOf(":") + 1,"").toInt();
}

void GroupPanel::removeSelectedItem()
{
	QList<QTreeWidgetItem*> selectedItems = groupWidget.groupTree->selectedItems();
	if(selectedItems.isEmpty()) return;
	
	Controller * ctrl = activeScene->activeObject()->controller;

	foreach(QTreeWidgetItem* item, selectedItems)
	{
		int itemId = getItemId(item);
		QString itemType = item->text(0).at(0);

		// For groups
		if(itemType.startsWith("g"))
		{
			ctrl->groups.erase( itemId );
		}
		
		// For segments
		if(itemType.startsWith("s"))
		{
			int groupId = getItemId( item->parent() );
			ctrl->groups[groupId]->removeNode( itemId );
		}
	}

	// Clean-up (shouldn't be this class's responsibility?)
	std::vector<int> emptyGroups;
	
	for(std::map<int, Group*>::iterator it = ctrl->groups.begin(); it != ctrl->groups.end(); it++){
		Group * group = it->second;
		if(!group->nodes.size())
			emptyGroups.push_back(it->first);
	}

	foreach(int group, emptyGroups)
		ctrl->groups.erase(group);

	updateWidget();
}
