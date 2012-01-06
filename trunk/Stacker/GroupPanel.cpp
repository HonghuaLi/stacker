#include "GroupPanel.h"
#include <fstream>
#include <QFileDialog>

#include "SymmetryGroup.h"
#include "JointGroup.h"
#include "ConcentricGroup.h"
#include "CoplanarGroup.h"


GroupPanel::GroupPanel( QWidget * parent) : QWidget(parent)
{
	groupWidget.setupUi(this);

	connect(groupWidget.removeButton, SIGNAL(clicked()), SLOT(removeSelectedItem()));
	connect(groupWidget.saveGroupsButton, SIGNAL(clicked()), SLOT(saveGroups()));
	connect(groupWidget.loadGroupsButton, SIGNAL(clicked()), SLOT(loadGroups()));
	connect(groupWidget.clearButton, SIGNAL(clicked()), SLOT(clearGroups()));

	// Strings to show in tree, ordered as in group type enum
	groupTypes.push_back("SYMMETRY");
	groupTypes.push_back("JOINT");
	groupTypes.push_back("CONCENTRIC");
	groupTypes.push_back("COPLANNAR");
	groupTypes.push_back("SELF_SYMMETRY");
	groupTypes.push_back("SELF_ROT_SYMMETRY");
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

	foreach(Group* group, ctrl->groups)
	{
		QTreeWidgetItem *groupItem = new QTreeWidgetItem(groupWidget.groupTree);

		groupItem->setText(0, QString("g%1:%2").arg(groupTypes[group->type]).arg(group->id));

		foreach(QString node, group->nodes)
		{
			QTreeWidgetItem *item = new QTreeWidgetItem(groupItem);
			item->setText(0, QString("segment:%1").arg(node));
		}
	}
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
	
	Controller * ctrl = activeScene->activeObject()->controller;

	foreach(QTreeWidgetItem* item, selectedItems)
	{
		QString itemId = getItemId(item);
		QString itemType = item->text(0).at(0);

		// For groups
		if(itemType.startsWith("g"))
		{
			ctrl->groups.erase( ctrl->groups.find(itemId) );
		}
		
		// For segments
		if(itemType.startsWith("s"))
		{
			QString groupId = getItemId( item->parent() );
			ctrl->groups[groupId]->removeNode( groupId );
		}
	}

	// Clean-up (shouldn't be this class's responsibility?)
	std::vector<QString> emptyGroups;
	
	foreach(Group* group, ctrl->groups)
	{
		if(!group->nodes.size())
			emptyGroups.push_back(group->id);
	}

	foreach(QString group, emptyGroups)
		ctrl->groups.erase(ctrl->groups.find(group));

	updateWidget();
}

void GroupPanel::saveGroups()
{
	Controller * ctrl = activeScene->activeObject()->controller;
	if (ctrl == NULL) 
	{
		std::cout << "There is no controller.\n";
		return;
	}

	if (ctrl->groups.empty())
	{
		std::cout << "There are no groups.\n";
		return;
	}

	QString fileName = QFileDialog::getSaveFileName(0, "Export Groups", "", "Group File (*.grp)"); 
	std::ofstream outF(qPrintable(fileName), std::ios::out);

	foreach(Group* group, ctrl->groups)
	{
		outF << qPrintable(groupTypes[group->type]) << '\t'; 
		group->save(outF);
		outF << '\n';
	}

	foreach(Primitive * prim, ctrl->getPrimitives())
	{
		int nb = prim->symmPlanes.size();
		
		if(nb > 0)
		{
			outF << qPrintable(groupTypes[SELF_SYMMETRY]) << "\t" << qPrintable(prim->id) << "\t" << nb << std::endl;
		}

		if (prim->isRotationalSymmetry)
		{
			outF << qPrintable(groupTypes[SELF_ROT_SYMMETRY]) << "\t" << qPrintable(prim->id) << std::endl;
		}
	}

	outF.close();
	std::cout << "Groups have been saved.\n";
}

void GroupPanel::loadGroups()
{
	Controller * ctrl = activeScene->activeObject()->controller;
	if (ctrl == NULL) 
	{
		std::cout << "There is no controller.\n";
		return;
	}

	QString fileName = QFileDialog::getOpenFileName(0, "Import Groups", "", "Group File (*.grp)"); 
	std::ifstream inF(qPrintable(fileName), std::ios::in);

	if (!inF) return;

	while (inF)
	{
		std::string str;
		inF >> str;
		int type = groupTypes.indexOf(str.c_str());
		if (type == -1) break;

		Group* newGroup = NULL;

		switch (type)
		{
		case SYMMETRY:
			newGroup = new SymmetryGroup(ctrl, SYMMETRY);
			break;
		case CONCENTRIC:
			newGroup = new ConcentricGroup(ctrl, CONCENTRIC);
			break;
		case COPLANNAR:
			newGroup = new CoplanarGroup(ctrl, COPLANNAR);
			break;
		case JOINT:
			newGroup = new JointGroup(ctrl, JOINT);
			break;
		case SELF_SYMMETRY:
			{
				inF >> str;
				QString primId = QString(str.c_str());
				int nb_fold = 0; inF >> nb_fold;
				ctrl->getPrimitive(primId)->setSymmetryPlanes(nb_fold);
				break;
			}
		case SELF_ROT_SYMMETRY:
			{
				inF >> str;
				QString primId = QString(str.c_str());
				ctrl->getPrimitive(primId)->isRotationalSymmetry = true;
				break;
			}
		}

		if(newGroup)
		{
			newGroup->load(inF);
			ctrl->groups[newGroup->id] = newGroup;

			if(type == JOINT)
			{
				JointGroup * jg = (JointGroup*)newGroup;

				Point joint;
				inF >> joint.x() >> joint.y() >> joint.z();

				QVector<QString> segments;

				foreach(QString node, jg->nodes)
					segments.push_back(node);
				
				jg->process(segments, joint);
			}
		}
	}

	std::cout << "Groups have been loaded.\n";
	updateWidget();
}

void GroupPanel::clearGroups()
{
	Controller * ctrl = activeScene->activeObject()->controller;
	ctrl->groups.clear();

	// Clear self symmetry
	foreach(Primitive * prim, ctrl->getPrimitives())
		prim->symmPlanes.clear();

	std::cout << "Groups cleared.\n";
	updateWidget();
}

