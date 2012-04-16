#include "Utility/Macros.h"
#include "MeshBrowserWidget.h"
#include "QuickMeshViewer.h"
#include <QFileDialog>

MeshBrowserWidget::MeshBrowserWidget()
{
	// Setup dialog from designed form
	dialog.setupUi(this);
	thumbWidget = dialog.thumbnailWidget;

	countX = 3;
	countY = 3;

	viewers.resize(countX);
	for(int i = 0; i < countX; i++){
		viewers[i].resize(countY);
		for(int j = 0; j < countY; j++){
			viewers[i][j] = new QuickMeshViewer();
			connect(viewers[i][j], SIGNAL(gotFocus(QuickMeshViewer*)), SLOT(setActiveViewer(QuickMeshViewer*)));
		}
	}

	numActiveViewers = 0;
	activeViewer = viewers[0][0];

	for(int i = 0; i < countX; i++)
		for(int j = 0; j < countY; j++)
			dialog.thumbLayout->addWidget(viewers[i][j], i, j);

	// Default path
	path = "";

	// Connections
	connect(dialog.pathButton, SIGNAL(clicked()), SLOT(changePath()));
	connect(this, SIGNAL(pathChanged(QString)), dialog.folderLabel, SLOT(setText(QString)));
	connect(this, SIGNAL(pathChanged(QString)), SLOT(loadMeshes(QString)));
}

void MeshBrowserWidget::changePath()
{
	QFileDialog fd;	fd.setOption(QFileDialog::ShowDirsOnly);
	path = fd.getExistingDirectory(0, "Select folder", path);

	emit(pathChanged(path));

	loadMeshes(path);
}

void MeshBrowserWidget::showEvent( QShowEvent * event )
{
	changePath();

	activeViewer->setFocus();
}

void MeshBrowserWidget::showNumViewers( int n )
{
	int count = 0, activeCount = 0;

	for(int i = 0; i < countX; i++){
		for(int j = 0; j < countY; j++){
			if(count++ < n)
			{
				viewers[i][j]->isActive = true;
				viewers[i][j]->resetView();
				activeCount++;
			}
			else
				viewers[i][j]->isActive = false;

			viewers[i][j]->updateGL();
		}
	}

	numActiveViewers = activeCount;
}

void MeshBrowserWidget::loadMeshes(QString using_path)
{
	path = using_path;

	// Get list of files
	QStringList filters;
	filters << "*.obj" << "*.off";
	files = QDir(path).entryList(filters);

	showNumViewers(files.size());

	int numPages = ceil(double(files.size()) / double(countX * countY));

	dialog.scrollBar->setRange(0, numPages);
	dialog.scrollBar->setValue(0);

	if(files.size())
		loadCurrentMeshes();
}

void MeshBrowserWidget::loadCurrentMeshes()
{
	int numViewers = countX * countY;

	int index = dialog.scrollBar->value() * numViewers;

	int curActive = Min(numViewers, files.size() - index);

	int c = 0;

	for(int i = 0; i < countX; i++)
		for(int j = 0; j < countY; j++)
		{
			viewers[i][j]->resetView();
			viewers[i][j]->clearMesh();
		}

	for(int i = 0; i < countX; i++){
		for(int j = 0; j < countY; j++){
			if(index + c > files.size() - 1) return;
			viewers[i][j]->loadMesh(path + "\\" + files[index + c]);
			c++; if(c > curActive) return;
		}
	}
}

void MeshBrowserWidget::setActiveViewer( QuickMeshViewer* v)
{
	activeViewer = v;
}

QString MeshBrowserWidget::selectedFile()
{
	if(activeViewer) return activeViewer->meshFileName();

	return "";
}
