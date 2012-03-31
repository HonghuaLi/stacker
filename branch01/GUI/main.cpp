#include "global.h"
#include "Workspace.h"
#include <QtGui/QApplication>
#include <QGLFormat>

int main(int argc, char *argv[])
{
	DEFAULT_FILE_PATH = "";

	QApplication a(argc, argv);

	// Anti-aliasing
	QGLFormat glf = QGLFormat::defaultFormat();
	glf.setSamples(8);
	QGLFormat::setDefaultFormat(glf);

	Workspace w;
	w.move(QPoint(0,0));
	w.show();

	return a.exec();
}
