
#include <QtGui/QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);
	MainWindow mainWindow;
	mainWindow.show();
	app.connect( &app, SIGNAL( lastWindowClosed() ),
				 &app, SLOT( quit() ) );
	return app.exec();
}
