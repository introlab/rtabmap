
#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include "Plot.h"

// Note : compiled with -DPLOT_WIDGET_OUT_OF_LIB to
// remove rtabmap's namespace and UtiLite dependency of Plot.
class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow() {
		Plot * plot = new Plot(this);
		plot->setObjectName("Figure 1");
		plot->setMaxVisibleItems(25);
		this->setCentralWidget(plot); // ownership transferred
		PlotCurve * curveA = new PlotCurve("Curve A", this);
		PlotCurve * curveB = new PlotCurve("Curve B", this);
		curveA->setPen(QPen(Qt::red));
		curveB->setPen(QPen(Qt::blue));
		plot->addCurve(curveA); // ownership transferred
		plot->addCurve(curveB); // ownership transferred
		connect(this, SIGNAL(valueUpdatedA(float)), curveA, SLOT(addValue(float)));
		connect(this, SIGNAL(valueUpdatedB(float)), curveB, SLOT(addValue(float)));
		connect(&timer_, SIGNAL(timeout()), this, SLOT(updateCounter()));
		timer_.start(100);
		qsrand(1);
	}
	~MainWindow() {}
public slots:
	void updateCounter() {
		emit valueUpdatedA(qrand() % 100);
		emit valueUpdatedB(qrand() % 50);
	}
signals:
	void valueUpdatedA(float);
	void valueUpdatedB(float);
private:
	QTimer timer_;
};

#endif /* MAINWINDOW_H_ */
