
#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtGui/QSlider>
#include <QtGui/QHBoxLayout>
#include <QtCore/QTimer>
#include "Plot.h"

// Note : compiled with -DPLOT_WIDGET_OUT_OF_LIB to
// remove rtabmap's namespace and UtiLite dependency of Plot.
class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow() {
		//Plot
		Plot * plot = new Plot(this);
		plot->setObjectName("Figure 1");
		plot->setMaxVisibleItems(50);
		plot->showRefreshRate(true);
		PlotCurve * curveA = new PlotCurve("Curve A", this);
		PlotCurve * curveB = new PlotCurve("Curve B", this);
		curveA->setPen(QPen(Qt::red));
		curveB->setPen(QPen(Qt::blue));
		plot->addCurve(curveA); // ownership transferred
		plot->addCurve(curveB); // ownership transferred
		connect(this, SIGNAL(valueUpdatedA(float)), curveA, SLOT(addValue(float)));
		connect(this, SIGNAL(valueUpdatedB(float)), curveB, SLOT(addValue(float)));

		//Control
		QSlider * slider = new QSlider(Qt::Vertical, this);
		slider->setMinimum(1); // Hz
		slider->setMaximum(100); // Hz
		slider->setValue(10);
		connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setRate(int)));

		// layout
		QWidget * placeHolder = new QWidget(this);
		this->setCentralWidget(placeHolder);
		QHBoxLayout * hlayout = new QHBoxLayout(placeHolder);
		hlayout->addWidget(plot, 1);
		hlayout->addWidget(slider);

		connect(&timer_, SIGNAL(timeout()), this, SLOT(updateCounter()));
		setRate(slider->value());
		qsrand(1);
	}
	~MainWindow() {}
public slots:
	void updateCounter() {
		emit valueUpdatedA(qrand() % 100);
		emit valueUpdatedB(qrand() % 50);
	}
	void setRate(int rate) {
		timer_.start(1000/rate);
	}
signals:
	void valueUpdatedA(float);
	void valueUpdatedB(float);
private:
	QTimer timer_;
};

#endif /* MAINWINDOW_H_ */
