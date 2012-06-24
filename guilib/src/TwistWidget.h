/*
 * TwistWidget.h
 *
 *  Created on: Dec 11, 2011
 *      Author: MatLab
 */

#ifndef TWISTWIDGET_H_
#define TWISTWIDGET_H_

#include <QtGui/QWidget>

class QGridLayout;

namespace rtabmap {

class TwistGridWidget : public QWidget
{
	Q_OBJECT
public:
	TwistGridWidget(QWidget * parent = 0, Qt::WindowFlags f = 0);
	virtual ~TwistGridWidget() {}

public slots:
	void addTwist(float x, float y, float z, float roll, float pitch, float yaw, int row=0, int col=0);

private:
	QGridLayout * _grid;
};

class TwistWidget : public QWidget
{
	Q_OBJECT

public:
	TwistWidget(float x, float y, float z, float roll, float pitch, float yaw, QWidget * parent, Qt::WindowFlags f = 0);
	virtual ~TwistWidget() {}

	void setData(float x, float y, float z, float roll, float pitch, float yaw);

protected:
	virtual void paintEvent(QPaintEvent * event);

private:
	float _x;
	float _y;
	float _z;
	float _roll;
	float _pitch;
	float _yaw;
};

}

#endif /* TWISTWIDGET_H_ */
