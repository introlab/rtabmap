/*
 * TwistWidget.cpp
 *
 *  Created on: Dec 11, 2011
 *      Author: MatLab
 */

#include "TwistWidget.h"

#include <utilite/ULogger.h>
#include <QtGui/QPainter>
#include <QtCore/qmath.h>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>

#define SIZE 50

namespace rtabmap {

TwistGridWidget::TwistGridWidget(QWidget * parent, Qt::WindowFlags f) :
		QWidget(parent, f)
{
	_grid = new QGridLayout(this);
	_grid->setContentsMargins(0,0,0,0);
	_grid->addWidget(new QLabel(tr("Current"), this), 0, 0);
	_grid->addWidget(new QLabel(tr("Prediction"), this), 1, 0);
}

void TwistGridWidget::addTwist(float x, float y, float z, float roll, float pitch, float yaw, int row, int col)
{
	if(_grid->itemAtPosition(row, col+1))
	{
		QLayoutItem * item = _grid->itemAtPosition(row, col+1);
		if(item)
		{
			QWidget * w = item->widget();
			if(w)
			{
				_grid->removeItem(item);
				w->deleteLater();
			}
		}
	}
	_grid->addWidget(new TwistWidget(x, y, z, roll, pitch, yaw, this), row, col+1);
}

TwistWidget::TwistWidget(float x, float y, float z, float roll, float pitch, float yaw, QWidget * parent, Qt::WindowFlags f) :
		QWidget(parent, f)
{
	if(qAbs(z) > 0.00001)
	{
		UWARN("%f,%f,%f %f,%f,%f", x,y,z, roll,pitch,yaw);
	}
	//linear
	_x = x;
	_y = y;
	_z = z;

	//angular
	_roll = roll;
	_pitch = pitch;
	_yaw = yaw;

	this->setFixedSize(SIZE,SIZE);
	this->setMinimumSize(SIZE,SIZE);
}

void TwistWidget::paintEvent(QPaintEvent * event)
{
	QPainter painter(this);

	painter.setPen(QPen(QBrush(Qt::black), 1, Qt::DashLine));
	painter.drawEllipse(0,0,SIZE,SIZE);

	painter.translate(SIZE/2,SIZE/2);
	painter.rotate(-90);
	const float pi = 3.14159f;

	painter.save();
	if(qAbs(_x) < 0.0001 && qAbs(_y) < 0.0001)
	{
		painter.setBrush(Qt::red);
		painter.setPen(Qt::NoPen);
		painter.drawEllipse(-5,-5,10,10);
	}
	else
	{
		painter.setPen(QPen(QBrush(Qt::red), 3));
		painter.rotate(-qAtan2(_y, _x)*180.0f/pi);
		painter.drawLine(0,0,SIZE/2,0);
	}
	painter.restore();

	painter.save();
	if(qAbs(_yaw) < 0.0001)
	{
		painter.setBrush(Qt::green);
		painter.setPen(Qt::NoPen);
		painter.drawEllipse(-3,-3,6,6);
	}
	else
	{
		painter.setPen(QPen(QBrush(Qt::green), 3));
		painter.rotate(-_yaw*180.0f/pi);
		painter.drawLine(0,0,50,0);
	}
	painter.restore();
}

}
