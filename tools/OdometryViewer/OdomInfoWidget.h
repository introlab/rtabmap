/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMINFOWIDGET_H_
#define ODOMINFOWIDGET_H_

#include <rtabmap/utilite/UEventsHandler.h>
#include <QWidget>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/OdometryInfo.h>

class QLabel;

namespace rtabmap{
class ImageView;
}

class OdomInfoWidget : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	OdomInfoWidget(QWidget * parent = 0);
	virtual ~OdomInfoWidget();
public slots:
	void processOdomInfo(const rtabmap::SensorData & data, const rtabmap::OdometryInfo & info);
protected:
	virtual void closeEvent(QCloseEvent* event);
	void handleEvent(UEvent * event);

private:
	rtabmap::ImageView* imageView_;
	QLabel* label_;
	UTimer timer_;
	bool processingOdomInfo_;
	double receivingRate_;
	double lastTime_;
	bool odomImageShow_;
	bool odomImageDepthShow_;
};

#endif /* ODOMINFOWIDGET_H_ */
