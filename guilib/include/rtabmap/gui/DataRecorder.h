/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef RTABMAP_DATARECORDER_H_
#define RTABMAP_DATARECORDER_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/utilite/UEventsHandler.h>
#include <QWidget>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMutex.h>

class QLabel;

namespace rtabmap {

class Memory;
class ImageView;

class RTABMAP_GUI_EXPORT DataRecorder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	DataRecorder(QWidget * parent = 0);
	bool init(const QString & path, bool recordInRAM = true);

	void closeRecorder();

	virtual ~DataRecorder();

	const QString & path() const {return path_;}

public Q_SLOTS:
	void addData(const rtabmap::SensorData & data, const Transform & pose = Transform(), const cv::Mat & infMatrix = cv::Mat::eye(6,6,CV_64FC1));
	void showImage(const cv::Mat & image, const cv::Mat & depth);
protected:
	virtual void closeEvent(QCloseEvent* event);
	bool handleEvent(UEvent * event);

private:
	UMutex memoryMutex_;
	Memory * memory_;
	ImageView* imageView_;
	QLabel* label_;
	UTimer timer_;
	QString path_;
	bool processingImages_;
	int count_;
	int totalSizeKB_;
};

} /* namespace rtabmap */
#endif /* DATARECORDER_H_ */
