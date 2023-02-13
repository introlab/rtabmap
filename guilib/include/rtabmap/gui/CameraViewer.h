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

#ifndef RTABMAP_CAMERAVIEWER_H_
#define RTABMAP_CAMERAVIEWER_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/utilite/UEventsHandler.h>
#include <QDialog>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

class QSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace rtabmap {

class ImageView;
class CloudViewer;

class RTABMAP_GUI_EXPORT CameraViewer : public QDialog, public UEventsHandler
{
	Q_OBJECT
public:
	CameraViewer(
		QWidget * parent = 0,
		const ParametersMap & parameters = ParametersMap());
	virtual ~CameraViewer();

public Q_SLOTS:
	void showImage(const rtabmap::SensorData & data);
protected:
	virtual bool handleEvent(UEvent * event);

private:
	ImageView* imageView_;
	CloudViewer* cloudView_;
	bool processingImages_;
	QSpinBox * decimationSpin_;
	ParametersMap parameters_;
	QPushButton * pause_;
	QLabel * imageSizeLabel_;
	QCheckBox * showCloudCheckbox_;
	QCheckBox * showScanCheckbox_;
};

} /* namespace rtabmap */
#endif /* CAMERAVIEWER_H_ */
