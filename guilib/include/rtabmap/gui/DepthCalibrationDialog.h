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

#ifndef RTABMAP_DEPTHCALIBRATIONDIALOG_H_
#define RTABMAP_DEPTHCALIBRATIONDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QDialog>
#include <QMap>
#include <QtCore/QSettings>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>

class Ui_DepthCalibrationDialog;

namespace clams {
class DiscreteDepthDistortionModel;
}

namespace rtabmap {

class ProgressDialog;

class RTABMAP_GUI_EXPORT DepthCalibrationDialog : public QDialog
{
	Q_OBJECT

public:
	DepthCalibrationDialog(QWidget * parent = 0);
	virtual ~DepthCalibrationDialog();

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");

	void calibrate(const std::map<int, Transform> & poses,
			const QMap<int, Signature> & cachedSignatures,
			const QString & workingDirectory,
			const ParametersMap & parameters);

Q_SIGNALS:
	void configChanged();

public Q_SLOTS:
	void restoreDefaults();

private Q_SLOTS:
	void saveModel();
	void cancel();

private:
	Ui_DepthCalibrationDialog * _ui;
	ProgressDialog * _progressDialog;
	bool _canceled;
	clams::DiscreteDepthDistortionModel * _model;
	QString _workingDirectory;
	cv::Size _imageSize;
};

} /* namespace rtabmap */

#endif /* GUILIB_SRC_DEPTHCALIBRATIONDIALOG_H_ */
