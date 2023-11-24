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

#ifndef RTABMAP_CALIBRATIONDIALOG_H_
#define RTABMAP_CALIBRATIONDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QDialog>
#include <QSettings>
#include <opencv2/opencv.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>

#include <rtabmap/utilite/UEventsHandler.h>

class Ui_calibrationDialog;

namespace rtabmap {

class RTABMAP_GUI_EXPORT CalibrationDialog  : public QDialog, public UEventsHandler
{
	Q_OBJECT;

public:
	CalibrationDialog(bool stereo = false, const QString & savingDirectory = ".", bool switchImages = false, QWidget * parent = 0);
	virtual ~CalibrationDialog();

	bool isCalibrated() const {return models_[0].isValidForProjection() && (stereo_?models_[1].isValidForProjection():true);}
	const rtabmap::CameraModel & getLeftCameraModel() const {return models_[0];}
	const rtabmap::CameraModel & getRightCameraModel() const {return models_[1];}
	const rtabmap::StereoCameraModel & getStereoCameraModel() const {return stereoModel_;}
	bool isProcessing() const {return processingData_;}
	int getStereoPairs() const {return (int)stereoImagePoints_[0].size();}
	int boardWidth() const;
	int boardHeight() const;
	double squareSize() const;

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");
	void resetSettings();

	void setCameraName(const QString & name);
	void setProgressVisibility(bool visible);
	void setSwitchedImages(bool switched);
	void setFisheyeImages(bool enabled);
	void setStereoMode(bool stereo, const QString & leftSuffix = "left", const QString & rightSuffix = "right");
	void setSavingDirectory(const QString & savingDirectory) {savingDirectory_ = savingDirectory;}

	StereoCameraModel stereoCalibration(const CameraModel & left, const CameraModel & right, bool ignoreStereoRectification) const;

public Q_SLOTS:
	void setBoardWidth(int width);
	void setBoardHeight(int height);
	void setSquareSize(double size);
	void setExpectedStereoBaseline(double length);
	void setMaxScale(int scale);

	void processImages(const cv::Mat & imageLeft, const cv::Mat & imageRight, const QString & cameraName);
	void calibrate();
	void restart();
	bool save();

private Q_SLOTS:
	void unlock();

protected:
	virtual void closeEvent(QCloseEvent* event);
	virtual bool handleEvent(UEvent * event);

private:
	float getArea(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize);
	float getSkew(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize);

	// x -> [0, 1] (left, right)
	// y -> [0, 1] (top, bottom)
	// size -> [0, 1] (small -> big)
	// skew -> [0, 1] (low, high)
	void getParams(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize, const cv::Size & imageSize,
			float & x, float & y, float & size, float & skew);

private:
	// parameters
	bool stereo_;
	QString leftSuffix_;
	QString rightSuffix_;
	QString savingDirectory_;

	QString cameraName_;
	bool processingData_;
	bool savedCalibration_;

	std::vector<std::vector<std::vector<cv::Point2f> > > imagePoints_;
	std::vector<std::vector<std::vector<float> > > imageParams_;
	std::vector<std::vector<std::vector<cv::Point2f> > > stereoImagePoints_;
	std::vector<cv::Size > imageSize_;
	std::vector<rtabmap::CameraModel> models_;
	rtabmap::StereoCameraModel stereoModel_;
	std::vector<unsigned short> minIrs_;
	std::vector<unsigned short> maxIrs_;

	Ui_calibrationDialog * ui_;
};

} /* namespace rtabmap */
#endif /* CALIBRATIONDIALOG_H_ */
