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

#ifndef CALIBRATIONDIALOG_H_
#define CALIBRATIONDIALOG_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QtGui/QDialog>
#include <opencv2/opencv.hpp>

#include <rtabmap/utilite/UEventsHandler.h>

class Ui_calibrationDialog;

namespace rtabmap {

class RTABMAPGUI_EXP CalibrationDialog  : public QDialog, public UEventsHandler
{
	Q_OBJECT;

public:
	CalibrationDialog(QWidget * parent = 0);
	virtual ~CalibrationDialog();

	bool isCalibrated() const {return calibrated_;}
	const cv::Mat & cameraMatrix() const {return cameraMatrix_;} // Matrix K
	const cv::Mat & distCoeffs() const {return distCoeffs_;} // Matrix D
	float fx() const {return cameraMatrix_.at<double>(0,0);} // K(0)
	float fy() const {return cameraMatrix_.at<double>(1,1);} // K(4)
	float cx() const {return cameraMatrix_.at<double>(0,2);} // K(2)
	float cy() const {return cameraMatrix_.at<double>(1,2);} // K(5)

public slots:
	void setBoardWidth(int width);
	void setBoardHeight(int height);
	void setSquareSize(double size);

private slots:
	void processImage(const cv::Mat & image);
	void restart();
	void calibrate();

protected:
	virtual void closeEvent(QCloseEvent* event);
	virtual void handleEvent(UEvent * event);

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
	cv::Size boardSize_; // innner squares
	float squareSize_; // m

	std::vector<std::vector<cv::Point2f> > imagePoints_;
	std::vector<std::vector<float> > imageParams_;
	cv::Size imageSize_;
	bool calibrated_;
	cv::Mat cameraMatrix_;
	cv::Mat distCoeffs_;

	Ui_calibrationDialog * ui_;
};

} /* namespace rtabmap */
#endif /* CALIBRATIONDIALOG_H_ */
