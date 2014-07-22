/*
 * CalibrationDialog.h
 *
 *  Created on: 2014-07-22
 *      Author: Mathieu
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

private slots:
	void processImage(const cv::Mat & image);
	void restart();
	void calibrate();
	void exit();

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
