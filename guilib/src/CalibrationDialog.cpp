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

#include "rtabmap/gui/CalibrationDialog.h"
#include "ui_calibrationDialog.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/gui/UCv2Qt.h>

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

#define COUNT_MIN 12

CalibrationDialog::CalibrationDialog(QWidget * parent) :
	QDialog(parent),
	boardSize_(8,6),
	squareSize_(0.033),
	calibrated_(false),
	cameraMatrix_(cv::Mat::eye(3, 3, CV_64F)),
	distCoeffs_(cv::Mat::zeros(8, 1, CV_64F))
{
	qRegisterMetaType<cv::Mat>("cv::Mat");

	ui_ = new Ui_calibrationDialog();
	ui_->setupUi(this);

	connect(ui_->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(ui_->pushButton_restart, SIGNAL(clicked()), this, SLOT(restart()));

	connect(ui_->spinBox_boardWidth, SIGNAL(valueChanged(int)), this, SLOT(setBoardWidth(int)));
	connect(ui_->spinBox_boardHeight, SIGNAL(valueChanged(int)), this, SLOT(setBoardHeight(int)));
	connect(ui_->doubleSpinBox_squareSize, SIGNAL(valueChanged(double)), this, SLOT(setSquareSize(double)));

	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
	connect(ui_->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));

	ui_->image_view->setFocus();

	ui_->progressBar_count->setMaximum(COUNT_MIN);
	ui_->progressBar_count->setFormat("%v");

	this->restart();
}

CalibrationDialog::~CalibrationDialog()
{
	this->unregisterFromEventsManager();
	delete ui_;
}

void CalibrationDialog::setBoardWidth(int width)
{
	if(width != boardSize_.width)
	{
		boardSize_.width = width;
		this->restart();
	}
}

void CalibrationDialog::setBoardHeight(int height)
{
	if(height != boardSize_.height)
	{
		boardSize_.height = height;
		this->restart();
	}
}

void CalibrationDialog::setSquareSize(double size)
{
	if(size != squareSize_)
	{
		squareSize_ = size;
		this->restart();
	}
}

void CalibrationDialog::closeEvent(QCloseEvent* event)
{
	this->unregisterFromEventsManager();
}

void CalibrationDialog::handleEvent(UEvent * event)
{
	if(event->getClassName().compare("CameraEvent") == 0)
	{
		rtabmap::CameraEvent * e = (rtabmap::CameraEvent *)event;
		if(e->getCode() == rtabmap::CameraEvent::kCodeImage ||
		   e->getCode() == rtabmap::CameraEvent::kCodeImageDepth)
		{
			QMetaObject::invokeMethod(this, "processImage", Q_ARG(cv::Mat, e->data().image()));
		}
	}
}

void CalibrationDialog::processImage(const cv::Mat & image)
{
	imageSize_ = image.size();
	std::vector<cv::Point2f> pointBuf;
	bool found = cv::findChessboardCorners( image, boardSize_, pointBuf,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

	if ( found) // If done with success,
	{
		// improve the found corners' coordinate accuracy for chessboard
		cv::Mat viewGray;
		cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);

		int border = 8; // minimum distance from border
		bool reject = false;
		for(unsigned int i=0; i<pointBuf.size(); ++i)
		{
			if(pointBuf[i].x < border || pointBuf[i].x > image.cols-border ||
			   pointBuf[i].y < border || pointBuf[i].y > image.rows-border)
			{
				reject = false;
				break;
			}
		}

		if(!reject)
		{
			float minSquareDistance = -1.0f;
			for(unsigned int i=0; i<pointBuf.size()-1; ++i)
			{
				float d = cv::norm(pointBuf[i] - pointBuf[i+1]);
				if(minSquareDistance == -1.0f || minSquareDistance > d)
				{
					minSquareDistance = d;
				}
			}
			float radius = minSquareDistance/2.0f +0.5f;
			cv::cornerSubPix( viewGray, pointBuf, cv::Size(radius, radius), cv::Size(-1,-1),
					cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));

			// verify if view is different from any previous samples
			std::vector<float> params(4, 0);
			getParams(pointBuf, boardSize_, imageSize_, params[0], params[1], params[2], params[3]);

			bool add = true;
			for(unsigned int i=0; i<imageParams_.size(); ++i)
			{
				if(fabs(params[0] - imageParams_[i].at(0)) < 0.1 && // x
					fabs(params[1] - imageParams_[i].at(1)) < 0.1 && // y
					fabs(params[2] - imageParams_[i].at(2)) < 0.1 && // size
					fabs(params[3] - imageParams_[i].at(3)) < 0.1) // skew
				{
					add = false;
				}
			}
			if(add)
			{
				UINFO("Added board. (x=%f, y=%f, size=%f, skew=%f)", params[0], params[1], params[2], params[3]);
				imagePoints_.push_back(pointBuf);
				imageParams_.push_back(params);

				// update statistics
				std::vector<float> xRange(2, imageParams_[0].at(0));
				std::vector<float> yRange(2, imageParams_[0].at(1));
				std::vector<float> sizeRange(2, imageParams_[0].at(2));
				std::vector<float> skewRange(2, imageParams_[0].at(3));
				for(unsigned int i=1; i<imageParams_.size(); ++i)
				{
					xRange[0] = imageParams_[i].at(0) < xRange[0] ? imageParams_[i].at(0) : xRange[0];
					xRange[1] = imageParams_[i].at(0) > xRange[1] ? imageParams_[i].at(0) : xRange[1];
					yRange[0] = imageParams_[i].at(1) < yRange[0] ? imageParams_[i].at(1) : yRange[0];
					yRange[1] = imageParams_[i].at(1) > yRange[1] ? imageParams_[i].at(1) : yRange[1];
					sizeRange[0] = imageParams_[i].at(2) < sizeRange[0] ? imageParams_[i].at(2) : sizeRange[0];
					sizeRange[1] = imageParams_[i].at(2) > sizeRange[1] ? imageParams_[i].at(2) : sizeRange[1];
					skewRange[0] = imageParams_[i].at(3) < skewRange[0] ? imageParams_[i].at(3) : skewRange[0];
					skewRange[1] = imageParams_[i].at(3) > skewRange[1] ? imageParams_[i].at(3) : skewRange[1];
				}
				UINFO("Stats:");
				UINFO("  Count = %d", (int)imagePoints_.size());
				UINFO("  x =    [%f -> %f]", xRange[0], xRange[1]);
				UINFO("  y =    [%f -> %f]", yRange[0], yRange[1]);
				UINFO("  size = [%f -> %f]", sizeRange[0], sizeRange[1]);
				UINFO("  skew = [%f -> %f]", skewRange[0], skewRange[1]);

				float xGood = xRange[1] - xRange[0];
				float yGood = yRange[1] - yRange[0];
				float sizeGood = sizeRange[1] - sizeRange[0];
				float skewGood = skewRange[1] - skewRange[0];

				if((int)imagePoints_.size() > ui_->progressBar_count->maximum())
				{
					ui_->progressBar_count->setMaximum((int)imagePoints_.size());
				}
				ui_->progressBar_count->setValue((int)imagePoints_.size());
				ui_->progressBar_x->setValue(xGood*100);
				ui_->progressBar_y->setValue(yGood*100);
				ui_->progressBar_size->setValue(sizeGood*100);
				ui_->progressBar_skew->setValue(skewGood*100);

				if(imagePoints_.size() >= COUNT_MIN && xGood > 0.5 && yGood > 0.5 && sizeGood > 0.4 && skewGood > 0.5)
				{
					ui_->pushButton_calibrate->setEnabled(true);
				}
			}
		}

		// Draw the corners.
		cv::drawChessboardCorners( image, boardSize_, cv::Mat(pointBuf), found );
	}

	if(calibrated_ && ui_->checkBox_rectified->isChecked())
	{
		cv::Mat temp = image.clone();
		cv::undistort(temp, image, cameraMatrix_, distCoeffs_);
	}

	//show frame
	ui_->image_view->setImage(uCvMat2QImage(image));
}

void CalibrationDialog::restart()
{
	// restart
	calibrated_ = false;
	imagePoints_.clear();
	imageParams_.clear();

	ui_->pushButton_calibrate->setEnabled(false);
	ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	ui_->checkBox_rectified->setEnabled(false);

	ui_->progressBar_count->reset();
	ui_->progressBar_count->setMaximum(COUNT_MIN);
	ui_->progressBar_x->reset();
	ui_->progressBar_y->reset();
	ui_->progressBar_size->reset();
	ui_->progressBar_skew->reset();

	ui_->label_fx->setNum(0);
	ui_->label_fy->setNum(0);
	ui_->label_cx->setNum(0);
	ui_->label_cy->setNum(0);
	ui_->label_error->setNum(0);
	ui_->lineEdit_K->clear();
	ui_->lineEdit_D->clear();
}

void CalibrationDialog::calibrate()
{
	//calibrate
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	// compute board corner positions
	 for( int i = 0; i < boardSize_.height; ++i )
		for( int j = 0; j < boardSize_.width; ++j )
			objectPoints[0].push_back(cv::Point3f(float( j*squareSize_ ), float( i*squareSize_ ), 0));

	objectPoints.resize(imagePoints_.size(),objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = cv::calibrateCamera(objectPoints,
			imagePoints_,
			imageSize_,
			cameraMatrix_,
			 distCoeffs_,
			 rvecs,
			 tvecs,
			 CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

	std::cout << "cameraMatrix = " << cameraMatrix_ << std::endl;
	std::cout << "distCoeffs = " << distCoeffs_ << std::endl;

	UINFO("Re-projection error reported by calibrateCamera: %f", rms);

	calibrated_ = checkRange(cameraMatrix_) && checkRange(distCoeffs_);

	// compute reprojection errors
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	reprojErrs.resize(objectPoints.size());

	for( i = 0; i < (int)objectPoints.size(); ++i )
	{
		cv::projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix_,
					   distCoeffs_, imagePoints2);
		err = cv::norm(cv::Mat(imagePoints_[i]), cv::Mat(imagePoints2), CV_L2);

		int n = (int)objectPoints[i].size();
		reprojErrs[i] = (float) std::sqrt(err*err/n);
		totalErr        += err*err;
		totalPoints     += n;
	}

	totalAvgErr =  std::sqrt(totalErr/totalPoints);

	UINFO("%s. avg re projection error = %f", calibrated_ ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

	if(calibrated_)
	{
		ui_->label_fx->setNum(cameraMatrix_.at<double>(0,0)); // K(0)
		ui_->label_fy->setNum(cameraMatrix_.at<double>(1,1)); // K(4)
		ui_->label_cx->setNum(cameraMatrix_.at<double>(0,2)); // K(2)
		ui_->label_cy->setNum(cameraMatrix_.at<double>(1,2)); // K(5)
		ui_->label_error->setNum(totalAvgErr);

		std::stringstream strK, strD;
		strK << cameraMatrix_;
		strD << distCoeffs_;
		ui_->lineEdit_K->setText(strK.str().c_str());
		ui_->lineEdit_D->setText(strD.str().c_str());

		ui_->checkBox_rectified->setEnabled(true);
		ui_->checkBox_rectified->setChecked(true);

		ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
	}
}

float CalibrationDialog::getArea(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize)
{
	//Get 2d image area of the detected checkerboard.
	//The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
	//|p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.

	cv::Point2f up_left = corners[0];
	cv::Point2f up_right = corners[boardSize.width-1];
	cv::Point2f down_right = corners[corners.size()-1];
	cv::Point2f down_left = corners[corners.size()-boardSize.width];
	cv::Point2f a = up_right - up_left;
	cv::Point2f b = down_right - up_right;
	cv::Point2f c = down_left - down_right;
	cv::Point2f p = b + c;
	cv::Point2f q = a + b;
	return std::fabs(p.x*q.y - p.y*q.x) / 2.0f;
}

float CalibrationDialog::getSkew(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize)
{
   // Get skew for given checkerboard detection.
   // Scaled to [0,1], which 0 = no skew, 1 = high skew
   // Skew is proportional to the divergence of three outside corners from 90 degrees.

	cv::Point2f up_left = corners[0];
	cv::Point2f up_right = corners[boardSize.width-1];
	cv::Point2f down_right = corners[corners.size()-1];


	//  Return angle between lines ab, bc
	cv::Point2f ab = up_left - up_right;
	cv::Point2f cb = down_right - up_right;
	float angle =  std::acos(ab.dot(cb) / (cv::norm(ab) * cv::norm(cb)));

	float r = 2.0f * std::fabs((CV_PI / 2.0f) - angle);
	return r > 1.0f?1.0f:r;
}

// x -> [0, 1] (left, right)
// y -> [0, 1] (top, bottom)
// size -> [0, 1] (small -> big)
// skew -> [0, 1] (low, high)
void CalibrationDialog::getParams(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize, const cv::Size & imageSize,
		float & x, float & y, float & size, float & skew)
{
	float area = getArea(corners, boardSize);
	size = std::sqrt(area / (imageSize.width * imageSize.height));
	skew = getSkew(corners, boardSize);
	float meanX = 0.0f;
	float meanY = 0.0f;
	for(unsigned int i=0; i<corners.size(); ++i)
	{
		meanX += corners[i].x;
		meanY += corners[i].y;
	}
	meanX /= corners.size();
	meanY /= corners.size();
	x = meanX / imageSize.width;
	y = meanY / imageSize.height;
}

} /* namespace rtabmap */
