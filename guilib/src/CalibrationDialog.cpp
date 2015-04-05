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

#include <QFileDialog>
#include <QMessageBox>

#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/gui/UCv2Qt.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

namespace rtabmap {

#define COUNT_MIN 40

CalibrationDialog::CalibrationDialog(bool stereo, QWidget * parent) :
	QDialog(parent),
	boardSize_(8,6),
	squareSize_(0.033),
	stereo_(stereo),
	calibrated_(false),
	processingData_(false)
{
	imagePoints_.resize(2);
	imageParams_.resize(2);
	imageSize_.resize(2);

	qRegisterMetaType<cv::Mat>("cv::Mat");

	ui_ = new Ui_calibrationDialog();
	ui_->setupUi(this);

	if(!stereo_)
	{
		ui_->progressBar_x_2->setVisible(false);
		ui_->progressBar_y_2->setVisible(false);
		ui_->progressBar_size_2->setVisible(false);
		ui_->progressBar_skew_2->setVisible(false);
		ui_->image_view_2->setVisible(false);
		ui_->label_fx_2->setVisible(false);
		ui_->label_fy_2->setVisible(false);
		ui_->label_cx_2->setVisible(false);
		ui_->label_cy_2->setVisible(false);
		ui_->label_baseline->setVisible(false);
		ui_->label_baseline_name->setVisible(false);
		ui_->lineEdit_K_2->setVisible(false);
		ui_->lineEdit_D_2->setVisible(false);
		ui_->lineEdit_R_2->setVisible(false);
		ui_->lineEdit_P_2->setVisible(false);
	}

	connect(ui_->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(ui_->pushButton_restart, SIGNAL(clicked()), this, SLOT(restart()));
	connect(ui_->pushButton_save, SIGNAL(clicked()), this, SLOT(save()));

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
	if(!processingData_)
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			rtabmap::CameraEvent * e = (rtabmap::CameraEvent *)event;
			if(e->getCode() == rtabmap::CameraEvent::kCodeImage ||
			   e->getCode() == rtabmap::CameraEvent::kCodeImageDepth)
			{
				processingData_ = true;
				QMetaObject::invokeMethod(this, "processImages",
						Q_ARG(cv::Mat, e->data().image()),
						Q_ARG(cv::Mat, e->data().depthOrRightImage()),
						Q_ARG(QString, QString(e->cameraName().c_str())));
			}
		}
	}
}

void CalibrationDialog::processImages(const cv::Mat & imageLeft, const cv::Mat & imageRight, const QString & cameraName)
{
	processingData_ = true;
	if(cameraName_.isEmpty())
	{
		cameraName_ = "0000";
		if(!cameraName.isEmpty())
		{
			cameraName_ = cameraName;
		}
	}
	if(ui_->label_serial->text().isEmpty())
	{
		ui_->label_serial->setText(cameraName_);

	}
	std::vector<cv::Mat> images(2);
	images[0] = imageLeft;
	images[1] = imageRight;
	imageSize_[0] = imageLeft.size();
	imageSize_[1] = imageRight.size();

	std::vector<std::vector<cv::Point2f> > pointBuf(2);
	std::vector<std::vector<float> > params(2, std::vector<float>(4, 0));
	bool boardAccepted[2] = {false};

	for(int id=0; id<(stereo_?2:1); ++id)
	{
		cv::Mat viewGray;
		if(!images[id].empty())
		{
			if(images[id].channels() == 3)
			{
				cvtColor(images[id], viewGray, cv::COLOR_BGR2GRAY);
			}
			else
			{
				viewGray = images[id];
				cvtColor(viewGray, images[id], cv::COLOR_GRAY2BGR); // convert to show detected points in color
			}
		}
		else
		{
			UERROR("Image %d is empty!! Should not!", id);
		}

		bool found = false;
		if(!viewGray.empty())
		{
			int maxScale = 1;
			for( int scale = 1; scale <= maxScale; scale++ )
			{
				cv::Mat timg;
				if( scale == 1 )
					timg = viewGray;
				else
					cv::resize(viewGray, timg, cv::Size(), scale, scale);
				found = cv::findChessboardCorners(timg, boardSize_, pointBuf[id],
						CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if(found)
				{
					if( scale > 1 )
					{
						cv::Mat cornersMat(pointBuf[id]);
						cornersMat *= 1./scale;
					}
					break;
				}
			}
		}

		if(found) // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			float minSquareDistance = -1.0f;
			for(unsigned int i=0; i<pointBuf[id].size()-1; ++i)
			{
				float d = cv::norm(pointBuf[id][i] - pointBuf[id][i+1]);
				if(minSquareDistance == -1.0f || minSquareDistance > d)
				{
					minSquareDistance = d;
				}
			}
			float radius = minSquareDistance/2.0f +0.5f;
			cv::cornerSubPix( viewGray, pointBuf[id], cv::Size(radius, radius), cv::Size(-1,-1),
					cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));

			boardAccepted[id] = true;
			getParams(pointBuf[id], boardSize_, imageSize_[id], params[id][0], params[id][1], params[id][2], params[id][3]);

			// Draw the corners.
			cv::drawChessboardCorners(images[id], boardSize_, cv::Mat(pointBuf[id]), found);
		}
	}

	bool readyToCalibrate[2] = {false};
	if(boardAccepted[0] && (!stereo_ || boardAccepted[1]))
	{
		// verify if view is different from any previous samples
		bool addSample = true;
		for(int id=0; id<(stereo_?2:1); ++id)
		{
			for(unsigned int i=0; i<imageParams_[id].size(); ++i)
			{
				if(fabs(params[id][0] - imageParams_[id][i].at(0)) < 0.1 && // x
					fabs(params[id][1] - imageParams_[id][i].at(1)) < 0.1 && // y
					fabs(params[id][2] - imageParams_[id][i].at(2)) < 0.05 && // size
					fabs(params[id][3] - imageParams_[id][i].at(3)) < 0.1) // skew
				{
					addSample = false;
				}
			}
			if(addSample)
			{
				break;
			}
		}

		if(addSample)
		{
			for(int id=0; id<(stereo_?2:1); ++id)
			{
				UINFO("Added board. (x=%f, y=%f, size=%f, skew=%f)", params[id][0], params[id][1], params[id][2], params[id][3]);
				imagePoints_[id].push_back(pointBuf[id]);
				imageParams_[id].push_back(params[id]);

				// update statistics
				std::vector<float> xRange(2, imageParams_[id][0].at(0));
				std::vector<float> yRange(2, imageParams_[id][0].at(1));
				std::vector<float> sizeRange(2, imageParams_[id][0].at(2));
				std::vector<float> skewRange(2, imageParams_[id][0].at(3));
				for(unsigned int i=1; i<imageParams_[id].size(); ++i)
				{
					xRange[0] = imageParams_[id][i].at(0) < xRange[0] ? imageParams_[id][i].at(0) : xRange[0];
					xRange[1] = imageParams_[id][i].at(0) > xRange[1] ? imageParams_[id][i].at(0) : xRange[1];
					yRange[0] = imageParams_[id][i].at(1) < yRange[0] ? imageParams_[id][i].at(1) : yRange[0];
					yRange[1] = imageParams_[id][i].at(1) > yRange[1] ? imageParams_[id][i].at(1) : yRange[1];
					sizeRange[0] = imageParams_[id][i].at(2) < sizeRange[0] ? imageParams_[id][i].at(2) : sizeRange[0];
					sizeRange[1] = imageParams_[id][i].at(2) > sizeRange[1] ? imageParams_[id][i].at(2) : sizeRange[1];
					skewRange[0] = imageParams_[id][i].at(3) < skewRange[0] ? imageParams_[id][i].at(3) : skewRange[0];
					skewRange[1] = imageParams_[id][i].at(3) > skewRange[1] ? imageParams_[id][i].at(3) : skewRange[1];
				}
				UINFO("Stats:");
				UINFO("  Count = %d", (int)imagePoints_[id].size());
				UINFO("  x =    [%f -> %f]", xRange[0], xRange[1]);
				UINFO("  y =    [%f -> %f]", yRange[0], yRange[1]);
				UINFO("  size = [%f -> %f]", sizeRange[0], sizeRange[1]);
				UINFO("  skew = [%f -> %f]", skewRange[0], skewRange[1]);

				float xGood = xRange[1] - xRange[0];
				float yGood = yRange[1] - yRange[0];
				float sizeGood = sizeRange[1] - sizeRange[0];
				float skewGood = skewRange[1] - skewRange[0];

				if(id == 0)
				{
					ui_->progressBar_x->setValue(xGood*100);
					ui_->progressBar_y->setValue(yGood*100);
					ui_->progressBar_size->setValue(sizeGood*100);
					ui_->progressBar_skew->setValue(skewGood*100);
				}
				else
				{
					ui_->progressBar_x_2->setValue(xGood*100);
					ui_->progressBar_y_2->setValue(yGood*100);
					ui_->progressBar_size_2->setValue(sizeGood*100);
					ui_->progressBar_skew_2->setValue(skewGood*100);
				}

				if(imagePoints_[id].size() >= COUNT_MIN && xGood > 0.5 && yGood > 0.5 && sizeGood > 0.4 && skewGood > 0.5)
				{
					readyToCalibrate[id] = true;
				}
			}

			if((int)imagePoints_[0].size() > ui_->progressBar_count->maximum())
			{
				ui_->progressBar_count->setMaximum((int)imagePoints_[0].size());
			}
			ui_->progressBar_count->setValue((int)imagePoints_[0].size());
		}
	}

	if(!stereo_ && readyToCalibrate[0])
	{
		ui_->pushButton_calibrate->setEnabled(true);
		ui_->pushButton_save->setEnabled(true);
	}
	else if(stereo_ && readyToCalibrate[0] && readyToCalibrate[1])
	{
		ui_->pushButton_calibrate->setEnabled(true);
		ui_->pushButton_save->setEnabled(true);
	}

	if(calibrated_ && ui_->checkBox_rectified->isChecked())
	{
		if(!stereo_)
		{
			images[0] = model_.rectifyImage(images[0]);
		}
		else
		{
			images[0] = stereoModel_.left().rectifyImage(images[0]);
			images[1] = stereoModel_.right().rectifyImage(images[1]);
		}
	}

	if(ui_->checkBox_showHorizontalLines->isChecked())
	{
		for(int id=0; id<(stereo_?2:1); ++id)
		{
			int step = imageSize_[id].height/12;
			for(int i=step; i<imageSize_[id].height; i+=step)
			{
				cv::line(images[id], cv::Point(0, i), cv::Point(imageSize_[id].width, i), CV_RGB(0,255,0));
			}
		}
	}

	//show frame
	ui_->image_view->setImage(uCvMat2QImage(images[0]).mirrored(ui_->checkBox_mirror->isChecked(), false));
	if(stereo_)
	{
		ui_->image_view_2->setImage(uCvMat2QImage(images[1]).mirrored(ui_->checkBox_mirror->isChecked(), false));
	}
	processingData_ = false;
}

void CalibrationDialog::restart()
{
	// restart
	calibrated_ = false;
	imagePoints_[0].clear();
	imagePoints_[1].clear();
	imageParams_[0].clear();
	imageParams_[1].clear();
	model_ = CameraModel();
	stereoModel_ = StereoCameraModel();
	cameraName_.clear();

	ui_->pushButton_calibrate->setEnabled(false);
	ui_->pushButton_save->setEnabled(false);
	ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	ui_->checkBox_rectified->setEnabled(false);

	ui_->progressBar_count->reset();
	ui_->progressBar_count->setMaximum(COUNT_MIN);
	ui_->progressBar_x->reset();
	ui_->progressBar_y->reset();
	ui_->progressBar_size->reset();
	ui_->progressBar_skew->reset();

	ui_->progressBar_x_2->reset();
	ui_->progressBar_y_2->reset();
	ui_->progressBar_size_2->reset();
	ui_->progressBar_skew_2->reset();

	ui_->label_serial->clear();
	ui_->label_fx->setNum(0);
	ui_->label_fy->setNum(0);
	ui_->label_cx->setNum(0);
	ui_->label_cy->setNum(0);
	ui_->label_baseline->setNum(0);
	ui_->label_error->setNum(0);
	ui_->lineEdit_K->clear();
	ui_->lineEdit_D->clear();
	ui_->lineEdit_R->clear();
	ui_->lineEdit_P->clear();
	ui_->label_fx_2->setNum(0);
	ui_->label_fy_2->setNum(0);
	ui_->label_cx_2->setNum(0);
	ui_->label_cy_2->setNum(0);
	ui_->lineEdit_K_2->clear();
	ui_->lineEdit_D_2->clear();
	ui_->lineEdit_R_2->clear();
	ui_->lineEdit_P_2->clear();
}

void CalibrationDialog::calibrate()
{
	UINFO("Calibration...");
	processingData_ = true;
	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	// compute board corner positions
	for( int i = 0; i < boardSize_.height; ++i )
		for( int j = 0; j < boardSize_.width; ++j )
			objectPoints[0].push_back(cv::Point3f(float( j*squareSize_ ), float( i*squareSize_ ), 0));

	objectPoints.resize(imagePoints_[0].size(),objectPoints[0]);

	if(!stereo_)
	{
		//calibrate
		std::vector<cv::Mat> rvecs, tvecs;
		std::vector<float> reprojErrs;
		cv::Mat K, D;
		K = cv::Mat::eye(3,3,CV_64FC1);

		//Find intrinsic and extrinsic camera parameters
		double rms = cv::calibrateCamera(objectPoints,
				imagePoints_[0],
				imageSize_[0],
				K,
				D,
				rvecs,
				tvecs,
				CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

		std::cout << "cameraMatrix = " << K << std::endl;
		std::cout << "distCoeffs = " << D << std::endl;

		UINFO("Re-projection error reported by calibrateCamera: %f", rms);

		calibrated_ = checkRange(K) && checkRange(D);

		// compute reprojection errors
		std::vector<cv::Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		reprojErrs.resize(objectPoints.size());

		for( i = 0; i < (int)objectPoints.size(); ++i )
		{
			cv::projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], K, D, imagePoints2);
			err = cv::norm(cv::Mat(imagePoints_[0][i]), cv::Mat(imagePoints2), CV_L2);

			int n = (int)objectPoints[i].size();
			reprojErrs[i] = (float) std::sqrt(err*err/n);
			totalErr        += err*err;
			totalPoints     += n;
		}

		double totalAvgErr =  std::sqrt(totalErr/totalPoints);

		UINFO("%s. avg re projection error = %f", calibrated_ ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

		if(calibrated_)
		{
			cv::Mat P(3,4,CV_64FC1);
			P.at<double>(2,3) = 1;
			K.copyTo(P.colRange(0,3).rowRange(0,3));

			model_ = CameraModel(cameraName_.toStdString(), imageSize_[0], K, D, cv::Mat::eye(3,3,CV_64FC1), P);

			ui_->label_fx->setNum(model_.fx());
			ui_->label_fy->setNum(model_.fy());
			ui_->label_cx->setNum(model_.cx());
			ui_->label_cy->setNum(model_.cy());
			ui_->label_error->setNum(totalAvgErr);

			std::stringstream strK, strD, strR, strP;
			strK << model_.K();
			strD << model_.D();
			strR << model_.R();
			strP << model_.P();
			ui_->lineEdit_K->setText(strK.str().c_str());
			ui_->lineEdit_D->setText(strD.str().c_str());
			ui_->lineEdit_R->setText(strR.str().c_str());
			ui_->lineEdit_P->setText(strP.str().c_str());

			ui_->checkBox_rectified->setEnabled(true);
			ui_->checkBox_rectified->setChecked(true);

			ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
			ui_->pushButton_save->setEnabled(true);
		}
	}
	else
	{
		UINFO("stereo calibration...");
		cv::Size imageSize = imageSize_[0];
		std::vector<cv::Mat> K(2), D(2);
		K[0] = cv::Mat::eye(3, 3, CV_64F);
		K[1] = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat R, T, E, F;

		double rms = cv::stereoCalibrate(objectPoints, imagePoints_[0], imagePoints_[1],
						K[0], D[0],
						K[1], D[1],
						imageSize, R, T, E, F,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5),
						cv::CALIB_FIX_ASPECT_RATIO +
						cv::CALIB_ZERO_TANGENT_DIST +
						cv::CALIB_SAME_FOCAL_LENGTH +
						cv::CALIB_RATIONAL_MODEL +
						cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);
		UINFO("stereo calibration... done with RMS error=%f", rms);

		calibrated_ = checkRange(K[0]) && checkRange(D[0]) && checkRange(K[1]) && checkRange(D[1]);

		double err = 0;
		int npoints = 0;
		std::vector<cv::Vec3f> lines[2];
		UINFO("Computing avg re-projection error...");
		for(unsigned int i = 0; i < imagePoints_[0].size(); i++ )
		{
			int npt = (int)imagePoints_[0][i].size();
			cv::Mat imgpt[2];
			for(int k = 0; k < 2; k++ )
			{
				imgpt[k] = cv::Mat(imagePoints_[k][i]);
				cv::undistortPoints(imgpt[k], imgpt[k], K[k], D[k], cv::Mat(), K[k]);
				computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
			}
			for(int j = 0; j < npt; j++ )
			{
				double errij = fabs(imagePoints_[0][i][j].x*lines[1][j][0] +
									 imagePoints_[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
							   fabs(imagePoints_[1][i][j].x*lines[0][j][0] +
								    imagePoints_[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
				err += errij;
			}
			npoints += npt;
		}
		double totalAvgErr = err/(double)npoints;

		UINFO("%s. avg re projection error = %f", calibrated_ ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

		if(calibrated_)
		{

			cv::Mat R1, R2, P1, P2, Q;
			cv::Rect validRoi[2];

			cv::stereoRectify(K[0], D[0],
							K[1], D[1],
							imageSize, R, T, R1, R2, P1, P2, Q,
							cv::CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);

			UINFO("Valid ROI1 = %d,%d,%d,%d  ROI2 = %d,%d,%d,%d",
					validRoi[0].x, validRoi[0].y, validRoi[0].width, validRoi[0].height,
					validRoi[1].x, validRoi[1].y, validRoi[1].width, validRoi[1].height);

			stereoModel_ = StereoCameraModel(cameraName_.toStdString(), imageSize, K[0], D[0], R1, P1, K[1], D[1], R2, P2);

			ui_->label_fx->setNum(stereoModel_.left().fx());
			ui_->label_fy->setNum(stereoModel_.left().fy());
			ui_->label_cx->setNum(stereoModel_.left().cx());
			ui_->label_cy->setNum(stereoModel_.left().cy());
			ui_->label_fx_2->setNum(stereoModel_.right().fx());
			ui_->label_fy_2->setNum(stereoModel_.right().fy());
			ui_->label_cx_2->setNum(stereoModel_.right().cx());
			ui_->label_cy_2->setNum(stereoModel_.right().cy());
			ui_->label_baseline->setVisible(stereoModel_.baseline());
			ui_->label_error->setNum(totalAvgErr);

			std::stringstream strK, strD, strR, strP;
			strK << stereoModel_.left().K();
			strD << stereoModel_.left().D();
			strR << stereoModel_.left().R();
			strP << stereoModel_.left().P();
			ui_->lineEdit_K->setText(strK.str().c_str());
			ui_->lineEdit_D->setText(strD.str().c_str());
			ui_->lineEdit_R->setText(strR.str().c_str());
			ui_->lineEdit_P->setText(strP.str().c_str());
			strK.clear();
			strD.clear();
			strR.clear();
			strP.clear();
			strK << stereoModel_.right().K();
			strD << stereoModel_.right().D();
			strR << stereoModel_.right().R();
			strP << stereoModel_.right().P();
			ui_->lineEdit_K_2->setText(strK.str().c_str());
			ui_->lineEdit_D_2->setText(strD.str().c_str());
			ui_->lineEdit_R_2->setText(strR.str().c_str());
			ui_->lineEdit_P_2->setText(strP.str().c_str());

			ui_->checkBox_rectified->setEnabled(true);
			ui_->checkBox_rectified->setChecked(true);

			ui_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
			ui_->pushButton_save->setEnabled(true);
		}
	}

	UINFO("End calibration");

	processingData_ = false;
}

void CalibrationDialog::save()
{
	processingData_ = true;
	if(!stereo_)
	{
		UASSERT(model_.isValid());
		QString cameraName = model_.name().c_str();
		QString filePath = QFileDialog::getSaveFileName(this, tr("Export"), cameraName+".yaml", "*.yaml");

		if(!filePath.isEmpty())
		{
			if(model_.save(filePath.toStdString()))
			{
				QMessageBox::information(this, tr("Export"), tr("Calibration file saved to \"%1\".").arg(filePath));
				UINFO("Saved \"%s\"!", filePath.toStdString().c_str());
			}
			else
			{
				UERROR("Error saving \"%s\"", filePath.toStdString().c_str());
			}
		}
	}
	else
	{
		UASSERT(stereoModel_.isValid());
		QString cameraName = stereoModel_.name().c_str();
		QString filePath = QFileDialog::getSaveFileName(this, tr("Export"), cameraName, "*.yaml");
		std::string name = UFile::getName(filePath.toStdString());
		std::string dir = UDirectory::getDir(filePath.toStdString());
		if(!name.empty())
		{
			std::string base = (dir+UDirectory::separator()+name).c_str();
			std::string leftPath = base+"_left.yaml";
			std::string rightPath = base+"_right.yaml";
			if(stereoModel_.save(dir, name))
			{
				QMessageBox::information(this, tr("Export"), tr("Calibration files saved to \"%1\" and \"%2\".").
						arg(leftPath.c_str()).arg(rightPath.c_str()));
				UINFO("Saved \"%s\" and \"%s\"!", leftPath.c_str(), rightPath.c_str());
			}
			else
			{
				UERROR("Error saving \"%s\" and \"%s\"", leftPath.c_str(), rightPath.c_str());
			}
		}
	}
	processingData_ = false;
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
