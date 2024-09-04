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

#include "rtabmap/gui/CalibrationDialog.h"
#include "ui_calibrationDialog.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/calib3d/calib3d_c.h>
#endif
#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
#include <rtabmap/core/stereo/stereoRectifyFisheye.h>
#endif

#include <QFileDialog>
#include <QMessageBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QTextStream>
#include <QtGlobal>

#if QT_VERSION >= QT_VERSION_CHECK(5,14,0)
#define ENDL Qt::endl
#else
#define ENDL endl
#endif

#include <rtabmap/core/SensorEvent.h>
#include <rtabmap/utilite/UCv2Qt.h>

#include <rtabmap/utilite/ULogger.h>

#ifdef HAVE_CHARUCO
#define kArucoDictNameSize 21
static const char * kArucoDictNames[kArucoDictNameSize] = {
"4X4_50",
"4X4_100",
"4X4_250",
"4X4_1000",
"5X5_50",
"5X5_100",
"5X5_250",
"5X5_1000",
"6X6_50",
"6X6_100",
"6X6_250",
"6X6_1000",
"7X7_50",
"7X7_100",
"7X7_250",
"7X7_1000",
"ARUCO_ORIGINAL",
"APRILTAG_16h5",
"APRILTAG_25h9",
"APRILTAG_36h10",
"APRILTAG_36h11"
};
#endif


namespace rtabmap {

#define COUNT_MIN 70

CalibrationDialog::CalibrationDialog(bool stereo, const QString & savingDirectory, bool switchImages, QWidget * parent) :
	QDialog(parent),
	stereo_(stereo),
	leftSuffix_("left"),
	rightSuffix_("right"),
	savingDirectory_(savingDirectory),
	processingData_(false),
	savedCalibration_(false),
	currentId_(0)
{
	imagePoints_.resize(2);
	objectPoints_.resize(2);
	imageParams_.resize(2);
	imageIds_.resize(2);
	imageSize_.resize(2);
	stereoImagePoints_.resize(2);
	models_.resize(2);

	minIrs_.resize(2);
	maxIrs_.resize(2);
	minIrs_[0] = 0x0000;
	maxIrs_[0] = 0x7fff;
	minIrs_[1] = 0x0000;
	maxIrs_[1] = 0x7fff;

	qRegisterMetaType<cv::Mat>("cv::Mat");

	ui_ = new Ui_calibrationDialog();
	ui_->setupUi(this);

	connect(ui_->toolButton_generateBoard, SIGNAL(clicked()), this, SLOT(generateBoard()));
	connect(ui_->pushButton_calibrate, SIGNAL(clicked()), this, SLOT(calibrate()));
	connect(ui_->pushButton_restart, SIGNAL(clicked()), this, SLOT(restart()));
	connect(ui_->pushButton_save, SIGNAL(clicked()), this, SLOT(save()));
	connect(ui_->checkBox_switchImages, SIGNAL(stateChanged(int)), this, SLOT(restart()));
	connect(ui_->checkBox_unlock, SIGNAL(stateChanged(int)), SLOT(unlock()));

	connect(ui_->comboBox_board_type, SIGNAL(currentIndexChanged(int)), this, SLOT(setBoardType(int)));
	connect(ui_->comboBox_marker_dictionary, SIGNAL(currentIndexChanged(int)), this, SLOT(setMarkerDictionary(int)));
	connect(ui_->spinBox_boardWidth, SIGNAL(valueChanged(int)), this, SLOT(setBoardWidth(int)));
	connect(ui_->spinBox_boardHeight, SIGNAL(valueChanged(int)), this, SLOT(setBoardHeight(int)));
	connect(ui_->doubleSpinBox_squareSize, SIGNAL(valueChanged(double)), this, SLOT(setSquareSize(double)));
	connect(ui_->doubleSpinBox_markerLength, SIGNAL(valueChanged(double)), this, SLOT(setMarkerLength(double)));
	connect(ui_->doubleSpinBox_subpixel_error, SIGNAL(valueChanged(double)), this, SLOT(setSubpixelMaxError(double)));
	connect(ui_->checkBox_subpixel_refinement, SIGNAL(toggled(bool)), this, SLOT(setSubpixelRefinement(bool)));
	connect(ui_->checkBox_saveCalibrationData, SIGNAL(toggled(bool)), this, SLOT(setCalibrationDataSaved(bool)));
	connect(ui_->doubleSpinBox_stereoBaseline, SIGNAL(valueChanged(double)), this, SLOT(setExpectedStereoBaseline(double)));
	connect(ui_->spinBox_maxScale, SIGNAL(valueChanged(int)), this, SLOT(setMaxScale(int)));

	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(close()));

	ui_->image_view->setFocus();

	ui_->progressBar_count->setMaximum(COUNT_MIN);
	ui_->progressBar_count->setFormat("%v");
	ui_->progressBar_count_2->setMaximum(COUNT_MIN);
	ui_->progressBar_count_2->setFormat("%v");

	ui_->radioButton_raw->setChecked(true);

	ui_->checkBox_switchImages->setChecked(switchImages);

	ui_->comboBox_calib_model->setCurrentIndex(1);

	this->setStereoMode(stereo_);

	timestamp_ = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");

#ifndef HAVE_CHARUCO
	ui_->comboBox_board_type->setItemData(1, 0, Qt::UserRole - 1);
	ui_->comboBox_board_type->setItemData(2, 0, Qt::UserRole - 1);
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 6)
	ui_->comboBox_board_type->setItemData(2, 0, Qt::UserRole - 1);
#elif CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 8
	ui_->comboBox_board_type->setItemData(1, 0, Qt::UserRole - 1);
#endif
}

CalibrationDialog::~CalibrationDialog()
{
	this->unregisterFromEventsManager();
	delete ui_;
}

void CalibrationDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("board_type", ui_->comboBox_board_type->currentIndex());
	settings.setValue("board_width", ui_->spinBox_boardWidth->value());
	settings.setValue("board_height", ui_->spinBox_boardHeight->value());
	settings.setValue("board_square_size", ui_->doubleSpinBox_squareSize->value());
	settings.setValue("marker_type", ui_->comboBox_marker_dictionary->currentIndex());
	settings.setValue("marker_length", ui_->doubleSpinBox_markerLength->value());
	settings.setValue("subpixel_refinement", ui_->checkBox_subpixel_refinement->isChecked());
	settings.setValue("subpixel_max_error", ui_->doubleSpinBox_subpixel_error->value());
	settings.setValue("calibration_data_saved", ui_->checkBox_saveCalibrationData->isChecked());
	settings.setValue("max_scale", ui_->spinBox_maxScale->value());
	settings.setValue("geometry", this->saveGeometry());
	settings.setValue("calibration_model", ui_->comboBox_calib_model->currentIndex());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void CalibrationDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	this->setBoardType(settings.value("board_type", ui_->comboBox_board_type->currentIndex()).toInt());
	this->setBoardWidth(settings.value("board_width", ui_->spinBox_boardWidth->value()).toInt());
	this->setBoardHeight(settings.value("board_height", ui_->spinBox_boardHeight->value()).toInt());
	this->setSquareSize(settings.value("board_square_size", ui_->doubleSpinBox_squareSize->value()).toDouble());
	this->setMarkerDictionary(settings.value("marker_type", ui_->comboBox_marker_dictionary->currentIndex()).toInt());
	this->setMarkerLength(settings.value("marker_length", ui_->doubleSpinBox_markerLength->value()).toDouble());
	this->setSubpixelRefinement(settings.value("subpixel_refinement", ui_->checkBox_subpixel_refinement->isChecked()).toBool());
	this->setSubpixelMaxError(settings.value("subpixel_max_error", ui_->doubleSpinBox_subpixel_error->value()).toDouble());
	this->setCalibrationDataSaved(settings.value("calibration_data_saved", ui_->checkBox_saveCalibrationData->isChecked()).toBool());
	this->setMaxScale(settings.value("max_scale", ui_->spinBox_maxScale->value()).toDouble());
	int model = settings.value("calibration_model", ui_->comboBox_calib_model->currentIndex()).toInt();
	if(model == 0)
	{
		this->setFisheyeModel();
	}
	else if(model ==2)
	{
		this->setRationalModel();
	}
	else
	{
		this->setPlumbobModel();
	}
	QByteArray bytes = settings.value("geometry", QByteArray()).toByteArray();
	if(!bytes.isEmpty())
	{
		this->restoreGeometry(bytes);
	}
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void CalibrationDialog::resetSettings()
{
	this->setBoardType(0);
	this->setBoardWidth(8);
	this->setBoardHeight(6);
	this->setSquareSize(0.033);
	this->setMarkerLength(0.02475);
}

cv::Mat drawChessboard(int squareSize, int boardWidth, int boardHeight, int borderSize)
{
    int imageWidth = squareSize*boardWidth + borderSize;
	int imageHeight = squareSize*boardHeight + borderSize;
    cv::Mat chessboard(imageWidth, imageHeight, CV_8UC1, 255);
	unsigned char color = 0;
	for(int i=borderSize;i<imageHeight-borderSize; i=i+squareSize) {
		color=~color;
		for(int j=borderSize;j<imageWidth-borderSize;j=j+squareSize) {
			cv::Mat roi=chessboard(cv::Rect(i,j,squareSize,squareSize));
			roi.setTo(color);
			color=~color;
		}
	}
	return chessboard;
}

void CalibrationDialog::generateBoard()
{
	int squareSizeInPixels = 200;
	cv::Mat image;
	QString filename;
	QTextStream stream(&filename);
#ifdef HAVE_CHARUCO
	if(ui_->comboBox_board_type->currentIndex() >= 1 )
	{
		try {
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
			charucoBoard_->generateImage(
				cv::Size(squareSizeInPixels*ui_->spinBox_boardWidth->value(),
						squareSizeInPixels*ui_->spinBox_boardHeight->value()), 
				image, 
				squareSizeInPixels/4, 1);
#else
			charucoBoard_->draw(
				cv::Size(squareSizeInPixels*ui_->spinBox_boardWidth->value(),
						squareSizeInPixels*ui_->spinBox_boardHeight->value()), 
				image, 
				squareSizeInPixels/4, 1);
#endif

			int arucoDict = ui_->comboBox_marker_dictionary->currentIndex();
			stream << "charuco_" << (arucoDict<kArucoDictNameSize?kArucoDictNames[arucoDict]:"NA") << "_" 
				<< ui_->spinBox_boardWidth->value() << "x" << ui_->spinBox_boardHeight->value() 
				<< "_ratio" << float(ui_->doubleSpinBox_markerLength->value())/float(ui_->doubleSpinBox_squareSize->value());
		}
		catch(const cv::Exception & e)
		{
			UERROR("%f", e.what());
			QMessageBox::critical(this, tr("Generating Board"),
				tr("Cannot generate the board. Make sure the dictionary "
				   "selected is big enough for the board size. Error:\"%1\"").arg(e.what()));
			return;
		}
	}
	else
#endif
	{
		image = drawChessboard(
			squareSizeInPixels, 
			ui_->spinBox_boardWidth->value(),
			ui_->spinBox_boardHeight->value(), 
			squareSizeInPixels/4);

		stream << "/chessboard_" << ui_->spinBox_boardWidth->value() << "x" << ui_->spinBox_boardHeight->value();

	}
	QString filePath = QFileDialog::getSaveFileName(this, tr("Save"), savingDirectory_+"/"+filename+".png", "*.png");
	if(!filePath.isEmpty())
	{
		cv::imwrite(filePath.toStdString(), image);
	}
}

void CalibrationDialog::setCameraName(const QString & name)
{
	cameraName_ = name;
}

void CalibrationDialog::setProgressVisibility(bool visible)
{
	ui_->groupBox_progress->setVisible(visible);
}

void CalibrationDialog::setSwitchedImages(bool switched)
{
	ui_->checkBox_switchImages->setChecked(switched);
}

void CalibrationDialog::setFisheyeModel()
{
	ui_->comboBox_calib_model->setCurrentIndex(0);
}

void CalibrationDialog::setPlumbobModel()
{
	ui_->comboBox_calib_model->setCurrentIndex(1);
}

void CalibrationDialog::setRationalModel()
{
	ui_->comboBox_calib_model->setCurrentIndex(2);
}

void CalibrationDialog::setStereoMode(bool stereo, const QString & leftSuffix, const QString & rightSuffix)
{
	leftSuffix_ = leftSuffix;
	rightSuffix_ = rightSuffix;
	this->restart();

	ui_->groupBox_progress->setVisible(true);
	stereo_ = stereo;
	ui_->progressBar_x_2->setVisible(stereo_);
	ui_->progressBar_y_2->setVisible(stereo_);
	ui_->progressBar_size_2->setVisible(stereo_);
	ui_->progressBar_skew_2->setVisible(stereo_);
	ui_->progressBar_count_2->setVisible(stereo_);
	ui_->label_right->setVisible(stereo_);
	ui_->image_view_2->setVisible(stereo_);
	ui_->label_fx_2->setVisible(stereo_);
	ui_->label_fy_2->setVisible(stereo_);
	ui_->label_cx_2->setVisible(stereo_);
	ui_->label_cy_2->setVisible(stereo_);
	ui_->label_fovx_2->setVisible(stereo_);
	ui_->label_fovy_2->setVisible(stereo_);
	ui_->label_error_2->setVisible(stereo_);
	ui_->label_baseline->setVisible(stereo_);
	ui_->label_baseline_name->setVisible(stereo_);
	ui_->label_stereoError->setVisible(stereo_);
	ui_->lineEdit_K_2->setVisible(stereo_);
	ui_->lineEdit_D_2->setVisible(stereo_);
	ui_->lineEdit_R_2->setVisible(stereo_);
	ui_->lineEdit_P_2->setVisible(stereo_);
	ui_->radioButton_stereoRectified->setVisible(stereo_);
	ui_->checkBox_switchImages->setVisible(stereo_);
	ui_->doubleSpinBox_stereoBaseline->setVisible(stereo_);
	ui_->label_stereoBaseline->setVisible(stereo_);
}

int CalibrationDialog::boardWidth() const
{
	return ui_->spinBox_boardWidth->value();
}
int CalibrationDialog::boardHeight() const
{
	return ui_->spinBox_boardHeight->value();
}
double CalibrationDialog::squareSize() const
{
	return ui_->doubleSpinBox_squareSize->value();
}
double CalibrationDialog::markerLength() const
{
	return ui_->doubleSpinBox_markerLength->value();
}

void CalibrationDialog::setBoardType(int type)
{
	if(type != ui_->comboBox_board_type->currentIndex())
	{
		ui_->comboBox_board_type->setCurrentIndex(type);
	}
	this->restart();
}

void CalibrationDialog::setMarkerDictionary(int dictionary)
{
	if(dictionary != ui_->comboBox_marker_dictionary->currentIndex())
	{
		ui_->comboBox_marker_dictionary->setCurrentIndex(dictionary);
	}
	this->restart();
}

void CalibrationDialog::setBoardWidth(int width)
{
	if(width != ui_->spinBox_boardWidth->value())
	{
		ui_->spinBox_boardWidth->setValue(width);
	}
	this->restart();
}

void CalibrationDialog::setBoardHeight(int height)
{
	if(height != ui_->spinBox_boardHeight->value())
	{
		ui_->spinBox_boardHeight->setValue(height);
	}
	this->restart();
}

void CalibrationDialog::setSquareSize(double size)
{
	if(size != ui_->doubleSpinBox_squareSize->value())
	{
		ui_->doubleSpinBox_squareSize->setValue(size);
	}
	if(ui_->doubleSpinBox_markerLength->value() >= ui_->doubleSpinBox_squareSize->value())
	{
		if(ui_->comboBox_board_type->currentIndex()==0)
		{
			ui_->doubleSpinBox_markerLength->setValue(ui_->doubleSpinBox_squareSize->value()-0.000001);
		}
		else
		{
			UWARN("Marker length (%f) cannot be larger than square size (%f), setting square size to %f. Decrease marker length first.", 
				ui_->doubleSpinBox_markerLength->value(),
				ui_->doubleSpinBox_squareSize->value(),
				ui_->doubleSpinBox_squareSize->value()+0.000001);
			ui_->doubleSpinBox_squareSize->setValue(ui_->doubleSpinBox_markerLength->value()+0.000001);
		}
	}
	this->restart();
}

void CalibrationDialog::setMarkerLength(double length)
{
	if(length != ui_->doubleSpinBox_markerLength->value())
	{
		ui_->doubleSpinBox_markerLength->setValue(length);
	}
	if(ui_->doubleSpinBox_markerLength->value() >= ui_->doubleSpinBox_squareSize->value())
	{
		UWARN("Marker length (%f) cannot be larger than square size (%f), setting marker length to %f. Increase square size first.", 
			ui_->doubleSpinBox_markerLength->value(),
			ui_->doubleSpinBox_squareSize->value(),
			ui_->doubleSpinBox_markerLength->value()-0.000001);
		ui_->doubleSpinBox_markerLength->setValue(ui_->doubleSpinBox_squareSize->value()-0.000001);
	}
	this->restart();
}

void CalibrationDialog::setSubpixelRefinement(bool enabled)
{
	if(enabled != ui_->checkBox_subpixel_refinement->isChecked())
	{
		ui_->checkBox_subpixel_refinement->setChecked(enabled);
	}
	this->restart();
}

void CalibrationDialog::setSubpixelMaxError(double value)
{
	if(value != ui_->doubleSpinBox_subpixel_error->value())
	{
		ui_->doubleSpinBox_subpixel_error->setValue(value);
	}
	this->restart();
}

void CalibrationDialog::setCalibrationDataSaved(bool enabled)
{
	if(enabled != ui_->checkBox_saveCalibrationData->isChecked())
	{
		ui_->checkBox_saveCalibrationData->setChecked(enabled);
	}
	this->restart();
}

void CalibrationDialog::setExpectedStereoBaseline(double length)
{
	if(length != ui_->doubleSpinBox_stereoBaseline->value())
	{
		ui_->doubleSpinBox_stereoBaseline->setValue(length);
	}
}

void CalibrationDialog::setMaxScale(int scale)
{
	if(scale != ui_->spinBox_maxScale->value())
	{
		ui_->spinBox_maxScale->setValue(scale);
	}
}

void CalibrationDialog::closeEvent(QCloseEvent* event)
{
	if(!savedCalibration_ && models_[0].isValidForRectification() &&
			(!stereo_ ||
					(stereoModel_.isValidForRectification() &&
					(!ui_->label_baseline->isVisible() || stereoModel_.baseline() > 0.0))))
	{
		QMessageBox::StandardButton b = QMessageBox::question(this, tr("Save calibration?"),
				tr("The camera is calibrated but you didn't "
				   "save the calibration, do you want to save it?"),
				QMessageBox::Yes | QMessageBox::Ignore | QMessageBox::Cancel, QMessageBox::Yes);
		event->ignore();
		if(b == QMessageBox::Yes)
		{
			if(this->save())
			{
				event->accept();
			}
		}
		else if(b == QMessageBox::Ignore)
		{
			event->accept();
		}
	}
	else
	{
		event->accept();
	}
	if(event->isAccepted())
	{
		this->unregisterFromEventsManager();
	}
	cameraName_.clear();
}

bool CalibrationDialog::handleEvent(UEvent * event)
{
	if(!processingData_)
	{
		if(event->getClassName().compare("SensorEvent") == 0)
		{
			rtabmap::SensorEvent * e = (rtabmap::SensorEvent *)event;
			if(e->getCode() == rtabmap::SensorEvent::kCodeData)
			{
				processingData_ = true;
				QMetaObject::invokeMethod(this, "processImages",
						Q_ARG(cv::Mat, e->data().imageRaw()),
						Q_ARG(cv::Mat, e->data().depthOrRightRaw()),
						Q_ARG(QString, QString(e->cameraName().c_str())));
			}
		}
	}
	return false;
}

#ifdef HAVE_CHARUCO
void matchCharucoImagePoints(
		const cv::aruco::CharucoBoard &board,
		const std::vector< cv::Point2f > & detectedCorners,
		const std::vector< int > & detectedIds,
		std::vector< cv::Point3f > & objectPoints)
{
	UASSERT(detectedIds.size() == detectedCorners.size());
	objectPoints.clear();

#if CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 7)
    objectPoints.reserve(detectedIds.size());

    // look for detected markers that belong to the board and get their information
    for(size_t i = 0; i < detectedIds.size(); i++) {
       	int pointId = detectedIds[i];
		UASSERT(pointId >= 0 && pointId < (int)board.chessboardCorners.size());
		objectPoints.push_back(board.chessboardCorners[pointId]);
    }
#else
	cv::Mat imgPts;
	board.matchImagePoints(detectedCorners, detectedIds, objectPoints, imgPts);
#endif
}
#endif

// Modified from original versoin in opencv_contrib to remove "id="
void drawDetectedCornersCharuco(cv::InputOutputArray image, cv::InputArray charucoCorners,
                                cv::InputArray charucoIds = cv::noArray(),
                                cv::Scalar cornerColor = cv::Scalar(255, 0, 0)) {

    CV_Assert(image.getMat().total() != 0 &&
              (image.getMat().channels() == 1 || image.getMat().channels() == 3));
    CV_Assert((charucoCorners.getMat().total() == charucoIds.getMat().total()) ||
              charucoIds.getMat().total() == 0);

    unsigned int nCorners = (unsigned int)charucoCorners.getMat().total();
    for(unsigned int i = 0; i < nCorners; i++) {
        cv::Point2f corner = charucoCorners.getMat().at< cv::Point2f >(i);

        // draw first corner mark
        cv::rectangle(image, corner - cv::Point2f(3, 3), corner + cv::Point2f(3, 3), cornerColor, 1, cv::LINE_AA);

        // draw ID
        if(charucoIds.total() != 0) {
            int id = charucoIds.getMat().at< int >(i);
            std::stringstream s;
            s << id;
            cv::putText(image, s.str(), corner + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cornerColor, 2);
        }
    }
}

void CalibrationDialog::processImages(const cv::Mat & imageLeft, const cv::Mat & imageRight, const QString & cameraName)
{
	UDEBUG("Processing images");
	processingData_ = true;
	if(cameraName_.isEmpty() && !cameraName.isEmpty())
	{
		cameraName_ = cameraName;
	}
	else if(cameraName.isEmpty())
	{
		cameraName_ = "0000";
	}

	if(ui_->label_serial->text().compare(cameraName_)!=0)
	{
		ui_->label_serial->setText(cameraName_);
	}

	std::vector<cv::Mat> inputRawImages(2);
	if(ui_->checkBox_switchImages->isChecked())
	{
		inputRawImages[0] = imageRight;
		inputRawImages[1] = imageLeft;
	}
	else
	{
		inputRawImages[0] = imageLeft;
		inputRawImages[1] = imageRight;
	}

	std::vector<cv::Mat> images(2);
	images[0] = inputRawImages[0];
	images[1] = inputRawImages[1];
	imageSize_[0] = images[0].size();
	imageSize_[1] = images[1].size();

	bool boardFound[2] = {false};
	bool boardAccepted[2] = {false};
	bool readyToCalibrate[2] = {false};

	std::vector<std::vector<cv::Point2f> > pointBuf(2);
	std::vector<std::vector<cv::Point3f> > objectBuf(2);
	std::vector<std::vector< int > > pointIds(2);

	bool depthDetected = false;
	for(int id=0; id<(stereo_?2:1); ++id)
	{
		cv::Mat viewGray;
		if(!images[id].empty())
		{
			if(images[id].type() == CV_16UC1)
			{
				double min, max;
				cv::minMaxLoc(images[id], &min, &max);
				UDEBUG("Camera IR %d: min=%f max=%f", id, min, max);
				if(minIrs_[id] == 0)
				{
					minIrs_[id] = min;
				}
				if(maxIrs_[id] == 0x7fff)
				{
					maxIrs_[id] = max;
				}

				depthDetected = true;
				//assume IR image: convert to gray scaled
				const float factor = 255.0f / float((maxIrs_[id] - minIrs_[id]));
				viewGray = cv::Mat(images[id].rows, images[id].cols, CV_8UC1);
				for(int i=0; i<images[id].rows; ++i)
				{
					for(int j=0; j<images[id].cols; ++j)
					{
						viewGray.at<unsigned char>(i, j) = (unsigned char)std::min(float(std::max(images[id].at<unsigned short>(i,j) - minIrs_[id], 0)) * factor, 255.0f);
					}
				}
				cvtColor(viewGray, images[id], cv::COLOR_GRAY2BGR); // convert to show detected points in color
			}
			else if(images[id].channels() == 3)
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

		minIrs_[id] = 0;
		maxIrs_[id] = 0x7FFF;

		//Dot it only if not yet calibrated
		if(!ui_->pushButton_save->isEnabled())
		{
			std::vector< int > markerIds;
			std::vector< std::vector< cv::Point2f > > markerCorners;
			cv::Size boardSize(ui_->spinBox_boardWidth->value(), ui_->spinBox_boardHeight->value());
			if(!viewGray.empty())
			{
				int flags = CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE;

				if(!viewGray.empty())
				{
					int maxScale = ui_->spinBox_maxScale->value();
					for( int scale = 1; scale <= maxScale; scale++ )
					{
						cv::Mat timg;
						if( scale == 1 )
							timg = viewGray;
						else
							cv::resize(viewGray, timg, cv::Size(), scale, scale, CV_INTER_CUBIC);

#ifdef HAVE_CHARUCO
						if(ui_->comboBox_board_type->currentIndex() >= 1 )
						{
							std::vector< std::vector< cv::Point2f > > rejected;
							UASSERT(charucoBoard_.get());

							// detect markers
							UDEBUG("Detecting aruco markers...");
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
							UASSERT(arucoDetector_.get());
							arucoDetector_->detectMarkers(timg, markerCorners, markerIds, rejected);
#else
							cv::aruco::detectMarkers(timg, markerDictionary_, markerCorners, markerIds, arucoDetectorParams_, rejected);
#endif
							// refine strategy to detect more markers
							UDEBUG("Refining aruco markers...");
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
							arucoDetector_->refineDetectedMarkers(timg, *charucoBoard_, markerCorners, markerIds, rejected);
#else
							cv::aruco::refineDetectedMarkers(timg, charucoBoard_, markerCorners, markerIds, rejected);
#endif
							// interpolate charuco corners
							UDEBUG("Finding charuco corners (markers=%ld)...", markerCorners.size());
							if(markerIds.size() > 0)
							{
								UASSERT(markerIds.size() == markerCorners.size());
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
								UASSERT(charucoDetector_.get());
								charucoDetector_->detectBoard(timg, pointBuf[id], pointIds[id], markerCorners, markerIds);
#else
								cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, timg, charucoBoard_, pointBuf[id], pointIds[id], cv::noArray(), cv::noArray(), 1);
#endif
								UDEBUG("Found %ld charuco corners (requires 12)", pointBuf[id].size());
								if(pointBuf[id].size() >= 12) {
									// Match image points
									matchCharucoImagePoints(*charucoBoard_, pointBuf[id], pointIds[id], objectBuf[id]);
									boardFound[id] = !objectBuf[id].empty() && objectBuf[id].size() == pointBuf[id].size();
								}
							}
						}
						else // standard checkerboard
#endif
						{
							boardFound[id] = cv::findChessboardCorners(timg, boardSize, pointBuf[id], flags);
							objectBuf[id] = chessboardPoints_;
							pointIds[id] = chessboardPointIds_;
						}

						if(boardFound[id])
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
			}

			if(boardFound[id]) // If done with success,
			{
				// refine corners?
				std::vector<cv::Point2f> originalPoints = pointBuf[id];
				std::vector<cv::Point2f> rejectedPoints;
				std::vector<int> rejectedPointIds;
				if(ui_->checkBox_subpixel_refinement->isChecked())
				{
					// improve the found corners' coordinate accuracy
					float minSquareDistance = -1.0f;
					for(unsigned int i=0; i<pointBuf[id].size()-1; ++i)
					{
						float d = cv::norm(pointBuf[id][i] - pointBuf[id][i+1]);
						if(minSquareDistance == -1.0f || minSquareDistance > d)
						{
							minSquareDistance = d;
						}
					}
					float ratio = ui_->comboBox_board_type->currentIndex() >= 1 ?6.0f:2.0f;
					float radius = minSquareDistance==-1.0f?5.0f:(minSquareDistance/ratio);
					cv::cornerSubPix( viewGray, pointBuf[id], cv::Size(radius, radius), cv::Size(-1,-1),
							cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
					
					// Filter points that drifted to far (caused by reflection or bad subpixel gradient)
					float threshold = ui_->doubleSpinBox_subpixel_error->value();
					if(threshold>0)
					{
						std::vector<cv::Point3f> filteredObjectPts;
						std::vector<cv::Point2f> filteredPoints;
						std::vector<int> filteredPointIds;
						for(size_t i=0; i<pointBuf[id].size(); ++i)
						{
							float d = cv::norm(pointBuf[id][i] - originalPoints[i]);
							if(d<threshold)
							{
								filteredObjectPts.push_back(objectBuf[id][i]);
								filteredPoints.push_back(pointBuf[id][i]);
								filteredPointIds.push_back(pointIds[id][i]);
							}
							else
							{
								UWARN("Filtered point: subpix error for image=%d cam=%d pt=%d radius=%f: %f > %f", currentId_, id, pointIds[id][i], radius, d, threshold);
								rejectedPoints.push_back(pointBuf[id][i]);
								rejectedPointIds.push_back(pointIds[id][i]);						
							}
						}
						objectBuf[id] = filteredObjectPts;
						pointBuf[id] = filteredPoints;
						pointIds[id] = filteredPointIds;
					}
				}

				// Draw the corners.
				images[id] = images[id].clone();
#ifdef HAVE_CHARUCO
				if(ui_->comboBox_board_type->currentIndex() >= 1 ) {
					if(markerIds.size() > 0)
						cv::aruco::drawDetectedMarkers(images[id], markerCorners, cv::noArray(), cv::Scalar(255,0,0));
				}
#endif
				if(pointBuf[id].size() > 0)
					drawDetectedCornersCharuco(images[id], pointBuf[id], pointIds[id], cv::Scalar(0,255,0)); // Accepted Green
				if(rejectedPoints.size() > 0)
					drawDetectedCornersCharuco(images[id], rejectedPoints, rejectedPointIds, cv::Scalar(0,0,255)); // Rejected Red

				if(pointBuf[id].size() < rejectedPoints.size())
				{
					// don't add if more than 50% of valid points were filtered
					UWARN("Ignoring whole board of image %d cam=%d because too many points were filtered.", currentId_, id);
					boardFound[id] = false;
				}
				else
				{
					std::vector<float> params(4,0);
					getParams(originalPoints, boardSize, imageSize_[id], params[0], params[1], params[2], params[3]);
					if(ui_->comboBox_board_type->currentIndex() >= 1 )
					{
						//params[2] = float(pointBuf[id].size()) / float(boardSize.width * boardSize.height); // number of markers seen
						float area = getArea(markerCorners[markerCorners.size()/2], cv::Size(4,4)) * (boardSize.width*boardSize.height);
						params[2] = std::sqrt(area / (imageSize_[id].width * imageSize_[id].height));
						params[2] = params[2]>1?1:params[2];
						params[3] = getSkew(markerCorners[markerCorners.size()/2]);
					}

					bool addSample = true;
					if(!ui_->checkBox_keep_all->isChecked())
					{
						for(unsigned int i=0; i<imageParams_[id].size(); ++i)
						{
							if(fabs(params[0] - imageParams_[id][i].at(0)) < (ui_->comboBox_board_type->currentIndex() >= 1 ?0.2:0.1)*ui_->doubleSpinBox_sample_factor->value() && // x
								fabs(params[1] - imageParams_[id][i].at(1)) < (ui_->comboBox_board_type->currentIndex() >= 1 ?0.2:0.1)*ui_->doubleSpinBox_sample_factor->value() && // y
								fabs(params[2] - imageParams_[id][i].at(2)) < 0.05*ui_->doubleSpinBox_sample_factor->value() && // size
								(params[3]==0 || params[3]==1.0f || imageParams_[id][i].at(3) == 0 || imageParams_[id][i].at(3) == 1.0f || fabs(params[3] - imageParams_[id][i].at(3)) < 0.1*ui_->doubleSpinBox_sample_factor->value())) // skew
							{
								addSample = false;
								break;
							}
						}
					}
					if(addSample)
					{
						boardAccepted[id] = true;
						imageIds_[id].push_back(currentId_);
						imagePoints_[id].push_back(pointBuf[id]);
						imageParams_[id].push_back(params);
						objectPoints_[id].push_back(objectBuf[id]);
						UINFO("[%d] Added board %d, total=%d. (x=%f, y=%f, size=%f, skew=%f)", id, currentId_, (int)imagePoints_[id].size(), params[0], params[1], params[2], params[3]);

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
							if(imageParams_[id][i].at(3) != 0 && imageParams_[id][i].at(3) != 1) {
								if(skewRange[0] == 0 || skewRange[0] == 1)
								{
									skewRange[0] = imageParams_[id][i].at(3);
									skewRange[1] = imageParams_[id][i].at(3);
								}
								else
								{
									skewRange[0] = imageParams_[id][i].at(3) < skewRange[0] ? imageParams_[id][i].at(3) : skewRange[0];
									skewRange[1] = imageParams_[id][i].at(3) > skewRange[1] ? imageParams_[id][i].at(3) : skewRange[1];
								}
							}
						}
						//UINFO("Stats [%d]:", id);
						//UINFO("  Count = %d", (int)imagePoints_[id].size());
						//UINFO("  x =    [%f -> %f]", xRange[0], xRange[1]);
						//UINFO("  y =    [%f -> %f]", yRange[0], yRange[1]);
						//UINFO("  size = [%f -> %f]", sizeRange[0], sizeRange[1]);
						//UINFO("  skew = [%f -> %f]", skewRange[0], skewRange[1]);

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
							if((int)imagePoints_[id].size() > ui_->progressBar_count->maximum())
							{
								ui_->progressBar_count->setMaximum((int)imagePoints_[id].size());
							}
							ui_->progressBar_count->setValue((int)imagePoints_[id].size());
						}
						else
						{
							ui_->progressBar_x_2->setValue(xGood*100);
							ui_->progressBar_y_2->setValue(yGood*100);
							ui_->progressBar_size_2->setValue(sizeGood*100);
							ui_->progressBar_skew_2->setValue(skewGood*100);

							if((int)imagePoints_[id].size() > ui_->progressBar_count_2->maximum())
							{
								ui_->progressBar_count_2->setMaximum((int)imagePoints_[id].size());
							}
							ui_->progressBar_count_2->setValue((int)imagePoints_[id].size());
						}

						if(imagePoints_[id].size() >= COUNT_MIN/2 &&
								xGood > 0.5 &&
								yGood > 0.5 &&
								(sizeGood > 0.4 || (ui_->comboBox_calib_model->currentIndex()==0 && sizeGood > 0.25)) &&
								skewGood > 0.5)
						{
							readyToCalibrate[id] = true;
						}

						//update IR values
						if(inputRawImages[id].type() == CV_16UC1)
						{
							//update min max IR if the chessboard was found
							minIrs_[id] = 0xFFFF;
							maxIrs_[id] = 0;
							for(size_t i = 0; i < pointBuf[id].size(); ++i)
							{
								const cv::Point2f &p = pointBuf[id][i];
								cv::Rect roi(std::max(0, (int)p.x - 3), std::max(0, (int)p.y - 3), 6, 6);

								roi.width = std::min(roi.width, inputRawImages[id].cols - roi.x);
								roi.height = std::min(roi.height, inputRawImages[id].rows - roi.y);

								//find minMax in the roi
								double min, max;
								cv::minMaxLoc(inputRawImages[id](roi), &min, &max);
								if(min < minIrs_[id])
								{
									minIrs_[id] = min;
								}
								if(max > maxIrs_[id])
								{
									maxIrs_[id] = max;
								}
							}
						}
					}
					else
					{
						//break;
					}
				}
			}
		}
	}
	ui_->label_baseline->setVisible(!depthDetected);
	ui_->label_baseline_name->setVisible(!depthDetected);
	ui_->label_stereoError->setVisible(!depthDetected);

	if(ui_->checkBox_saveCalibrationData->isChecked() && (boardAccepted[0] || boardAccepted[1]))
	{
		for(int id=0; id<(stereo_?2:1); ++id)
		{
			QString rawImagesDir = savingDirectory_+"/"+cameraName_+"_"+timestamp_+(stereo_?"/"+(id==0?leftSuffix_:rightSuffix_):"images");
			QString imagesWithBoardDir = rawImagesDir+"_board_detection";
			if(!QDir(rawImagesDir).exists())
			{
				UINFO("Creating dir %s", rawImagesDir.toStdString().c_str());
				QDir().mkpath(rawImagesDir);
			}
			if(!QDir(imagesWithBoardDir).exists())
			{
				UINFO("Creating dir %s", imagesWithBoardDir.toStdString().c_str());
				QDir().mkpath(imagesWithBoardDir);
			}
			cv::imwrite((rawImagesDir+"/"+QString::number(currentId_)+".png").toStdString(), inputRawImages[id]);
			cv::imwrite((imagesWithBoardDir+"/"+QString::number(currentId_)+".jpg").toStdString(), images[id]);
		}
	}

	if(stereo_ && boardFound[0] && boardFound[1] && (boardAccepted[0] || boardAccepted[1]))
	{
		// Find same corners detected in both boards
		std::vector< int > combinedIds;
		std::vector<cv::Point2f> leftCorners;
		std::vector<cv::Point2f> rightCorners;
		std::vector<cv::Point3f> objectPoints;
		for(size_t i=0; i<pointIds[0].size(); ++i)
		{
			for(size_t j=0; j<pointIds[1].size(); ++j)
			{
				if(pointIds[0][i] == pointIds[1][j])
				{
					leftCorners.push_back(pointBuf[0][i]);
					rightCorners.push_back(pointBuf[1][j]);
					objectPoints.push_back(objectBuf[0][i]);
					combinedIds.push_back(pointIds[0][i]);
					break;
				}
			}
		}
		if(objectPoints.size() >=6)
		{
			stereoImagePoints_[0].push_back(leftCorners);
			stereoImagePoints_[1].push_back(rightCorners);
			stereoObjectPoints_.push_back(objectPoints);

			stereoImageIds_.push_back(currentId_);
			UINFO("Added board %d for stereo image points (size=%d)", currentId_, (int)stereoImagePoints_[0].size());
		}
	}

	if(!stereo_ && readyToCalibrate[0])
	{
		unlock();
	}
	else if(stereo_ && readyToCalibrate[0] && readyToCalibrate[1] && stereoImagePoints_[0].size())
	{
		unlock();
	}

	if(ui_->radioButton_rectified->isChecked())
	{
		if(models_[0].isValidForRectification())
		{
			images[0] = models_[0].rectifyImage(images[0]);
		}
		if(models_[1].isValidForRectification())
		{
			images[1] = models_[1].rectifyImage(images[1]);
		}
	}
	else if(ui_->radioButton_stereoRectified->isChecked() &&
			(stereoModel_.left().isValidForRectification() &&
			stereoModel_.right().isValidForRectification()&&
			(!ui_->label_baseline->isVisible() || stereoModel_.baseline() > 0.0)))
	{
		images[0] = stereoModel_.left().rectifyImage(images[0]);
		images[1] = stereoModel_.right().rectifyImage(images[1]);
	}

	if(ui_->checkBox_showHorizontalLines->isChecked())
	{
		for(int id=0; id<(stereo_?2:1); ++id)
		{
			int step = imageSize_[id].height/16;
			for(int i=step; i<imageSize_[id].height; i+=step)
			{
				cv::line(images[id], cv::Point(0, i), cv::Point(imageSize_[id].width, i), CV_RGB(0,255,0));
			}
		}
	}

	ui_->label_left->setText(tr("%1x%2").arg(images[0].cols).arg(images[0].rows));

	//show frame
	ui_->image_view->setImage(uCvMat2QImage(images[0]).mirrored(ui_->checkBox_mirror->isChecked(), false));
	if(stereo_)
	{
		ui_->label_right->setText(tr("%1x%2").arg(images[1].cols).arg(images[1].rows));
		ui_->image_view_2->setImage(uCvMat2QImage(images[1]).mirrored(ui_->checkBox_mirror->isChecked(), false));
	}
	processingData_ = false;
	++currentId_;
}

void CalibrationDialog::restart()
{
	// restart
	if(!savingDirectory_.isEmpty() && !cameraName_.isEmpty() && !savedCalibration_ && ui_->comboBox_board_type->isEnabled())
	{
		//overwrite previous data not used.
		QDir(savingDirectory_+"/"+cameraName_+"_"+timestamp_).removeRecursively();
	}
	else
	{
		timestamp_ = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
	}
	savedCalibration_ = false;
	currentId_ = 0;
	imagePoints_[0].clear();
	imagePoints_[1].clear();
	objectPoints_[0].clear();
	objectPoints_[1].clear();
	imageParams_[0].clear();
	imageParams_[1].clear();
	imageIds_[0].clear();
	imageIds_[1].clear();
	stereoImagePoints_[0].clear();
	stereoImagePoints_[1].clear();
	stereoImageIds_.clear();
	stereoObjectPoints_.clear();
	models_[0] = CameraModel();
	models_[1] = CameraModel();
	stereoModel_ = StereoCameraModel();
	minIrs_[0] = 0x0000;
	maxIrs_[0] = 0x7fff;
	minIrs_[1] = 0x0000;
	maxIrs_[1] = 0x7fff;

	ui_->comboBox_board_type->setEnabled(true);
	ui_->comboBox_marker_dictionary->setEnabled(true);
	ui_->spinBox_boardWidth->setEnabled(true);
	ui_->spinBox_boardHeight->setEnabled(true);
	ui_->doubleSpinBox_squareSize->setEnabled(true);
	ui_->doubleSpinBox_markerLength->setEnabled(true);
	ui_->checkBox_subpixel_refinement->setEnabled(true);
	ui_->doubleSpinBox_subpixel_error->setEnabled(true);
	ui_->checkBox_saveCalibrationData->setEnabled(true);

	ui_->pushButton_calibrate->setEnabled(ui_->checkBox_unlock->isChecked());
	ui_->pushButton_save->setEnabled(false);
	ui_->radioButton_raw->setChecked(true);
	ui_->radioButton_rectified->setEnabled(false);
	ui_->radioButton_stereoRectified->setEnabled(false);

	ui_->progressBar_count->reset();
	ui_->progressBar_count->setMaximum(COUNT_MIN);
	ui_->progressBar_x->reset();
	ui_->progressBar_y->reset();
	ui_->progressBar_size->reset();
	ui_->progressBar_skew->reset();

	ui_->progressBar_count_2->reset();
	ui_->progressBar_count_2->setMaximum(COUNT_MIN);
	ui_->progressBar_x_2->reset();
	ui_->progressBar_y_2->reset();
	ui_->progressBar_size_2->reset();
	ui_->progressBar_skew_2->reset();

	ui_->label_serial->clear();
	ui_->label_fx->setNum(0);
	ui_->label_fy->setNum(0);
	ui_->label_cx->setNum(0);
	ui_->label_cy->setNum(0);
	ui_->label_fovx->setNum(0);
	ui_->label_fovy->setNum(0);
	ui_->label_baseline->setNum(0);
	ui_->label_stereoError->setNum(0);
	ui_->label_error->setNum(0);
	ui_->label_error_2->setNum(0);
	ui_->lineEdit_K->clear();
	ui_->lineEdit_D->clear();
	ui_->lineEdit_R->clear();
	ui_->lineEdit_P->clear();
	ui_->label_fx_2->setNum(0);
	ui_->label_fy_2->setNum(0);
	ui_->label_cx_2->setNum(0);
	ui_->label_cy_2->setNum(0);
	ui_->label_fovx_2->setNum(0);
	ui_->label_fovy_2->setNum(0);
	ui_->lineEdit_K_2->clear();
	ui_->lineEdit_D_2->clear();
	ui_->lineEdit_R_2->clear();
	ui_->lineEdit_P_2->clear();

	chessboardPoints_.clear();
	chessboardPointIds_.clear();
#ifdef HAVE_CHARUCO
	markerDictionary_.release();
	arucoDetectorParams_.release();
	charucoBoard_.release();
	if(ui_->comboBox_board_type->currentIndex() >= 1 )
	{
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
		arucoDetectorParams_.reset(new cv::aruco::DetectorParameters());
#else
		arucoDetectorParams_ = cv::aruco::DetectorParameters::create();
#endif

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=3)
		arucoDetectorParams_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
#else
		arucoDetectorParams_->doCornerRefinement = true;
#endif

		int arucoDictionary = ui_->comboBox_marker_dictionary->currentIndex();
		if(arucoDictionary >= 17)
		{
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION <4 || (CV_MINOR_VERSION ==4 && CV_SUBMINOR_VERSION<2)))
			UERROR("Cannot set AprilTag dictionary. OpenCV version should be at least 3.4.2, "
					"current version is %s.", CV_VERSION);

			// Dictionary to use: 
			// DICT_ARUCO_4X4_50=0, DICT_ARUCO_4X4_100=1, DICT_ARUCO_4X4_250=2, DICT_ARUCO_4X4_1000=3,
			// DICT_ARUCO_5X5_50=4, DICT_ARUCO_5X5_100=5, DICT_ARUCO_5X5_250=6, DICT_ARUCO_5X5_1000=7,
			// DICT_ARUCO_6X6_50=8, DICT_ARUCO_6X6_100=9, DICT_ARUCO_6X6_250=10, DICT_ARUCO_6X6_1000=11,
			// DICT_ARUCO_7X7_50=12, DICT_ARUCO_7X7_100=13, DICT_ARUCO_7X7_250=14, DICT_ARUCO_7X7_1000=15,
			// DICT_ARUCO_ORIGINAL = 16, DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19,
			// DICT_APRILTAG_36h11=20
			//
			arucoDictionary = 0;
#else
			arucoDetectorParams_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
#endif
		}

#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
		markerDictionary_.reset(new cv::aruco::Dictionary());
		*markerDictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(arucoDictionary));
#elif CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >=2)
		markerDictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(arucoDictionary));
#else
		markerDictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(arucoDictionary));
#endif
		UDEBUG("Creating charuco board: %dx%d square=%f marker=%f aruco dict=%d", 
			ui_->spinBox_boardWidth->value(),
			ui_->spinBox_boardHeight->value(), 
			ui_->doubleSpinBox_squareSize->value(), 
			ui_->doubleSpinBox_markerLength->value(),
			arucoDictionary);

#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
		charucoBoard_.reset(new cv::aruco::CharucoBoard(
			cv::Size(ui_->spinBox_boardWidth->value(), ui_->spinBox_boardHeight->value()), 
			ui_->doubleSpinBox_squareSize->value(), 
			ui_->doubleSpinBox_markerLength->value(), 
			*markerDictionary_));
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 8)
		charucoBoard_->setLegacyPattern(ui_->comboBox_board_type->currentIndex()==1);
#endif
		arucoDetector_.reset(new cv::aruco::ArucoDetector(*markerDictionary_, *arucoDetectorParams_));
		charucoDetector_.reset(new cv::aruco::CharucoDetector(*charucoBoard_, cv::aruco::CharucoParameters(), *arucoDetectorParams_));
#else
		charucoBoard_ = cv::aruco::CharucoBoard::create(
			ui_->spinBox_boardWidth->value(),
			ui_->spinBox_boardHeight->value(), 
			ui_->doubleSpinBox_squareSize->value(), 
			ui_->doubleSpinBox_markerLength->value(), 
			markerDictionary_);
#endif
	}
	else //checkerboard
#endif
	{
		for( int i = 0; i < ui_->spinBox_boardHeight->value(); ++i ) {
			for( int j = 0; j < ui_->spinBox_boardWidth->value(); ++j ) {
				chessboardPoints_.push_back(cv::Point3f(float( j*ui_->doubleSpinBox_squareSize->value() ), float( i*ui_->doubleSpinBox_squareSize->value() ), 0));
				chessboardPointIds_.push_back(i*ui_->spinBox_boardWidth->value() + j);
			}
		}
	}

	ui_->comboBox_marker_dictionary->setVisible(ui_->comboBox_board_type->currentIndex() >= 1 );
	ui_->doubleSpinBox_markerLength->setVisible(ui_->comboBox_board_type->currentIndex() >= 1 );
	ui_->label_markerDictionary->setVisible(ui_->comboBox_board_type->currentIndex() >= 1 );
	ui_->label_markerLength->setVisible(ui_->comboBox_board_type->currentIndex() >= 1 );
}

void CalibrationDialog::unlock()
{
	ui_->pushButton_calibrate->setEnabled(true);
}

void CalibrationDialog::calibrate()
{
	processingData_ = true;
	savedCalibration_ = false;

	ui_->comboBox_board_type->setEnabled(false);
	ui_->comboBox_marker_dictionary->setEnabled(false);
	ui_->spinBox_boardWidth->setEnabled(false);
	ui_->spinBox_boardHeight->setEnabled(false);
	ui_->doubleSpinBox_squareSize->setEnabled(false);
	ui_->doubleSpinBox_markerLength->setEnabled(false);
	ui_->checkBox_subpixel_refinement->setEnabled(false);
	ui_->doubleSpinBox_subpixel_error->setEnabled(false);
	ui_->checkBox_saveCalibrationData->setEnabled(false);

	QMessageBox mb(QMessageBox::Information,
			tr("Calibrating..."),
			tr("Operation in progress..."));
	mb.show();
	QApplication::processEvents();
	uSleep(100); // hack make sure the text in the QMessageBox is shown...
	QApplication::processEvents();
	
	// Logging
	QFile logFile;
	QString dummyOutput;
	QTextStream logStream(&dummyOutput);
	if(ui_->checkBox_saveCalibrationData->isChecked())
	{
		logFile.setFileName(savingDirectory_+"/"+cameraName_+"_"+timestamp_+"/"+"log.txt");
		if (logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
			logStream.setDevice(&logFile);
		}
	}

	std::cout << "Board type = " << ui_->comboBox_board_type->currentIndex() << std::endl;
	std::cout << "Board width = " << ui_->spinBox_boardWidth->value() << std::endl;
	std::cout << "Board height = " << ui_->spinBox_boardHeight->value() << std::endl;
	std::cout << "Square size = " << ui_->doubleSpinBox_squareSize->value() << std::endl;
	std::cout << "Subpixel refinement = " << ui_->checkBox_subpixel_refinement->isChecked() << std::endl;
	std::cout << "Subpixel max error = " << ui_->doubleSpinBox_subpixel_error->value() << std::endl;
	logStream << "Board type = " << ui_->comboBox_board_type->currentIndex() << ENDL;
	logStream << "Board width = " << ui_->spinBox_boardWidth->value() << ENDL;
	logStream << "Board height = " << ui_->spinBox_boardHeight->value() << ENDL;
	logStream << "Square size = " << ui_->doubleSpinBox_squareSize->value() << ENDL;
	logStream << "Subpixel refinement = " << ui_->checkBox_subpixel_refinement->isChecked() << ENDL;
	logStream << "Subpixel max error = " << ui_->doubleSpinBox_subpixel_error->value() << ENDL;
	if(ui_->comboBox_board_type->currentIndex() >= 1 )
	{
		std::cout << "Marker dictionary = " << ui_->comboBox_marker_dictionary->currentIndex() << std::endl;
		std::cout << "Marker length = " << ui_->doubleSpinBox_markerLength->value() << std::endl;
		logStream << "Marker dictionary = " << ui_->comboBox_marker_dictionary->currentIndex() << ENDL;
		logStream << "Marker length = " << ui_->doubleSpinBox_markerLength->value() << ENDL;
	}

	for(int id=0; id<(stereo_?2:1); ++id)
	{
		UINFO("Calibrating camera %d (samples=%d)", id, (int)imagePoints_[id].size());
		logStream << "Calibrating camera " << id << " (samples=" << imagePoints_[id].size() << ")" << ENDL;

		//calibrate
		std::vector<cv::Mat> rvecs, tvecs;
		std::vector<float> reprojErrs;
		cv::Mat K, D;
		K = cv::Mat::eye(3,3,CV_64FC1);
		UINFO("calibrate!");
		//Find intrinsic and extrinsic camera parameters
		double rms = 0.0;
#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
		bool fishEye = ui_->comboBox_calib_model->currentIndex()==0;

		if(fishEye)
		{
			try
			{
				rms = cv::fisheye::calibrate(
					objectPoints_[id],
					imagePoints_[id],
					imageSize_[id],
					K,
					D,
					rvecs,
					tvecs,
					cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
					cv::fisheye::CALIB_CHECK_COND |
					cv::fisheye::CALIB_FIX_SKEW);
			}
			catch(const cv::Exception & e)
			{
				UERROR("Error: %s (try restarting the calibration)", e.what());
				QMessageBox::warning(this, tr("Calibration failed!"), tr("Error: %1 (try restarting the calibration)").arg(e.what()));
				processingData_ = false;
				return;
			}
		}
		else
#endif
		{
			cv::Mat stdDevsMatInt, stdDevsMatExt;
			cv::Mat perViewErrorsMat;
			rms = cv::calibrateCamera(
					objectPoints_[id],
					imagePoints_[id],
					imageSize_[id],
					K,
					D,
					rvecs,
					tvecs,
					stdDevsMatInt,
					stdDevsMatExt,
					perViewErrorsMat,
					ui_->comboBox_calib_model->currentIndex()==2?cv::CALIB_RATIONAL_MODEL:0);
			if((int)imageIds_[id].size() == perViewErrorsMat.rows)
			{
				UINFO("Per view errors:");
				logStream << "Per view errors:" << ENDL;
				for(int i=0; i<perViewErrorsMat.rows; ++i)
				{
					UINFO("Image %d: %f", imageIds_[id][i], perViewErrorsMat.at<double>(i,0));
					logStream << "Image " << imageIds_[id][i] << ": " << perViewErrorsMat.at<double>(i,0) << ENDL;
				}
			}
		}

		UINFO("Re-projection error reported by calibrateCamera: %f", rms);
		logStream << "Re-projection error reported by calibrateCamera: " << rms << ENDL;

		// compute reprojection errors
		std::vector<cv::Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		reprojErrs.resize(objectPoints_[id].size());

		for( i = 0; i < (int)objectPoints_[id].size(); ++i )
		{
#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
			if(fishEye)
			{
				cv::fisheye::projectPoints( cv::Mat(objectPoints_[id][i]), imagePoints2, rvecs[i], tvecs[i], K, D);
			}
			else
#endif
			{
				cv::projectPoints( cv::Mat(objectPoints_[id][i]), rvecs[i], tvecs[i], K, D, imagePoints2);
			}
			err = cv::norm(cv::Mat(imagePoints_[id][i]), cv::Mat(imagePoints2), CV_L2);

			int n = (int)objectPoints_[id][i].size();
			reprojErrs[i] = (float) std::sqrt(err*err/n);
			totalErr        += err*err;
			totalPoints     += n;
		}

		double totalAvgErr =  std::sqrt(totalErr/totalPoints);

		UINFO("avg re projection error = %f", totalAvgErr);
		logStream << "avg re projection error = " << totalAvgErr << ENDL;

		cv::Mat P(3,4,CV_64FC1);
		P.at<double>(2,3) = 1;
		K.copyTo(P.colRange(0,3).rowRange(0,3));

#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
		if(fishEye)
		{
			// Convert to unified distortion model (k1,k2,p1,p2,k3,k4)
			cv::Mat newD = cv::Mat::zeros(1,6,CV_64FC1);
			newD.at<double>(0,0) = D.at<double>(0,0);
			newD.at<double>(0,1) = D.at<double>(0,1);
			newD.at<double>(0,4) = D.at<double>(0,2);
			newD.at<double>(0,5) = D.at<double>(0,3);
			D = newD;
		}
#endif

		models_[id] = CameraModel(cameraName_.toStdString(), imageSize_[id], K, D, cv::Mat::eye(3,3,CV_64FC1), P);

		std::cout << "K = " << K << std::endl;
		std::cout << "D = " << D << std::endl;
		std::cout << "width = " << imageSize_[id].width << std::endl;
		std::cout << "height = " << imageSize_[id].height << std::endl;
		UINFO("FOV horizontal=%f vertical=%f", models_[id].horizontalFOV(), models_[id].verticalFOV());

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION > 2)
		std::string strStream;
		logStream << "K = " << (strStream << K).c_str() << ENDL;
		strStream.clear();
		logStream << "D = " << (strStream << D).c_str() << ENDL;
#endif
		logStream << "width = " << imageSize_[id].width << ENDL;
		logStream << "height = " << imageSize_[id].height << ENDL;
		logStream << "FOV horizontal=" << models_[id].horizontalFOV() << " vertical=" << models_[id].verticalFOV() << ENDL;

		if(id == 0)
		{
			ui_->label_fx->setNum(models_[id].fx());
			ui_->label_fy->setNum(models_[id].fy());
			ui_->label_cx->setNum(models_[id].cx());
			ui_->label_cy->setNum(models_[id].cy());
			ui_->label_fovx->setNum(models_[id].horizontalFOV());
			ui_->label_fovy->setNum(models_[id].verticalFOV());
			ui_->label_error->setNum(totalAvgErr);

			std::stringstream strK, strD, strR, strP;
			strK << models_[id].K_raw();
			strD << models_[id].D_raw();
			strR << models_[id].R();
			strP << models_[id].P();
			ui_->lineEdit_K->setText(strK.str().c_str());
			ui_->lineEdit_D->setText(strD.str().c_str());
			ui_->lineEdit_R->setText(strR.str().c_str());
			ui_->lineEdit_P->setText(strP.str().c_str());
		}
		else
		{
			ui_->label_fx_2->setNum(models_[id].fx());
			ui_->label_fy_2->setNum(models_[id].fy());
			ui_->label_cx_2->setNum(models_[id].cx());
			ui_->label_cy_2->setNum(models_[id].cy());
			ui_->label_fovx_2->setNum(models_[id].horizontalFOV());
			ui_->label_fovy_2->setNum(models_[id].verticalFOV());
			ui_->label_error_2->setNum(totalAvgErr);

			std::stringstream strK, strD, strR, strP;
			strK << models_[id].K_raw();
			strD << models_[id].D_raw();
			strR << models_[id].R();
			strP << models_[id].P();
			ui_->lineEdit_K_2->setText(strK.str().c_str());
			ui_->lineEdit_D_2->setText(strD.str().c_str());
			ui_->lineEdit_R_2->setText(strR.str().c_str());
			ui_->lineEdit_P_2->setText(strP.str().c_str());
		}
	}

	if(stereo_ && models_[0].isValidForRectification() && models_[1].isValidForRectification())
	{
		stereoModel_ = stereoCalibration(models_[0], models_[1], false, &logStream);

		if(stereoModel_.isValidForProjection()  &&
			ui_->doubleSpinBox_stereoBaseline->value() > 0 &&
		   stereoModel_.baseline() != ui_->doubleSpinBox_stereoBaseline->value())
		{
			UWARN("Expected stereo baseline is set to %f m, but computed baseline is %f m. Rescaling baseline...",
					ui_->doubleSpinBox_stereoBaseline->value(), stereoModel_.baseline());
			cv::Mat P = stereoModel_.right().P().clone();
			P.at<double>(0,3) = -P.at<double>(0,0)*ui_->doubleSpinBox_stereoBaseline->value();
			double scale = ui_->doubleSpinBox_stereoBaseline->value() / stereoModel_.baseline();
			UWARN("Scale %f (setting square size from %f to %f)", scale, ui_->doubleSpinBox_squareSize->value(), ui_->doubleSpinBox_squareSize->value()*scale);
			logStream << "Baseline rescaled from " << stereoModel_.baseline() << " to " << ui_->doubleSpinBox_stereoBaseline->value() << " scale=" << scale << ENDL;
			ui_->doubleSpinBox_squareSize->setValue(ui_->doubleSpinBox_squareSize->value()*scale);
			stereoModel_ = StereoCameraModel(
					stereoModel_.name(),
					stereoModel_.left().imageSize(),stereoModel_.left().K_raw(), stereoModel_.left().D_raw(), stereoModel_.left().R(), stereoModel_.left().P(),
					stereoModel_.right().imageSize(), stereoModel_.right().K_raw(), stereoModel_.right().D_raw(), stereoModel_.right().R(), P,
					stereoModel_.R(), stereoModel_.T()*scale, stereoModel_.E(), stereoModel_.F(), stereoModel_.localTransform());
		}

		std::stringstream strR1, strP1, strR2, strP2;
		strR1 << stereoModel_.left().R();
		strP1 << stereoModel_.left().P();
		strR2 << stereoModel_.right().R();
		strP2 << stereoModel_.right().P();
		ui_->lineEdit_R->setText(strR1.str().c_str());
		ui_->lineEdit_P->setText(strP1.str().c_str());
		ui_->lineEdit_R_2->setText(strR2.str().c_str());
		ui_->lineEdit_P_2->setText(strP2.str().c_str());

		ui_->label_fovx->setNum(stereoModel_.left().horizontalFOV());
		ui_->label_fovx_2->setNum(stereoModel_.right().horizontalFOV());
		ui_->label_fovy->setNum(stereoModel_.left().verticalFOV());
		ui_->label_fovy_2->setNum(stereoModel_.right().verticalFOV());
		ui_->label_baseline->setNum(stereoModel_.baseline());
		//ui_->label_error_stereo->setNum(totalAvgErr);
		UINFO("Baseline=%f FOV horizontal=%f vertical=%f", stereoModel_.baseline(), stereoModel_.left().horizontalFOV(), stereoModel_.left().verticalFOV());
		logStream << "Baseline = " << stereoModel_.baseline() << ENDL;
		logStream << "Stereo horizontal FOV = " << stereoModel_.left().horizontalFOV() << ENDL;
		logStream << "Stereo vertical FOV = " << stereoModel_.left().verticalFOV() << ENDL;
	}

	if(stereo_)
	{
		if(models_[0].isValidForRectification())
		{
			models_[0].initRectificationMap();
		}
		if(models_[1].isValidForRectification())
		{
			models_[1].initRectificationMap();
		}
		if(models_[0].isValidForRectification() || models_[1].isValidForRectification())
		{
			ui_->radioButton_rectified->setEnabled(true);
		}
		if(stereoModel_.isValidForRectification())
		{
			stereoModel_.initRectificationMap();
			ui_->radioButton_stereoRectified->setEnabled(true);
			ui_->radioButton_stereoRectified->setChecked(true);
			ui_->pushButton_save->setEnabled(true);

			if(ui_->checkBox_saveCalibrationData->isChecked())
			{
				stereoModel_.save((savingDirectory_+"/"+cameraName_+"_"+timestamp_).toStdString(), false);
			}
		}
		else
		{
			ui_->radioButton_rectified->setChecked(ui_->radioButton_rectified->isEnabled());
		}
	}
	else if(models_[0].isValidForRectification())
	{
		models_[0].initRectificationMap();
		ui_->radioButton_rectified->setEnabled(true);
		ui_->radioButton_rectified->setChecked(true);
		ui_->pushButton_save->setEnabled(true);

		if(ui_->checkBox_saveCalibrationData->isChecked())
		{
			models_[0].save((savingDirectory_+"/"+cameraName_+"_"+timestamp_).toStdString());
		}
	}

	UINFO("End calibration");
	processingData_ = false;
	logFile.close();
}

StereoCameraModel CalibrationDialog::stereoCalibration(const CameraModel & left, const CameraModel & right, bool ignoreStereoRectification, QTextStream * logStream) const
{
	StereoCameraModel output;
	if (stereoImagePoints_[0].empty())
	{
		UERROR("No stereo correspondences!");
		return output;
	}
	UINFO("stereo calibration (samples=%d)...", (int)stereoImagePoints_[0].size());
	if(logStream) (*logStream) << "stereo calibration (samples=" << stereoImagePoints_[0].size() <<")..." << ENDL;

	if (left.K_raw().empty() || left.D_raw().empty())
	{
		UERROR("Empty intrinsic parameters (K, D) for the %s camera! Aborting stereo calibration...", leftSuffix_.toStdString().c_str());
		return output;
	}
	if (right.K_raw().empty() || right.D_raw().empty())
	{
		UERROR("Empty intrinsic parameters (K, D) for the %s camera! Aborting stereo calibration...", rightSuffix_.toStdString().c_str());
		return output;
	}

	if (left.imageSize() != imageSize_[0])
	{
		UERROR("left model (%dx%d) has not the same size as the processed images (%dx%d)",
			left.imageSize().width, left.imageSize().height,
			imageSize_[0].width, imageSize_[0].height);
		return output;
	}
	if (right.imageSize() != imageSize_[1])
	{
		UERROR("right model (%dx%d) has not the same size as the processed images (%dx%d)",
			right.imageSize().width, right.imageSize().height,
			imageSize_[1].width, imageSize_[1].height);
		return output;
	}

	cv::Size imageSize = imageSize_[0].width > imageSize_[1].width ? imageSize_[0] : imageSize_[1];
	cv::Mat R, T, E, F;

	double rms = 0.0;
#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
	bool fishEye = left.D_raw().cols == 6;
	// calibrate extrinsic
	if(fishEye)
	{
		cv::Vec3d Tvec;
		cv::Vec4d D_left(left.D_raw().at<double>(0,0), left.D_raw().at<double>(0,1), left.D_raw().at<double>(0,4), left.D_raw().at<double>(0,5));
		cv::Vec4d D_right(right.D_raw().at<double>(0,0), right.D_raw().at<double>(0,1), right.D_raw().at<double>(0,4), right.D_raw().at<double>(0,5));

		UASSERT(stereoImagePoints_[0].size() == stereoImagePoints_[1].size());
		std::vector<std::vector<cv::Point2d> > leftPoints(stereoImagePoints_[0].size());
		std::vector<std::vector<cv::Point2d> > rightPoints(stereoImagePoints_[1].size());
		for(unsigned int i =0; i<stereoImagePoints_[0].size(); ++i)
		{
			UASSERT(stereoImagePoints_[0][i].size() == stereoImagePoints_[1][i].size());
			leftPoints[i].resize(stereoImagePoints_[0][i].size());
			rightPoints[i].resize(stereoImagePoints_[1][i].size());
			for(unsigned int j =0; j<stereoImagePoints_[0][i].size(); ++j)
			{
				leftPoints[i][j].x = stereoImagePoints_[0][i][j].x;
				leftPoints[i][j].y = stereoImagePoints_[0][i][j].y;
				rightPoints[i][j].x = stereoImagePoints_[1][i][j].x;
				rightPoints[i][j].y = stereoImagePoints_[1][i][j].y;
			}
		}

		try
		{
			rms = cv::fisheye::stereoCalibrate(
					stereoObjectPoints_,
					leftPoints,
					rightPoints,
					left.K_raw(), D_left, right.K_raw(), D_right,
					imageSize, R, Tvec,
					cv::fisheye::CALIB_FIX_INTRINSIC,
					cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5));
			UINFO("stereo calibration... done with RMS error=%f", rms);
		}
		catch(const cv::Exception & e)
		{
			UERROR("Error: %s (try restarting the calibration)", e.what());
			return output;
		}

		std::cout << "R = " << R << std::endl;
		std::cout << "T = " << Tvec << std::endl;

		if(imageSize_[0] == imageSize_[1] && !ignoreStereoRectification)
		{
			UINFO("Compute stereo rectification");

			cv::Mat R1, R2, P1, P2, Q;
			stereoRectifyFisheye(
					left.K_raw(), D_left,
					right.K_raw(), D_right,
					imageSize, R, Tvec, R1, R2, P1, P2, Q,
					cv::CALIB_ZERO_DISPARITY, 0, imageSize);

			// Very hard to get good results with this one:
			/*double balance = 0.0, fov_scale = 1.0;
			cv::fisheye::stereoRectify(
					left.K_raw(), D_left,
					right.K_raw(), D_right,
					imageSize, R, Tvec, R1, R2, P1, P2, Q,
					cv::CALIB_ZERO_DISPARITY, imageSize, balance, fov_scale);*/

			std::cout << "R1 = " << R1 << std::endl;
			std::cout << "R2 = " << R2 << std::endl;
			std::cout << "P1 = " << P1 << std::endl;
			std::cout << "P2 = " << P2 << std::endl;

			// Re-zoom to original focal distance
			if(P1.at<double>(0,0) < 0)
			{
				P1.at<double>(0,0) *= -1;
				P1.at<double>(1,1) *= -1;
			}
			if(P2.at<double>(0,0) < 0)
			{
				P2.at<double>(0,0) *= -1;
				P2.at<double>(1,1) *= -1;
			}
			if(P2.at<double>(0,3) > 0)
			{
				P2.at<double>(0,3) *= -1;
			}
			P2.at<double>(0,3) = P2.at<double>(0,3) * left.K_raw().at<double>(0,0) / P2.at<double>(0,0);
			P1.at<double>(0,0) = P1.at<double>(1,1) = left.K_raw().at<double>(0,0);
			P2.at<double>(0,0) = P2.at<double>(1,1) = left.K_raw().at<double>(0,0);

			std::cout << "P1n = " << P1 << std::endl;
			std::cout << "P2n = " << P2 << std::endl;


			cv::Mat T(3,1,CV_64FC1);
			T.at <double>(0,0) = Tvec[0];
			T.at <double>(1,0) = Tvec[1];
			T.at <double>(2,0) = Tvec[2];
			output = StereoCameraModel(
							cameraName_.toStdString(),
							imageSize_[0], left.K_raw(), left.D_raw(), R1, P1,
							imageSize_[1], right.K_raw(), right.D_raw(), R2, P2,
							R, T, E, F);
		}
		else
		{
			UDEBUG("%s", cameraName_.toStdString().c_str());
			//Kinect, ignore the stereo rectification
			output = StereoCameraModel(
							cameraName_.toStdString(),
							imageSize_[0], left.K_raw(), left.D_raw(), left.R(), left.P(),
							imageSize_[1], right.K_raw(), right.D_raw(), right.R(), right.P(),
							R, T, E, F);
		}
	}
	else
#endif
	{
#if CV_MAJOR_VERSION < 3
		rms = cv::stereoCalibrate(
				stereoObjectPoints_,
				stereoImagePoints_[0],
				stereoImagePoints_[1],
				left.K_raw(), left.D_raw(),
				right.K_raw(), right.D_raw(),
				imageSize, R, T, E, F,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5),
				cv::CALIB_FIX_INTRINSIC | (ui_->comboBox_calib_model->currentIndex()==2?cv::CALIB_RATIONAL_MODEL:0));
#elif CV_MAJOR_VERSION == 3 and (CV_MINOR_VERSION < 4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION < 1))
		//OpenCV < 3.4.1
		rms = cv::stereoCalibrate(
				stereoObjectPoints_,
				stereoImagePoints_[0],
				stereoImagePoints_[1],
				left.K_raw(), left.D_raw(),
				right.K_raw(), right.D_raw(),
				imageSize, R, T, E, F,
				cv::CALIB_FIX_INTRINSIC | (ui_->comboBox_calib_model->currentIndex()==2?cv::CALIB_RATIONAL_MODEL:0),
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5));
#else
		cv::Mat perViewErrorsMat;
		rms = cv::stereoCalibrate(
				stereoObjectPoints_,
				stereoImagePoints_[0],
				stereoImagePoints_[1],
				left.K_raw(), left.D_raw(),
				right.K_raw(), right.D_raw(),
				imageSize, R, T, E, F,
				perViewErrorsMat,
				cv::CALIB_FIX_INTRINSIC | (ui_->comboBox_calib_model->currentIndex()==2?cv::CALIB_RATIONAL_MODEL:0),
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5));
		if((int)stereoImageIds_.size() == perViewErrorsMat.rows)
		{
			UINFO("Per stereo view errors: %dx%d", perViewErrorsMat.rows, perViewErrorsMat.cols);
			if(logStream) (*logStream) << "Per stereo view errors:" << ENDL;
			for(int i=0; i<perViewErrorsMat.rows; ++i)
			{
				UINFO("Image %d: %f <-> %f", stereoImageIds_[i], perViewErrorsMat.at<double>(i,0), perViewErrorsMat.at<double>(i,1));
				if(logStream) (*logStream) << "Image " << stereoImageIds_[i] << ": " << perViewErrorsMat.at<double>(i,0) << " <-> " << perViewErrorsMat.at<double>(i,0) << ENDL;
			}
		}
#endif
		UINFO("stereo calibration... done with RMS error=%f", rms);
		if(logStream) (*logStream) << "stereo calibration... done with RMS error=" << rms << ENDL;
		ui_->label_stereoError->setNum(rms);

		std::cout << "R = " << R << std::endl;
		std::cout << "T = " << T << std::endl;
		std::cout << "E = " << E << std::endl;
		std::cout << "F = " << F << std::endl;

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION > 2)
		std::string strStream;
		if(logStream) (*logStream) << "R = " << (strStream<<R).c_str() << ENDL;
		strStream.clear();
		if(logStream) (*logStream) << "T = " << (strStream<<T).c_str() << ENDL;
		strStream.clear();
		if(logStream) (*logStream) << "E = " << (strStream<<E).c_str() << ENDL;
		strStream.clear();
		if(logStream) (*logStream) << "F = " << (strStream<<F).c_str() << ENDL;
		strStream.clear();
#endif

		if(imageSize_[0] == imageSize_[1] && !ignoreStereoRectification)
		{
			UINFO("Compute stereo rectification");

			cv::Mat R1, R2, P1, P2, Q;
			cv::stereoRectify(left.K_raw(), left.D_raw(),
							right.K_raw(), right.D_raw(),
							imageSize, R, T, R1, R2, P1, P2, Q,
							cv::CALIB_ZERO_DISPARITY, 0, imageSize);

			std::cout << "R1 = " << R1 << std::endl;
			std::cout << "P1 = " << P1 << std::endl;
			std::cout << "R2 = " << R2 << std::endl;
			std::cout << "P2 = " << P2 << std::endl;

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION > 2)
			if(logStream) (*logStream) << "R1 = " << (strStream<<R1).c_str() << ENDL;
			strStream.clear();
			if(logStream) (*logStream) << "P1 = " << (strStream<<P1).c_str() << ENDL;
			strStream.clear();
			if(logStream) (*logStream) << "R2 = " << (strStream<<R2).c_str() << ENDL;
			strStream.clear();
			if(logStream) (*logStream) << "P2 = " << (strStream<<P2).c_str() << ENDL;
#endif

			double err = 0;
			int npoints = 0;
			std::vector<cv::Vec3f> lines[2];
			UINFO("Computing re-projection errors...");
			if(logStream) (*logStream) << "Computing re-projection epipolar errors..." << ENDL;
			for(unsigned int i = 0; i < stereoImagePoints_[0].size(); i++ )
			{
				int npt = (int)stereoImagePoints_[0][i].size();

				cv::Mat imgpt0 = cv::Mat(stereoImagePoints_[0][i]);
				cv::Mat imgpt1 = cv::Mat(stereoImagePoints_[1][i]);
				cv::undistortPoints(imgpt0, imgpt0, left.K_raw(), left.D_raw(), R1, P1);
				cv::undistortPoints(imgpt1, imgpt1, right.K_raw(), right.D_raw(), R2, P2);
				computeCorrespondEpilines(imgpt0, 1, F, lines[0]);
				computeCorrespondEpilines(imgpt1, 2, F, lines[1]);

				double sampleErr = 0.0;
				for(int j = 0; j < npt; j++ )
				{
					double errij = fabs(stereoImagePoints_[0][i][j].x*lines[1][j][0] +
										stereoImagePoints_[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
								   fabs(stereoImagePoints_[1][i][j].x*lines[0][j][0] +
										stereoImagePoints_[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
					sampleErr += errij;
				}
				UINFO("Stereo image %d: %f", stereoImageIds_[i], sampleErr/npt);
				if(logStream) (*logStream) << "Stereo image " << stereoImageIds_[i] << ": " << sampleErr/npt << ENDL;
				err += sampleErr;
				npoints += npt;
			}
			double totalAvgErr = err/(double)npoints;
			UINFO("stereo avg re projection error = %f", totalAvgErr);
			if(logStream) (*logStream) << "stereo avg re projection error = " << totalAvgErr << ENDL;

			output = StereoCameraModel(
							cameraName_.toStdString(),
							imageSize_[0], left.K_raw(), left.D_raw(), R1, P1,
							imageSize_[1], right.K_raw(), right.D_raw(), R2, P2,
							R, T, E, F);
		}
		else
		{
			UDEBUG("%s", cameraName_.toStdString().c_str());
			//Kinect, ignore the stereo rectification
			output = StereoCameraModel(
							cameraName_.toStdString(),
							imageSize_[0], left.K_raw(), left.D_raw(), left.R(), left.P(),
							imageSize_[1], right.K_raw(), right.D_raw(), right.R(), right.P(),
							R, T, E, F);
		}
	}
	return output;
}

bool CalibrationDialog::save()
{
	bool saved = false;
	processingData_ = true;
	if(!stereo_)
	{
		UASSERT(models_[0].isValidForRectification());
		QString cameraName = models_[0].name().c_str();
		QString filePath = QFileDialog::getSaveFileName(this, tr("Export"), savingDirectory_+"/"+cameraName+".yaml", "*.yaml");

		if(!filePath.isEmpty())
		{
			QString name = QFileInfo(filePath).baseName();
			QString dir = QFileInfo(filePath).absoluteDir().absolutePath();
			models_[0].setName(name.toStdString());
			if(models_[0].save(dir.toStdString()))
			{
				QMessageBox::information(this, tr("Export"), tr("Calibration file saved to \"%1\".").arg(filePath));
				UINFO("Saved \"%s\"!", filePath.toStdString().c_str());
				savedCalibration_ = true;
				saved = true;
			}
			else
			{
				UERROR("Error saving \"%s\"", filePath.toStdString().c_str());
			}
		}
	}
	else
	{
		UASSERT(stereoModel_.left().isValidForRectification() &&
				stereoModel_.right().isValidForRectification());
		QString cameraName = stereoModel_.name().c_str();
		QString filePath = QFileDialog::getSaveFileName(this, tr("Export"), savingDirectory_ + "/" + cameraName, "*.yaml");
		QString name = QFileInfo(filePath).baseName();
		QString dir = QFileInfo(filePath).absoluteDir().absolutePath();
		if(!name.isEmpty())
		{
			bool switched = ui_->checkBox_switchImages->isChecked();
			stereoModel_.setName(name.toStdString(), switched?rightSuffix_.toStdString():leftSuffix_.toStdString(), switched?leftSuffix_.toStdString():rightSuffix_.toStdString());
			std::string base = (dir+QDir::separator()+name).toStdString();
			std::string leftPath = base+"_"+stereoModel_.getLeftSuffix()+".yaml";
			std::string rightPath = base+"_"+stereoModel_.getRightSuffix()+".yaml";
			std::string posePath = base+"_pose.yaml";
			if(stereoModel_.save(dir.toStdString(), false))
			{
				QMessageBox::information(this, tr("Export"), tr("Calibration files saved:\n  \"%1\"\n  \"%2\"\n  \"%3\".").
						arg(leftPath.c_str()).arg(rightPath.c_str()).arg(posePath.c_str()));
				UINFO("Saved \"%s\" and \"%s\"!", leftPath.c_str(), rightPath.c_str());
				savedCalibration_ = true;
				saved = true;
			}
			else
			{
				UERROR("Error saving \"%s\" and \"%s\"", leftPath.c_str(), rightPath.c_str());
			}
		}
	}
	processingData_ = false;
	return saved;
}

float CalibrationDialog::getArea(const std::vector<cv::Point2f> & corners, const cv::Size & boardSize)
{
	//Get 2d image area of the detected checkerboard.
	//The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
	//|p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.

	cv::Point2f up_left;
	cv::Point2f up_right;
	cv::Point2f down_right;
	cv::Point2f down_left;
	if((int)corners.size() == (boardSize.width * boardSize.height))
	{
		up_left = corners[0];
		up_right = corners[boardSize.width-1];
		down_right = corners[corners.size()-1];
		down_left = corners[corners.size()-boardSize.width];
	}
	else
	{
		cv::Rect rect = cv::boundingRect(corners);
		up_left = cv::Point2f(rect.x, rect.y);
		up_right = cv::Point2f(rect.x+rect.width, rect.y);
		down_right = cv::Point2f(rect.x+rect.width, rect.y+rect.height);
		down_left = cv::Point2f(rect.x, rect.y+rect.height);
	}
	cv::Point2f a = up_right - up_left;
	cv::Point2f b = down_right - up_right;
	cv::Point2f c = down_left - down_right;
	cv::Point2f p = b + c;
	cv::Point2f q = a + b;
	return std::fabs(p.x*q.y - p.y*q.x) / 2.0f;
}

float CalibrationDialog::getSkew(const std::vector<cv::Point2f> & fourCorners)
{
	UASSERT(fourCorners.size() == 4);
	std::vector<cv::Point2f> corners = fourCorners;
	corners.resize(3);
	return getSkew(corners, cv::Size(2,1));
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
