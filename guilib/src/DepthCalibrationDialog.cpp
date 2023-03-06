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

#include <rtabmap/gui/DepthCalibrationDialog.h>
#include "ui_depthCalibrationDialog.h"

#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/ImageView.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UCv2Qt.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"

#include "rtabmap/core/clams/slam_calibrator.h"
#include "rtabmap/core/clams/frame_projector.h"

#include <QPushButton>
#include <QDir>
#include <QFileInfo>
#include <QUrl>
#include <QtGui/QDesktopServices>
#include <QMessageBox>
#include <QFileDialog>

namespace rtabmap {

DepthCalibrationDialog::DepthCalibrationDialog(QWidget *parent) :
			QDialog(parent),
			_canceled(false),
			_model(0)
{
	_ui = new Ui_DepthCalibrationDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));
	connect(_ui->buttonBox->button(QDialogButtonBox::Save), SIGNAL(clicked()), this, SLOT(saveModel()));
	connect(_ui->buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), this, SLOT(accept()));
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setText("Calibrate");

	restoreDefaults();

	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_minDepth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_coneRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_coneStdDevThresh, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_laserScan, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->spinBox_bin_width, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_bin_height, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_bin_depth, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_smoothing, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_maxDepthModel, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	_ui->buttonBox->button(QDialogButtonBox::Ok)->setFocus();

	_progressDialog = new ProgressDialog(this);
	_progressDialog->setVisible(false);
	_progressDialog->setAutoClose(true, 2);
	_progressDialog->setMinimumWidth(600);
	_progressDialog->setCancelButtonVisible(true);

	connect(_progressDialog, SIGNAL(canceled()), this, SLOT(cancel()));
}

DepthCalibrationDialog::~DepthCalibrationDialog()
{
	delete _ui;
	delete _model;
}

void DepthCalibrationDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	settings.setValue("decimation", _ui->spinBox_decimation->value());
	settings.setValue("max_depth", _ui->doubleSpinBox_maxDepth->value());
	settings.setValue("min_depth", _ui->doubleSpinBox_minDepth->value());
	settings.setValue("voxel",_ui->doubleSpinBox_voxelSize->value());
	settings.setValue("cone_radius",_ui->doubleSpinBox_coneRadius->value());
	settings.setValue("cone_stddev_thresh",_ui->doubleSpinBox_coneStdDevThresh->value());
	settings.setValue("laser_scan",_ui->checkBox_laserScan->isChecked());

	settings.setValue("bin_width",_ui->spinBox_bin_width->value());
	settings.setValue("bin_height",_ui->spinBox_bin_height->value());
	settings.setValue("bin_depth",_ui->doubleSpinBox_bin_depth->value());
	settings.setValue("smoothing",_ui->spinBox_smoothing->value());
	settings.setValue("max_model_depth",_ui->doubleSpinBox_maxDepthModel->value());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void DepthCalibrationDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	_ui->spinBox_decimation->setValue(settings.value("decimation", _ui->spinBox_decimation->value()).toInt());
	_ui->doubleSpinBox_maxDepth->setValue(settings.value("max_depth", _ui->doubleSpinBox_maxDepth->value()).toDouble());
	_ui->doubleSpinBox_minDepth->setValue(settings.value("min_depth", _ui->doubleSpinBox_minDepth->value()).toDouble());
	_ui->doubleSpinBox_voxelSize->setValue(settings.value("voxel", _ui->doubleSpinBox_voxelSize->value()).toDouble());
	_ui->doubleSpinBox_coneRadius->setValue(settings.value("cone_radius", _ui->doubleSpinBox_coneRadius->value()).toDouble());
	_ui->doubleSpinBox_coneStdDevThresh->setValue(settings.value("cone_stddev_thresh", _ui->doubleSpinBox_coneStdDevThresh->value()).toDouble());
	_ui->checkBox_laserScan->setChecked(settings.value("laser_scan", _ui->checkBox_laserScan->isChecked()).toBool());

	_ui->spinBox_bin_width->setValue(settings.value("bin_width", _ui->spinBox_bin_width->value()).toInt());
	_ui->spinBox_bin_height->setValue(settings.value("bin_height", _ui->spinBox_bin_height->value()).toInt());
	_ui->doubleSpinBox_bin_depth->setValue(settings.value("bin_depth", _ui->doubleSpinBox_bin_depth->value()).toDouble());
	_ui->spinBox_smoothing->setValue(settings.value("smoothing", _ui->spinBox_smoothing->value()).toInt());
	_ui->doubleSpinBox_maxDepthModel->setValue(settings.value("max_model_depth", _ui->doubleSpinBox_maxDepthModel->value()).toDouble());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void DepthCalibrationDialog::restoreDefaults()
{
	_ui->spinBox_decimation->setValue(1);
	_ui->doubleSpinBox_maxDepth->setValue(3.5);
	_ui->doubleSpinBox_minDepth->setValue(0);
	_ui->doubleSpinBox_voxelSize->setValue(0.01);
	_ui->doubleSpinBox_coneRadius->setValue(0.02);
	_ui->doubleSpinBox_coneStdDevThresh->setValue(0.1); // 0.03
	_ui->checkBox_laserScan->setChecked(false);
	_ui->checkBox_resetModel->setChecked(true);

	_ui->spinBox_bin_width->setValue(8);
	_ui->spinBox_bin_height->setValue(6);
	if(_imageSize.width > 0 && _imageSize.height > 0)
	{
		size_t bin_width, bin_height;
		clams::DiscreteDepthDistortionModel::getBinSize(_imageSize.width, _imageSize.height, bin_width, bin_height);
		_ui->spinBox_bin_width->setValue(bin_width);
		_ui->spinBox_bin_height->setValue(bin_height);
	}
	_ui->doubleSpinBox_bin_depth->setValue(2.0),
	_ui->spinBox_smoothing->setValue(1);
	_ui->doubleSpinBox_maxDepthModel->setValue(10.0);
}

void DepthCalibrationDialog::saveModel()
{
	if(_model && _model->getTrainingSamples())
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save distortion model to ..."), _workingDirectory+QDir::separator()+"distortion_model.bin", tr("Distortion model (*.bin *.txt)"));
		if(!path.isEmpty())
		{
			//
			// Save depth calibration
			//
			cv::Mat results = _model->visualize(ULogger::level() == ULogger::kDebug?_workingDirectory.toStdString():"");
			_model->save(path.toStdString());

			if(!results.empty())
			{
				QString name = QString(path).replace(".bin", ".png", Qt::CaseInsensitive).replace(".txt", ".png", Qt::CaseInsensitive);
				cv::imwrite(name.toStdString(), results);
				QDesktopServices::openUrl(QUrl::fromLocalFile(name));
			}

			QMessageBox::information(this, tr("Depth Calibration"), tr("Distortion model saved to \"%1\"!").arg(path));
		}
	}
}

void DepthCalibrationDialog::cancel()
{
	_canceled = true;
	_progressDialog->appendText(tr("Canceled!"));
}

void DepthCalibrationDialog::calibrate(
		const std::map<int, Transform> & poses,
			const QMap<int, Signature> & cachedSignatures,
			const QString & workingDirectory,
			const ParametersMap & parameters)
{
	_canceled = false;
	_workingDirectory = workingDirectory;
	_ui->buttonBox->button(QDialogButtonBox::Save)->setEnabled(_model && _model->getTrainingSamples()>0);
	if(_model)
	{
		_ui->label_trainingSamples->setNum((int)_model->getTrainingSamples());
	}

	_ui->label_width->setText("NA");
	_ui->label_height->setText("NA");
	_imageSize = cv::Size();
	CameraModel model;
	if(cachedSignatures.size())
	{
		const Signature & s = cachedSignatures.begin().value();
		const SensorData & data = s.sensorData();
		cv::Mat depth;
		data.uncompressDataConst(0, &depth);
		if(data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForProjection() && !depth.empty())
		{
			// use depth image size
			_imageSize = depth.size();
			_ui->label_width->setNum(_imageSize.width);
			_ui->label_height->setNum(_imageSize.height);

			if(_imageSize.width % _ui->spinBox_bin_width->value() != 0 ||
				_imageSize.height % _ui->spinBox_bin_height->value() != 0)
			{
				size_t bin_width, bin_height;
				clams::DiscreteDepthDistortionModel::getBinSize(_imageSize.width, _imageSize.height, bin_width, bin_height);
				_ui->spinBox_bin_width->setValue(bin_width);
				_ui->spinBox_bin_height->setValue(bin_height);
			}
		}
		else if(data.cameraModels().size() > 1)
		{
			QMessageBox::warning(this, tr("Depth Calibration"),tr("Multi-camera not supported!"));
			return;
		}
		else if(data.cameraModels().size() != 1)
		{
			QMessageBox::warning(this, tr("Depth Calibration"), tr("Camera model not found."));
			return;
		}
		else if(data.cameraModels().size() == 1 && !data.cameraModels()[0].isValidForProjection())
		{
			QMessageBox::warning(this, tr("Depth Calibration"), tr("Camera model %1 not valid for projection.").arg(s.id()));
			return;
		}
		else
		{
			QMessageBox::warning(this, tr("Depth Calibration"), tr("Depth image cannot be found in the cache, make sure to update cache before doing calibration."));
			return;
		}
	}
	else
	{
		QMessageBox::warning(this, tr("Depth Calibration"), tr("No signatures detected! Map is empty!?"));
		return;
	}

	if(this->exec() == QDialog::Accepted)
	{
		if(_model && _ui->checkBox_resetModel->isChecked())
		{
			delete _model;
			_model = 0;
		}

		if(_ui->doubleSpinBox_maxDepthModel->value() < _ui->doubleSpinBox_bin_depth->value())
		{
			QMessageBox::warning(this, tr("Wrong parameter"), tr("Maximum model depth should be higher than bin depth, setting to bin depth x5."));
			_ui->doubleSpinBox_maxDepthModel->setValue(_ui->doubleSpinBox_bin_depth->value() * 5.0);
		}

		_progressDialog->setMaximumSteps(poses.size()*2 + 3);
		if(_ui->doubleSpinBox_voxelSize->value() > 0.0)
		{
			_progressDialog->setMaximumSteps(_progressDialog->maximumSteps()+1);
		}
		_progressDialog->resetProgress();
		_progressDialog->show();

		std::map<int, rtabmap::SensorData> sequence;

		// Create the map
		pcl::PointCloud<pcl::PointXYZ>::Ptr  map(new pcl::PointCloud<pcl::PointXYZ>);
		int index=1;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end() && !_canceled; ++iter, ++index)
		{
			int points = 0;
			if(!iter->second.isNull())
			{
				pcl::IndicesPtr indices(new std::vector<int>);
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					SensorData data = s.sensorData();

					cv::Mat  depth;
					LaserScan laserScan;
					data.uncompressData(0, &depth, _ui->checkBox_laserScan->isChecked()?&laserScan:0);
					if(data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForProjection() && !depth.empty())
					{
						UASSERT(iter->first == data.id());
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

						if(_ui->checkBox_laserScan->isChecked())
						{
							cloud = util3d::laserScanToPointCloud(laserScan);
							indices->resize(cloud->size());
							for(unsigned int i=0; i<indices->size(); ++i)
							{
								indices->at(i) = i;
							}
						}
						else
						{
							cloud = util3d::cloudFromSensorData(
									data,
									_ui->spinBox_decimation->value(),
									_ui->doubleSpinBox_maxDepth->value(),
									_ui->doubleSpinBox_minDepth->value(),
									indices.get(),
									parameters);
						}

						if(indices->size())
						{
							if(_ui->doubleSpinBox_voxelSize->value() > 0.0)
							{
								cloud = util3d::voxelize(cloud, indices, _ui->doubleSpinBox_voxelSize->value());
							}

							cloud = util3d::transformPointCloud(cloud, iter->second);

							points+=cloud->size();

							*map += *cloud;

							sequence.insert(std::make_pair(iter->first, data));

							cv::Size size = depth.size();
							if(_model &&
								(_model->getWidth()!=size.width ||
								 _model->getHeight()!=size.height))
							{
								QString msg = tr("Depth images (%1x%2) in the map don't have the "
										   "same size then in the current model (%3x%4). You may want "
										   "to check \"Reset previous model\" before trying again.")
										   .arg(size.width).arg(size.height)
										   .arg(_model->getWidth()).arg(_model->getHeight());
								QMessageBox::warning(this, tr("Depth Calibration"), msg);
								_progressDialog->appendText(msg, Qt::darkRed);
								_progressDialog->setAutoClose(false);
								return;
							}
						}
					}
					else
					{
						_progressDialog->appendText(tr("Not suitable camera model found for node %1, ignoring this node!").arg(iter->first), Qt::darkYellow);
						_progressDialog->setAutoClose(false);
					}
				}
				else
				{
					UERROR("Cloud %d not found in cache!", iter->first);
				}
			}
			else
			{
				UERROR("transform is null!?");
			}

			if(points>0)
			{
				_progressDialog->appendText(tr("Generated cloud %1 with %2 points (%3/%4).")
						.arg(iter->first).arg(points).arg(index).arg(poses.size()));
			}
			else
			{
				_progressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(index).arg(poses.size()));
			}
			_progressDialog->incrementStep();
			QApplication::processEvents();
		}

		if(!_canceled && map->size() && sequence.size())
		{
			if(_ui->doubleSpinBox_voxelSize->value() > 0.0)
			{
				_progressDialog->appendText(tr("Voxel filtering (%1 m) of the merged point cloud (%2 points)")
						.arg(_ui->doubleSpinBox_voxelSize->value())
						.arg(map->size()));
				QApplication::processEvents();
				QApplication::processEvents();

				map = util3d::voxelize(map, _ui->doubleSpinBox_voxelSize->value());
				_progressDialog->incrementStep();
			}

			if(ULogger::level() == ULogger::kDebug)
			{
				//
				// Show 3D map with frustums
				//
				QDialog * window = new QDialog(this->parentWidget()?this->parentWidget():this, Qt::Window);
				window->setAttribute(Qt::WA_DeleteOnClose, true);
				window->setWindowTitle(tr("Map"));
				window->setMinimumWidth(800);
				window->setMinimumHeight(600);

				CloudViewer * viewer = new CloudViewer(window);
				viewer->setCameraLockZ(false);
				viewer->setFrustumShown(true);

				QVBoxLayout *layout = new QVBoxLayout();
				layout->addWidget(viewer);
				window->setLayout(layout);
				connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

				window->show();

				uSleep(500);

				_progressDialog->appendText(tr("Viewing the cloud (%1 points and %2 poses)...").arg(map->size()).arg(sequence.size()));
				_progressDialog->incrementStep();
				viewer->addCloud("map", map);
				for(std::map<int, SensorData>::iterator iter=sequence.begin(); iter!=sequence.end(); ++iter)
				{
					Transform baseToCamera = iter->second.cameraModels()[0].localTransform();
					viewer->addOrUpdateFrustum(uFormat("frustum%d",iter->first), poses.at(iter->first), baseToCamera, 0.2, QColor(), iter->second.cameraModels()[0].fovX(), iter->second.cameraModels()[0].fovY());
				}
				_progressDialog->appendText(tr("Viewing the cloud (%1 points and %2 poses)... done.").arg(map->size()).arg(sequence.size()));

				viewer->update();
			}

			_progressDialog->appendText(tr("CLAMS depth calibration..."));
			QApplication::processEvents();
			QApplication::processEvents();

			QDialog * dialog = new QDialog(this->parentWidget()?this->parentWidget():this, Qt::Window);
			dialog->setAttribute(Qt::WA_DeleteOnClose, true);
			dialog->setWindowTitle(tr("Original/Map"));
			dialog->setMinimumWidth(_imageSize.width);
			ImageView * imageView1 = new ImageView(dialog);
			imageView1->setMinimumSize(320, 240);
			ImageView * imageView2 = new ImageView(dialog);
			imageView2->setMinimumSize(320, 240);
			QVBoxLayout * vlayout = new QVBoxLayout();
			vlayout->setContentsMargins(0,0,0,0);
			vlayout->addWidget(imageView1, 1);
			vlayout->addWidget(imageView2, 1);
			dialog->setLayout(vlayout);
			if(ULogger::level() == ULogger::kDebug)
			{
				dialog->show();
			}

			//clams::DiscreteDepthDistortionModel model = clams::calibrate(sequence, poses, map);
			const cv::Size & imageSize = _imageSize;
			if(_model == 0)
			{
				size_t bin_width = _ui->spinBox_bin_width->value();
				size_t bin_height = _ui->spinBox_bin_height->value();
				if(imageSize.width % _ui->spinBox_bin_width->value() != 0 ||
				   imageSize.height % _ui->spinBox_bin_height->value() != 0)
				{
					size_t bin_width, bin_height;
					clams::DiscreteDepthDistortionModel::getBinSize(imageSize.width, imageSize.height, bin_width, bin_height);
					_ui->spinBox_bin_width->setValue(bin_width);
					_ui->spinBox_bin_height->setValue(bin_height);
				}
				_model = new clams::DiscreteDepthDistortionModel(
						imageSize.width,
						imageSize.height,
						bin_width,
						bin_height,
						_ui->doubleSpinBox_bin_depth->value(),
						_ui->spinBox_smoothing->value(),
						_ui->doubleSpinBox_maxDepthModel->value());
			}
			UASSERT(_model->getWidth() == imageSize.width && _model->getHeight() == imageSize.height);

			// -- For all selected frames, accumulate training examples
			//    in the distortion model.
			size_t counts;
			index = 0;
			for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin(); iter != poses.end() && !_canceled; ++iter)
			{
			  size_t idx = iter->first;
			  std::map<int, rtabmap::SensorData>::const_iterator ster = sequence.find(idx);
			  if(ster!=sequence.end())
			  {
				  cv::Mat depthImage;
				  ster->second.uncompressDataConst(0, &depthImage);

				  if(ster->second.cameraModels().size() == 1 && ster->second.cameraModels()[0].isValidForProjection() && !depthImage.empty())
				  {
					  cv::Mat mapDepth;
					  CameraModel model = ster->second.cameraModels()[0];
					  if(model.imageWidth() != depthImage.cols)
					  {
						  UASSERT_MSG(model.imageHeight() % depthImage.rows == 0, uFormat("rgb=%d depth=%d", model.imageHeight(), depthImage.rows).c_str());
						  model = model.scaled(double(depthImage.rows) / double(model.imageHeight()));
					  }
					  clams::FrameProjector projector(model);
					  mapDepth = projector.estimateMapDepth(
							  map,
							  iter->second.inverse(),
							  depthImage,
							 _ui->doubleSpinBox_coneRadius->value(),
							 _ui->doubleSpinBox_coneStdDevThresh->value());

					  if(ULogger::level() == ULogger::kDebug)
					  {
						  imageView1->setImage(uCvMat2QImage(depthImage));
						  imageView2->setImage(uCvMat2QImage(mapDepth));
					  }

					  counts = _model->accumulate(mapDepth, depthImage);
					  _progressDialog->appendText(tr("Added %1 training examples from node %2 (%3/%4).").arg(counts).arg(iter->first).arg(++index).arg(sequence.size()));
				  }
				 }
			  _progressDialog->incrementStep();
			  QApplication::processEvents();
			}

			_progressDialog->appendText(tr("CLAMS depth calibration... done!"));
			QApplication::processEvents();

			if(!_canceled)
			{
				this->saveModel();
			}
		}
		else
		{
			QMessageBox::warning(this, tr("Depth Calibration"), tr("The resulting map is empty!"));
		}
		_progressDialog->setValue(_progressDialog->maximumSteps());
	}
}

} /* namespace rtabmap */
