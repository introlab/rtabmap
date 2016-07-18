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

#include "ExportScansDialog.h"
#include "ui_exportScansDialog.h"

#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UStl.h"

#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Graph.h"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <QPushButton>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>

namespace rtabmap {

ExportScansDialog::ExportScansDialog(QWidget *parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportScansDialog();
	_ui->setupUi(this);

	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	restoreDefaults();

	connect(_ui->checkBox_binary, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_normalKSearch, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_regenerate, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_decimation, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->groupBox_filtering, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_filteringRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_filteringMinNeighbors, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->checkBox_assemble, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_voxelSize_assembled, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	_progressDialog = new ProgressDialog(this);
	_progressDialog->setVisible(false);
	_progressDialog->setAutoClose(true, 2);
	_progressDialog->setMinimumWidth(600);
}

ExportScansDialog::~ExportScansDialog()
{
	delete _ui;
}

void ExportScansDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("binary", _ui->checkBox_binary->isChecked());
	settings.setValue("normals_k", _ui->spinBox_normalKSearch->value());

	settings.setValue("regenerate", _ui->groupBox_regenerate->isChecked());
	settings.setValue("regenerate_decimation", _ui->spinBox_decimation->value());

	settings.setValue("filtering", _ui->groupBox_filtering->isChecked());
	settings.setValue("filtering_radius", _ui->doubleSpinBox_filteringRadius->value());
	settings.setValue("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value());

	settings.setValue("assemble", _ui->checkBox_assemble->isChecked());
	settings.setValue("assemble_voxel",_ui->doubleSpinBox_voxelSize_assembled->value());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportScansDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	_ui->checkBox_binary->setChecked(settings.value("binary", _ui->checkBox_binary->isChecked()).toBool());
	_ui->spinBox_normalKSearch->setValue(settings.value("normals_k", _ui->spinBox_normalKSearch->value()).toInt());

	_ui->groupBox_regenerate->setChecked(settings.value("regenerate", _ui->groupBox_regenerate->isChecked()).toBool());
	_ui->spinBox_decimation->setValue(settings.value("regenerate_decimation", _ui->spinBox_decimation->value()).toInt());

	_ui->groupBox_filtering->setChecked(settings.value("filtering", _ui->groupBox_filtering->isChecked()).toBool());
	_ui->doubleSpinBox_filteringRadius->setValue(settings.value("filtering_radius", _ui->doubleSpinBox_filteringRadius->value()).toDouble());
	_ui->spinBox_filteringMinNeighbors->setValue(settings.value("filtering_min_neighbors", _ui->spinBox_filteringMinNeighbors->value()).toInt());

	_ui->checkBox_assemble->setChecked(settings.value("assemble", _ui->checkBox_assemble->isChecked()).toBool());
	_ui->doubleSpinBox_voxelSize_assembled->setValue(settings.value("assemble_voxel", _ui->doubleSpinBox_voxelSize_assembled->value()).toDouble());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportScansDialog::restoreDefaults()
{
	_ui->checkBox_binary->setChecked(true);
	_ui->spinBox_normalKSearch->setValue(20);

	_ui->groupBox_regenerate->setChecked(false);
	_ui->spinBox_decimation->setValue(1);

	_ui->groupBox_filtering->setChecked(false);
	_ui->doubleSpinBox_filteringRadius->setValue(0.02);
	_ui->spinBox_filteringMinNeighbors->setValue(2);

	_ui->checkBox_assemble->setChecked(true);
	_ui->doubleSpinBox_voxelSize_assembled->setValue(0.01);

	this->update();
}

void ExportScansDialog::setSaveButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(false);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(true);
	_ui->checkBox_binary->setVisible(true);
	_ui->label_binaryFile->setVisible(true);
}

void ExportScansDialog::setOkButton()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setVisible(true);
	_ui->buttonBox->button(QDialogButtonBox::Save)->setVisible(false);
	_ui->checkBox_binary->setVisible(false);
	_ui->label_binaryFile->setVisible(false);
}

void ExportScansDialog::enableRegeneration(bool enabled)
{
	if(!enabled)
	{
		_ui->groupBox_regenerate->setChecked(false);
	}
	_ui->groupBox_regenerate->setEnabled(enabled);
}

void ExportScansDialog::exportScans(
		const std::map<int, Transform> & poses,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, cv::Mat> & createdScans,
		const QString & workingDirectory)
{
	std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;

	setSaveButton();

	if(getExportedScans(
			poses,
			mapIds,
			cachedSignatures,
			createdScans,
			workingDirectory,
			clouds))
	{
		saveScans(workingDirectory, poses, clouds, _ui->checkBox_binary->isChecked());
		_progressDialog->setValue(_progressDialog->maximumSteps());
	}
}

void ExportScansDialog::viewScans(
		const std::map<int, Transform> & poses,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, cv::Mat> & createdScans,
		const QString & workingDirectory)
{
	std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;

	setOkButton();
	if(getExportedScans(
			poses,
			mapIds,
			cachedSignatures,
			createdScans,
			workingDirectory,
			clouds))
	{
		QDialog * window = new QDialog(this->parentWidget()?this->parentWidget():this, Qt::Window);
		window->setAttribute(Qt::WA_DeleteOnClose, true);
		window->setWindowTitle(tr("Scans (%1 nodes)").arg(clouds.size()));
		window->setMinimumWidth(800);
		window->setMinimumHeight(600);

		CloudViewer * viewer = new CloudViewer(window);
		viewer->setCameraLockZ(false);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(viewer);
		window->setLayout(layout);
		connect(window, SIGNAL(finished(int)), viewer, SLOT(clear()));

		window->show();

		uSleep(500);
		if(clouds.size())
		{
			for(std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr>::iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
			{
				_progressDialog->appendText(tr("Viewing the cloud %1 (%2 points)...").arg(iter->first).arg(iter->second->size()));
				_progressDialog->incrementStep();

				QColor color = Qt::gray;
				int mapId = uValue(mapIds, iter->first, -1);
				if(mapId >= 0)
				{
					color = (Qt::GlobalColor)(mapId % 12 + 7 );
				}
				viewer->addCloud(uFormat("cloud%d",iter->first), iter->second, iter->first>0?poses.at(iter->first):Transform::getIdentity());
				_progressDialog->appendText(tr("Viewing the cloud %1 (%2 points)... done.").arg(iter->first).arg(iter->second->size()));
			}
		}

		_progressDialog->setValue(_progressDialog->maximumSteps());
		viewer->update();
	}
}

bool ExportScansDialog::getExportedScans(
		const std::map<int, Transform> & poses,
		const std::map<int, int> & mapIds,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, cv::Mat> & createdClouds,
		const QString & workingDirectory,
		std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> & cloudsWithNormals)
{
	enableRegeneration(cachedSignatures.size());
	if(this->exec() == QDialog::Accepted)
	{
		_progressDialog->resetProgress();
		_progressDialog->show();
		int mul = 1;
		if(_ui->checkBox_assemble->isChecked())
		{
			mul+=1;
		}
		mul+=1; // normals
		_progressDialog->setMaximumSteps(int(poses.size())*mul+1);

		std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds = this->getScans(
				poses,
				cachedSignatures,
				createdClouds);

		if(_ui->checkBox_assemble->isChecked())
		{
			_progressDialog->appendText(tr("Assembling %1 clouds...").arg(clouds.size()));
			QApplication::processEvents();

			pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<int> rawCameraIndices;
			int i =0;
			pcl::PointCloud<pcl::PointNormal>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointNormal>);
			for(std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr>::iterator iter=clouds.begin();
				iter!= clouds.end();
				++iter)
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointNormal>);
				transformed = util3d::transformPointCloud(iter->second, poses.at(iter->first));

				*assembledCloud += *transformed;

				rawCameraIndices.resize(assembledCloud->size(), iter->first);

				_progressDialog->appendText(tr("Assembled cloud %1, total=%2 (%3/%4).").arg(iter->first).arg(assembledCloud->size()).arg(++i).arg(clouds.size()));
				_progressDialog->incrementStep();
				QApplication::processEvents();
			}

			pcl::copyPointCloud(*assembledCloud, *rawAssembledCloud);

			if(_ui->doubleSpinBox_voxelSize_assembled->value())
			{
				_progressDialog->appendText(tr("Voxelize cloud (%1 points, voxel size = %2 m)...")
						.arg(assembledCloud->size())
						.arg(_ui->doubleSpinBox_voxelSize_assembled->value()));
				QApplication::processEvents();

				assembledCloud = util3d::voxelize(
						assembledCloud,
						_ui->doubleSpinBox_voxelSize_assembled->value());

				if(_ui->spinBox_normalKSearch->value() > 0)
				{
					_progressDialog->appendText(tr("Compute normals (%1 points)...")
							.arg(assembledCloud->size()));
					QApplication::processEvents();

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::copyPointCloud(*assembledCloud, *cloudXYZ);

					pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudXYZ, _ui->spinBox_normalKSearch->value());
					pcl::concatenateFields(*cloudXYZ, *normals, *assembledCloud);

					_progressDialog->appendText(tr("Update %1 normals with %2 camera views...")
							.arg(assembledCloud->size()).arg(poses.size()));
							util3d::adjustNormalsToViewPoints(
									poses,
									rawAssembledCloud,
									rawCameraIndices,
									assembledCloud);
				}
			}

			clouds.clear();
			clouds.insert(std::make_pair(0, assembledCloud));
		}

		//fill cloudWithNormals
		for(std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr>::iterator iter=clouds.begin();
			iter!= clouds.end();
			++iter)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals = iter->second;

			cloudsWithNormals.insert(std::make_pair(iter->first, cloudWithNormals));

			_progressDialog->incrementStep();
			QApplication::processEvents();
		}

		return true;
	}
	return false;
}

std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> ExportScansDialog::getScans(
		const std::map<int, Transform> & poses,
		const QMap<int, Signature> & cachedSignatures,
		const std::map<int, cv::Mat> & createdScans) const
{
	std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;
	int i=0;
	pcl::PointCloud<pcl::PointNormal>::Ptr previousCloud;
	pcl::IndicesPtr previousIndices;
	Transform previousPose;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		int points = 0;
		if(!iter->second.isNull())
		{
			cv::Mat scan;
			if(_ui->groupBox_regenerate->isChecked())
			{
				if(cachedSignatures.contains(iter->first))
				{
					const Signature & s = cachedSignatures.find(iter->first).value();
					SensorData d = s.sensorData();
					d.uncompressData(0, 0, &scan);
					if(!scan.empty())
					{
						if(_ui->spinBox_decimation->value() > 1)
						{
							scan = util3d::downsample(scan, _ui->spinBox_decimation->value());
						}
					}
				}
				else
				{
					UERROR("Scan %d not found in cache!", iter->first);
				}
			}
			else
			{
				scan = uValue(createdScans, iter->first, cv::Mat());
			}

			if(!scan.empty())
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
				if(scan.channels() == 6 && _ui->doubleSpinBox_voxelSize_assembled->value() == 0.0)
				{
					cloud = util3d::laserScanToPointCloudNormal(scan);
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ = util3d::laserScanToPointCloud(scan);

					if(_ui->doubleSpinBox_voxelSize_assembled->value() > 0.0)
					{
						cloudXYZ = util3d::voxelize(
								cloudXYZ,
								_ui->doubleSpinBox_voxelSize_assembled->value());
					}

					if(!_ui->checkBox_assemble->isChecked() && _ui->spinBox_normalKSearch->value() > 0)
					{
						pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloudXYZ, _ui->spinBox_normalKSearch->value());
						pcl::concatenateFields(*cloudXYZ, *normals, *cloud);
					}
					else
					{
						pcl::copyPointCloud(*cloudXYZ, *cloud);
					}
				}

				if(cloud->size())
				{
					if(_ui->groupBox_filtering->isChecked() &&
						_ui->doubleSpinBox_filteringRadius->value() > 0.0f &&
						_ui->spinBox_filteringMinNeighbors->value() > 0)
					{
						pcl::IndicesPtr indices = util3d::radiusFiltering(cloud, _ui->doubleSpinBox_filteringRadius->value(), _ui->spinBox_filteringMinNeighbors->value());
						pcl::PointCloud<pcl::PointNormal>::Ptr tmp(new pcl::PointCloud<pcl::PointNormal>);
						pcl::copyPointCloud(*cloud, *indices, *tmp);
						cloud = tmp;
					}

					clouds.insert(std::make_pair(iter->first, cloud));
					points = cloud->size();
				}
			}
		}
		else
		{
			UERROR("transform is null!?");
		}

		if(points>0)
		{
			_progressDialog->appendText(tr("Generated cloud %1 with %2 points (%3/%4).")
					.arg(iter->first).arg(points).arg(++i).arg(poses.size()));
		}
		else
		{
			_progressDialog->appendText(tr("Ignored cloud %1 (%2/%3).").arg(iter->first).arg(++i).arg(poses.size()));
		}
		_progressDialog->incrementStep();
		QApplication::processEvents();
	}

	return clouds;
}


void ExportScansDialog::saveScans(
		const QString & workingDirectory,
		const std::map<int, Transform> & poses,
		const std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr> & clouds,
		bool binaryMode)
{
	if(clouds.size() == 1)
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save scan to ..."), workingDirectory+QDir::separator()+"scan.ply", tr("Point cloud data (*.ply *.pcd)"));
		if(!path.isEmpty())
		{
			if(clouds.begin()->second->size())
			{
				_progressDialog->appendText(tr("Saving the scan (%1 points)...").arg(clouds.begin()->second->size()));

				bool success =false;
				if(QFileInfo(path).suffix() == "pcd")
				{
					success = pcl::io::savePCDFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "ply")
				{
					success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else if(QFileInfo(path).suffix() == "")
				{
					//use ply by default
					path += ".ply";
					success = pcl::io::savePLYFile(path.toStdString(), *clouds.begin()->second, binaryMode) == 0;
				}
				else
				{
					UERROR("Extension not recognized! (%s) Should be one of (*.ply *.pcd).", QFileInfo(path).suffix().toStdString().c_str());
				}
				if(success)
				{
					_progressDialog->incrementStep();
					_progressDialog->appendText(tr("Saving the scan (%1 points)... done.").arg(clouds.begin()->second->size()));

					QMessageBox::information(this, tr("Save successful!"), tr("Scan saved to \"%1\"").arg(path));
				}
				else
				{
					QMessageBox::warning(this, tr("Save failed!"), tr("Failed to save to \"%1\"").arg(path));
				}
			}
			else
			{
				QMessageBox::warning(this, tr("Save failed!"), tr("Scan is empty..."));
			}
		}
	}
	else if(clouds.size())
	{
		QString path = QFileDialog::getExistingDirectory(this, tr("Save scans to (*.ply *.pcd)..."), workingDirectory, 0);
		if(!path.isEmpty())
		{
			bool ok = false;
			QStringList items;
			items.push_back("ply");
			items.push_back("pcd");
			QString suffix = QInputDialog::getItem(this, tr("File format"), tr("Which format?"), items, 0, false, &ok);

			if(ok)
			{
				QString prefix = QInputDialog::getText(this, tr("File prefix"), tr("Prefix:"), QLineEdit::Normal, "scan", &ok);

				if(ok)
				{
					for(std::map<int, pcl::PointCloud<pcl::PointNormal>::Ptr >::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
					{
						if(iter->second->size())
						{
							pcl::PointCloud<pcl::PointNormal>::Ptr transformedCloud;
							transformedCloud = util3d::transformPointCloud(iter->second, poses.at(iter->first));

							QString pathFile = path+QDir::separator()+QString("%1%2.%3").arg(prefix).arg(iter->first).arg(suffix);
							bool success =false;
							if(suffix == "pcd")
							{
								success = pcl::io::savePCDFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
							}
							else if(suffix == "ply")
							{
								success = pcl::io::savePLYFile(pathFile.toStdString(), *transformedCloud, binaryMode) == 0;
							}
							else
							{
								UFATAL("Extension not recognized! (%s)", suffix.toStdString().c_str());
							}
							if(success)
							{
								_progressDialog->appendText(tr("Saved scan %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
							else
							{
								_progressDialog->appendText(tr("Failed saving scan %1 (%2 points) to %3.").arg(iter->first).arg(iter->second->size()).arg(pathFile));
							}
						}
						else
						{
							_progressDialog->appendText(tr("Scan %1 is empty!").arg(iter->first));
						}
						_progressDialog->incrementStep();
						QApplication::processEvents();
					}
				}
			}
		}
	}
}

}
