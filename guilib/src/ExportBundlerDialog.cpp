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

#include "rtabmap/gui/ExportBundlerDialog.h"
#include "ui_exportBundlerDialog.h"
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/util3d_transforms.h>
#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>
#include <QTextStream>

namespace rtabmap {

ExportBundlerDialog::ExportBundlerDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportBundlerDialog();
	_ui->setupUi(this);

	connect(_ui->toolButton_path, SIGNAL(clicked()), this, SLOT(getPath()));

	restoreDefaults();
	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	connect(_ui->doubleSpinBox_laplacianVariance, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_linearSpeed, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_angularSpeed, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->groupBox_export_points, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->sba_iterations, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_sbaType, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_sbaType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateVisibility()));
	connect(_ui->sba_rematchFeatures, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA) && !Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		_ui->groupBox_export_points->setEnabled(false);
		_ui->groupBox_export_points->setChecked(false);
	}
	else if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		_ui->comboBox_sbaType->setItemData(1, 0, Qt::UserRole - 1);
		_ui->comboBox_sbaType->setCurrentIndex(0);
	}
	else if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		_ui->comboBox_sbaType->setItemData(0, 0, Qt::UserRole - 1);
		_ui->comboBox_sbaType->setCurrentIndex(1);
	}

	_ui->lineEdit_path->setText(QDir::currentPath());

	updateVisibility();
}

ExportBundlerDialog::~ExportBundlerDialog()
{
	delete _ui;
}

void ExportBundlerDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("maxLinearSpeed", _ui->doubleSpinBox_linearSpeed->value());
	settings.setValue("maxAngularSpeed", _ui->doubleSpinBox_angularSpeed->value());
	settings.setValue("laplacianThr", _ui->doubleSpinBox_laplacianVariance->value());
	settings.setValue("exportPoints", _ui->groupBox_export_points->isChecked());
	settings.setValue("sba_iterations", _ui->sba_iterations->value());
	settings.setValue("sba_type", _ui->comboBox_sbaType->currentIndex());
	settings.setValue("sba_variance", _ui->sba_variance->value());
	settings.setValue("sba_rematch_features", _ui->sba_rematchFeatures->isChecked());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportBundlerDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	_ui->doubleSpinBox_linearSpeed->setValue(settings.value("maxLinearSpeed", _ui->doubleSpinBox_linearSpeed->value()).toDouble());
	_ui->doubleSpinBox_angularSpeed->setValue(settings.value("maxAngularSpeed", _ui->doubleSpinBox_angularSpeed->value()).toDouble());
	_ui->doubleSpinBox_laplacianVariance->setValue(settings.value("laplacianThr", _ui->doubleSpinBox_laplacianVariance->value()).toDouble());
	_ui->groupBox_export_points->setChecked(settings.value("exportPoints", _ui->groupBox_export_points->isChecked()).toBool());
	_ui->sba_iterations->setValue(settings.value("sba_iterations", _ui->sba_iterations->value()).toInt());
	_ui->comboBox_sbaType->setCurrentIndex((Optimizer::Type)settings.value("sba_type", _ui->comboBox_sbaType->currentIndex()).toInt());
	_ui->sba_variance->setValue(settings.value("sba_variance", _ui->sba_variance->value()).toDouble());
	_ui->sba_rematchFeatures->setChecked(settings.value("sba_rematch_features", _ui->sba_rematchFeatures->isChecked()).toBool());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportBundlerDialog::setWorkingDirectory(const QString & path)
{
	_ui->lineEdit_path->setText((path.isEmpty()?QDir::currentPath():path) + "/bundler");
}

void ExportBundlerDialog::restoreDefaults()
{
	_ui->doubleSpinBox_linearSpeed->setValue(0);
	_ui->doubleSpinBox_angularSpeed->setValue(0);
	_ui->doubleSpinBox_laplacianVariance->setValue(0);
	_ui->groupBox_export_points->setChecked(false);
	_ui->sba_iterations->setValue(20);
	if(Optimizer::isAvailable(Optimizer::kTypeG2O) || !Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		_ui->comboBox_sbaType->setCurrentIndex(0);
	}
	else
	{
		_ui->comboBox_sbaType->setCurrentIndex(1);
	}

	_ui->sba_variance->setValue(1.0);
	_ui->sba_rematchFeatures->setChecked(true);
}

void ExportBundlerDialog::updateVisibility()
{
	_ui->sba_variance->setVisible(_ui->comboBox_sbaType->currentIndex() == 0);
	_ui->label_variance->setVisible(_ui->comboBox_sbaType->currentIndex() == 0);
}

void ExportBundlerDialog::getPath()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("Exporting cameras in Bundler format..."), _ui->lineEdit_path->text());
	if(!path.isEmpty())
	{
		_ui->lineEdit_path->setText(path);
	}
}

void ExportBundlerDialog::exportBundler(
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const QMap<int, Signature> & signatures,
		const ParametersMap & parameters)
{
	if(this->exec() != QDialog::Accepted)
	{
		return;
	}
	QString path = _ui->lineEdit_path->text();
	if(!path.isEmpty())
	{
		if(!QDir(path).mkpath("."))
		{
			QMessageBox::warning(this, tr("Exporting cameras..."), tr("Failed creating directory %1.").arg(path));
			return;
		}

		std::map<int, cv::Point3f> points3DMap;
		std::map<int, std::map<int, FeatureBA> > wordReferences;
		std::map<int, Transform> newPoses = poses;
		if(_ui->groupBox_export_points->isEnabled() && _ui->groupBox_export_points->isChecked())
		{
			std::map<int, Transform> posesOut;
			std::multimap<int, Link> linksOut;
			Optimizer::Type sbaType = _ui->comboBox_sbaType->currentIndex()==0?Optimizer::kTypeG2O:Optimizer::kTypeCVSBA;
			UASSERT(Optimizer::isAvailable(sbaType));
			ParametersMap parametersSBA = parameters;
			uInsert(parametersSBA, std::make_pair(Parameters::kOptimizerIterations(), uNumber2Str(_ui->sba_iterations->value())));
			uInsert(parametersSBA, std::make_pair(Parameters::kg2oPixelVariance(), uNumber2Str(_ui->sba_variance->value())));
			Optimizer * sba = Optimizer::create(sbaType, parametersSBA);
			sba->getConnectedGraph(poses.begin()->first, poses, links, posesOut, linksOut);
			newPoses = sba->optimizeBA(
					posesOut.begin()->first,
					posesOut,
					linksOut,
					signatures.toStdMap(),
					points3DMap,
					wordReferences,
					_ui->sba_rematchFeatures->isChecked());
			delete sba;

			if(newPoses.empty())
			{
				QMessageBox::warning(this, tr("Exporting cameras..."), tr("SBA optimization failed! Cannot export with 3D points.").arg(path));
				return;
			}
		}

		// export cameras and images
		QFile fileOut(path+QDir::separator()+"cameras.out");
		QFile fileList(path+QDir::separator()+"list.txt");
		QFile fileListKeys(path+QDir::separator()+"list_keys.txt");
		QDir(path).mkdir("images");
		if(wordReferences.size())
		{
			QDir(path).mkdir("keys");
		}
		if(fileOut.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			if(fileList.open(QIODevice::WriteOnly | QIODevice::Text))
			{
				std::map<int, Transform> cameras;
				std::map<int, int> cameraIndexes;
				int camIndex = 0;
				std::map<int, QColor> colors;
				for(std::map<int, Transform>::const_iterator iter=newPoses.begin(); iter!=newPoses.end(); ++iter)
				{
					QMap<int, Signature>::const_iterator ster = signatures.find(iter->first);
					if(ster!= signatures.end())
					{
						cv::Mat image = ster.value().sensorData().imageRaw();
						if(image.empty())
						{
							ster.value().sensorData().uncompressDataConst(&image, 0, 0, 0);
						}

						double maxLinearVel = _ui->doubleSpinBox_linearSpeed->value();
						double maxAngularVel = _ui->doubleSpinBox_angularSpeed->value();
						double laplacianThr = _ui->doubleSpinBox_laplacianVariance->value();
						bool blurryImage = false;
						const std::vector<float> & velocity = ster.value().getVelocity();
						if(maxLinearVel>0.0 || maxAngularVel>0.0)
						{
							if(velocity.size() == 6)
							{
								float transVel = uMax3(fabs(velocity[0]), fabs(velocity[1]), fabs(velocity[2]));
								float rotVel = uMax3(fabs(velocity[3]), fabs(velocity[4]), fabs(velocity[5]));
								if(maxLinearVel>0.0 && transVel > maxLinearVel)
								{
									UWARN("Fast motion detected for camera %d (speed=%f m/s > thr=%f m/s), camera is ignored for texturing.", iter->first, transVel, maxLinearVel);
									blurryImage = true;
								}
								else if(maxAngularVel>0.0 && rotVel > maxAngularVel)
								{
									UWARN("Fast motion detected for camera %d (speed=%f rad/s > thr=%f rad/s), camera is ignored for texturing.", iter->first, rotVel, maxAngularVel);
									blurryImage = true;
								}
							}
							else
							{
								UWARN("Camera motion filtering is set, but velocity of camera %d is not available.", iter->first);
							}
						}

						if(!blurryImage && !image.empty() && laplacianThr>0.0)
						{
							cv::Mat imgLaplacian;
							cv::Laplacian(image, imgLaplacian, CV_16S);
							cv::Mat m, s;
							cv::meanStdDev(imgLaplacian, m, s);
							double stddev_pxl = s.at<double>(0);
							double var = stddev_pxl*stddev_pxl;
							if(var < laplacianThr)
							{
								blurryImage = true;
								UWARN("Camera's image %d is detected as blurry (var=%f < thr=%f), camera is ignored for texturing.", iter->first, var, laplacianThr);
							}
						}
						if(!blurryImage)
						{
							cameras.insert(*iter);
							cameraIndexes.insert(std::make_pair(iter->first, camIndex++));
							QString p = QString("images")+QDir::separator()+tr("%1.jpg").arg(iter->first);
							p = path+QDir::separator()+p;
							if(cv::imwrite(p.toStdString(), image))
							{
								UINFO("saved image %s", p.toStdString().c_str());
							}
							else
							{
								UERROR("Failed to save image %s", p.toStdString().c_str());
							}

							//
							// Descriptors
							//
							// The file format starts with 2 integers giving the total number of
							// keypoints and the length of the descriptor vector for each keypoint
							// (128). Then the location of each keypoint in the image is specified by
							// 4 floating point numbers giving subpixel row and column location,
							// scale, and orientation (in radians from -PI to PI).  Obviously, these
							// numbers are not invariant to viewpoint, but can be used in later
							// stages of processing to check for geometric consistency among matches.
							// Finally, the invariant descriptor vector for the keypoint is given as
							// a list of 128 integers in range [0,255].  Keypoints from a new image
							// can be matched to those from previous images by simply looking for the
							// descriptor vector with closest Euclidean distance among all vectors
							// from previous images.
							//
							if(wordReferences.size())
							{
								std::list<FeatureBA> descriptors;
								for(std::map<int, std::map<int, FeatureBA> >::iterator jter=wordReferences.begin(); jter!=wordReferences.end(); ++jter)
								{
									for(std::map<int, FeatureBA>::iterator kter=jter->second.begin(); kter!=jter->second.end(); ++kter)
									{
										if(kter->first == iter->first)
										{
											if(!kter->second.descriptor.empty())
											{
												descriptors.push_back(kter->second);
											}

											if(colors.find(jter->first) == colors.end())
											{
												if(!image.empty() &&
												   kter->second.kpt.pt.x >= 0.0f && (int)kter->second.kpt.pt.x < image.cols &&
												   kter->second.kpt.pt.y >= 0.0f && (int)kter->second.kpt.pt.y < image.rows)
												{
													UASSERT(image.type() == CV_8UC3 || image.type() == CV_8UC1);
													QColor c;
													if(image.channels() == 3)
													{
														cv::Vec3b & pixel = image.at<cv::Vec3b>((int)kter->second.kpt.pt.y, (int)kter->second.kpt.pt.x);
														c.setRgb(pixel[2], pixel[1], pixel[0]);
													}
													else // grayscale
													{
														unsigned char & pixel = image.at<unsigned char>((int)kter->second.kpt.pt.y, (int)kter->second.kpt.pt.x);
														c.setRgb(pixel, pixel, pixel);
													}
													colors.insert(std::make_pair(jter->first, c));
												}
											}
										}
									}
								}

								QString p = QString("keys")+QDir::separator()+tr("%1.key").arg(iter->first);
								p = path+QDir::separator()+p;
								QFile fileKey(p);
								if(fileKey.open(QIODevice::WriteOnly | QIODevice::Text))
								{
									if(descriptors.size())
									{
										QTextStream key(&fileKey);
										key << descriptors.size() << " " << descriptors.front().descriptor.cols << "\n";
										for(std::list<FeatureBA>::iterator dter=descriptors.begin(); dter!=descriptors.end(); ++dter)
										{
											// unpack octave value to get the scale set by SIFT (https://github.com/opencv/opencv/issues/4554)
											int octave = dter->kpt.octave & 255;
											octave = octave < 128 ? octave : (-128 | octave);
											float scale = octave >= 0 ? 1.f/(1 << octave) : (float)(1 << -octave);

											key << dter->kpt.pt.x << " " << dter->kpt.pt.y << " " << scale << " " << dter->kpt.angle << "\n";
											for(int i=0; i<dter->descriptor.cols; ++i)
											{
												if(dter->descriptor.type() == CV_8U)
												{
													key << " " << (int)dter->descriptor.at<unsigned char>(i);
												}
												else // assume CV_32F
												{
													key << " " << (int)dter->descriptor.at<float>(i);
												}
												if((i+1)%20 == 0 && i+1 < dter->descriptor.cols)
												{
													key << "\n";
												}
											}
											key << "\n";
										}
									}
									else
									{
										UWARN("No descriptors saved for frame %d in file %s. "
												"Descriptors may not have been saved in the nodes. "
												"Verify that parameter %s was true during mapping.",
												iter->first, p.toStdString().c_str(),
												Parameters::kMemRawDescriptorsKept().c_str());
									}
									fileKey.close();
								}
							}
						}
					}
					else
					{
						UWARN("Could not find node data for pose %d", iter->first);
					}
				}

				static const Transform opengl_world_T_rtabmap_world(
						 0.0f, -1.0f, 0.0f, 0.0f,
						 0.0f,  0.0f, 1.0f, 0.0f,
						-1.0f,  0.0f, 0.0f, 0.0f);

				static const Transform optical_rotation_inv(
						 0.0f, -1.0f,  0.0f, 0.0f,
						 0.0f,  0.0f, -1.0f, 0.0f,
						 1.0f,  0.0f,  0.0f, 0.0f);

				QTextStream out(&fileOut);
				QTextStream list(&fileList);
				out << "# Bundle file v0.3\n";
				out << cameras.size() << " " << points3DMap.size() << "\n";

				//
				//  Each camera entry <cameraI> contains the estimated camera intrinsics and extrinsics, and has the form:
				//
				//	<f> <k1> <k2>   [the focal length, followed by two radial distortion coeffs]
				//	<R>             [a 3x3 matrix representing the camera rotation]
				//	<t>             [a 3-vector describing the camera translation]
				//
				//  The cameras are specified in the order they appear in the list of images.
				//
				for(std::map<int, Transform>::iterator iter=cameras.begin(); iter!=cameras.end(); ++iter)
				{
					QString p = QString("images")+QDir::separator()+tr("%1.jpg").arg(iter->first);
					list << p << "\n";

					Transform localTransform;
					QMap<int, Signature>::const_iterator ster = signatures.find(iter->first);
					UASSERT(ster!=signatures.end());
					if(ster.value().sensorData().cameraModels().size())
					{
						out << ster.value().sensorData().cameraModels().at(0).fx() << " 0 0\n";
						localTransform = ster.value().sensorData().cameraModels().at(0).localTransform();
					}
					else if(ster.value().sensorData().stereoCameraModels().size())
					{
						out << ster.value().sensorData().stereoCameraModels()[0].left().fx() << " 0 0\n";
						localTransform = ster.value().sensorData().stereoCameraModels()[0].left().localTransform();
					}

					Transform pose = iter->second;
					if(!localTransform.isNull())
					{
						pose*=localTransform*optical_rotation_inv;
					}
					Transform poseGL = opengl_world_T_rtabmap_world*pose.inverse();

					out << poseGL.r11() << " " << poseGL.r12() << " " << poseGL.r13() << "\n";
					out << poseGL.r21() << " " << poseGL.r22() << " " << poseGL.r23() << "\n";
					out << poseGL.r31() << " " << poseGL.r32() << " " << poseGL.r33() << "\n";
					out << poseGL.x()   << " " << poseGL.y()   << " " << poseGL.z()   << "\n";
				}
				if(wordReferences.size())
				{
					if(fileListKeys.open(QIODevice::WriteOnly | QIODevice::Text))
					{
						QTextStream listKeys(&fileListKeys);
						for(std::map<int, Transform>::iterator iter=cameras.begin(); iter!=cameras.end(); ++iter)
						{
							QString p = QString("keys")+QDir::separator()+tr("%1.key").arg(iter->first);
							listKeys << p << "\n";
						}
						fileListKeys.close();
					}
				}

				//
				//  Each point entry has the form:
				//
				//		<position>      [a 3-vector describing the 3D position of the point]
				//		<color>         [a 3-vector describing the RGB color of the point]
				//		<view list>     [a list of views the point is visible in]
				//
				//	The view list begins with the length of the list (i.e., the number of cameras
				//	the point is visible in). The list is then given as a list of quadruplets
				//	<camera> <key> <x> <y>, where <camera> is a camera index, <key> the index
				//	of the SIFT keypoint where the point was detected in that camera, and <x>
				//	and <y> are the detected positions of that keypoint. Both indices are
				//	0-based (e.g., if camera 0 appears in the list, this corresponds to the
				//	first camera in the scene file and the first image in "list.txt"). The
				//	pixel positions are floating point numbers in a coordinate system where
				//	the origin is the center of the image, the x-axis increases to the right,
				//	and the y-axis increases towards the top of the image. Thus, (-w/2, -h/2)
				//	is the lower-left corner of the image, and (w/2, h/2) is the top-right
				//	corner (where w and h are the width and height of the image).
				//
				std::map<int, int> descriptorIndexes;
				for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
				{
					std::map<int, std::map<int, FeatureBA> >::iterator jter = wordReferences.find(iter->first);
					out << iter->second.x << " " << iter->second.y << " " << iter->second.z << "\n";
					UASSERT(colors.find(iter->first) != colors.end());
					QColor & c = colors.at(iter->first);
					out << c.red() << " " << c.green() << " " << c.blue() << "\n";
					out << jter->second.size();
					for(std::map<int, FeatureBA>::iterator kter = jter->second.begin(); kter!=jter->second.end(); ++kter)
					{
						// <camera> <key> <x> <y>
						int camId = kter->first;
						UASSERT(signatures.contains(camId));
						UASSERT(cameraIndexes.find(camId) != cameraIndexes.end());
						const Signature & s = signatures[camId];
						cv::Point2f pt;
						if(signatures[camId].sensorData().cameraModels().size())
						{
							pt.x = kter->second.kpt.pt.x - s.sensorData().cameraModels().at(0).cx();
							pt.y = kter->second.kpt.pt.y - s.sensorData().cameraModels().at(0).cy();
						}
						else if(signatures[camId].sensorData().stereoCameraModels().size())
						{
							pt.x = kter->second.kpt.pt.x - s.sensorData().stereoCameraModels()[0].left().cx();
							pt.y = kter->second.kpt.pt.y - s.sensorData().stereoCameraModels()[0].left().cy();
						}
						descriptorIndexes.insert(std::make_pair(camId, 0));
						out << " " << cameraIndexes.at(camId) << " " << descriptorIndexes.at(camId)++ << " " << pt.x << " " << -pt.y;
					}
					out << "\n";
				}

				fileList.close();
				fileOut.close();

				QMessageBox::information(this,
						tr("Exporting cameras in Bundler format..."),
						tr("%1 cameras/images and %2 points exported to directory \"%3\".%4")
						.arg(newPoses.size())
						.arg(points3DMap.size())
						.arg(path)
						.arg(newPoses.size()>cameras.size()?tr(" %1/%2 cameras ignored for too fast motion and/or blur level.").arg(newPoses.size()-cameras.size()).arg(newPoses.size()):""));
			}
			else
			{
				fileOut.close();
				QMessageBox::warning(this, tr("Exporting cameras..."), tr("Failed opening file %1 for writing.").arg(path+QDir::separator()+"list.txt"));
			}
		}
		else
		{
			QMessageBox::warning(this, tr("Exporting cameras..."), tr("Failed opening file %1 for writing.").arg(path+QDir::separator()+"cameras.out"));
		}
	}
}

}
