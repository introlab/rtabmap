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

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_features.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/gui/ImageView.h>
#include <rtabmap/gui/KeypointItem.h>
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <fstream>
#include <string>
#include <QApplication>
#include <QDialog>
#include <QHBoxLayout>
#include <QMultiMap>
#include <QString>
#include <opencv2/core/core.hpp>

using namespace rtabmap;

void showUsage()
{
	printf("\n\nUsage:\n"
			"   rtabmap-matcher [Options] from.png to.png\n"
			"Examples:\n"
			"   rtabmap-matcher --Vis/CorNNType 5 --Vis/PnPReprojError 3 from.png to.png\n"
			"   rtabmap-matcher --Vis/CorNNDR 0.8 from.png to.png\n"
			"   rtabmap-matcher --Vis/FeatureType 11 --SuperPoint/ModelPath \"superpoint.pt\" --Vis/CorNNType 6 --PyMatcher/Path \"~/SuperGluePretrainedNetwork/rtabmap_superglue.py\" from.png to.png\n"
			"   rtabmap-matcher --Vis/FeatureType 1 --Vis/CorNNType 6 --PyMatcher/Path \"~/OANet/demo/rtabmap_oanet.py\" --PyMatcher/Model \"~/OANet/model/gl3d/sift-4000/model_best.pth\" from.png to.png\n"
			"   rtabmap-matcher --calibration calib.yaml --from_depth from_depth.png --to_depth to_depth.png from.png to.png\n"
			"   rtabmap-matcher --calibration calibFrom.yaml --calibration_to calibTo.yaml --from_depth from_depth.png --to_depth to_depth.png from.png to.png\n"
			"   rtabmap-matcher --calibration calib.yaml --Vis/FeatureType 2 --Vis/MaxFeatures 10000 --Vis/CorNNType 7 from.png to.png\n"
			"\n"
			"Note: Use \"Vis/\" parameters for feature stuff.\n"
			"Options:\n"
			"   --calibration \"calibration.yaml\" Calibration file. If not set, a\n"
			"                                        fake one is created from image's\n"
			"                                        size (which may not be optimal).\n"
			"                                        Required if from_depth option is set.\n"
			"                                        Assuming same calibration for both images\n"
			"                                        if --calibration_to is not set.\n"
			"   --calibration_to \"calibration.yaml\" Calibration file for \"to\" image. If not set,\n"
			"                                           the same calibration of --calibration option is\n"
			"                                           used for \"to\" image.\n"
			"   --from_depth \"from_depth.png\"    Depth or right image file of the first image.\n"
			"                                        If not set, 2D->2D estimation is done by \n"
			"                                        default. For 3D->2D estimation, from_depth\n"
			"                                        should be set.\n"
			"   --to_depth \"to_depth.png\"        Depth or right image file of the second image.\n"
			"                                        For 3D->3D estimation, from_depth and to_depth\n"
			"                                        should be both set.\n"
			"\n\n"
			"%s\n",
			Parameters::showUsage());
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 3)
	{
		showUsage();
	}

	ULogger::setLevel(ULogger::kWarning);
	ULogger::setType(ULogger::kTypeConsole);

	std::string fromDepthPath;
	std::string toDepthPath;
	std::string calibrationPath;
	std::string calibrationToPath;
	for(int i=1; i<argc-2; ++i)
	{
		if(strcmp(argv[i], "--from_depth") == 0)
		{
			++i;
			if(i<argc-2)
			{
				fromDepthPath = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(strcmp(argv[i], "--to_depth") == 0)
		{
			++i;
			if(i<argc-2)
			{
				toDepthPath = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(strcmp(argv[i], "--calibration") == 0)
		{
			++i;
			if(i<argc-2)
			{
				calibrationPath = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(strcmp(argv[i], "--calibration_to") == 0)
		{
			++i;
			if(i<argc-2)
			{
				calibrationToPath = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
	}

	printf("Options\n");
	printf("  --calibration = \"%s\"\n", calibrationPath.c_str());
	if(!calibrationToPath.empty())
	{
		printf("  --calibration_to = \"%s\"\n", calibrationToPath.c_str());
	}
	printf("  --from_depth  = \"%s\"\n", fromDepthPath.c_str());
	printf("  --to_depth    = \"%s\"\n", toDepthPath.c_str());

	ParametersMap parameters = Parameters::parseArguments(argc, argv);
	parameters.insert(ParametersPair(Parameters::kRegRepeatOnce(), "false"));

	cv::Mat imageFrom = cv::imread(argv[argc-2], cv::IMREAD_COLOR);
	cv::Mat imageTo = cv::imread(argv[argc-1], cv::IMREAD_COLOR);

	if(!imageFrom.empty() && !imageTo.empty())
	{
		//////////////////
		// Load data
		//////////////////

		cv::Mat fromDepth;
		cv::Mat toDepth;
		if(!calibrationPath.empty())
		{
			if(!fromDepthPath.empty())
			{
				fromDepth = cv::imread(fromDepthPath, cv::IMREAD_UNCHANGED);
				if(fromDepth.type() == CV_8UC3)
				{
					cv::cvtColor(fromDepth, fromDepth, cv::COLOR_BGR2GRAY);
				}
				else if(fromDepth.empty())
				{
					printf("Failed loading from_depth image: \"%s\"!", fromDepthPath.c_str());
				}
			}
			if(!toDepthPath.empty())
			{
				toDepth = cv::imread(toDepthPath, cv::IMREAD_UNCHANGED);
				if(toDepth.type() == CV_8UC3)
				{
					cv::cvtColor(toDepth, toDepth, cv::COLOR_BGR2GRAY);
				}
				else if(toDepth.empty())
				{
					printf("Failed loading to_depth image: \"%s\"!", toDepthPath.c_str());
				}
			}
			UASSERT(toDepth.empty() || (!fromDepth.empty() && fromDepth.type() == toDepth.type()));
		}
		else if(!fromDepthPath.empty() || !fromDepthPath.empty())
		{
			printf("A calibration file should be provided if depth images are used!\n");
			showUsage();
		}

		CameraModel model;
		StereoCameraModel stereoModel;
		CameraModel modelTo;
		StereoCameraModel stereoModelTo;
		if(!fromDepth.empty())
		{
			if(fromDepth.type() != CV_8UC1)
			{
				if(!model.load(UDirectory::getDir(calibrationPath), uSplit(UFile::getName(calibrationPath), '.').front()))
				{
					printf("Failed to load calibration file \"%s\"!\n", calibrationPath.c_str());
					exit(-1);
				}
				if(calibrationToPath.empty())
				{
					modelTo = model;
				}
			}
			else // fromDepth.type() == CV_8UC1
			{
				if(!stereoModel.load(UDirectory::getDir(calibrationPath), uSplit(UFile::getName(calibrationPath), '.').front()))
				{
					printf("Failed to load calibration file \"%s\"!\n", calibrationPath.c_str());
					exit(-1);
				}
				if(calibrationToPath.empty())
				{
					stereoModelTo = stereoModel;
				}
			}
			if(!calibrationToPath.empty())
			{
				if(toDepth.empty() || toDepth.type() != CV_8UC1)
				{
					if(!modelTo.load(UDirectory::getDir(calibrationToPath), uSplit(UFile::getName(calibrationToPath), '.').front()))
					{
						printf("Failed to load calibration file \"%s\"!\n", calibrationToPath.c_str());
						exit(-1);
					}
				}
				else // toDepth.type() == CV_8UC1
				{
					if(!stereoModelTo.load(UDirectory::getDir(calibrationToPath), uSplit(UFile::getName(calibrationToPath), '.').front()))
					{
						printf("Failed to load calibration file \"%s\"!\n", calibrationToPath.c_str());
						exit(-1);
					}
				}
			}
		}
		else if(!calibrationPath.empty())
		{
			if(!model.load(UDirectory::getDir(calibrationPath), uSplit(UFile::getName(calibrationPath), '.').front()))
			{
				printf("Failed to load calibration file \"%s\"!\n", calibrationPath.c_str());
				exit(-1);
			}
			if(!calibrationToPath.empty())
			{
				if(!modelTo.load(UDirectory::getDir(calibrationToPath), uSplit(UFile::getName(calibrationToPath), '.').front()))
				{
					printf("Failed to load calibration file \"%s\"!\n", calibrationToPath.c_str());
					exit(-1);
				}
			}
			else
			{
				modelTo = model;
			}
		}
		else
		{
			printf("Using fake calibration model \"from\" (image size=%dx%d): fx=%d fy=%d cx=%d cy=%d\n",
					imageFrom.cols, imageFrom.rows, imageFrom.cols/2, imageFrom.cols/2, imageFrom.cols/2, imageFrom.rows/2);
			model = CameraModel(imageFrom.cols/2, imageFrom.cols/2, imageFrom.cols/2, imageFrom.rows/2); // Fake model
			model.setImageSize(imageFrom.size());
			printf("Using fake calibration model \"to\" (image size=%dx%d): fx=%d fy=%d cx=%d cy=%d\n",
					imageTo.cols, imageTo.rows, imageTo.cols/2, imageTo.cols/2, imageTo.cols/2, imageTo.rows/2);
			modelTo = CameraModel(imageTo.cols/2, imageTo.cols/2, imageTo.cols/2, imageTo.rows/2); // Fake model
			modelTo.setImageSize(imageTo.size());
		}

		Signature dataFrom;
		Signature dataTo;
		if(model.isValidForProjection())
		{
			printf("Mono calibration model detected.\n");
			dataFrom = SensorData(imageFrom, fromDepth, model, 1);
			dataTo = SensorData(imageTo, toDepth, modelTo, 2);
		}
		else //stereo
		{
			printf("Stereo calibration model detected.\n");
			dataFrom = SensorData(imageFrom, fromDepth, stereoModel, 1);
			dataTo = SensorData(imageTo, toDepth, stereoModelTo, 2);
		}

		//////////////////
		// Registration
		//////////////////

		if(fromDepth.empty())
		{
			parameters.insert(ParametersPair(Parameters::kVisEstimationType(), "2")); // Set 2D->2D estimation for mono images
			parameters.insert(ParametersPair(Parameters::kVisEpipolarGeometryVar(), "1")); //Unknown scale
			printf("Calibration not set, setting %s=1 and %s=2 by default (2D->2D estimation)\n", Parameters::kVisEpipolarGeometryVar().c_str(), Parameters::kVisEstimationType().c_str());
		}
		RegistrationVis reg(parameters);
		RegistrationInfo info;


		// Do it one time before to make sure everything is loaded to get realistic timing for matching only.
		reg.computeTransformationMod(dataFrom, dataTo, Transform(), &info);

		UTimer timer;
		Transform t = reg.computeTransformationMod(dataFrom, dataTo, Transform(), &info);
		double matchingTime = timer.ticks();
		printf("Time matching and motion estimation (excluding feature detection): %fs\n", matchingTime);

		//////////////////
		// Visualization
		//////////////////

		if(reg.getNNType()==6 &&
		   !dataFrom.getWordsDescriptors().empty() &&
		   dataFrom.getWordsDescriptors().type()!=CV_32F)
		{
			UWARN("PyMatcher is selected for matching but binary features "
				  "are not compatible. BruteForce with CrossCheck (%s=5) "
				  "has been used instead.", Parameters::kVisCorNNType().c_str());
		}

		QApplication app(argc, argv);
		QDialog dialog;
		float reprojError = Parameters::defaultVisPnPReprojError();
		std::string pyMatcherPath;
		Parameters::parse(parameters, Parameters::kVisPnPReprojError(), reprojError);
		Parameters::parse(parameters, Parameters::kPyMatcherPath(), pyMatcherPath);
		dialog.setWindowTitle(QString("Matches (%1/%2) %3 sec [%4=%5 (%6) %7=%8 (%9)%10 %11=%12 (%13) %14=%15]")
				.arg(info.inliers)
				.arg(info.matches)
				.arg(matchingTime)
				.arg(Parameters::kVisFeatureType().c_str())
				.arg(reg.getDetector()?reg.getDetector()->getType():-1)
				.arg(reg.getDetector()?Feature2D::typeName(reg.getDetector()->getType()).c_str():"?")
				.arg(Parameters::kVisCorNNType().c_str())
				.arg(reg.getNNType())
				.arg(reg.getNNType()<VWDictionary::kNNUndef?VWDictionary::nnStrategyName((VWDictionary::NNStrategy)reg.getNNType()).c_str():
						reg.getNNType()==5||(reg.getNNType()==6&&!dataFrom.getWordsDescriptors().empty()&& dataFrom.getWordsDescriptors().type()!=CV_32F)?"BFCrossCheck":
						reg.getNNType()==6?QString(uSplit(UFile::getName(pyMatcherPath), '.').front().c_str()).replace("rtabmap_", ""):
						reg.getNNType()==7?"GMS":"?")
				.arg(reg.getNNType()<5?QString(" %1=%2").arg(Parameters::kVisCorNNDR().c_str()).arg(reg.getNNDR()):"")
				.arg(Parameters::kVisEstimationType().c_str())
				.arg(reg.getEstimationType())
				.arg(reg.getEstimationType()==0?"3D->3D":reg.getEstimationType()==1?"3D->2D":reg.getEstimationType()==2?"2D->2D":"?")
				.arg(Parameters::kVisPnPReprojError().c_str())
				.arg(reprojError));

		CloudViewer * viewer = 0;
		if(!t.isNull())
		{
			viewer = new CloudViewer(&dialog);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFrom = util3d::cloudRGBFromSensorData(dataFrom.sensorData());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTo = util3d::cloudRGBFromSensorData(dataTo.sensorData());
			viewer->addCloud(uFormat("cloud_%d", dataFrom.id()), cloudFrom, Transform::getIdentity(), Qt::magenta);
			viewer->addCloud(uFormat("cloud_%d", dataTo.id()), cloudTo, t, Qt::cyan);
			viewer->addOrUpdateCoordinate(uFormat("frame_%d", dataTo.id()), t, 0.2);
			viewer->setGridShown(true);

			if(reg.getEstimationType() == 2)
			{
				// triangulate 3D words based on the transform computed
				std::map<int, int> wordsFrom = uMultimapToMapUnique(dataFrom.getWords());
				std::map<int, int> wordsTo = uMultimapToMapUnique(dataTo.getWords());
				std::map<int, cv::KeyPoint> kptsFrom;
				std::map<int, cv::KeyPoint> kptsTo;
				for(std::map<int, int>::iterator iter=wordsFrom.begin(); iter!=wordsFrom.end(); ++iter)
				{
					kptsFrom.insert(std::make_pair(iter->first, dataFrom.getWordsKpts()[iter->second]));
				}
				for(std::map<int, int>::iterator iter=wordsTo.begin(); iter!=wordsTo.end(); ++iter)
				{
					kptsTo.insert(std::make_pair(iter->first, dataTo.getWordsKpts()[iter->second]));
				}
				std::map<int, cv::Point3f> points3d = util3d::generateWords3DMono(
						kptsFrom,
						kptsTo,
						model.isValidForProjection()?model:stereoModel.left(),
						t);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWordsFrom(new pcl::PointCloud<pcl::PointXYZ>);
				cloudWordsFrom->resize(points3d.size());
				int i=0;
				for(std::multimap<int, cv::Point3f>::const_iterator iter=points3d.begin();
					iter!=points3d.end();
					++iter)
				{
					cloudWordsFrom->at(i++) = pcl::PointXYZ(iter->second.x, iter->second.y, iter->second.z);
				}
				if(cloudWordsFrom->size())
				{
					cloudWordsFrom = rtabmap::util3d::removeNaNFromPointCloud(cloudWordsFrom);
				}
				if(cloudWordsFrom->size())
				{
					viewer->addCloud("wordsFrom", cloudWordsFrom, Transform::getIdentity(), Qt::yellow);
					viewer->setCloudPointSize("wordsFrom", 5);
				}
			}
			else
			{
				if(!dataFrom.getWords3().empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWordsFrom(new pcl::PointCloud<pcl::PointXYZ>);
					cloudWordsFrom->resize(dataFrom.getWords3().size());
					int i=0;
					for(std::multimap<int, int>::const_iterator iter=dataFrom.getWords().begin();
						iter!=dataFrom.getWords().end();
						++iter)
					{
						const cv::Point3f & pt = dataFrom.getWords3()[iter->second];
						cloudWordsFrom->at(i++) = pcl::PointXYZ(pt.x, pt.y, pt.z);
					}
					if(cloudWordsFrom->size())
					{
						cloudWordsFrom = rtabmap::util3d::removeNaNFromPointCloud(cloudWordsFrom);
					}
					if(cloudWordsFrom->size())
					{
						viewer->addCloud("wordsFrom", cloudWordsFrom, Transform::getIdentity(), Qt::magenta);
						viewer->setCloudPointSize("wordsFrom", 5);
					}
				}
				if(!dataTo.getWords3().empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWordsTo(new pcl::PointCloud<pcl::PointXYZ>);
					cloudWordsTo->resize(dataTo.getWords3().size());
					int i=0;
					for(std::multimap<int, int>::const_iterator iter=dataTo.getWords().begin();
						iter!=dataTo.getWords().end();
						++iter)
					{
						const cv::Point3f & pt = dataTo.getWords3()[iter->second];
						cloudWordsTo->at(i++) = pcl::PointXYZ(pt.x, pt.y, pt.z);
					}
					if(cloudWordsTo->size())
					{
						cloudWordsTo = rtabmap::util3d::removeNaNFromPointCloud(cloudWordsTo);
					}
					if(cloudWordsTo->size())
					{
						viewer->addCloud("wordsTo", cloudWordsTo, t, Qt::cyan);
						viewer->setCloudPointSize("wordsTo", 5);
					}
				}
			}
		}

		QBoxLayout * mainLayout = new QHBoxLayout();
		mainLayout->setContentsMargins(0, 0, 0, 0);
		mainLayout->setSpacing(0);
		QBoxLayout * layout;
		bool vertical=true;
		if(imageFrom.cols > imageFrom.rows)
		{
			dialog.setMinimumWidth(640*(viewer?2:1));
			dialog.setMinimumHeight(640*imageFrom.rows/imageFrom.cols*2);
			layout = new QVBoxLayout();
		}
		else
		{
			dialog.setMinimumWidth((640*imageFrom.cols/imageFrom.rows*2)*(viewer?2:1));
			dialog.setMinimumHeight(640);
			layout = new QHBoxLayout();
			vertical = false;
		}

		ImageView * viewA = new ImageView(&dialog);
		ImageView * viewB = new ImageView(&dialog);

		layout->setSpacing(0);
		layout->addWidget(viewA, 1);
		layout->addWidget(viewB, 1);

		mainLayout->addLayout(layout, 1);
		if(viewer)
		{
			mainLayout->addWidget(viewer, 1);
		}

		dialog.setLayout(mainLayout);

		dialog.show();

		viewA->setImage(uCvMat2QImage(imageFrom));
		viewA->setAlpha(200);
		if(!fromDepth.empty())
		{
			viewA->setImageDepth(uCvMat2QImage(fromDepth, false, uCvQtDepthRedToBlue));
			viewA->setImageDepthShown(true);
		}
		viewB->setImage(uCvMat2QImage(imageTo));
		viewB->setAlpha(200);
		if(!toDepth.empty())
		{
			viewB->setImageDepth(uCvMat2QImage(toDepth, false, uCvQtDepthRedToBlue));
			viewB->setImageDepthShown(true);
		}
		std::multimap<int, cv::KeyPoint> keypointsFrom;
		std::multimap<int, cv::KeyPoint> keypointsTo;
		if(!dataFrom.getWordsKpts().empty())
		{
			for(std::map<int, int>::const_iterator iter=dataFrom.getWords().begin(); iter!=dataFrom.getWords().end(); ++iter)
			{
				keypointsFrom.insert(keypointsFrom.end(), std::make_pair(iter->first, dataFrom.getWordsKpts()[iter->second]));
			}
		}
		if(!dataTo.getWordsKpts().empty())
		{
			for(std::map<int, int>::const_iterator iter=dataTo.getWords().begin(); iter!=dataTo.getWords().end(); ++iter)
			{
				keypointsTo.insert(keypointsTo.end(), std::make_pair(iter->first, dataTo.getWordsKpts()[iter->second]));
			}
		}
		viewA->setFeatures(keypointsFrom);
		viewB->setFeatures(keypointsTo);
		std::set<int> inliersSet(info.inliersIDs.begin(), info.inliersIDs.end());

		const QMultiMap<int, KeypointItem*> & wordsA = viewA->getFeatures();
		const QMultiMap<int, KeypointItem*> & wordsB = viewB->getFeatures();
		if(wordsA.size() && wordsB.size())
		{
			QList<int> ids = wordsA.uniqueKeys();
			for(int i=0; i<ids.size(); ++i)
			{
				if(ids[i] > 0 && wordsA.count(ids[i]) == 1 && wordsB.count(ids[i]) == 1)
				{
					// Add lines
					// Draw lines between corresponding features...
					float scaleAX = viewA->viewScale();
					float scaleBX = viewB->viewScale();

					float scaleDiff = viewA->viewScale() / viewB->viewScale();
					float deltaAX = 0;
					float deltaAY = 0;

					if(vertical)
					{
						deltaAY = viewA->height()/scaleAX;
					}
					else
					{
						deltaAX = viewA->width()/scaleAX;
					}

					float deltaBX = 0;
					float deltaBY = 0;

					if(vertical)
					{
						deltaBY = viewB->height()/scaleBX;
					}
					else
					{
						deltaBX = viewA->width()/scaleBX;
					}

					const KeypointItem * kptA = wordsA.value(ids[i]);
					const KeypointItem * kptB = wordsB.value(ids[i]);

					QColor cA = viewA->getDefaultMatchingLineColor();
					QColor cB = viewB->getDefaultMatchingLineColor();
					if(inliersSet.find(ids[i])!=inliersSet.end())
					{
						cA = viewA->getDefaultMatchingFeatureColor();
						cB = viewB->getDefaultMatchingFeatureColor();
						viewA->setFeatureColor(ids[i], viewA->getDefaultMatchingFeatureColor());
						viewB->setFeatureColor(ids[i], viewB->getDefaultMatchingFeatureColor());
					}
					else
					{
						viewA->setFeatureColor(ids[i], viewA->getDefaultMatchingLineColor());
						viewB->setFeatureColor(ids[i], viewB->getDefaultMatchingLineColor());
					}

					viewA->addLine(
							kptA->rect().x()+kptA->rect().width()/2,
							kptA->rect().y()+kptA->rect().height()/2,
							kptB->rect().x()/scaleDiff+kptB->rect().width()/scaleDiff/2+deltaAX,
							kptB->rect().y()/scaleDiff+kptB->rect().height()/scaleDiff/2+deltaAY,
							cA);

					viewB->addLine(
							kptA->rect().x()*scaleDiff+kptA->rect().width()*scaleDiff/2-deltaBX,
							kptA->rect().y()*scaleDiff+kptA->rect().height()*scaleDiff/2-deltaBY,
							kptB->rect().x()+kptB->rect().width()/2,
							kptB->rect().y()+kptB->rect().height()/2,
							cB);
				}
			}
			viewA->update();
			viewB->update();
		}

		printf("Transform: %s\n", t.prettyPrint().c_str());
		printf("Features: from=%d to=%d\n", (int)dataFrom.getWords().size(), (int)dataTo.getWords().size());
		printf("Matches: %d\n", info.matches);
		printf("Inliers: %d (%s=%d)\n", info.inliers, Parameters::kVisMinInliers().c_str(), reg.getMinInliers());
		app.exec();
		delete viewer;
	}
	else
	{
		printf("Failed loading images %s and %s\n!", argv[argc-2], argv[argc-1]);
	}


	return 0;
}

