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

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/gui/UCv2Qt.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"
#include <QtGui/QApplication>
#include <QtGui/QGraphicsLineItem>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtCore/QTime>
#include <QtGui/QGraphicsEffect>


using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-epipolar_geometry image1.jpg image2.jpg\n");
	exit(1);
}

class RTABMAP_EXP OdometryMono : public Odometry
{
public:
	OdometryMono(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap()) :
		Odometry(parameters),
		flowWinSize_(Parameters::defaultOdomFlowWinSize()),
		flowIterations_(Parameters::defaultOdomFlowIterations()),
		flowEps_(Parameters::defaultOdomFlowEps()),
		flowMaxLevel_(Parameters::defaultOdomFlowMaxLevel()),
		subPixWinSize_(Parameters::defaultOdomSubPixWinSize()),
		subPixIterations_(Parameters::defaultOdomSubPixIterations()),
		subPixEps_(Parameters::defaultOdomSubPixEps()),
		refCorners3D_(new pcl::PointCloud<pcl::PointXYZ>)
	{
		Parameters::parse(parameters, Parameters::kOdomFlowWinSize(), flowWinSize_);
		Parameters::parse(parameters, Parameters::kOdomFlowIterations(), flowIterations_);
		Parameters::parse(parameters, Parameters::kOdomFlowEps(), flowEps_);
		Parameters::parse(parameters, Parameters::kOdomFlowMaxLevel(), flowMaxLevel_);
		Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize_);
		Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations_);
		Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps_);

		ParametersMap::const_iterator iter;
		Feature2D::Type detectorStrategy = (Feature2D::Type)Parameters::defaultOdomFeatureType();
		if((iter=parameters.find(Parameters::kOdomFeatureType())) != parameters.end())
		{
			detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
		}
		feature2D_ = Feature2D::create(detectorStrategy, parameters);

		ParametersMap customParameters;
		customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uValue(parameters, Parameters::kOdomBowNNType(), uNumber2Str(Parameters::defaultOdomBowNNType()))));
		customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uValue(parameters, Parameters::kOdomBowNNDR(), uNumber2Str(Parameters::defaultOdomBowNNDR()))));
		customParameters.insert(ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
		dictionary_ = new VWDictionary(customParameters);
	}

	virtual ~OdometryMono()
	{
		delete feature2D_;
		delete dictionary_;
	}

private:
	virtual Transform computeTransform(const SensorData & data, int * quality = 0, int * features = 0, int * localMapSize = 0)
	{
		UTimer timer;
		Transform output;

		int inliers = 0;
		int correspondences = 0;

		cv::Mat newFrame;
		// convert to grayscale
		if(data.image().channels() > 1)
		{
			cv::cvtColor(data.image(), newFrame, cv::COLOR_BGR2GRAY);
		}
		else
		{
			newFrame = data.image().clone();
		}

		UDEBUG("lastCorners_.size()=%d lastFrame_=%d", (int)refCorners_.size(), refFrame_.empty()?0:1);
		if(!refFrame_.empty() && refCorners_.size())
		{
			if(refCorners3D_->size())
			{
				//PnP
				UDEBUG("PnP");

				std::vector<cv::KeyPoint> newKpts;
				std::vector<cv::Point2f> newCorners;
				cv::Mat newDescriptors;
				if(data.keypoints().size())
				{
					cv::KeyPoint::convert(data.keypoints(), newCorners);
					newKpts = data.keypoints();
					newDescriptors = data.descriptors();
				}
				else
				{
					// generate kpts
					cv::Rect roi = Feature2D::computeRoi(newFrame, this->getRoiRatios());
					newKpts = feature2D_->generateKeypoints(newFrame, this->getMaxFeatures(), roi);
					Feature2D::limitKeypoints(newKpts, this->getMaxFeatures());

					if(newKpts.size())
					{
						//extract descriptors (before subpixel)
						newDescriptors = feature2D_->generateDescriptors(newFrame, newKpts);

						cv::KeyPoint::convert(newKpts, newCorners);

						if(subPixWinSize_ > 0 && subPixIterations_ > 0)
						{
							UDEBUG("cv::cornerSubPix() begin");
							cv::cornerSubPix(newFrame, newCorners,
								cv::Size( subPixWinSize_, subPixWinSize_ ),
								cv::Size( -1, -1 ),
								cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, subPixIterations_, subPixEps_ ) );
							UDEBUG("cv::cornerSubPix() end");

							for(unsigned int i=0; i<newCorners.size(); ++i)
							{
								newKpts[i].pt = newCorners[i];
							}
						}
					}
				}

				//matching using visual words dictionary
				std::vector<int> newWordIds = uListToVector(dictionary_->addNewWords(newDescriptors, 2));
				UDEBUG("");
				UASSERT((int)newKpts.size() == newDescriptors.rows);
				UASSERT(newKpts.size() == newWordIds.size());
				std::multimap<int, cv::KeyPoint> newWords;
				for(unsigned int i=0; i<newWordIds.size(); ++i)
				{
					newWords.insert(std::make_pair(newWordIds[i], newKpts[i]));
				}
				UDEBUG("newWords=%d", (int)newWords.size());
				std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
				if(EpipolarGeometry::findPairsUnique(refWords_, newWords, pairs) > this->getMinInliers())
				{
					UDEBUG("pairs = %d", (int)pairs.size());
					// now that we have correspondences, set data for PnP
					std::vector<cv::Point3f> objectPoints(pairs.size());
					std::vector<cv::Point2f> imagePoints(pairs.size());
					int i=0;
					std::vector<cv::KeyPoint> a,b;
					for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter = pairs.begin();
						iter!=pairs.end();
						++iter)
					{
						pcl::PointXYZ pt3 = refCorners3D_->at(iter->first-1); // id and index should match
						objectPoints[i] = cv::Point3f(pt3.x, pt3.y, pt3.z);
						imagePoints[i] = iter->second.second.pt;
						UDEBUG("ref (%f %f) new (%f %f) pt (%f %f %f)",
								iter->second.first.pt.x, iter->second.first.pt.y,
								iter->second.second.pt.x, iter->second.second.pt.y,
								pt3.x, pt3.y, pt3.z);
						a.push_back(iter->second.first);
						b.push_back(iter->second.second);
						++i;
					}

					UDEBUG("");
					cv::Mat K = (cv::Mat_<double>(3,3) <<
						data.fx(), 0, data.cx(),
						0, data.fyOrBaseline(), data.cy(),
						0, 0, 1);
					cv::Mat rvec, tvec;
					std::vector<int> inliers;
					cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false, 100, 8., 100, inliers);
					UDEBUG("");
					UDEBUG("inliers=%d/%d", (int)inliers.size(), (int)objectPoints.size());

					/*
					/// Debug draw matches
					std::vector<cv::DMatch> good_matches(inliers.size());
					for(i=0; i<(int)good_matches.size(); ++i)
					{
						good_matches[i].trainIdx = inliers[i];
						good_matches[i].queryIdx = inliers[i];
					}

					cv::Mat imgInliers;
					cv::drawMatches( refFrame_, a, newFrame, b,
								   good_matches, imgInliers, cv::Scalar::all(-1), cv::Scalar::all(-1),
								   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
					UWARN("saved test.png");
					cv::imwrite("test.png", imgInliers);
					cv::imwrite("testa.png", refFrame_);
					cv::imwrite("testb.png", newFrame);
					/// Debug draw matches
					 */

					if((int)inliers.size() > this->getMinInliers())
					{
						cv::Mat R(3,3,CV_64FC1);
						cv::Rodrigues(rvec, R);

						std::cout << "R: " << R << std::endl;
						std::cout << "T: " << tvec << std::endl;

						//R = R.t();  // rotation of inverse
						//tvec = -R * tvec; // translation of inverse

						//UDEBUG("camera movement:");
						//std::cout << "R: " << R << std::endl;
						//std::cout << "T: " << tvec << std::endl;

						output = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
										   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
										   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));
						output = data.localTransform() * output.inverse() * data.localTransform().inverse();
						output = this->getPose().inverse() * refCorners3DPose_ * output;
					}
					else
					{
						UWARN("PnP not enough inliers (%d < %d), rejecting the transform...", (int)inliers.size(), this->getMinInliers());
					}
				}
				else
				{
					UWARN("Not enough pairs found (%d)...", (int)pairs.size());
				}

				// remove new words from dictionary
				for(unsigned int i=0; i<newWordIds.size(); ++i)
				{
					dictionary_->removeAllWordRef(newWordIds[i], 2);
				}
				dictionary_->deleteUnusedWords();
			}
			else
			{
				//flow

				UDEBUG("flow");
				// Find features in the new left image
				std::vector<unsigned char> status;
				std::vector<float> err;
				std::vector<cv::Point2f> flowCorners = refCornersGuess_;
				std::vector<cv::Point2f> refCorners = refCorners_;
				std::vector<cv::KeyPoint> refKpts = refKpts_;
				cv::Mat refDescriptors = refDescriptors_;
				UDEBUG("cv::calcOpticalFlowPyrLK() begin (ref=%d guess=%d)", (int)refCorners.size(), (int)flowCorners.size());
				cv::calcOpticalFlowPyrLK(
						refFrame_,
						newFrame,
						refCorners,
						flowCorners,
						status,
						err,
						cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
						cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
				UDEBUG("cv::calcOpticalFlowPyrLK() end");

				UDEBUG("Filtering optical flow outliers...");
				std::vector<cv::Point2f> tmpFlowCorners(status.size());
				std::vector<cv::Point2f> tmpRefCorners(status.size());
				std::vector<cv::KeyPoint> tmpRefKpts(status.size());
				cv::Mat tmpRefDescriptors;
				int oi = 0;
				float flow = 0;
				float minFlow = 50;

				UASSERT(flowCorners.size() == status.size());
				UASSERT(refCorners.size() == status.size());
				UASSERT(refKpts.size() == status.size());
				UASSERT(refDescriptors.rows == (int)status.size());
				for(unsigned int i=0; i<status.size(); ++i)
				{
					if(status[i] && refCornersMask_[i])
					{
						float dx = refCorners[i].x - flowCorners[i].x;
						float dy = refCorners[i].y - flowCorners[i].y;
						float tmp = std::sqrt(dx*dx + dy*dy);
						flow+=tmp;

						tmpFlowCorners[oi] = flowCorners[i];
						tmpRefCorners[oi] = refCorners[i];
						tmpRefKpts[oi] = refKpts[i];
						tmpRefDescriptors.push_back(refDescriptors.row(i));
						++oi;

						UDEBUG("%d = ref(%f %f) flow(%f %f) = %f", i,
							refCorners[i].x, refCorners[i].y,
							flowCorners[i].x, flowCorners[i].y,
							tmp);
					}
					else
					{
						refCornersMask_[i] = 0;
					}
				}
				if(oi)
				{
					flow /=float(oi);
				}
				tmpFlowCorners.resize(oi);
				tmpRefCorners.resize(oi);
				tmpRefKpts.resize(oi);
				UDEBUG("Filtering optical flow outliers...done! (inliers=%d/%d)", oi, (int)status.size());

				if(flow > minFlow && oi > this->getMinInliers())
				{
					flowCorners = tmpFlowCorners;
					refCorners = tmpRefCorners;
					refKpts = tmpRefKpts;
					refDescriptors = tmpRefDescriptors;

					UDEBUG("flow=%f", flow);
					// compute fundamental matrix
					UDEBUG("Find fundamental matrix");
					status.clear();
					cv::Mat F = cv::findFundamentalMat(refCorners, flowCorners, status, cv::RANSAC, 3.0, 0.99);
					std::cout << "F=" << F << std::endl;

					if(!F.empty())
					{
						UDEBUG("Filtering fundamental matrix outliers...");
						tmpFlowCorners.resize(status.size());
						tmpRefCorners.resize(status.size());
						tmpRefKpts.resize(status.size());
						tmpRefDescriptors = cv::Mat();
						oi = 0;
						UASSERT(flowCorners.size() == status.size());
						UASSERT(refCorners.size() == status.size());
						UASSERT(refKpts.size() == status.size());
						UASSERT(refDescriptors.rows == (int)status.size());
						for(unsigned int i=0; i<status.size(); ++i)
						{
							if(status[i])
							{
								tmpFlowCorners[oi] = flowCorners[i];
								tmpRefCorners[oi] = refCorners[i];
								tmpRefKpts[oi] = refKpts[i];
								tmpRefDescriptors.push_back(refDescriptors.row(i));
								++oi;
							}
						}
						tmpFlowCorners.resize(oi);
						tmpRefCorners.resize(oi);
						tmpRefKpts.resize(oi);
						flowCorners = tmpFlowCorners;
						refCorners = tmpRefCorners;
						refKpts = tmpRefKpts;
						refDescriptors = tmpRefDescriptors;
						UDEBUG("Filtering fundamental matrix outliers...done! (inliers=%d/%d)", oi, (int)status.size());

						if(refCorners.size())
						{
							std::vector<cv::Point2f> lastCornersRefined;
							std::vector<cv::Point2f> newCornersRefined;
							//UDEBUG("Correcting matches...");
							cv::correctMatches(F, refCorners, flowCorners, lastCornersRefined, newCornersRefined);
							refCorners = lastCornersRefined;
							flowCorners = newCornersRefined;
							//UDEBUG("Correcting matches...done!");

							UDEBUG("Computing P...");
							cv::Mat K = (cv::Mat_<double>(3,3) <<
										data.fx(), 0, data.cx(),
										0, data.fyOrBaseline(), data.cy(),
										0, 0, 1);
							//std::cout << "K=" << K << std::endl;
							cv::Mat Kinv = K.inv();
							//std::cout << "Kinv=" << Kinv << std::endl;
							cv::Mat E = K.t()*F*K;
							std::cout << "E=" << E << std::endl;

							//normalize coordinates
							cv::Mat x(3, refCorners.size(), CV_64FC1);
							cv::Mat xp(3, refCorners.size(), CV_64FC1);
							for(unsigned int i=0; i<refCorners.size(); ++i)
							{
								x.at<double>(0, i) = refCorners[i].x;
								x.at<double>(1, i) = refCorners[i].y;
								x.at<double>(2, i) = 1;

								xp.at<double>(0, i) = flowCorners[i].x;
								xp.at<double>(1, i) = flowCorners[i].y;
								xp.at<double>(2, i) = 1;

								//UDEBUG("ptA= %f %f %f", ptA.at<double>(0, i), ptA.at<double>(1, i), ptA.at<double>(2, i));
							}

							cv::Mat x_norm = Kinv * x;
							cv::Mat xp_norm = Kinv * xp;
							x_norm = x_norm.rowRange(0,2);
							xp_norm = xp_norm.rowRange(0,2);
							x = x.rowRange(0,2);
							xp = xp.rowRange(0,2);

							cv::Mat P = EpipolarGeometry::findPFromE(E, x_norm, xp_norm);
							if(!P.empty())
							{
								cv::Mat P0 = cv::Mat::zeros(3, 4, CV_64FC1);
								P0.at<double>(0,0) = 1;
								P0.at<double>(1,1) = 1;
								P0.at<double>(2,2) = 1;

								UDEBUG("Computing P...done!");
								std::cout << "P=" << P << std::endl;

								//scale
								//P.col(3) /= 10.0;

								cv::Mat R, T;
								EpipolarGeometry::findRTFromP(P, R, T);
								//std::cout << "R=" << R << std::endl;
								//std::cout << "T=" << T << std::endl;

								UDEBUG("");
								//cv::Mat pts4D;
								std::vector<double> reprojErrors;
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
								EpipolarGeometry::triangulatePoints(x_norm, xp_norm, P0, P, cloud, reprojErrors);
								//cv::triangulatePoints(P0, P, x_norm, xp_norm, pts4D);

								tmpRefCorners.resize(cloud->size());
								tmpRefKpts.resize(cloud->size());
								tmpRefDescriptors = cv::Mat();
								refCorners3D_->resize(cloud->size());
								oi = 0;
								UASSERT(refCorners.size() == cloud->size());
								UASSERT(refKpts.size() == cloud->size());
								UASSERT(refDescriptors.rows == (int)cloud->size());
								for(unsigned int i=0; i<cloud->size(); ++i)
								{
									if(cloud->at(i).z>0)
									{
										refCorners3D_->at(oi) = cloud->at(i);
										tmpRefCorners[oi] = refCorners[i];
										tmpRefKpts[oi] = refKpts[i];
										tmpRefDescriptors.push_back(refDescriptors.row(i));
										++oi;
									}
								}
								refCorners3D_->resize(oi);
								tmpRefCorners.resize(oi);
								tmpRefKpts.resize(oi);
								refCorners = tmpRefCorners;
								refKpts = tmpRefKpts;
								refDescriptors = tmpRefDescriptors;
								UDEBUG("Filtering triangulation outliers...done! (inliers=%d/%d)", oi, (int)cloud->size());

								//refCorners3D_ = util3d::transformPointCloud<pcl::PointXYZ>(refCorners3D_, data.localTransform());

								refCorners3DPose_ = this->getPose();
								dictionary_->clear();
								refCorners_ = refCorners;
								refKpts_ = refKpts;
								refDescriptors_ = refDescriptors;
								std::vector<int> wordsId = uListToVector(dictionary_->addNewWords(refDescriptors_, 1));
								refWords_.clear();
								UASSERT(wordsId.size() == refCorners_.size());
								for(unsigned int i=0; i<wordsId.size(); ++i)
								{
									refWords_.insert(std::make_pair(wordsId[i], refKpts[i]));
								}
								dictionary_->update();

								output = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0)/*/T.at<double>(3)*/,
												   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1)/*/T.at<double>(3)*/,
												   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2)/*/T.at<double>(3)*/);
								output = data.localTransform() * output.inverse() * data.localTransform().inverse();
							}
							else
							{
								UFATAL("No valid camera matrix found!");
							}
						}
					}
				}
				else
				{
					UWARN("Flow not enough high! flow=%f ki=%d", flow, oi);
					refCornersGuess_ = flowCorners;
				}
			}
		}
		else
		{
			//return Identity
			output = Transform::getIdentity();

			std::vector<cv::KeyPoint> newKpts;
			std::vector<cv::Point2f> newCorners;
			cv::Mat newDescriptors;
			if(data.keypoints().size())
			{
				cv::KeyPoint::convert(data.keypoints(), newCorners);
				newKpts = data.keypoints();
				newDescriptors = data.descriptors();
			}
			else
			{
				// generate kpts
				cv::Rect roi = Feature2D::computeRoi(newFrame, this->getRoiRatios());
				newKpts = feature2D_->generateKeypoints(newFrame, this->getMaxFeatures(), roi);
				Feature2D::limitKeypoints(newKpts, this->getMaxFeatures());

				if(newKpts.size())
				{
					//extract descriptors (before subpixel)
					newDescriptors = feature2D_->generateDescriptors(newFrame, newKpts);

					cv::KeyPoint::convert(newKpts, newCorners);

					if(subPixWinSize_ > 0 && subPixIterations_ > 0)
					{
						UDEBUG("cv::cornerSubPix() begin");
						cv::cornerSubPix(newFrame, newCorners,
							cv::Size( subPixWinSize_, subPixWinSize_ ),
							cv::Size( -1, -1 ),
							cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, subPixIterations_, subPixEps_ ) );
						UDEBUG("cv::cornerSubPix() end");

						for(unsigned int i=0; i<newCorners.size(); ++i)
						{
							newKpts[i].pt = newCorners[i];
						}
					}
				}
			}

			if((int)newCorners.size() > this->getMinInliers())
			{
				refFrame_ = newFrame;
				refCorners_ = newCorners;
				refKpts_ = newKpts;
				refDescriptors_ = newDescriptors;
				refCornersGuess_ = newCorners;
				refCornersMask_.resize(newCorners.size(), 1);
				UASSERT(refCorners_.size() == refKpts_.size());
				UASSERT(refDescriptors_.rows == (int)refKpts_.size());
			}
			else
			{
				UWARN("Too low 2D corners (%d), ignoring new frame...",
						(int)newCorners.size());
			}
		}

		UINFO("Odom update time = %fs tf=[%s] inliers=%d/%d, transform accepted=%s",
				timer.elapsed(),
				output.prettyPrint().c_str(),
				inliers,
				correspondences,
				!output.isNull()?"true":"false");

		return output;

	}
private:
	//Parameters:
	int flowWinSize_;
	int flowIterations_;
	double flowEps_;
	int flowMaxLevel_;

	int subPixWinSize_;
	int subPixIterations_;
	double subPixEps_;

	Feature2D * feature2D_;
	VWDictionary * dictionary_;

	cv::Mat refFrame_;
	std::vector<cv::Point2f> refCorners_;
	std::vector<cv::Point2f> refCornersGuess_;
	std::vector<unsigned char> refCornersMask_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr refCorners3D_;
	Transform refCorners3DPose_;
	std::vector<cv::KeyPoint> refKpts_;
	cv::Mat refDescriptors_;
	std::multimap<int, cv::KeyPoint> refWords_;
};

class MainWidget : public QWidget
{
public:
	MainWidget(const cv::Mat & image1,
			   const cv::Mat & image2,
			   const std::multimap<int, cv::KeyPoint> & words1,
			   const std::multimap<int, cv::KeyPoint> & words2,
			   const std::vector<uchar> & status)
	{
		view1_ = new ImageView(this);
		this->setLayout(new QHBoxLayout());
		this->layout()->setSpacing(0);
		this->layout()->setContentsMargins(0,0,0,0);
		this->layout()->addWidget(view1_);
		view1_->setSceneRect(0,0,(float)image1.cols, (float)image1.rows);
		view1_->setLinesShown(true);
		view1_->setFeaturesShown(false);

		QGraphicsPixmapItem * item1 = view1_->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(image1)));
		QGraphicsPixmapItem * item2 = view1_->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(image2)));

		QGraphicsOpacityEffect * effect1 = new QGraphicsOpacityEffect();
		QGraphicsOpacityEffect * effect2 = new QGraphicsOpacityEffect();
		effect1->setOpacity(0.5);
		effect2->setOpacity(0.5);
		item1->setGraphicsEffect(effect1);
		item2->setGraphicsEffect(effect2);

		item1->setVisible(view1_->isImageShown());
		item2->setVisible(view1_->isImageShown());

		drawKeypoints(words1, words2, status);
	}
protected:
	virtual void showEvent(QShowEvent* event)
	{
		resizeEvent(0);
	}
	virtual void resizeEvent(QResizeEvent* event)
	{
		view1_->fitInView(view1_->sceneRect(), Qt::KeepAspectRatio);
		view1_->resetZoom();
	}
private:
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords, const std::vector<uchar> & status)
	{
		UTimer timer;

		timer.start();
		KeypointItem * item = 0;
		int alpha = 10*255/100;
		QList<QPair<cv::Point2f, cv::Point2f> > uniqueCorrespondences;
		QList<bool> inliers;
		int j=0;
		for(std::multimap<int, cv::KeyPoint>::const_iterator i = refWords.begin(); i != refWords.end(); ++i )
		{
			const cv::KeyPoint & r = (*i).second;
			int id = (*i).first;

			QString info = QString( "WordRef = %1\n"
									"Laplacian = %2\n"
									"Dir = %3\n"
									"Hessian = %4\n"
									"X = %5\n"
									"Y = %6\n"
									"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
			float radius = r.size*1.2/9.*2;
			if(uContains(loopWords, id))
			{
				// PINK = FOUND IN LOOP SIGNATURE
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 255, alpha));
				//To draw lines... get only unique correspondences
				if(uValues(refWords, id).size() == 1 && uValues(loopWords, id).size() == 1)
				{
					uniqueCorrespondences.push_back(QPair<cv::Point2f, cv::Point2f>(r.pt, uValues(loopWords, id).begin()->pt));
					inliers.push_back(status[j++]);
				}
			}
			else if(refWords.count(id) > 1)
			{
				// YELLOW = NEW and multiple times
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));
			}
			else
			{
				// GREEN = NEW
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(0, 255, 0, alpha));
			}
			item->setVisible(view1_->isFeaturesShown());
			view1_->scene()->addItem(item);
			item->setZValue(1);
		}
		ULOGGER_DEBUG("source time = %f s", timer.ticks());

		// Draw lines between corresponding features...
		UASSERT(uniqueCorrespondences.size() == inliers.size());
		QList<bool>::iterator jter = inliers.begin();
		for(QList<QPair<cv::Point2f, cv::Point2f> >::iterator iter = uniqueCorrespondences.begin();
			iter!=uniqueCorrespondences.end();
			++iter)
		{
			QGraphicsLineItem * item = view1_->scene()->addLine(
					iter->first.x,
					iter->first.y,
					iter->second.x,
					iter->second.y,
					*jter?QPen(Qt::cyan):QPen(Qt::red));
			item->setVisible(view1_->isLinesShown());
			item->setZValue(1);
			++jter;
		}
	}

private:
	ImageView * view1_;
};

std::multimap<int, cv::KeyPoint> aggregate(const std::list<int> & wordIds, const std::vector<cv::KeyPoint> & keypoints)
{
	std::multimap<int, cv::KeyPoint> words;
	std::vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
	for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
		++kpIter;
	}
	return words;
}



int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	cv::Mat image1;
    cv::Mat image2;
	if(argc == 3)
	{
		image1 = cv::imread(argv[1]);
		image2 = cv::imread(argv[2]);
	}
	else
	{
		showUsage();
	}

	QTime timer;
	timer.start();

	// Extract words
	timer.start();
   VWDictionary dictionary;
   ParametersMap param;
   param.insert(ParametersPair(Parameters::kSURFExtended(), "true"));
   param.insert(ParametersPair(Parameters::kSURFHessianThreshold(), "100"));
   SURF detector(param);
   std::vector<cv::KeyPoint> kpts1 = detector.generateKeypoints(image1);
   std::vector<cv::KeyPoint> kpts2 = detector.generateKeypoints(image2);
   cv::Mat descriptors1 = detector.generateDescriptors(image1, kpts1);
   cv::Mat descriptors2 = detector.generateDescriptors(image2, kpts2);
   UINFO("detect/extract features = %d ms", timer.elapsed());

   timer.start();
   std::list<int> wordIds1 = dictionary.addNewWords(descriptors1, 1);
   std::list<int> wordIds2 = dictionary.addNewWords(descriptors2, 2);
   UINFO("quantization to words = %d ms", timer.elapsed());

   std::multimap<int, cv::KeyPoint> words1 = aggregate(wordIds1, kpts1);
   std::multimap<int, cv::KeyPoint> words2 = aggregate(wordIds2, kpts2);

   // Find pairs
   timer.start();
   std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
   EpipolarGeometry::findPairsUnique(words1, words2, pairs);
   UINFO("find pairs = %d ms", timer.elapsed());

   // Find fundamental matrix
   timer.start();
   std::vector<uchar> status;
   cv::Mat fundamentalMatrix = EpipolarGeometry::findFFromWords(pairs, status);
   UINFO("inliers = %d/%d", uSum(status), pairs.size());
   UINFO("find F = %d ms", timer.elapsed());
   if(!fundamentalMatrix.empty())
   {
	   int i = 0;
	   int goodCount = 0;
	   for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
		{
			if(status[i])
			{
				// the output of the correspondences can be easily copied in MatLab
				if(goodCount==0)
				{
					printf("x=[%f %f %d]; xp=[%f %f %d];\n",
							iter->second.first.pt.x,
							iter->second.first.pt.y,
							iter->first,
							iter->second.second.pt.x,
							iter->second.second.pt.y,
							iter->first);
				}
				else
				{
					printf("x=[x;[%f %f %d]]; xp=[xp;[%f %f %d]];\n",
							iter->second.first.pt.x,
							iter->second.first.pt.y,
							iter->first,
							iter->second.second.pt.x,
							iter->second.second.pt.y,
							iter->first);
				}
				++goodCount;
			}
			++i;
		}

		// Show the fundamental matrix
		std::cout << "F=" << fundamentalMatrix << std::endl;

		// Intrinsic parameters K of the camera (guest... non-calibrated camera)
		cv::Mat k = cv::Mat::zeros(3,3,CV_64FC1);
		k.at<double>(0,0) = image1.cols; // focal x
		k.at<double>(1,1) = image1.rows; // focal y
		k.at<double>(2,2) = 1;
		k.at<double>(0,2) = image1.cols/2; // center x in pixels
		k.at<double>(1,2) = image1.rows/2; // center y in pixels

		// Use essential matrix E=K'*F*K

		cv::Mat e = k.t()*fundamentalMatrix*k;

		//remove K from points xe = inv(K)*x
		cv::Mat x1(2, goodCount, CV_64FC1);
		cv::Mat x2(2, goodCount, CV_64FC1);
		i=0;
		int j=0;
		cv::Mat invK = k.inv();
		for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
		{
			if(status[i])
			{
				cv::Mat tmp(3,1,CV_64FC1);
				tmp.at<double>(0,0) = iter->second.first.pt.x;
				tmp.at<double>(1,0) = iter->second.first.pt.y;
				tmp.at<double>(2,0) = 1;
				tmp = invK*tmp;
				x1.at<double>(0,j) = tmp.at<double>(0,0);
				x1.at<double>(1,j) = tmp.at<double>(1,0);
				tmp.at<double>(0,0) = iter->second.second.pt.x;
				tmp.at<double>(1,0) = iter->second.second.pt.y;
				tmp.at<double>(2,0) = 1;
				tmp = invK*tmp;
				x2.at<double>(0,j) = tmp.at<double>(0,0);
				x2.at<double>(1,j) = tmp.at<double>(1,0);
				UDEBUG("i=%d j=%d, x1=[%f,%f] x2=[%f,%f]", i, j, x1.at<double>(0,j), x1.at<double>(1,j), x2.at<double>(0,j), x2.at<double>(1,j));
				++j;
			}
			++i;
		}

		std::cout<<"K=" << k << std::endl;
		timer.start();
		//std::cout<<"e=" << e << std::endl;
		cv::Mat p = EpipolarGeometry::findPFromE(e, x1, x2);
		cv::Mat p0 = cv::Mat::zeros(3, 4, CV_64FC1);
		p0.at<double>(0,0) = 1;
		p0.at<double>(1,1) = 1;
		p0.at<double>(2,2) = 1;
		UINFO("find P from F = %d ms", timer.elapsed());

		std::cout<<"P=" << p << std::endl;

		//find 4D homogeneous points
		cv::Mat x4d;
		timer.start();
		cv::triangulatePoints(p0, p, x1, x2, x4d);
		UINFO("find X (triangulate) = %d ms", timer.elapsed());

		//Show 4D points
		for(int i=0; i<x4d.cols; ++i)
		{
			x4d.at<double>(0,i) = x4d.at<double>(0,i)/x4d.at<double>(3,i);
			x4d.at<double>(1,i) = x4d.at<double>(1,i)/x4d.at<double>(3,i);
			x4d.at<double>(2,i) = x4d.at<double>(2,i)/x4d.at<double>(3,i);
			x4d.at<double>(3,i) = x4d.at<double>(3,i)/x4d.at<double>(3,i);
			if(i==0)
			{
				printf("X=[%f;%f;%f;%f];\n",
					x4d.at<double>(0,i),
					x4d.at<double>(1,i),
					x4d.at<double>(2,i),
					x4d.at<double>(3,i));
			}
			else
			{
				printf("X=[X [%f;%f;%f;%f]];\n",
					x4d.at<double>(0,i),
					x4d.at<double>(1,i),
					x4d.at<double>(2,i),
					x4d.at<double>(3,i));
			}
		}

		//Show rotation/translation of the second camera
		cv::Mat r;
		cv::Mat t;
		EpipolarGeometry::findRTFromP(p, r, t);
		std::cout<< "R=" << r << std::endl;
		std::cout<< "t=" << t << std::endl;

		//GUI
		QApplication app(argc, argv);
		MainWidget mainWidget(image1, image2, words1, words2, status);
		mainWidget.show();
		app.exec();
	}
	else
	{
		UINFO("Fundamental matrix not found...");
	}

    return 0;
}
