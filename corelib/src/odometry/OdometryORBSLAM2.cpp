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

#include "rtabmap/core/odometry/OdometryORBSLAM2.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UDirectory.h"
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_ORB_SLAM2
#include <System.h>
#include <thread>

using namespace std;

namespace ORB_SLAM2 {
// Override original Tracking object to comment all rendering stuff
class Tracker: public Tracking
{
public:
	Tracker(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
	             KeyFrameDatabase* pKFDB, const std::string &strSettingPath, const int sensor, long unsigned int maxFeatureMapSize) :
	            	 Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor),
	            	 maxFeatureMapSize_(maxFeatureMapSize)
	{

	}
private:
	long unsigned int maxFeatureMapSize_;

protected:
	void Track()
	{
	    if(mState==NO_IMAGES_YET)
	    {
	        mState = NOT_INITIALIZED;
	    }

	    mLastProcessedState=mState;

	    // Get Map Mutex -> Map cannot be changed
	    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

	    if(mState==NOT_INITIALIZED)
	    {
	       // if(mSensor==System::STEREO || mSensor==System::RGBD)
	            StereoInitialization();
	        //else
	        //    MonocularInitialization();

	        //mpFrameDrawer->Update(this);

	        if(mState!=OK)
	            return;
	    }
	    else
	    {
	        // System is initialized. Track Frame.
	        bool bOK;

	        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
	        if(!mbOnlyTracking)
	        {
	            // Local Mapping is activated. This is the normal behaviour, unless
	            // you explicitly activate the "only tracking" mode.

	            if(mState==OK || mState==LOST)
	            {
	                // Local Mapping might have changed some MapPoints tracked in last frame
	                CheckReplacedInLastFrame();

	                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
	                {
	                    bOK = TrackReferenceKeyFrame();
	                }
	                else
	                {
	                    bOK = TrackWithMotionModel();
	                    if(!bOK)
	                        bOK = TrackReferenceKeyFrame();
	                }
	                if(bOK)
	                {
	                	mState=OK;
	                }
	            }
	            else
	            {
	                bOK = Relocalization();
	            }
	        }
	        else
	        {
	            // Localization Mode: Local Mapping is deactivated

	            if(mState==LOST)
	            {
	                bOK = Relocalization();
	            }
	            else
	            {
	                if(!mbVO)
	                {
	                    // In last frame we tracked enough MapPoints in the map

	                    if(!mVelocity.empty())
	                    {
	                        bOK = TrackWithMotionModel();
	                    }
	                    else
	                    {
	                        bOK = TrackReferenceKeyFrame();
	                    }
	                }
	                else
	                {
	                    // In last frame we tracked mainly "visual odometry" points.

	                    // We compute two camera poses, one from motion model and one doing relocalization.
	                    // If relocalization is sucessfull we choose that solution, otherwise we retain
	                    // the "visual odometry" solution.

	                    bool bOKMM = false;
	                    bool bOKReloc = false;
	                    std::vector<MapPoint*> vpMPsMM;
	                    std::vector<bool> vbOutMM;
	                    cv::Mat TcwMM;
	                    if(!mVelocity.empty())
	                    {
	                        bOKMM = TrackWithMotionModel();
	                        vpMPsMM = mCurrentFrame.mvpMapPoints;
	                        vbOutMM = mCurrentFrame.mvbOutlier;
	                        TcwMM = mCurrentFrame.mTcw.clone();
	                    }
	                    bOKReloc = Relocalization();

	                    if(bOKMM && !bOKReloc)
	                    {
	                        mCurrentFrame.SetPose(TcwMM);
	                        mCurrentFrame.mvpMapPoints = vpMPsMM;
	                        mCurrentFrame.mvbOutlier = vbOutMM;

	                        if(mbVO)
	                        {
	                            for(int i =0; i<mCurrentFrame.N; i++)
	                            {
	                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
	                                {
	                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
	                                }
	                            }
	                        }
	                    }
	                    else if(bOKReloc)
	                    {
	                        mbVO = false;
	                    }

	                    bOK = bOKReloc || bOKMM;
	                }
	            }
	        }

	        mCurrentFrame.mpReferenceKF = mpReferenceKF;

	        // If we have an initial estimation of the camera pose and matching. Track the local map.
	        if(!mbOnlyTracking)
	        {
	            if(bOK)
	                bOK = TrackLocalMap();
	        }
	        else
	        {
	            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
	            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
	            // the camera we will use the local map again.
	            if(bOK && !mbVO)
	                bOK = TrackLocalMap();
	        }

	        if(bOK)
	            mState = OK;
	        else
	            mState=LOST;

	        // Update drawer
	        //mpFrameDrawer->Update(this);

	        // If tracking were good, check if we insert a keyframe
	        if(bOK)
	        {
	            // Update motion model
	            if(!mLastFrame.mTcw.empty())
	            {
	                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
	                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
	                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
	                mVelocity = mCurrentFrame.mTcw*LastTwc;
	            }
	            else
	                mVelocity = cv::Mat();

	            //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	            // Clean VO matches
	            for(int i=0; i<mCurrentFrame.N; i++)
	            {
	                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	                if(pMP)
	                    if(pMP->Observations()<1)
	                    {
	                        mCurrentFrame.mvbOutlier[i] = false;
	                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
	                    }
	            }

	            // Delete temporal MapPoints
	            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
	            {
	                MapPoint* pMP = *lit;
	                delete pMP;
	            }
	            mlpTemporalPoints.clear();

	            // Check if we need to insert a new keyframe
	            if(NeedNewKeyFrame())
	            {
	                CreateNewKeyFrame();
	            }

	            if(maxFeatureMapSize_ > 0)
				{
					//limit size of the feature map, keep last X recent ones
					if(mpMap->KeyFramesInMap()>1 && mpMap->MapPointsInMap()>maxFeatureMapSize_)
					{
						std::vector<KeyFrame*> kfs = mpMap->GetAllKeyFrames();
						std::map<long unsigned int, KeyFrame*> kfsSorted;
						for(unsigned int i=1; i<kfs.size(); ++i)
						{
							kfsSorted.insert(std::make_pair(kfs[i]->mnId, kfs[i]));
						}
						KeyFrame * lastFrame = kfsSorted.rbegin()->second;
						std::vector<MapPoint*> mapPoints = mpMap->GetAllMapPoints();
						std::map<long unsigned int, MapPoint*> mapPointsSorted;
						for(unsigned int i=0; i<mapPoints.size(); ++i)
						{
							mapPointsSorted.insert(std::make_pair(mapPoints[i]->mnId, mapPoints[i]));
						}

						for(std::map<long unsigned int, MapPoint*>::iterator iter=mapPointsSorted.begin();
							iter != mapPointsSorted.end() && mpMap->MapPointsInMap()>maxFeatureMapSize_;
							++iter)
						{
							if(!iter->second->IsInKeyFrame(lastFrame))
							{
								// FIXME: Memory leak: ORB_SLAM2 doesn't delete after removing from the map...
								// Not sure when it is safe to delete it, as if I delete just
								// after setting the bad flag, the app crashes.
								iter->second->SetBadFlag();
							}
						}
						// remove kfs without observations
						for(std::map<long unsigned int, KeyFrame*>::iterator iter=kfsSorted.begin();
							iter != kfsSorted.end();
							++iter)
						{
							if(iter->second!=lastFrame && iter->second->GetMapPoints().size()==0)
							{
								// FIXME: Memory leak: ORB_SLAM2 doesn't delete after removing from the map...
								// Not sure when it is safe to delete it, as if I delete just
								// after setting the bad flag, the app crashes.
								iter->second->SetErase();
							}
							else
							{
								break;
							}
						}
					}
				}

	            // We allow points with high innovation (considererd outliers by the Huber Function)
	            // pass to the new keyframe, so that bundle adjustment will finally decide
	            // if they are outliers or not. We don't want next frame to estimate its position
	            // with those points so we discard them in the frame.
	            for(int i=0; i<mCurrentFrame.N;i++)
	            {
	                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
	                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
	            }
	        }

	        // Reset if the camera get lost soon after initialization
	        if(mState==LOST)
	        {
	            //if(mpMap->KeyFramesInMap()<=5)
	            {
	               	UWARN("Track lost...");
	                 return;

	            }
	        }

	        if(!mCurrentFrame.mpReferenceKF)
	            mCurrentFrame.mpReferenceKF = mpReferenceKF;

	        mLastFrame = Frame(mCurrentFrame);
	    }

	    // Store frame pose information to retrieve the complete camera trajectory afterwards.
	    if(!mCurrentFrame.mTcw.empty())
	    {
	        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
	        mlRelativeFramePoses.push_back(Tcr);
	        mlpReferences.push_back(mpReferenceKF);
	        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
	        mlbLost.push_back(mState==LOST);
	    }
	    else
	    {
	        // This can happen if tracking is lost
	        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
	        mlpReferences.push_back(mlpReferences.back());
	        mlFrameTimes.push_back(mlFrameTimes.back());
	        mlbLost.push_back(mState==LOST);
	    }
	}

	void StereoInitialization()
	{
	    if(mCurrentFrame.N>500)
	    {
	        // Set Frame pose to the origin
	        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

	        // Create KeyFrame
	        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	        // Insert KeyFrame in the map
	        mpMap->AddKeyFrame(pKFini);

	        // Create MapPoints and asscoiate to KeyFrame
	        for(int i=0; i<mCurrentFrame.N;i++)
	        {
	            float z = mCurrentFrame.mvDepth[i];
	            if(z>0)
	            {
	                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
	                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
	                pNewMP->AddObservation(pKFini,i);
	                pKFini->AddMapPoint(pNewMP,i);
	                pNewMP->ComputeDistinctiveDescriptors();
	                pNewMP->UpdateNormalAndDepth();
	                mpMap->AddMapPoint(pNewMP);

	                mCurrentFrame.mvpMapPoints[i]=pNewMP;
	            }
	        }

	        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

	        mpLocalMapper->InsertKeyFrame(pKFini);

	        mLastFrame = Frame(mCurrentFrame);
	        mnLastKeyFrameId=mCurrentFrame.mnId;
	        mpLastKeyFrame = pKFini;

	        mvpLocalKeyFrames.push_back(pKFini);
	        mvpLocalMapPoints=mpMap->GetAllMapPoints();
	        mpReferenceKF = pKFini;
	        mCurrentFrame.mpReferenceKF = pKFini;

	        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

	        //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

	        mState=OK;
	    }
	}

public:
	cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
	{
	    mImGray = imRectLeft;
	    cv::Mat imGrayRight = imRectRight;

	    if(mImGray.channels()==3)
	    {
	        if(mbRGB)
	        {
	            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
	        }
	        else
	        {
	            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
	        }
	    }
	    else if(mImGray.channels()==4)
	    {
	        if(mbRGB)
	        {
	            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
	        }
	        else
	        {
	            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
	        }
	    }
	    if(imGrayRight.channels()==3)
		{
			if(mbRGB)
			{
				cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
			}
			else
			{
				cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
			}
		}
		else if(imGrayRight.channels()==4)
		{
			if(mbRGB)
			{
				cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
			}
			else
			{
				cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
			}
		}

	    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

	    Track();

	    return mCurrentFrame.mTcw.clone();
	}

	cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
	{
	    mImGray = imRGB;
	    cv::Mat imDepth = imD;

	    if(mImGray.channels()==3)
	    {
	        if(mbRGB)
	            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
	        else
	            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
	    }
	    else if(mImGray.channels()==4)
	    {
	        if(mbRGB)
	            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
	        else
	            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
	    }

	    UASSERT(imDepth.type()==CV_32F);

	    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

	    Track();

	    return mCurrentFrame.mTcw.clone();
	}
};

// Hack to disable loop closing
class LoopCloser: public LoopClosing
{
public:
	LoopCloser(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale) :
		LoopClosing(pMap, pDB, pVoc, bFixScale)
	{
	}

public:
	void RunNoLoop()
	{
		mbFinished =false;

		while(1)
		{
			// just clear the buffer
			{
				unique_lock<mutex> lock(mMutexLoopQueue);
				mlpLoopKeyFrameQueue.clear();
			}

			ResetIfRequested();

			if(CheckFinish())
				break;

			usleep(1000000); // 1 sec
		}

		SetFinish();
	}
};

} // namespace ORB_SLAM2

class ORBSLAM2System
{
public:
	ORBSLAM2System(const rtabmap::ParametersMap & parameters) :
		mpVocabulary(0),
		mpKeyFrameDatabase(0),
		mpMap(0),
		mpTracker(0),
		mpLocalMapper(0),
		mpLoopCloser(0),
		mptLocalMapping(0),
		mptLoopClosing(0),
		parameters_(parameters)
	{
		std::string vocabularyPath;
		rtabmap::Parameters::parse(parameters, rtabmap::Parameters::kOdomORBSLAM2VocPath(), vocabularyPath);

		if(!vocabularyPath.empty())
		{
			//Load ORB Vocabulary
			vocabularyPath = uReplaceChar(vocabularyPath, '~', UDirectory::homeDir());
			UWARN("Loading ORB Vocabulary: \"%s\". This could take a while...", vocabularyPath.c_str());
			mpVocabulary = new ORB_SLAM2::ORBVocabulary();
			bool bVocLoad = mpVocabulary->loadFromTextFile(vocabularyPath);
			if(!bVocLoad)
			{
				UERROR("Failed to open vocabulary at %s", vocabularyPath.c_str());
				delete mpVocabulary;
				mpVocabulary = 0;
			}
			else
			{
				UWARN("Vocabulary loaded!");
			}
		}
		else
		{
			UERROR("ORBSLAM2 vocabulary path should be set! (Parameter name=\"%s\")", rtabmap::Parameters::kOdomORBSLAM2VocPath().c_str());
		}
	}

	bool init(const rtabmap::CameraModel & model, bool stereo, double baseline)
	{
		if(!mpVocabulary)
		{
			UERROR("Vocabulary not loaded!");
			return false;
		}

		this->shutdown();

		// Create configuration file
		std::string workingDir;
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kRtabmapWorkingDirectory(), workingDir);
		if(workingDir.empty())
		{
			workingDir = ".";
		}
		std::string configPath = workingDir+"/rtabmap_orbslam2.yaml";
		std::ofstream ofs (configPath, std::ofstream::out);
		ofs << "%YAML:1.0" << std::endl;
		ofs << std::endl;

		//# Camera calibration and distortion parameters (OpenCV)
		ofs << "Camera.fx: " << model.fx() << std::endl;
		ofs << "Camera.fy: " << model.fy() << std::endl;
		ofs << "Camera.cx: " << model.cx() << std::endl;
		ofs << "Camera.cy: " << model.cy() << std::endl;
		ofs << std::endl;

		if(model.D().cols < 4)
		{
			ofs << "Camera.k1: " << 0 << std::endl;
			ofs << "Camera.k2: " << 0 << std::endl;
			ofs << "Camera.p1: " << 0 << std::endl;
			ofs << "Camera.p2: " << 0 << std::endl;
			if(!stereo)
			{
				ofs << "Camera.k3: " << 0 << std::endl;
			}
		}
		if(model.D().cols >= 4)
		{
			ofs << "Camera.k1: " << model.D().at<double>(0,0) << std::endl;
			ofs << "Camera.k2: " << model.D().at<double>(0,1) << std::endl;
			ofs << "Camera.p1: " << model.D().at<double>(0,2) << std::endl;
			ofs << "Camera.p2: " << model.D().at<double>(0,3) << std::endl;
		}
		if(model.D().cols >= 5)
		{
			ofs << "Camera.k3: " << model.D().at<double>(0,4) << std::endl;
		}
		if(model.D().cols > 5)
		{
			UWARN("Unhandled camera distortion size %d, only 5 first coefficients used", model.D().cols);
		}
		ofs << std::endl;

		ofs << "Camera.width: " << model.imageWidth() << std::endl;
		ofs << "Camera.height: " << model.imageHeight() << std::endl;
		ofs << std::endl;

		//# IR projector baseline times fx (aprox.)
		if(baseline <= 0.0)
		{
			baseline = rtabmap::Parameters::defaultOdomORBSLAM2Bf();
			rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAM2Bf(), baseline);
		}
		ofs << "Camera.bf: " << model.fx()*baseline << std::endl;
		ofs << std::endl;

		//# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
		//Camera.RGB: 1
		ofs << "Camera.RGB: 1" << std::endl;
		ofs << std::endl;

		float fps = rtabmap::Parameters::defaultOdomORBSLAM2Fps();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAM2Fps(), fps);
		ofs << "Camera.fps: " << fps << std::endl;
		ofs << std::endl;

		//# Close/Far threshold. Baseline times.
		double thDepth = rtabmap::Parameters::defaultOdomORBSLAM2ThDepth();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAM2ThDepth(), thDepth);
		ofs << "ThDepth: " << thDepth << std::endl;
		ofs << std::endl;

		//# Deptmap values factor
		ofs << "DepthMapFactor: " << 1000.0 << std::endl;
		ofs << std::endl;

		//#--------------------------------------------------------------------------------------------
		//# ORB Parameters
		//#--------------------------------------------------------------------------------------------
		//# ORB Extractor: Number of features per image
		int features = rtabmap::Parameters::defaultOdomORBSLAM2MaxFeatures();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAM2MaxFeatures(), features);
		ofs << "ORBextractor.nFeatures: " << features << std::endl;
		ofs << std::endl;

		//# ORB Extractor: Scale factor between levels in the scale pyramid
		double scaleFactor = rtabmap::Parameters::defaultORBScaleFactor();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kORBScaleFactor(), scaleFactor);
		ofs << "ORBextractor.scaleFactor: " << scaleFactor << std::endl;
		ofs << std::endl;

		//# ORB Extractor: Number of levels in the scale pyramid
		int levels = rtabmap::Parameters::defaultORBNLevels();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kORBNLevels(), levels);
		ofs << "ORBextractor.nLevels: " << levels << std::endl;
		ofs << std::endl;

		//# ORB Extractor: Fast threshold
		//# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
		//# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
		//# You can lower these values if your images have low contrast
		int iniThFAST = rtabmap::Parameters::defaultFASTThreshold();
		int minThFAST = rtabmap::Parameters::defaultFASTMinThreshold();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kFASTThreshold(), iniThFAST);
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kFASTMinThreshold(), minThFAST);
		ofs << "ORBextractor.iniThFAST: " << iniThFAST << std::endl;
		ofs << "ORBextractor.minThFAST: " << minThFAST << std::endl;
		ofs << std::endl;

		int maxFeatureMapSize = rtabmap::Parameters::defaultOdomORBSLAM2MapSize();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAM2MapSize(), maxFeatureMapSize);

		ofs.close();

		//Create KeyFrame Database
		mpKeyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*mpVocabulary);

		//Create the Map
		mpMap = new ORB_SLAM2::Map();

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		mpTracker = new ORB_SLAM2::Tracker(0, mpVocabulary, 0, 0, mpMap, mpKeyFrameDatabase, configPath, stereo?ORB_SLAM2::System::STEREO:ORB_SLAM2::System::RGBD, maxFeatureMapSize);

		//Initialize the Local Mapping thread and launch
		mpLocalMapper = new ORB_SLAM2::LocalMapping(mpMap, false);

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new ORB_SLAM2::LoopCloser(mpMap, mpKeyFrameDatabase, mpVocabulary, true);

		mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
		mptLoopClosing = new thread(&ORB_SLAM2::LoopCloser::RunNoLoop, mpLoopCloser);

		//Set pointers between threads
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);
		mpTracker->SetViewer(0);

		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);

		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);

		return true;
	}

	virtual ~ORBSLAM2System()
	{
		shutdown();
		delete mpVocabulary;
	}

	void shutdown()
	{
		if(mpMap)
		{
			mpLocalMapper->RequestFinish();
			mpLoopCloser->RequestFinish();

			// Wait until all thread have effectively stopped
			while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
			{
				usleep(5000);
			}

			//cleanup!
			mptLoopClosing->join();
			delete mptLoopClosing;
			mptLoopClosing = 0;
			mptLocalMapping->join();
			delete mptLocalMapping;
			mptLocalMapping = 0;
			delete mpLoopCloser;
			mpLoopCloser=0;
			delete mpLocalMapper;
			mpLocalMapper=0;
			delete mpTracker;
			mpTracker=0;
			delete mpMap;
			mpMap=0;
			delete mpKeyFrameDatabase;
			mpKeyFrameDatabase=0;
		}
	}

public:
	// ORB vocabulary used for place recognition and feature matching.
	ORB_SLAM2::ORBVocabulary* mpVocabulary;

	// KeyFrame database for place recognition (relocalization and loop detection).
	ORB_SLAM2::KeyFrameDatabase* mpKeyFrameDatabase;

	// Map structure that stores the pointers to all KeyFrames and MapPoints.
	ORB_SLAM2::Map* mpMap;

	// Tracker. It receives a frame and computes the associated camera pose.
	// It also decides when to insert a new keyframe, create some new MapPoints and
	// performs relocalization if tracking fails.
	ORB_SLAM2::Tracker* mpTracker;

	// Local Mapper. It manages the local map and performs local bundle adjustment.
	ORB_SLAM2::LocalMapping* mpLocalMapper;

	// Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
	// a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
	ORB_SLAM2::LoopCloser* mpLoopCloser;

	 // System threads: Local Mapping, Loop Closing, Viewer.
	// The Tracking thread "lives" in the main execution thread that creates the System object.
	std::thread* mptLocalMapping;
	std::thread* mptLoopClosing;

	rtabmap::ParametersMap parameters_;
};
#endif

namespace rtabmap {

OdometryORBSLAM2::OdometryORBSLAM2(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_ORB_SLAM2
    ,
	orbslam2_(0),
	firstFrame_(true),
	previousPose_(Transform::getIdentity())
#endif
{
#ifdef RTABMAP_ORB_SLAM2
	orbslam2_ = new ORBSLAM2System(parameters);
#endif
}

OdometryORBSLAM2::~OdometryORBSLAM2()
{
#ifdef RTABMAP_ORB_SLAM2
	if(orbslam2_)
	{
		delete orbslam2_;
	}
#endif
}

void OdometryORBSLAM2::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_ORB_SLAM2
	if(orbslam2_)
	{
		orbslam2_->shutdown();
	}
	firstFrame_ = true;
	originLocalTransform_.setNull();
	previousPose_.setIdentity();
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryORBSLAM2::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;

#ifdef RTABMAP_ORB_SLAM2
	UTimer timer;

	if(data.imageRaw().empty() ||
		data.imageRaw().rows != data.depthOrRightRaw().rows ||
		data.imageRaw().cols != data.depthOrRightRaw().cols)
	{
		UERROR("Not supported input! RGB (%dx%d) and depth (%dx%d) should have the same size.",
				data.imageRaw().cols, data.imageRaw().rows, data.depthOrRightRaw().cols, data.depthOrRightRaw().rows);
		return t;
	}

	if(!((data.cameraModels().size() == 1 &&
			data.cameraModels()[0].isValidForReprojection()) ||
		(data.stereoCameraModel().isValidForProjection() &&
			data.stereoCameraModel().left().isValidForReprojection() &&
			data.stereoCameraModel().right().isValidForReprojection())))
	{
		UERROR("Invalid camera model!");
		return t;
	}

	bool stereo = data.cameraModels().size() == 0;

	cv::Mat covariance;
	if(orbslam2_->mpTracker == 0)
	{
		CameraModel model = data.cameraModels().size()==1?data.cameraModels()[0]:data.stereoCameraModel().left();
		if(!orbslam2_->init(model, stereo, data.stereoCameraModel().baseline()))
		{
			return t;
		}
	}

	cv::Mat Tcw;
	Transform localTransform;
	if(stereo)
	{
		localTransform = data.stereoCameraModel().localTransform();
		Tcw = ((ORB_SLAM2::Tracker*)orbslam2_->mpTracker)->GrabImageStereo(data.imageRaw(), data.rightRaw(), data.stamp());
	}
	else
	{
		localTransform = data.cameraModels()[0].localTransform();
		cv::Mat depth;
		if(data.depthRaw().type() == CV_32FC1)
		{
			depth = data.depthRaw();
		}
		else if(data.depthRaw().type() == CV_16UC1)
		{
			depth = util2d::cvtDepthToFloat(data.depthRaw());
		}
		Tcw = ((ORB_SLAM2::Tracker*)orbslam2_->mpTracker)->GrabImageRGBD(data.imageRaw(), depth, data.stamp());
	}

	Transform previousPoseInv = previousPose_.inverse();
	if(orbslam2_->mpTracker->mState == ORB_SLAM2::Tracking::LOST)
	{
		covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0f;
	}
	else if(Tcw.cols == 4 && Tcw.rows == 4)
	{
		Transform p = Transform(cv::Mat(Tcw, cv::Range(0,3), cv::Range(0,4)));

		if(!p.isNull())
		{
			if(!localTransform.isNull())
			{
				if(originLocalTransform_.isNull())
				{
					originLocalTransform_ = localTransform;
				}
				// transform in base frame
				p = originLocalTransform_ * p.inverse() * localTransform.inverse();
			}
			t = previousPoseInv*p;
		}
		previousPose_ = p;

		if(firstFrame_)
		{
			// just recovered of being lost, set high covariance
			covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0f;
			firstFrame_ = false;
		}
		else
		{
			float baseline = data.stereoCameraModel().baseline();
			if(baseline <= 0.0f)
			{
				baseline = rtabmap::Parameters::defaultOdomORBSLAM2Bf();
				rtabmap::Parameters::parse(orbslam2_->parameters_, rtabmap::Parameters::kOdomORBSLAM2Bf(), baseline);
			}
			double linearVar = 0.0001;
			if(baseline > 0.0f)
			{
				linearVar = baseline/8.0;
				linearVar *= linearVar;
			}

			covariance = cv::Mat::eye(6,6, CV_64FC1);
			covariance.at<double>(0,0) = linearVar;
			covariance.at<double>(1,1) = linearVar;
			covariance.at<double>(2,2) = linearVar;
			covariance.at<double>(3,3) = 0.0001;
			covariance.at<double>(4,4) = 0.0001;
			covariance.at<double>(5,5) = 0.0001;
		}
	}

	int totalMapPoints= 0;
	int totalKfs= 0;
	if(orbslam2_->mpMap)
	{
		totalMapPoints = orbslam2_->mpMap->MapPointsInMap();
		totalKfs = orbslam2_->mpMap->KeyFramesInMap();
	}

	if(info)
	{
		info->lost = t.isNull();
		info->type = (int)kTypeORBSLAM2;
		info->reg.covariance = covariance;
		info->localMapSize = totalMapPoints;
		info->localKeyFrames = totalKfs;

		if(this->isInfoDataFilled() && orbslam2_->mpTracker && orbslam2_->mpMap)
		{
			const std::vector<cv::KeyPoint> & kpts = orbslam2_->mpTracker->mCurrentFrame.mvKeys;
			info->reg.matchesIDs.resize(kpts.size());
			info->reg.inliersIDs.resize(kpts.size());
			int oi = 0;
			for (unsigned int i = 0; i < kpts.size(); ++i)
			{
				int wordId;
				if(orbslam2_->mpTracker->mCurrentFrame.mvpMapPoints[i] != 0)
				{
					wordId = orbslam2_->mpTracker->mCurrentFrame.mvpMapPoints[i]->mnId;
				}
				else
				{
					wordId = -(i+1);
				}
				info->words.insert(std::make_pair(wordId, kpts[i]));
				if(orbslam2_->mpTracker->mCurrentFrame.mvpMapPoints[i] != 0)
				{
					info->reg.matchesIDs[oi] = wordId;
					info->reg.inliersIDs[oi] = wordId;
					++oi;
				}
			}
			info->reg.matchesIDs.resize(oi);
			info->reg.inliersIDs.resize(oi);
			info->reg.inliers = oi;
			info->reg.matches = oi;

			std::vector<ORB_SLAM2::MapPoint*> mapPoints = orbslam2_->mpMap->GetAllMapPoints();
			Eigen::Affine3f fixRot = (this->getPose()*previousPoseInv*originLocalTransform_).toEigen3f();
			for (unsigned int i = 0; i < mapPoints.size(); ++i)
			{
				cv::Point3f pt(mapPoints[i]->GetWorldPos());
				pcl::PointXYZ ptt = pcl::transformPoint(pcl::PointXYZ(pt.x, pt.y, pt.z), fixRot);
				info->localMap.insert(std::make_pair(mapPoints[i]->mnId, cv::Point3f(ptt.x, ptt.y, ptt.z)));
			}
		}
	}

	UINFO("Odom update time = %fs, map points=%d, keyframes=%d, lost=%s", timer.elapsed(), totalMapPoints, totalKfs, t.isNull()?"true":"false");

#else
	UERROR("RTAB-Map is not built with ORB_SLAM2 support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
