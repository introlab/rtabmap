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
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "CameraMobile.h"
#include "util.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/util2d.h"
#include <glm/gtx/transform.hpp>

namespace rtabmap {

#define nullptr 0

//////////////////////////////
// CameraMobile
//////////////////////////////
const float CameraMobile::bilateralFilteringSigmaS = 2.0f;
const float CameraMobile::bilateralFilteringSigmaR = 0.075f;

const rtabmap::Transform CameraMobile::opticalRotation = Transform(
		0.0f,  0.0f,  1.0f, 0.0f,
	   -1.0f,  0.0f,  0.0f, 0.0f,
		0.0f, -1.0f,  0.0f, 0.0f);
const rtabmap::Transform CameraMobile::opticalRotationInv = Transform(
		0.0f, -1.0f,  0.0f, 0.0f,
	    0.0f,  0.0f, -1.0f, 0.0f,
		1.0f,  0.0f,  0.0f, 0.0f);

CameraMobile::CameraMobile(bool smoothing, float upstreamRelocalizationAccThr) :
		Camera(10),
		deviceTColorCamera_(Transform::getIdentity()),
		textureId_(0),
		uvs_initialized_(false),
		stampEpochOffset_(0.0),
		smoothing_(smoothing),
		colorCameraToDisplayRotation_(ROTATION_0),
		originUpdate_(true),
		upstreamRelocalizationAccThr_(upstreamRelocalizationAccThr),
		previousAnchorStamp_(0.0),
		dataGoodTracking_(true)
{
}

CameraMobile::~CameraMobile() {
	// Disconnect camera service
	close();
}

bool CameraMobile::init(const std::string &, const std::string &)
{
	deviceTColorCamera_ = opticalRotation;
	// clear semaphore
	if(dataReady_.value() > 0) {
		dataReady_.acquire(dataReady_.value());
	}
	return true;
}

void CameraMobile::close()
{
	UScopeMutex lock(dataMutex_);

	firstFrame_ = true;
	lastKnownGPS_ = GPS();
	lastEnvSensors_.clear();
	originOffset_ = Transform();
	originUpdate_ = true;
	dataPose_ = Transform();
	data_ = SensorData();
    dataGoodTracking_ = true;
    previousAnchorPose_.setNull();
    previousAnchorLinearVelocity_.clear();
    previousAnchorStamp_ = 0.0;

    if(textureId_ != 0)
    {
        glDeleteTextures(1, &textureId_);
        textureId_ = 0;
    }
    // in case someone is waiting on captureImage()
    dataReady_.release();
}

void CameraMobile::resetOrigin(const rtabmap::Transform & offset)
{
    manualOriginOffset_ = offset;
	originUpdate_ = true;
}

bool CameraMobile::getPose(double epochStamp, Transform & pose, cv::Mat & covariance, double maxWaitTime)
{
	pose.setNull();

	int maxWaitTimeMs = maxWaitTime * 1000;

	// Interpolate pose
	if(!poseBuffer_.empty())
	{
		poseMutex_.lock();
		int waitTry = 0;
		while(maxWaitTimeMs>0 && poseBuffer_.rbegin()->first < epochStamp && waitTry < maxWaitTimeMs)
		{
			poseMutex_.unlock();
			++waitTry;
			uSleep(1);
			poseMutex_.lock();
		}
		if(poseBuffer_.rbegin()->first < epochStamp)
		{
			if(maxWaitTimeMs > 0)
			{
				UWARN("Could not find poses to interpolate at time %f after waiting %d ms (latest is %f)...", epochStamp, maxWaitTimeMs, poseBuffer_.rbegin()->first);
			}
			else
			{
				UWARN("Could not find poses to interpolate at time %f (latest is %f)...", epochStamp, poseBuffer_.rbegin()->first);
			}
		}
		else
		{
			std::map<double, Transform>::const_iterator iterB = poseBuffer_.lower_bound(epochStamp);
			std::map<double, Transform>::const_iterator iterA = iterB;
			if(iterA != poseBuffer_.begin())
			{
				iterA = --iterA;
			}
			if(iterB == poseBuffer_.end())
			{
				iterB = --iterB;
			}
			if(iterA == iterB && epochStamp == iterA->first)
			{
				pose = iterA->second;
			}
			else if(epochStamp >= iterA->first && epochStamp <= iterB->first)
			{
				pose = iterA->second.interpolate((epochStamp-iterA->first) / (iterB->first-iterA->first), iterB->second);
			}
			else // stamp < iterA->first
			{
				UWARN("Could not find pose data to interpolate at time %f (earliest is %f). Are sensors synchronized?", epochStamp, iterA->first);
			}
		}
		poseMutex_.unlock();
	}
	return !pose.isNull();
}

void CameraMobile::poseReceived(const Transform & pose, double deviceStamp)
{
	// Pose reveived is the pose of the device in rtabmap coordinate
	if(!pose.isNull())
	{
		Transform p = pose;
		
		if(stampEpochOffset_ == 0.0)
		{
			stampEpochOffset_ = UTimer::now() - deviceStamp;
		}

		if(originUpdate_)
		{
			firstFrame_ = true;
			lastKnownGPS_ = GPS();
			lastEnvSensors_.clear();
			dataGoodTracking_ = true;
			previousAnchorPose_.setNull();
			previousAnchorLinearVelocity_.clear();
			previousAnchorStamp_ = 0.0;
            originOffset_ = manualOriginOffset_.isNull() ? pose.translation().inverse() : manualOriginOffset_;
			originUpdate_ = false;
		}
        
		double epochStamp = stampEpochOffset_ + deviceStamp;
		if(!originOffset_.isNull())
		{
			// Filter re-localizations from poses received
			rtabmap::Transform rawPose = originOffset_ * pose.translation(); // remove rotation to keep position in fixed frame
			// Remove upstream localization corrections by integrating pose from previous frame anchor
            bool showLog = false;
            if(upstreamRelocalizationAccThr_>0.0f && !previousAnchorPose_.isNull())
			{
				float dt = epochStamp - previousAnchorStamp_;
                std::vector<float> currentLinearVelocity(3);
                float dx = rawPose.x()-previousAnchorPose_.x();
                float dy = rawPose.y()-previousAnchorPose_.y();
                float dz = rawPose.z()-previousAnchorPose_.z();
                currentLinearVelocity[0] = dx / dt;
                currentLinearVelocity[1] = dy / dt;
                currentLinearVelocity[2] = dz / dt;
				if(!previousAnchorLinearVelocity_.empty() && uNorm(dx, dy, dz)>0.02)
				{
                    float ax = (currentLinearVelocity[0] - previousAnchorLinearVelocity_[0]) / dt;
                    float ay = (currentLinearVelocity[1] - previousAnchorLinearVelocity_[1]) / dt;
                    float az = (currentLinearVelocity[2] - previousAnchorLinearVelocity_[2]) / dt;
					float acceleration = sqrt(ax*ax + ay*ay + az*az);
					if(acceleration>=upstreamRelocalizationAccThr_)
					{
						// Only correct the translation to not lose rotation aligned
						// with gravity.
                        
                        // Use constant motion model to update current pose.
						rtabmap::Transform offset(previousAnchorLinearVelocity_[0] * dt,
                                                  previousAnchorLinearVelocity_[1] * dt,
                                                  previousAnchorLinearVelocity_[2] * dt,
                                                  0, 0, 0, 1);
						rtabmap::Transform newRawPose = offset * previousAnchorPose_;
						currentLinearVelocity = previousAnchorLinearVelocity_;
						originOffset_.x() += newRawPose.x() - rawPose.x();
                        originOffset_.y() += newRawPose.y() - rawPose.y();
                        originOffset_.z() += newRawPose.z() - rawPose.z();
                        UERROR("Upstream re-localization has been suppressed because of "
						     "high acceleration detected (%f m/s^2) causing a jump!",
								acceleration);
						dataGoodTracking_ = false;
                        post(new CameraInfoEvent(0, "UpstreamRelocationFiltered", uFormat("%.1f m/s^2", acceleration).c_str()));
                        showLog = true;
					}
				}
				previousAnchorLinearVelocity_ = currentLinearVelocity;
			}

            p = originOffset_*pose;
            previousAnchorPose_ = p;
            previousAnchorStamp_ = epochStamp;
            
            if(upstreamRelocalizationAccThr_>0.0f) {
                relocalizationDebugBuffer_.insert(std::make_pair(epochStamp, std::make_pair(pose, p)));
                if(relocalizationDebugBuffer_.size() > 60)
                {
                    relocalizationDebugBuffer_.erase(relocalizationDebugBuffer_.begin());
                }
                if(showLog) {
                    std::stringstream stream;
                    for(auto iter=relocalizationDebugBuffer_.begin(); iter!=relocalizationDebugBuffer_.end(); ++iter)
                    {
                        stream << iter->first - relocalizationDebugBuffer_.begin()->first
                        << " " << iter->second.first.x()
                        << " " << iter->second.first.y()
                        << " " << iter->second.first.z()
                        << " " << iter->second.second.x()
                        << " " << iter->second.second.y()
                        << " " << iter->second.second.z() << std::endl;
                    }
                    UERROR("timestamp original_xyz corrected_xyz:\n%s", stream.str().c_str());
                }
            }
		}

		{
			UScopeMutex lock(poseMutex_);
			poseBuffer_.insert(poseBuffer_.end(), std::make_pair(epochStamp, p));
			if(poseBuffer_.size() > 1000)
			{
				poseBuffer_.erase(poseBuffer_.begin());
			}
		}

		// send pose of the camera (with optical rotation)
		this->post(new PoseEvent(p * deviceTColorCamera_));
	}
}

bool CameraMobile::isCalibrated() const
{
	return model_.isValidForProjection();
}

void CameraMobile::setGPS(const GPS & gps)
{
	lastKnownGPS_ = gps;
}

void CameraMobile::addEnvSensor(int type, float value)
{
	lastEnvSensors_.insert(std::make_pair((EnvSensor::Type)type, EnvSensor((EnvSensor::Type)type, value)));
}

void CameraMobile::update(const SensorData & data, const Transform & pose, const glm::mat4 & viewMatrix, const glm::mat4 & projectionMatrix, const float * texCoord)
{
	UScopeMutex lock(dataMutex_);

    LOGD("CameraMobile::update pose=%s stamp=%f", pose.prettyPrint().c_str(), data.stamp());

	bool notify = !data_.isValid();
    
	data_ = data;
    dataPose_ = pose;

    viewMatrix_ = viewMatrix;
    projectionMatrix_ = projectionMatrix;
        
    if(textureId_ == 0)
    {
    	glGenTextures(1, &textureId_);
    }

    if(texCoord)
    {
        memcpy(transformed_uvs_, texCoord, 8*sizeof(float));
        uvs_initialized_ = true;
    }
    
    LOGD("CameraMobile::update textureId_=%d", (int)textureId_);

    if(textureId_ != 0 && texCoord != 0)
    {
        cv::Mat rgbImage;
        cv::cvtColor(data.imageRaw(), rgbImage, cv::COLOR_BGR2RGBA);
        
        glBindTexture(GL_TEXTURE_2D, textureId_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        //glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        //glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
        //glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImage.cols, rgbImage.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbImage.data);

        GLint error = glGetError();
        if(error != GL_NO_ERROR)
        {
            LOGE("OpenGL: Could not allocate texture (0x%x)\n", error);
            textureId_ = 0;
            return;
        }
    }

    if(data_.isValid())
    {
        postUpdate();

        if(notify)
        {
            dataReady_.release();
        }
    }
}

void CameraMobile::updateOnRender()
{
	UScopeMutex lock(dataMutex_);
	bool notify = !data_.isValid();

	data_ = updateDataOnRender(dataPose_);
	if(data_.isValid())
	{
		postUpdate();

		if(notify)
		{
			dataReady_.release();
		}
	}
}

SensorData CameraMobile::updateDataOnRender(Transform & pose)
{
	LOGE("To use CameraMobile::updateOnRender(), CameraMobile::updateDataOnRender() "
	     "should be overridden by inherited classes. Returning empty data!\n");
	return SensorData();
}

void CameraMobile::postUpdate()
{
	if(data_.isValid())
	{
		// adjust origin
		if(!originOffset_.isNull())
		{
			dataPose_ = originOffset_ * dataPose_;
			viewMatrix_ = glm::inverse(rtabmap::glmFromTransform(rtabmap::opengl_world_T_rtabmap_world * originOffset_ *rtabmap::rtabmap_world_T_opengl_world)*glm::inverse(viewMatrix_));
			occlusionModel_.setLocalTransform(originOffset_ * occlusionModel_.localTransform());
		}

		if(lastKnownGPS_.stamp() > 0.0 && data_.stamp()-lastKnownGPS_.stamp()<1.0)
		{
			data_.setGPS(lastKnownGPS_);
		}
		else if(lastKnownGPS_.stamp()>0.0)
		{
			LOGD("GPS too old (current time=%f, gps time = %f)", data_.stamp(), lastKnownGPS_.stamp());
		}

		if(lastEnvSensors_.size())
		{
			data_.setEnvSensors(lastEnvSensors_);
			lastEnvSensors_.clear();
		}

		if(smoothing_ && !data_.depthRaw().empty())
		{
			//UTimer t;
			data_.setDepthOrRightRaw(rtabmap::util2d::fastBilateralFiltering(data_.depthRaw(), bilateralFilteringSigmaS, bilateralFilteringSigmaR));
			//LOGD("Bilateral filtering, time=%fs", t.ticks());
		}

		// Rotate image depending on the camera orientation
		if(colorCameraToDisplayRotation_ == ROTATION_90)
		{
			UDEBUG("ROTATION_90");
			cv::Mat rgb, depth;
			cv::Mat rgbt(data_.imageRaw().cols, data_.imageRaw().rows, data_.imageRaw().type());
			cv::flip(data_.imageRaw(),rgb,1);
			cv::transpose(rgb,rgbt);
			rgb = rgbt;
			cv::Mat deptht(data_.depthRaw().cols, data_.depthRaw().rows, data_.depthRaw().type());
			cv::flip(data_.depthRaw(),depth,1);
			cv::transpose(depth,deptht);
			depth = deptht;
			CameraModel model = data_.cameraModels()[0];
			cv::Size sizet(model.imageHeight(), model.imageWidth());
			model = CameraModel(
					model.fy(),
					model.fx(),
					model.cy(),
					model.cx()>0?model.imageWidth()-model.cx():0,
					model.localTransform()*rtabmap::Transform(0,-1,0,0, 1,0,0,0, 0,0,1,0));
			model.setImageSize(sizet);
			data_.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data_.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = data_.keypoints()[i].pt.y;
				keypoints[i].pt.y = rgb.rows - data_.keypoints()[i].pt.x;
			}
			data_.setFeatures(keypoints, data_.keypoints3D(), cv::Mat());
		}
		else if(colorCameraToDisplayRotation_ == ROTATION_180)
		{
			UDEBUG("ROTATION_180");
			cv::Mat rgb, depth;
			cv::flip(data_.imageRaw(),rgb,1);
			cv::flip(rgb,rgb,0);
			cv::flip(data_.depthOrRightRaw(),depth,1);
			cv::flip(depth,depth,0);
			CameraModel model = data_.cameraModels()[0];
			cv::Size sizet(model.imageWidth(), model.imageHeight());
			model = CameraModel(
					model.fx(),
					model.fy(),
					model.cx()>0?model.imageWidth()-model.cx():0,
					model.cy()>0?model.imageHeight()-model.cy():0,
					model.localTransform()*rtabmap::Transform(0,0,0,0,0,1,0));
			model.setImageSize(sizet);
			data_.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data_.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = rgb.cols - data_.keypoints()[i].pt.x;
				keypoints[i].pt.y = rgb.rows - data_.keypoints()[i].pt.y;
			}
			data_.setFeatures(keypoints, data_.keypoints3D(), cv::Mat());
		}
		else if(colorCameraToDisplayRotation_ == ROTATION_270)
		{
			UDEBUG("ROTATION_270");
			cv::Mat rgb(data_.imageRaw().cols, data_.imageRaw().rows, data_.imageRaw().type());
			cv::transpose(data_.imageRaw(),rgb);
			cv::flip(rgb,rgb,1);
			cv::Mat depth(data_.depthOrRightRaw().cols, data_.depthOrRightRaw().rows, data_.depthOrRightRaw().type());
			cv::transpose(data_.depthOrRightRaw(),depth);
			cv::flip(depth,depth,1);
			CameraModel model = data_.cameraModels()[0];
			cv::Size sizet(model.imageHeight(), model.imageWidth());
			model = CameraModel(
					model.fy(),
					model.fx(),
					model.cy()>0?model.imageHeight()-model.cy():0,
					model.cx(),
					model.localTransform()*rtabmap::Transform(0,1,0,0, -1,0,0,0, 0,0,1,0));
			model.setImageSize(sizet);
			data_.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data_.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = rgb.cols - data_.keypoints()[i].pt.y;
				keypoints[i].pt.y = data_.keypoints()[i].pt.x;
			}
			data_.setFeatures(keypoints, data_.keypoints3D(), cv::Mat());
		}
	}
}

SensorData CameraMobile::captureImage(SensorCaptureInfo * info)
{
	SensorData data;
    bool firstFrame = true;
    bool dataGoodTracking = true;
    rtabmap::Transform dataPose;
	if(dataReady_.acquire(1, 15000))
	{
		UScopeMutex lock(dataMutex_);
		data = data_;
        dataPose = dataPose_;
        firstFrame = firstFrame_;
        dataGoodTracking = dataGoodTracking_;
        firstFrame_ = false;
        dataGoodTracking_ = true;
		data_ = SensorData();
        dataPose_.setNull();
	}
	if(data.isValid())
	{
		data.setGroundTruth(Transform());
		data.setStamp(stampEpochOffset_ + data.stamp());

		if(info)
		{
			// linear cov = 0.0001
			info->odomCovariance = cv::Mat::eye(6,6,CV_64FC1) * (firstFrame?9999.0:0.00001);
			if(!firstFrame)
			{
				// angular cov = 0.000001
                // roll/pitch should be fairly accurate with VIO input
				info->odomCovariance.at<double>(3,3) *= 0.01; // roll
				info->odomCovariance.at<double>(4,4) *= 0.01; // pitch
                if(!dataGoodTracking)
                {
                    UERROR("not good tracking!");
                    // add slightly more error on translation
                    // 0.001
                    info->odomCovariance.at<double>(0,0) *= 10; // x
                    info->odomCovariance.at<double>(1,1) *= 10; // y
                    info->odomCovariance.at<double>(2,2) *= 10; // z
                    info->odomCovariance.at<double>(5,5) *= 10; // yaw
                }
			}
			info->odomPose = dataPose;
		}
	}
	else
	{
		UWARN("CameraMobile::captureImage() invalid data!");
	}
	return data;
}

LaserScan CameraMobile::scanFromPointCloudData(
        const cv::Mat & pointCloudData,
        const Transform & pose,
        const CameraModel & model,
        const cv::Mat & rgb,
        std::vector<cv::KeyPoint> * kpts,
        std::vector<cv::Point3f> * kpts3D,
        int kptsSize)
{
    if(!pointCloudData.empty())
    {
        cv::Mat scanData(1, pointCloudData.cols, CV_32FC4);
        float * ptr = scanData.ptr<float>();
        const float * inPtr = pointCloudData.ptr<float>();
        int ic = pointCloudData.channels();
        UASSERT(pointCloudData.depth() == CV_32F && ic >= 3);
        
        int oi = 0;
        for(unsigned int i=0;i<pointCloudData.cols; ++i)
        {
            cv::Point3f pt(inPtr[i*ic], inPtr[i*ic + 1], inPtr[i*ic + 2]);
            pt = util3d::transformPoint(pt, pose.inverse()*rtabmap_world_T_opengl_world);
            ptr[oi*4] = pt.x;
            ptr[oi*4 + 1] = pt.y;
            ptr[oi*4 + 2] = pt.z;

            //get color from rgb image
            cv::Point3f org= pt;
            pt = util3d::transformPoint(pt, opticalRotationInv);
            if(pt.z > 0)
            {
                int u,v;
                model.reproject(pt.x, pt.y, pt.z, u, v);
                unsigned char r=255,g=255,b=255;
                if(model.inFrame(u, v))
                {
                    b=rgb.at<cv::Vec3b>(v,u).val[0];
                    g=rgb.at<cv::Vec3b>(v,u).val[1];
                    r=rgb.at<cv::Vec3b>(v,u).val[2];
                    if(kpts)
                        kpts->push_back(cv::KeyPoint(u,v,kptsSize));
                    if(kpts3D)
                        kpts3D->push_back(org);
                    
                    *(int*)&ptr[oi*4 + 3] = int(b) | (int(g) << 8) | (int(r) << 16);
                    ++oi;
                }
            }
            //confidence
            //*(int*)&ptr[i*4 + 3] = (int(pointCloudData[i*4 + 3] * 255.0f) << 8) | (int(255) << 16);

        }
        return LaserScan::backwardCompatibility(scanData.colRange(0, oi), 0, 10, rtabmap::Transform::getIdentity());
    }
    return LaserScan();
}

} /* namespace rtabmap */
