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

CameraMobile::CameraMobile(bool smoothing) :
		Camera(10),
		deviceTColorCamera_(Transform::getIdentity()),
		spinOncePreviousStamp_(0.0),
		textureId_(0),
		uvs_initialized_(false),
		previousStamp_(0.0),
		stampEpochOffset_(0.0),
		smoothing_(smoothing),
		colorCameraToDisplayRotation_(ROTATION_0),
		originUpdate_(false)
{
}

CameraMobile::~CameraMobile() {
	// Disconnect camera service
	close();
}

bool CameraMobile::init(const std::string &, const std::string &)
{
	deviceTColorCamera_ = opticalRotation;
	return true;
}

void CameraMobile::close()
{
	previousPose_.setNull();
	previousStamp_ = 0.0;
	lastKnownGPS_ = GPS();
	lastEnvSensors_.clear();
	originOffset_ = Transform();
	originUpdate_ = false;
	pose_ = Transform();
	data_ = SensorData();

    if(textureId_ != 0)
    {
        glDeleteTextures(1, &textureId_);
        textureId_ = 0;
    }
}

void CameraMobile::resetOrigin()
{
	previousPose_.setNull();
	previousStamp_ = 0.0;
	lastKnownGPS_ = GPS();
	lastEnvSensors_.clear();
	pose_ = Transform();
	data_ = SensorData();
	originUpdate_ = true;
}

void CameraMobile::poseReceived(const Transform & pose)
{
	if(!pose.isNull())
	{
		// send pose of the camera (without optical rotation)
		Transform p = pose*deviceTColorCamera_;
		if(originUpdate_)
		{
			originOffset_ = p.translation().inverse();
			originUpdate_ = false;
		}

		if(!originOffset_.isNull())
		{
			this->post(new PoseEvent(originOffset_*p));
		}
		else
		{
			this->post(new PoseEvent(p));
		}
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

void CameraMobile::setData(const SensorData & data, const Transform & pose, const glm::mat4 & viewMatrix, const glm::mat4 & projectionMatrix, const float * texCoord)
{
	LOGD("CameraMobile::setData pose=%s stamp=%f", pose.prettyPrint().c_str(), data.stamp());
	data_ = data;
    pose_ = pose;

    viewMatrix_ = viewMatrix;
    projectionMatrix_ = projectionMatrix;
    
    // adjust origin
    if(!originOffset_.isNull())
    {
        pose_ = originOffset_ * pose_;
        viewMatrix_ = glm::inverse(rtabmap::glmFromTransform(rtabmap::opengl_world_T_rtabmap_world * originOffset_ *rtabmap::rtabmap_world_T_opengl_world)*glm::inverse(viewMatrix_));
    }
    
    if(textureId_ == 0)
    {
    	glGenTextures(1, &textureId_);
    }

    if(texCoord)
    {
        memcpy(transformed_uvs_, texCoord, 8*sizeof(float));
        uvs_initialized_ = true;
    }
    
    LOGD("CameraMobile::setData textureId_=%d", (int)textureId_);

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
}

void CameraMobile::addEnvSensor(int type, float value)
{
	lastEnvSensors_.insert(std::make_pair((EnvSensor::Type)type, EnvSensor((EnvSensor::Type)type, value)));
}

void CameraMobile::spinOnce()
{
	if(!this->isRunning())
	{
		bool ignoreFrame = false;
		//float rate = 10.0f; // maximum 10 FPS for image data
		double now = UTimer::now();
		/*if(rate>0.0f)
		{
			if((spinOncePreviousStamp_>=0.0 && now>spinOncePreviousStamp_ && now - spinOncePreviousStamp_ < 1.0f/rate) ||
				((spinOncePreviousStamp_<=0.0 || now<=spinOncePreviousStamp_) && spinOnceFrameRateTimer_.getElapsedTime() < 1.0f/rate))
			{
				ignoreFrame = true;
			}
		}*/

		if(!ignoreFrame)
		{
			spinOnceFrameRateTimer_.start();
			spinOncePreviousStamp_ = now;
			mainLoop();
		}
		else
		{
			// just send pose
			capturePoseOnly();
		}
	}
}

void CameraMobile::mainLoopBegin()
{
	double t = cameraStartedTime_.elapsed();
	if(t < 5.0)
	{
		uSleep((5.0-t)*1000); // just to make sure that the camera is started
	}
}

void CameraMobile::mainLoop()
{
	CameraInfo info;
	SensorData data = this->captureImage(&info);

	if(data.isValid() && !info.odomPose.isNull())
	{
		if(lastKnownGPS_.stamp() > 0.0 && data.stamp()-lastKnownGPS_.stamp()<1.0)
		{
			data.setGPS(lastKnownGPS_);
		}
		else if(lastKnownGPS_.stamp()>0.0)
		{
			LOGD("GPS too old (current time=%f, gps time = %f)", data.stamp(), lastKnownGPS_.stamp());
		}

		if(lastEnvSensors_.size())
		{
			data.setEnvSensors(lastEnvSensors_);
			lastEnvSensors_.clear();
		}

		if(smoothing_ && !data.depthRaw().empty())
		{
			//UTimer t;
			data.setDepthOrRightRaw(rtabmap::util2d::fastBilateralFiltering(data.depthRaw(), bilateralFilteringSigmaS, bilateralFilteringSigmaR));
			//LOGD("Bilateral filtering, time=%fs", t.ticks());
		}

		// Rotate image depending on the camera orientation
		if(colorCameraToDisplayRotation_ == ROTATION_90)
		{
			UDEBUG("ROTATION_90");
			cv::Mat rgb, depth;
			cv::Mat rgbt(data.imageRaw().cols, data.imageRaw().rows, data.imageRaw().type());
			cv::flip(data.imageRaw(),rgb,1);
			cv::transpose(rgb,rgbt);
			rgb = rgbt;
			cv::Mat deptht(data.depthRaw().cols, data.depthRaw().rows, data.depthRaw().type());
			cv::flip(data.depthRaw(),depth,1);
			cv::transpose(depth,deptht);
			depth = deptht;
			CameraModel model = data.cameraModels()[0];
			cv::Size sizet(model.imageHeight(), model.imageWidth());
			model = CameraModel(
					model.fy(),
					model.fx(),
					model.cy(),
					model.cx()>0?model.imageWidth()-model.cx():0,
					model.localTransform()*rtabmap::Transform(0,-1,0,0, 1,0,0,0, 0,0,1,0));
			model.setImageSize(sizet);
			data.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = data.keypoints()[i].pt.y;
				keypoints[i].pt.y = rgb.rows - data.keypoints()[i].pt.x;
			}
			data.setFeatures(keypoints, data.keypoints3D(), cv::Mat());
		}
		else if(colorCameraToDisplayRotation_ == ROTATION_180)
		{
			UDEBUG("ROTATION_180");
			cv::Mat rgb, depth;
			cv::flip(data.imageRaw(),rgb,1);
			cv::flip(rgb,rgb,0);
			cv::flip(data.depthOrRightRaw(),depth,1);
			cv::flip(depth,depth,0);
			CameraModel model = data.cameraModels()[0];
			cv::Size sizet(model.imageWidth(), model.imageHeight());
			model = CameraModel(
					model.fx(),
					model.fy(),
					model.cx()>0?model.imageWidth()-model.cx():0,
					model.cy()>0?model.imageHeight()-model.cy():0,
					model.localTransform()*rtabmap::Transform(0,0,0,0,0,1,0));
			model.setImageSize(sizet);
			data.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = rgb.cols - data.keypoints()[i].pt.x;
				keypoints[i].pt.y = rgb.rows - data.keypoints()[i].pt.y;
			}
			data.setFeatures(keypoints, data.keypoints3D(), cv::Mat());
		}
		else if(colorCameraToDisplayRotation_ == ROTATION_270)
		{
			UDEBUG("ROTATION_270");
			cv::Mat rgb(data.imageRaw().cols, data.imageRaw().rows, data.imageRaw().type());
			cv::transpose(data.imageRaw(),rgb);
			cv::flip(rgb,rgb,1);
			cv::Mat depth(data.depthOrRightRaw().cols, data.depthOrRightRaw().rows, data.depthOrRightRaw().type());
			cv::transpose(data.depthOrRightRaw(),depth);
			cv::flip(depth,depth,1);
			CameraModel model = data.cameraModels()[0];
			cv::Size sizet(model.imageHeight(), model.imageWidth());
			model = CameraModel(
					model.fy(),
					model.fx(),
					model.cy()>0?model.imageHeight()-model.cy():0,
					model.cx(),
					model.localTransform()*rtabmap::Transform(0,1,0,0, -1,0,0,0, 0,0,1,0));
			model.setImageSize(sizet);
			data.setRGBDImage(rgb, depth, model);

			std::vector<cv::KeyPoint> keypoints = data.keypoints();
			for(size_t i=0; i<keypoints.size(); ++i)
			{
				keypoints[i].pt.x = rgb.cols - data.keypoints()[i].pt.y;
				keypoints[i].pt.y = data.keypoints()[i].pt.x;
			}
			data.setFeatures(keypoints, data.keypoints3D(), cv::Mat());
		}

		rtabmap::Transform pose = info.odomPose;
		data.setGroundTruth(Transform());

		// convert stamp to epoch
		bool firstFrame = previousPose_.isNull();
		if(firstFrame)
		{
			stampEpochOffset_ = UTimer::now()-data.stamp();
		}
		data.setStamp(stampEpochOffset_ + data.stamp());
		OdometryInfo info;
		if(!firstFrame)
		{
			info.interval = data.stamp()-previousStamp_;
			info.transform = previousPose_.inverse() * pose;
		}
		// linear cov = 0.0001
		info.reg.covariance = cv::Mat::eye(6,6,CV_64FC1) * (firstFrame?9999.0:0.0001);
		if(!firstFrame)
		{
			// angular cov = 0.000001
			info.reg.covariance.at<double>(3,3) *= 0.01;
			info.reg.covariance.at<double>(4,4) *= 0.01;
			info.reg.covariance.at<double>(5,5) *= 0.01;
		}
		LOGI("Publish odometry message (variance=%f)", firstFrame?9999:0.0001);
		this->post(new OdometryEvent(data, pose, info));
		previousPose_ = pose;
		previousStamp_ = data.stamp();
	}
	else if(!this->isKilled() && info.odomPose.isNull())
	{
		LOGW("Odometry lost");
		this->post(new OdometryEvent());
	}
}

SensorData CameraMobile::captureImage(CameraInfo * info)
{
	if(info)
	{
		info->odomPose = pose_;
	}
	return data_;
}

LaserScan CameraMobile::scanFromPointCloudData(
        const cv::Mat & pointCloudData,
        int points,
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
        for(unsigned int i=0;i<points; ++i)
        {
            cv::Point3f pt(inPtr[i*ic], inPtr[i*ic + 1], inPtr[i*ic + 2]);
            pt = util3d::transformPoint(pt, pose.inverse()*rtabmap_world_T_opengl_world);
            ptr[oi*4] = pt.x;
            ptr[oi*4 + 1] = pt.y;
            ptr[oi*4 + 2] = pt.z;

            //get color from rgb image
            cv::Point3f org= pt;
            pt = util3d::transformPoint(pt, opticalRotationInv);
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

            //confidence
            //*(int*)&ptr[i*4 + 3] = (int(pointCloudData[i*4 + 3] * 255.0f) << 8) | (int(255) << 16);

        }
        return LaserScan::backwardCompatibility(scanData.colRange(0, oi), 0, 10, rtabmap::Transform::getIdentity());
    }
    return LaserScan();
}

} /* namespace rtabmap */
