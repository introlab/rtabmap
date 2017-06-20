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

#ifndef UTIL_H_
#define UTIL_H_


#include <android/log.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/CameraModel.h>
#include <tango-gl/util.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/pcl_base.h>

class LogHandler : public UEventsHandler
{
public:
	LogHandler()
	{
		ULogger::setLevel(ULogger::kDebug);
		ULogger::setEventLevel(ULogger::kDebug);
		ULogger::setPrintThreadId(true);

		registerToEventsManager();
	}
protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("ULogEvent") == 0)
		{
			ULogEvent * logEvent = (ULogEvent*)event;
			if(logEvent->getCode() == ULogger::kDebug)
			{
				LOGD(logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kInfo)
			{
				LOGI(logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() == ULogger::kWarning)
			{
				LOGW(logEvent->getMsg().c_str());
			}
			else if(logEvent->getCode() >= ULogger::kError)
			{
				LOGE(logEvent->getMsg().c_str());
			}

		}
		return false;
	}
};

static const rtabmap::Transform opengl_world_T_tango_world(
		1.0f,  0.0f,  0.0f, 0.0f,
		0.0f,  0.0f,  1.0f, 0.0f,
		0.0f, -1.0f,  0.0f, 0.0f);

static const rtabmap::Transform rtabmap_world_T_tango_world(
		 0.0f, 1.0f, 0.0f, 0.0f,
	    -1.0f, 0.0f, 0.0f, 0.0f,
		 0.0f, 0.0f, 1.0f, 0.0f);

static const rtabmap::Transform tango_device_T_rtabmap_device(
		 0.0f, -1.0f, 0.0f, 0.0f,
	     0.0f,  0.0f, 1.0f, 0.0f,
		-1.0f,  0.0f, 0.0f, 0.0f);

static const rtabmap::Transform opengl_world_T_rtabmap_world(
		 0.0f, -1.0f, 0.0f, 0.0f,
		 0.0f,  0.0f, 1.0f, 0.0f,
		-1.0f,  0.0f, 0.0f, 0.0f);

static const rtabmap::Transform rtabmap_device_T_opengl_device(
		 0.0f, 0.0f, -1.0f, 0.0f,
		-1.0f, 0.0f,  0.0f, 0.0f,
		 0.0f, 1.0f,  0.0f, 0.0f);

inline glm::mat4 glmFromTransform(const rtabmap::Transform & transform)
{
	glm::mat4 mat(1.0f);
	// gl is column wise
	mat[0][0] = transform(0,0);
	mat[1][0] = transform(0,1);
	mat[2][0] = transform(0,2);
	mat[0][1] = transform(1,0);
	mat[1][1] = transform(1,1);
	mat[2][1] = transform(1,2);
	mat[0][2] = transform(2,0);
	mat[1][2] = transform(2,1);
	mat[2][2] = transform(2,2);

	mat[3][0] = transform(0,3);
	mat[3][1] = transform(1,3);
	mat[3][2] = transform(2,3);
	return mat;
}

inline rtabmap::Transform glmToTransform(const glm::mat4 & mat)
{
	rtabmap::Transform transform;
	// gl is column wise
	transform(0,0) = mat[0][0];
	transform(0,1) = mat[1][0];
	transform(0,2) = mat[2][0];
	transform(1,0) = mat[0][1];
	transform(1,1) = mat[1][1];
	transform(1,2) = mat[2][1];
	transform(2,0) = mat[0][2];
	transform(2,1) = mat[1][2];
	transform(2,2) = mat[2][2];

	transform(0,3) = mat[3][0];
	transform(1,3) = mat[3][1];
	transform(2,3) = mat[3][2];

	return transform;
}

class Mesh
{
public:
	Mesh() :
		cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
		normals(new pcl::PointCloud<pcl::Normal>),
		indices(new std::vector<int>),
		visible(true)
	{
		gains[0] = gains[1] = gains[2] = 1.0f;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // organized cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::IndicesPtr indices;
	std::vector<pcl::Vertices> polygons;
	std::vector<pcl::Vertices> polygonsLowRes;
	rtabmap::Transform pose; // in rtabmap coordinates
	bool visible;
	rtabmap::CameraModel cameraModel;
	double gains[3]; // RGB gains
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
    	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texCoords;
#else
    	std::vector<Eigen::Vector2f> texCoords;
#endif
	cv::Mat texture;
};

#endif /* UTIL_H_ */
