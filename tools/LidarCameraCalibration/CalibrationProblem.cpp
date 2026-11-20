/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "CalibrationProblem.h"

#include <rtabmap/core/util3d_transforms.h>

#include <algorithm>
#include <cmath>

using namespace rtabmap;

Transform correctionFrom(const double p[6])
{
	return Transform(p[0], p[1], p[2], p[3]*M_PI/180.0, p[4]*M_PI/180.0, p[5]*M_PI/180.0);
}

Eigen::Isometry3d correctionFromDouble(const double p[6])
{
	// As Transform(x, y, z, roll, pitch, yaw): yaw * pitch * roll.
	Eigen::Isometry3d C = Eigen::Isometry3d::Identity();
	C.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
	C.linear() = (Eigen::AngleAxisd(p[5]*M_PI/180.0, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(p[4]*M_PI/180.0, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(p[3]*M_PI/180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
	return C;
}

CalibrationProblem::CalibrationProblem(const std::vector<Frame> & frames, const std::vector<int> & nodes, float minDepth) :
	frames_(frames), nodes_(nodes), minDepth_(minDepth), evaluations_(0)
{
}

void CalibrationProblem::project(const Transform & C,
		const std::function<void(const Frame &, size_t, float, float)> & visit) const
{
	for(int k : nodes_)
	{
		const Frame & f = frames_[k];
		const Transform scanInCam = (f.scanToCam * C).inverse();
		const double fx = f.K.at<double>(0,0), fy = f.K.at<double>(1,1);
		const double cx = f.K.at<double>(0,2), cy = f.K.at<double>(1,2);
		for(size_t i = 0; i < f.edgePoints.size(); ++i)
		{
			const cv::Point3f pc = util3d::transformPoint(f.edgePoints[i], scanInCam);
			if(pc.z < minDepth_)
			{
				continue;
			}
			const float u = fx * pc.x / pc.z + cx, v = fy * pc.y / pc.z + cy;
			if(u < 0 || v < 0 || u >= f.gray.cols - 1 || v >= f.gray.rows - 1)
			{
				continue;
			}
			visit(f, i, u, v);
		}
	}
}

double CalibrationProblem::score(const Transform & C) const
{
	++evaluations_;
	double sum = 0.0;
	project(C, [&](const Frame & f, size_t i, float u, float v) {
		const int u0 = int(u), v0 = int(v);
		const float a = u - u0, b = v - v0;
		const float s =
				(1-a)*(1-b)*f.edgeScore.at<float>(v0, u0) + a*(1-b)*f.edgeScore.at<float>(v0, u0+1) +
				(1-a)*b*f.edgeScore.at<float>(v0+1, u0) + a*b*f.edgeScore.at<float>(v0+1, u0+1);
		sum += f.edgeWeights[i] * s;
	});
	// The frames' edge points are selected again between passes: not cached.
	double weights = 0.0;
	for(int k : nodes_)
	{
		for(float w : frames_[k].edgeWeights)
		{
			weights += w;
		}
	}
	return weights > 0.0 ? sum / weights : 0.0;
}

double CalibrationProblem::edgeDistance(const Frame & f, size_t i, const double p[6], double cap) const
{
	const cv::Point3f & pt = f.edgePoints[i];
	const Eigen::Vector3d pc =
			(f.scanToCam.toEigen3d() * correctionFromDouble(p)).inverse() * Eigen::Vector3d(pt.x, pt.y, pt.z);
	if(pc.z() < minDepth_)
	{
		return cap;
	}
	const double u = f.K.at<double>(0,0) * pc.x() / pc.z() + f.K.at<double>(0,2);
	const double v = f.K.at<double>(1,1) * pc.y() / pc.z() + f.K.at<double>(1,2);
	if(u < 0 || v < 0 || u >= f.edgeDistance.cols - 1 || v >= f.edgeDistance.rows - 1)
	{
		return cap;
	}
	const int u0 = int(u), v0 = int(v);
	const double a = u - u0, b = v - v0;
	const cv::Mat & d = f.edgeDistance;
	const double distance =
			(1-a)*(1-b)*d.at<float>(v0, u0) + a*(1-b)*d.at<float>(v0, u0+1) +
			(1-a)*b*d.at<float>(v0+1, u0) + a*b*d.at<float>(v0+1, u0+1);
	return std::min(distance, cap);
}
