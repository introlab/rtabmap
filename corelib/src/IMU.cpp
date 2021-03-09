/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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


#include <rtabmap/core/IMU.h>

namespace rtabmap {

void IMU::convertToBaseFrame()
{
	if(!localTransform_.isNull() && !localTransform_.rotation().isIdentity())
	{
		cv::Mat rotationMatrix, rotationMatrixT;
		localTransform_.rotationMatrix().convertTo(rotationMatrix, CV_64FC1);
		cv::transpose(rotationMatrix, rotationMatrixT);

		cv::Mat_<double> v = rotationMatrix * cv::Mat(linearAcceleration_);
		linearAcceleration_ = cv::Vec3d(v(0,0), v(0,1), v(0,2));
		if(!linearAccelerationCovariance_.empty())
		{
			linearAccelerationCovariance_ = rotationMatrix * linearAccelerationCovariance_ * rotationMatrixT;
		}

		v = rotationMatrix * cv::Mat(angularVelocity_);
		angularVelocity_ = cv::Vec3d(v(0,0), v(0,1), v(0,2));
		if(!angularVelocityCovariance_.empty())
		{
			angularVelocityCovariance_ = rotationMatrix * angularVelocityCovariance_ * rotationMatrixT;
		}

		if(!(orientation_[0] == 0.0 && orientation_[1] == 0.0 && orientation_[2] == 0.0))
		{
			// orientation includes roll and pitch but not yaw in local transform
			Eigen::Quaterniond qTheta =
					Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
					Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(localTransform_.theta(), Eigen::Vector3d::UnitZ());
			Eigen::Quaterniond q = qTheta * Eigen::Quaterniond(orientation_[3], orientation_[0], orientation_[1], orientation_[2]) * localTransform_.getQuaterniond().inverse();
			orientation_ = cv::Vec4d(q.x(),q.y(),q.z(),q.w());
			if(!orientationCovariance_.empty())
			{
				orientationCovariance_ = rotationMatrix * orientationCovariance_ * rotationMatrixT;
			}
		}

		localTransform_ = Transform(localTransform_.x(), localTransform_.y(), localTransform_.z(), 0,0,0);
	}
}

} //namespace rtabmap
