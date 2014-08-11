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

#include <rtabmap/core/Transform.h>

#include <pcl/common/eigen.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <iomanip>

namespace rtabmap {

Transform::Transform() : data_(12)
{
	data_[0] = 0.0f;
	data_[1] = 0.0f;
	data_[2] = 0.0f;
	data_[3] = 0.0f;
	data_[4] = 0.0f;
	data_[5] = 0.0f;
	data_[6] = 0.0f;
	data_[7] = 0.0f;
	data_[8] = 0.0f;
	data_[9] = 0.0f;
	data_[10] = 0.0f;
	data_[11] = 0.0f;
}

// rotation matrix r## and origin o##
Transform::Transform(float r11, float r12, float r13, float o14,
				     float r21, float r22, float r23, float o24,
				     float r31, float r32, float r33, float o34) :
	data_(12)
{
	data_[0] = r11;
	data_[1] = r12;
	data_[2] = r13;
	data_[3] = o14;
	data_[4] = r21;
	data_[5] = r22;
	data_[6] = r23;
	data_[7] = o24;
	data_[8] = r31;
	data_[9] = r32;
	data_[10] = r33;
	data_[11] = o34;
}

Transform::Transform(float x, float y, float z, float roll, float pitch, float yaw)
{
	Eigen::Affine3f t = pcl::getTransformation (x, y, z, roll, pitch, yaw);
	*this = util3d::transformFromEigen3f(t);
}

bool Transform::isNull() const
{
	return (data_[0] == 0.0f &&
			data_[1] == 0.0f &&
			data_[2] == 0.0f &&
			data_[3] == 0.0f &&
			data_[4] == 0.0f &&
			data_[5] == 0.0f &&
			data_[6] == 0.0f &&
			data_[7] == 0.0f &&
			data_[8] == 0.0f &&
			data_[9] == 0.0f &&
			data_[10] == 0.0f &&
			data_[11] == 0.0f) ||
			uIsNan(data_[0]) ||
			uIsNan(data_[1]) ||
			uIsNan(data_[2]) ||
			uIsNan(data_[3]) ||
			uIsNan(data_[4]) ||
			uIsNan(data_[5]) ||
			uIsNan(data_[6]) ||
			uIsNan(data_[7]) ||
			uIsNan(data_[8]) ||
			uIsNan(data_[9]) ||
			uIsNan(data_[10]) ||
			uIsNan(data_[11]);
}

bool Transform::isIdentity() const
{
	return data_[0] == 1.0f &&
			data_[1] == 0.0f &&
			data_[2] == 0.0f &&
			data_[3] == 0.0f &&
			data_[4] == 0.0f &&
			data_[5] == 1.0f &&
			data_[6] == 0.0f &&
			data_[7] == 0.0f &&
			data_[8] == 0.0f &&
			data_[9] == 0.0f &&
			data_[10] == 1.0f &&
			data_[11] == 0.0f;
}

void Transform::setNull()
{
	*this = Transform();
}

void Transform::setIdentity()
{
	*this = getIdentity();
}

Transform Transform::getIdentity()
{
	return Transform(1,0,0,0,
					 0,1,0,0,
					 0,0,1,0);
}

Transform Transform::inverse() const
{
	Eigen::Matrix4f m = util3d::transformToEigen4f(*this);
	return util3d::transformFromEigen4f(m.inverse());
}

Transform Transform::rotation() const
{
	return Transform(data_[0], data_[1], data_[2], 0,
					 data_[4], data_[5], data_[6], 0,
					 data_[8], data_[9], data_[10], 0);
}

Transform Transform::translation() const
{
	return Transform(1,0,0, data_[3],
					 0,1,0, data_[7],
					 0,0,1, data_[11]);
}

void Transform::getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const
{
	pcl::getTranslationAndEulerAngles(util3d::transformToEigen3f(*this), x, y, z, roll, pitch, yaw);
}

void Transform::getTranslation(float & x, float & y, float & z) const
{
	x = this->x();
	y = this->y();
	z = this->z();
}

float Transform::getNorm() const
{
	return std::sqrt(this->getNormSquared());
}

float Transform::getNormSquared() const
{
	return this->x()*this->x() + this->y()*this->y() + this->z()*this->z();
}

std::string Transform::prettyPrint() const
{
	float x,y,z,roll,pitch,yaw;
	getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
	return uFormat("xyz=%f,%f,%f rpy=%f,%f,%f", x,y,z, roll,pitch,yaw);
}

Transform Transform::operator*(const Transform & t) const
{
	Eigen::Matrix4f m1 = util3d::transformToEigen4f(*this);
	Eigen::Matrix4f m2 = util3d::transformToEigen4f(t);
	return util3d::transformFromEigen4f(m1*m2);
}

Transform & Transform::operator*=(const Transform & t)
{
	*this = *this * t;
	return *this;
}

bool Transform::operator==(const Transform & t) const
{
	return memcmp(data_.data(), t.data_.data(), data_.size() * sizeof(float)) == 0;
}

bool Transform::operator!=(const Transform & t) const
{
	return !(*this == t);
}

std::ostream& operator<<(std::ostream& os, const Transform& s)
{
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 4; ++j)
		{
			std::cout << std::left << std::setw(12) << s.data()[i*4 + j];
		}
		std::cout << std::endl;
	}
	return os;
}


}
