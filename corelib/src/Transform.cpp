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

#include <rtabmap/core/Transform.h>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <iomanip>

namespace rtabmap {

Transform::Transform() : data_(cv::Mat::zeros(3,4,CV_32FC1))
{
}

// rotation matrix r## and origin o##
Transform::Transform(
		float r11, float r12, float r13, float o14,
		float r21, float r22, float r23, float o24,
		float r31, float r32, float r33, float o34)
{
	data_ = (cv::Mat_<float>(3,4) <<
			r11, r12, r13, o14,
			r21, r22, r23, o24,
			r31, r32, r33, o34);
}

Transform::Transform(const cv::Mat & transformationMatrix)
{
	UASSERT(transformationMatrix.cols == 4 &&
			transformationMatrix.rows == 3 &&
			(transformationMatrix.type() == CV_32FC1 || transformationMatrix.type() == CV_64FC1));
	if(transformationMatrix.type() == CV_32FC1)
	{
		data_ = transformationMatrix;
	}
	else
	{
		transformationMatrix.convertTo(data_, CV_32F);
	}
}

Transform::Transform(float x, float y, float z, float roll, float pitch, float yaw)
{
	Eigen::Affine3f t = pcl::getTransformation (x, y, z, roll, pitch, yaw);
	*this = fromEigen3f(t);
}

Transform::Transform(float x, float y, float z, float qx, float qy, float qz, float qw) :
		data_(cv::Mat::zeros(3,4,CV_32FC1))
{
	Eigen::Matrix3f rotation = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
	data()[0] = rotation(0,0);
	data()[1] = rotation(0,1);
	data()[2] = rotation(0,2);
	data()[3] = x;
	data()[4] = rotation(1,0);
	data()[5] = rotation(1,1);
	data()[6] = rotation(1,2);
	data()[7] = y;
	data()[8] = rotation(2,0);
	data()[9] = rotation(2,1);
	data()[10] = rotation(2,2);
	data()[11] = z;
}

Transform::Transform(float x, float y, float theta)
{
	Eigen::Affine3f t = pcl::getTransformation (x, y, 0, 0, 0, theta);
	*this = fromEigen3f(t);
}

Transform Transform::clone() const
{
	return Transform(data_.clone());
}

bool Transform::isNull() const
{
	return (data_.empty() ||
			(data()[0] == 0.0f &&
			data()[1] == 0.0f &&
			data()[2] == 0.0f &&
			data()[3] == 0.0f &&
			data()[4] == 0.0f &&
			data()[5] == 0.0f &&
			data()[6] == 0.0f &&
			data()[7] == 0.0f &&
			data()[8] == 0.0f &&
			data()[9] == 0.0f &&
			data()[10] == 0.0f &&
			data()[11] == 0.0f) ||
			uIsNan(data()[0]) ||
			uIsNan(data()[1]) ||
			uIsNan(data()[2]) ||
			uIsNan(data()[3]) ||
			uIsNan(data()[4]) ||
			uIsNan(data()[5]) ||
			uIsNan(data()[6]) ||
			uIsNan(data()[7]) ||
			uIsNan(data()[8]) ||
			uIsNan(data()[9]) ||
			uIsNan(data()[10]) ||
			uIsNan(data()[11]));
}

bool Transform::isIdentity() const
{
	return data()[0] == 1.0f &&
			data()[1] == 0.0f &&
			data()[2] == 0.0f &&
			data()[3] == 0.0f &&
			data()[4] == 0.0f &&
			data()[5] == 1.0f &&
			data()[6] == 0.0f &&
			data()[7] == 0.0f &&
			data()[8] == 0.0f &&
			data()[9] == 0.0f &&
			data()[10] == 1.0f &&
			data()[11] == 0.0f;
}

void Transform::setNull()
{
	*this = Transform();
}

void Transform::setIdentity()
{
	*this = getIdentity();
}

float Transform::theta() const
{
	float roll, pitch, yaw;
	this->getEulerAngles(roll, pitch, yaw);
	return yaw;
}

bool Transform::isInvertible() const
{
	bool invertible = false;
	Eigen::Matrix4f inverse;
	Eigen::Matrix4f::RealScalar det;
	toEigen4f().computeInverseAndDetWithCheck(inverse, det, invertible);
	return invertible;
}

Transform Transform::inverse() const
{
	bool invertible = false;
	Eigen::Matrix4f inverse;
	Eigen::Matrix4f::RealScalar det;
	toEigen4f().computeInverseAndDetWithCheck(inverse, det, invertible);
	UASSERT_MSG(invertible, uFormat("This transform is not invertible! %s \n"
			"[%f %f %f %f;\n"
			" %f %f %f %f;\n"
			" %f %f %f %f;\n"
			" 0 0 0 1]", prettyPrint().c_str(),
			r11(), r12(), r13(), o14(),
			r21(), r22(), r23(), o24(),
			r31(), r32(), r33(), o34()).c_str());
	return fromEigen4f(inverse);
}

Transform Transform::rotation() const
{
	return Transform(
			data()[0], data()[1], data()[2], 0,
			data()[4], data()[5], data()[6], 0,
			data()[8], data()[9], data()[10], 0);
}

Transform Transform::translation() const
{
	return Transform(1,0,0, data()[3],
					 0,1,0, data()[7],
					 0,0,1, data()[11]);
}

Transform Transform::to3DoF() const
{
	float x,y,z,roll,pitch,yaw;
	this->getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
	return Transform(x,y,0, 0,0,yaw);
}

Transform Transform::to4DoF() const
{
	float x,y,z,roll,pitch,yaw;
	this->getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
	return Transform(x,y,z, 0,0,yaw);
}

bool Transform::is3DoF() const
{
	return is4DoF() && z() == 0.0;
}

bool Transform::is4DoF() const
{
	return r13() == 0.0 &&
		   r23() == 0.0 &&
		   r31() == 0.0 &&
		   r32() == 0.0 &&
		   r33() == 0.0;
}

cv::Mat Transform::rotationMatrix() const
{
	return data_.colRange(0, 3).clone();
}

cv::Mat Transform::translationMatrix() const
{
	return data_.col(3).clone();
}

void Transform::getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const
{
	pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);
}

void Transform::getEulerAngles(float & roll, float & pitch, float & yaw) const
{
	float x,y,z;
	pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);
}

void Transform::getTranslation(float & x, float & y, float & z) const
{
	x = this->x();
	y = this->y();
	z = this->z();
}

float Transform::getAngle(float x, float y, float z) const
{
	Eigen::Vector3f vA(x,y,z);
	Eigen::Vector3f vB = this->toEigen3f().linear()*Eigen::Vector3f(1,0,0);
	return pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
}

float Transform::getNorm() const
{
	return uNorm(this->x(), this->y(), this->z());
}

float Transform::getNormSquared() const
{
	return uNormSquared(this->x(), this->y(), this->z());
}

float Transform::getDistance(const Transform & t) const
{
	return uNorm(this->x()-t.x(), this->y()-t.y(), this->z()-t.z());
}

float Transform::getDistanceSquared(const Transform & t) const
{
	return uNormSquared(this->x()-t.x(), this->y()-t.y(), this->z()-t.z());
}

Transform Transform::interpolate(float t, const Transform & other) const
{
	Eigen::Quaternionf qa=this->getQuaternionf();
	Eigen::Quaternionf qb=other.getQuaternionf();
	Eigen::Quaternionf qres = qa.slerp(t, qb);

	float x = this->x() + t*(other.x() - this->x());
	float y = this->y() + t*(other.y() - this->y());
	float z = this->z() + t*(other.z() - this->z());

	return Transform(x,y,z, qres.x(), qres.y(), qres.z(), qres.w());
}

void Transform::normalizeRotation()
{
	if(!this->isNull())
	{
		Eigen::Affine3f m = toEigen3f();
		m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();
		*this = fromEigen3f(m);
	}
}

std::string Transform::prettyPrint() const
{
	if(this->isNull())
	{
		return uFormat("xyz=[null] rpy=[null]");
	}
	else
	{
		float x,y,z,roll,pitch,yaw;
		getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		return uFormat("xyz=%f,%f,%f rpy=%f,%f,%f", x,y,z, roll,pitch,yaw);
	}
}

Transform Transform::operator*(const Transform & t) const
{
	Eigen::Affine3f m = Eigen::Affine3f(toEigen4f()*t.toEigen4f());
	// make sure rotation is always normalized!
	m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();
	return fromEigen3f(m);
}

Transform & Transform::operator*=(const Transform & t)
{
	*this = *this * t;
	return *this;
}

bool Transform::operator==(const Transform & t) const
{
	return memcmp(data_.data, t.data_.data, data_.total() * sizeof(float)) == 0;
}

bool Transform::operator!=(const Transform & t) const
{
	return !(*this == t);
}

std::ostream& operator<<(std::ostream& os, const Transform& s)
{
	os << "[" << s.data()[0] << ", " << s.data()[1] << ", " << s.data()[2] << ", " << s.data()[3] << ";" << std::endl
	   << " " << s.data()[4] << ", " << s.data()[5] << ", " << s.data()[6] << ", " << s.data()[7] << ";" << std::endl
	   << " " << s.data()[8] << ", " << s.data()[9] << ", " << s.data()[10]<< ", " << s.data()[11] << "]";
	return os;
}

Eigen::Matrix4f Transform::toEigen4f() const
{
	Eigen::Matrix4f m;
	m << data()[0], data()[1], data()[2], data()[3],
		 data()[4], data()[5], data()[6], data()[7],
		 data()[8], data()[9], data()[10], data()[11],
		 0,0,0,1;
	return m;
}
Eigen::Matrix4d Transform::toEigen4d() const
{
	Eigen::Matrix4d m;
	m << data()[0], data()[1], data()[2], data()[3],
		 data()[4], data()[5], data()[6], data()[7],
		 data()[8], data()[9], data()[10], data()[11],
		 0,0,0,1;
	return m;
}

Eigen::Affine3f Transform::toEigen3f() const
{
	return Eigen::Affine3f(toEigen4f());
}

Eigen::Affine3d Transform::toEigen3d() const
{
	return Eigen::Affine3d(toEigen4d());
}

Eigen::Quaternionf Transform::getQuaternionf() const
{
	return Eigen::Quaternionf(this->toEigen3f().linear()).normalized();
}

Eigen::Quaterniond Transform::getQuaterniond() const
{
	return Eigen::Quaterniond(this->toEigen3d().linear()).normalized();
}

Transform Transform::getIdentity()
{
	return Transform(1,0,0,0, 0,1,0,0, 0,0,1,0);
}

Transform Transform::fromEigen4f(const Eigen::Matrix4f & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
Transform Transform::fromEigen4d(const Eigen::Matrix4d & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}

Transform Transform::fromEigen3f(const Eigen::Affine3f & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
Transform Transform::fromEigen3d(const Eigen::Affine3d & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}

Transform Transform::fromEigen3f(const Eigen::Isometry3f & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
Transform Transform::fromEigen3d(const Eigen::Isometry3d & matrix)
{
	return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
					 matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
					 matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}

/**
 * Format (3 values): x y z
 * Format (6 values): x y z roll pitch yaw
 * Format (7 values): x y z qx qy qz qw
 * Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33
 * Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
 */
Transform Transform::fromString(const std::string & string)
{
	std::list<std::string> list = uSplit(string, ' ');
	UASSERT_MSG(list.empty() || list.size() == 3 || list.size() == 6 || list.size() == 7 || list.size() == 9 || list.size() == 12,
			uFormat("Cannot parse \"%s\"", string.c_str()).c_str());

	std::vector<float> numbers(list.size());
	int i = 0;
	for(std::list<std::string>::iterator iter=list.begin(); iter!=list.end(); ++iter)
	{
		numbers[i++] = uStr2Float(*iter);
	}

	Transform t;
	if(numbers.size() == 3)
	{
		t = Transform(numbers[0], numbers[1], numbers[2]);
	}
	else if(numbers.size() == 6)
	{
		t = Transform(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);
	}
	else if(numbers.size() == 7)
	{

		t = Transform(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);
	}
	else if(numbers.size() == 9)
	{
		t = Transform(numbers[0], numbers[1], numbers[2], 0,
					  numbers[3], numbers[4], numbers[5], 0,
					  numbers[6], numbers[7], numbers[8], 0);
	}
	else if(numbers.size() == 12)
	{
		t = Transform(numbers[0], numbers[1], numbers[2], numbers[3],
					  numbers[4], numbers[5], numbers[6], numbers[7],
					  numbers[8], numbers[9], numbers[10], numbers[11]);
	}
	return t;
}

/**
 * Format (3 values): x y z
 * Format (6 values): x y z roll pitch yaw
 * Format (7 values): x y z qx qy qz qw
 * Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33
 * Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
 */
bool Transform::canParseString(const std::string & string)
{
	std::list<std::string> list = uSplit(string, ' ');
	return list.size() == 0 || list.size() == 3 || list.size() == 6 || list.size() == 7 || list.size() == 9 || list.size() == 12;
}

Transform Transform::getTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp)
{
	UASSERT(!tfBuffer.empty());
	std::map<double, Transform>::const_iterator imuIterB = tfBuffer.lower_bound(stamp);
	std::map<double, Transform>::const_iterator imuIterA = imuIterB;
	if(imuIterA != tfBuffer.begin())
	{
		imuIterA = --imuIterA;
	}
	if(imuIterB == tfBuffer.end())
	{
		imuIterB = --imuIterB;
	}
	Transform imuT;
	if(imuIterB->first == stamp)
	{
		imuT = imuIterB->second;
	}
	else if(imuIterA != imuIterB)
	{
		//interpolate:
		imuT = imuIterA->second.interpolate((stamp-imuIterA->first) / (imuIterB->first-imuIterA->first), imuIterB->second);
	}
	else if(stamp > imuIterB->first)
	{
		UWARN("No transform found for stamp %f! Latest is %f", stamp, imuIterB->first);
	}
	else
	{
		UWARN("No transform found for stamp %f! Earliest is %f", stamp, imuIterA->first);
	}
	return imuT;
}

Transform Transform::getClosestTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp,
				double * stampDiff)
{
	UASSERT(!tfBuffer.empty());
	std::map<double, Transform>::const_iterator imuIterB = tfBuffer.lower_bound(stamp);
	std::map<double, Transform>::const_iterator imuIterA = imuIterB;
	if(imuIterA != tfBuffer.begin())
	{
		imuIterA = --imuIterA;
	}
	if(imuIterB == tfBuffer.end())
	{
		imuIterB = --imuIterB;
	}
	Transform imuT;
	if(imuIterB->first == stamp || imuIterA == imuIterB)
	{
		imuT = imuIterB->second;
		if(stampDiff)
		{
			*stampDiff = fabs(imuIterB->first - stamp);
		}
	}
	else if(imuIterA != imuIterB)
	{
		//interpolate:
		imuT = imuIterA->second.interpolate((stamp-imuIterA->first) / (imuIterB->first-imuIterA->first), imuIterB->second);
		if(stampDiff)
		{
			*stampDiff = 0.0;
		}
	}
	return imuT;
}


}
