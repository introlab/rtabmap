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

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <rtabmap/core/RtabmapExp.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace rtabmap {

class RTABMAP_EXP Transform
{
public:

	// Zero by default
	Transform();
	// rotation matrix r## and origin o##
	Transform(float r11, float r12, float r13, float o14,
		      float r21, float r22, float r23, float o24,
			  float r31, float r32, float r33, float o34);
	// should have 3 rows, 4 cols and type CV_32FC1
	Transform(const cv::Mat & transformationMatrix);
	// x,y,z, roll,pitch,yaw
	Transform(float x, float y, float z, float roll, float pitch, float yaw);
	// x,y, theta
	Transform(float x, float y, float theta);

	float r11() const {return data()[0];}
	float r12() const {return data()[1];}
	float r13() const {return data()[2];}
	float r21() const {return data()[4];}
	float r22() const {return data()[5];}
	float r23() const {return data()[6];}
	float r31() const {return data()[8];}
	float r32() const {return data()[9];}
	float r33() const {return data()[10];}

	float o14() const {return data()[3];}
	float o24() const {return data()[7];}
	float o34() const {return data()[11];}

	float & operator[](int index) {return data()[index];}
	const float & operator[](int index) const {return data()[index];}

	bool isNull() const;
	bool isIdentity() const;

	void setNull();
	void setIdentity();

	const float * data() const {return (const float *)data_.data;}
	float * data() {return (float *)data_.data;}
	int size() const {return 12;}

	float & x() {return data()[3];}
	float & y() {return data()[7];}
	float & z() {return data()[11];}
	const float & x() const {return data()[3];}
	const float & y() const {return data()[7];}
	const float & z() const {return data()[11];}

	float theta() const;

	Transform inverse() const;
	Transform rotation() const;
	Transform translation() const;

	void getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const;
	void getEulerAngles(float & roll, float & pitch, float & yaw) const;
	void getTranslation(float & x, float & y, float & z) const;
	float getNorm() const;
	float getNormSquared() const;
	float getDistance(const Transform & t) const;
	float getDistanceSquared(const Transform & t) const;
	std::string prettyPrint() const;

	Transform operator*(const Transform & t) const;
	Transform & operator*=(const Transform & t);
	bool operator==(const Transform & t) const;
	bool operator!=(const Transform & t) const;

	Eigen::Matrix4f toEigen4f() const;
	Eigen::Matrix4d toEigen4d() const;
	Eigen::Affine3f toEigen3f() const;
	Eigen::Affine3d toEigen3d() const;

	Eigen::Quaternionf getQuaternionf() const;
	Eigen::Quaterniond getQuaterniond() const;

public:
	static Transform getIdentity();
	static Transform fromEigen4f(const Eigen::Matrix4f & matrix);
	static Transform fromEigen4d(const Eigen::Matrix4d & matrix);
	static Transform fromEigen3f(const Eigen::Affine3f & matrix);
	static Transform fromEigen3d(const Eigen::Affine3d & matrix);
	static Transform fromEigen3f(const Eigen::Isometry3f & matrix);
	static Transform fromEigen3d(const Eigen::Isometry3d & matrix);

private:
	cv::Mat data_;
};

RTABMAP_EXP std::ostream& operator<<(std::ostream& os, const Transform& s);

}

#endif /* TRANSFORM_H_ */
