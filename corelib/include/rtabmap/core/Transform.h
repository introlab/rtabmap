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

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <rtabmap/core/RtabmapExp.h>
#include <vector>
#include <string>
#include <map>
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
	// x,y,z, qx,qy,qz,qw
	Transform(float x, float y, float z, float qx, float qy, float qz, float qw);
	// x,y, theta
	Transform(float x, float y, float theta);

	Transform clone() const;

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
	float & operator()(int row, int col) {return data()[row*4 + col];}
	const float & operator()(int row, int col) const {return data()[row*4 + col];}

	bool isNull() const;
	bool isIdentity() const;

	void setNull();
	void setIdentity();

	const cv::Mat & dataMatrix() const {return data_;}
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

	bool isInvertible() const;
	Transform inverse() const;
	Transform rotation() const;
	Transform translation() const;
	Transform to3DoF() const;
	Transform to4DoF() const;
	bool is3DoF() const;
	bool is4DoF() const;

	cv::Mat rotationMatrix() const;
	cv::Mat translationMatrix() const;

	void getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const;
	void getEulerAngles(float & roll, float & pitch, float & yaw) const;
	void getTranslation(float & x, float & y, float & z) const;
	float getAngle(float x=1.0f, float y=0.0f, float z=0.0f) const;
	float getNorm() const;
	float getNormSquared() const;
	float getDistance(const Transform & t) const;
	float getDistanceSquared(const Transform & t) const;
	Transform interpolate(float t, const Transform & other) const;
	void normalizeRotation();
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

	static Transform opengl_T_rtabmap() {return Transform(
			 0.0f, -1.0f, 0.0f, 0.0f,
			 0.0f,  0.0f, 1.0f, 0.0f,
			-1.0f,  0.0f, 0.0f, 0.0f);}
	static Transform rtabmap_T_opengl() {return Transform(
			 0.0f,  0.0f,-1.0f, 0.0f,
			-1.0f,  0.0f, 0.0f, 0.0f,
			 0.0f,  1.0f, 0.0f, 0.0f);}

	/**
	 * Format (3 values): x y z
	 * Format (6 values): x y z roll pitch yaw
	 * Format (7 values): x y z qx qy qz qw
	 * Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33
	 * Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
	 */
	static Transform fromString(const std::string & string);
	static bool canParseString(const std::string & string);

	static Transform getTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp);
	RTABMAP_DEPRECATED(static Transform getClosestTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp,
				double * stampDiff), "Use Transform::getTransform() instead to get always accurate transforms.");

private:
	cv::Mat data_;
};

RTABMAP_EXP std::ostream& operator<<(std::ostream& os, const Transform& s);

class TransformStamped
{
public:
	TransformStamped(const Transform & transform, const double & stamp) :
		transform_(transform),
		stamp_(stamp)
	{}
	const Transform & transform() const {return transform_;}
	const double & stamp() const {return stamp_;}

private:
	Transform transform_;
	double stamp_;
};

}

#endif /* TRANSFORM_H_ */
