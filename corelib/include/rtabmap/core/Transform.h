/*
 * Transform.h
 *
 *  Created on: 2013-08-30
 *      Author: Mathieu
 */

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <rtabmap/core/RtabmapExp.h>
#include <vector>
#include <string>

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
	// x,y,z, roll,pitch,yaw
	Transform(float x, float y, float z, float roll, float pitch, float yaw);

	float & operator[](int index) {return data_[index];}
	const float & operator[](int index) const {return data_[index];}

	bool isNull() const;
	bool isIdentity() const;

	void setNull();
	void setIdentity();

	const float * data() const {return data_.data();}
	float * data() {return data_.data();}
	int size() const {return data_.size();}

	float & x() {return data_[3];}
	float & y() {return data_[7];}
	float & z() {return data_[11];}
	const float & x() const {return data_[3];}
	const float & y() const {return data_[7];}
	const float & z() const {return data_[11];}

	Transform inverse() const;
	Transform rotation() const;
	Transform translation() const;

	void getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const;
	void getTranslation(float & x, float & y, float & z) const;
	float getNorm() const;
	float getNormSquared() const;
	std::string prettyPrint() const;

	Transform operator*(const Transform & t) const;
	Transform & operator*=(const Transform & t);
	bool operator==(const Transform & t) const;
	bool operator!=(const Transform & t) const;

	static Transform getIdentity();

private:
	std::vector<float> data_;
};

RTABMAP_EXP std::ostream& operator<<(std::ostream& os, const Transform& s);

}

#endif /* TRANSFORM_H_ */
