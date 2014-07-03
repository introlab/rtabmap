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

	float r11() const {return data_[0];}
	float r12() const {return data_[1];}
	float r13() const {return data_[2];}
	float r21() const {return data_[4];}
	float r22() const {return data_[5];}
	float r23() const {return data_[6];}
	float r31() const {return data_[8];}
	float r32() const {return data_[9];}
	float r33() const {return data_[10];}

	float o14() const {return data_[3];}
	float o24() const {return data_[7];}
	float o34() const {return data_[11];}

	float & operator[](int index) {return data_[index];}
	const float & operator[](int index) const {return data_[index];}

	bool isNull() const;
	bool isIdentity() const;

	void setNull();
	void setIdentity();

	const float * data() const {return data_.data();}
	float * data() {return data_.data();}
	int size() const {return (int)data_.size();}

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
