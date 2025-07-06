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

#include <rtabmap/core/rtabmap_core_export.h>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace rtabmap {

/**
 * @class Transform
 * @brief Represents a 3D rigid body transformation (rotation + translation).
 *
 * This class provides an abstraction over 3D transformations using a 3x4 matrix representation,
 * with utilities for conversion to/from Eigen and OpenCV formats, interpolation, inversion,
 * DoF reduction, and distance calculations. It is fundamental to pose estimation and motion handling
 * within the RTAB-Map framework.
 *
 * The underlying data is stored in a `cv::Mat` (3x4, CV_32FC1), representing a rotation matrix and translation vector.
 */
class RTABMAP_CORE_EXPORT Transform
{
public:

	/**
     * @brief Default constructor. Initializes to a null (all zeros) transform.
     */
	Transform();
	/**
     * @brief Constructor from rotation matrix elements and translation components.
     * @param r11...r33 Rotation matrix components.
     * @param o14,o24,o34 Translation vector components.
     */
	Transform(float r11, float r12, float r13, float o14,
		      float r21, float r22, float r23, float o24,
			  float r31, float r32, float r33, float o34);
	/**
     * @brief Constructs a Transform from a 3x4 OpenCV matrix.
     * @param transformationMatrix A 3x4 CV_32FC1 matrix.
     */
	Transform(const cv::Mat & transformationMatrix);
	/**
     * @brief Constructs a Transform from position and Euler angles (in radians).
     * @param x, y, z     Translation components.
     * @param roll, pitch, yaw Euler rotation angles (ZYX-convention).
     */
	Transform(float x, float y, float z, float roll, float pitch, float yaw);
	/**
     * @brief Constructs a Transform from position and quaternion.
     * @param x, y, z Translation components.
     * @param qx, qy, qz, qw Quaternion rotation components.
     */
	Transform(float x, float y, float z, float qx, float qy, float qz, float qw);
	/**
     * @brief Constructs a 2D Transform (x, y, theta).
     */
	Transform(float x, float y, float theta);

	/**
     * @brief Returns a deep copy of the transform.
     */
	Transform clone() const;

	// --- Accessors (rotation matrix elements, translation) ---
	float r11() const {return data()[0];} //!< Rotation matrix element at row 1, col 1.
	float r12() const {return data()[1];}
	float r13() const {return data()[2];}
	float r21() const {return data()[4];}
	float r22() const {return data()[5];}
	float r23() const {return data()[6];}
	float r31() const {return data()[8];}
	float r32() const {return data()[9];}
	float r33() const {return data()[10];}

	float o14() const {return data()[3];} //!< Translation x
	float o24() const {return data()[7];} //!< Translation y
	float o34() const {return data()[11];} //!< Translation z

	float & operator[](int index) {return data()[index];}
	const float & operator[](int index) const {return data()[index];}
	float & operator()(int row, int col) {return data()[row*4 + col];}
	const float & operator()(int row, int col) const {return data()[row*4 + col];}

	/**
     * @brief Checks whether the transform is null (all zeros).
     */
	bool isNull() const;
	/**
     * @brief Checks whether the transform is identity.
     */
	bool isIdentity() const;

	/**
     * @brief Sets the transform to null (zero matrix).
     */
	void setNull();
	/**
     * @brief Sets the transform to the identity transform.
     */
	void setIdentity();

	/**
     * @brief Returns the internal OpenCV matrix (3x4).
     */
	const cv::Mat & dataMatrix() const {return data_;}
	/**
     * @brief Returns a pointer to the raw data (12 floats).
     */
	const float * data() const {return (const float *)data_.data;}
	float * data() {return (float *)data_.data;}

	/**
     * @brief Returns the number of float elements (always 12).
     */
	int size() const {return 12;}

	/// Translation getters/setters
	float & x() {return data()[3];} //!< Translation x
	float & y() {return data()[7];} //!< Translation y
	float & z() {return data()[11];} //!< Translation z
	const float & x() const {return data()[3];}
	const float & y() const {return data()[7];}
	const float & z() const {return data()[11];}

	/**
     * @brief Returns 2D orientation (theta) in radians.
     */
	float theta() const;

	/**
     * @brief Returns whether the transform is invertible.
     */
	bool isInvertible() const;
	/**
     * @brief Returns the inverse of the transform.
     */
	Transform inverse() const;
	/**
     * @brief Returns only the rotation component.
     */
	Transform rotation() const;
	/**
     * @brief Returns only the translation component.
     */
	Transform translation() const;
	/**
     * @brief Converts to 3 DoF (x, y, theta).
     */
	Transform to3DoF() const;
	/**
     * @brief Converts to 4 DoF (x, y, z, yaw).
     */
	Transform to4DoF() const;
	/**
     * @brief Checks if the transform is 3 DoF (no pitch/roll).
     */
	bool is3DoF() const;
	/**
     * @brief Checks if the transform is 4 DoF (no pitch).
     */
	bool is4DoF() const;

	/**
     * @brief Returns the 3x3 rotation matrix (cv::Mat).
     */
	cv::Mat rotationMatrix() const;
	/**
     * @brief Returns the 3x1 translation matrix (cv::Mat).
     */
	cv::Mat translationMatrix() const;

	/**
     * @brief Extracts translation and Euler angles (radians).
     */
	void getTranslationAndEulerAngles(float & x, float & y, float & z, float & roll, float & pitch, float & yaw) const;
	/**
     * @brief Extracts Euler angles (roll, pitch, yaw).
     */
	void getEulerAngles(float & roll, float & pitch, float & yaw) const;
	/**
     * @brief Extracts translation only.
     */
	void getTranslation(float & x, float & y, float & z) const;
	/**
     * @brief Returns angular difference (in radians) with another transform.
     */
	float getAngle(const Transform & t) const;
	/**
     * @brief Returns the Euclidean norm of the translation vector.
     */
	float getNorm() const;
	/**
     * @brief Returns the squared norm of the translation vector.
     */
	float getNormSquared() const;
	/**
     * @brief Returns the Euclidean distance to another transform.
     */
	float getDistance(const Transform & t) const;
	/**
     * @brief Returns the squared distance to another transform.
     */
	float getDistanceSquared(const Transform & t) const;
	/**
     * @brief Interpolates between this and another transform.
     * @param t Interpolation factor [0, 1].
     * @param other Target transform.
     */
	Transform interpolate(float t, const Transform & other) const;
	/**
     * @brief Normalizes the rotation matrix.
     */
	void normalizeRotation();
	/**
     * @brief Returns a string representation of the transform.
     */
	std::string prettyPrint() const;

	/// Operator overloads
	Transform operator*(const Transform & t) const;
	Transform & operator*=(const Transform & t);
	bool operator==(const Transform & t) const;
	bool operator!=(const Transform & t) const;

	// --- Eigen conversions ---
	Eigen::Matrix4f toEigen4f() const;
	Eigen::Matrix4d toEigen4d() const;
	Eigen::Affine3f toEigen3f() const;
	Eigen::Affine3d toEigen3d() const;

	Eigen::Quaternionf getQuaternionf() const;
	Eigen::Quaterniond getQuaterniond() const;

public:
	// --- Static helpers ---

    /**
     * @brief Returns identity transform.
     */
	static Transform getIdentity();

	/// Converts from Eigen representations
	static Transform fromEigen4f(const Eigen::Matrix4f & matrix);
	static Transform fromEigen4d(const Eigen::Matrix4d & matrix);
	static Transform fromEigen3f(const Eigen::Affine3f & matrix);
	static Transform fromEigen3d(const Eigen::Affine3d & matrix);
	static Transform fromEigen3f(const Eigen::Isometry3f & matrix);
	static Transform fromEigen3d(const Eigen::Isometry3d & matrix);
	static Transform fromEigen3f(const Eigen::Matrix<float, 3, 4> & matrix);
	static Transform fromEigen3d(const Eigen::Matrix<double, 3, 4> & matrix);

	/**
	 * @brief Returns the transform from RTAB-Map to OpenGL coordinate system.
	 * @note Coordinate systems:
	 * - OpenGL: x → right, y → up, z → out of the screen (toward viewer).
	 * - RTAB-Map: x → forward, y → left, z → up.
	 */
	static Transform opengl_T_rtabmap() {return Transform(
			 0.0f, -1.0f, 0.0f, 0.0f,
			 0.0f,  0.0f, 1.0f, 0.0f,
			-1.0f,  0.0f, 0.0f, 0.0f);}
	/**
	 * @brief Returns the transform from OpenGL to RTAB-Map coordinate system.
	 * @note Coordinate systems:
	 * - OpenGL: x → right, y → up, z → out of the screen (toward viewer).
	 * - RTAB-Map: x → forward, y → left, z → up.
	 */
	static Transform rtabmap_T_opengl() {return Transform(
			 0.0f,  0.0f,-1.0f, 0.0f,
			-1.0f,  0.0f, 0.0f, 0.0f,
			 0.0f,  1.0f, 0.0f, 0.0f);}

	/**
     * @brief Parses a transform from a string representation.
     * Supported formats:
     * - (3 values) "x y z"
     * - (6 values) "x y z roll pitch yaw" (ZYX-Euler convention)
     * - (7 values) "x y z qx qy qz qw"
     * - (9 values, 3x3 rotation) "r11 r12 r13 r21 r22 r23 r31 r32 r33"
	 * - (12 values, 3x4 rotation) "r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz"
     */
	static Transform fromString(const std::string & string);
	/**
     * @brief Checks if a string can be parsed into a transform.
     */
	static bool canParseString(const std::string & string);

	/**
     * @brief Retrieves the transform to a given timestamp.
     * @param tfBuffer Buffer of timestamped transforms.
     * @param stamp Requested timestamp.
	 * @return transform at the requested timestamp, interpolated if it doesn't fall on a transform 
	 *         with the exact same timestamp in the buffer. If the timestamp is older
	 *         than the oldest timestamp or newer than the latest timestamp in the buffer, a 
	 *         null transform is returned and a warning message is generated.
     */
	static Transform getTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp);
	/**
     * @deprecated Use getTransform() instead.
     */
	RTABMAP_DEPRECATED static Transform getClosestTransform(
				const std::map<double, Transform> & tfBuffer,
				const double & stamp,
				double * stampDiff);

private:
	cv::Mat data_; ///< 3x4 float matrix (rotation + translation)
};

/**
 * @brief Output stream operator for Transform.
 */
RTABMAP_CORE_EXPORT std::ostream& operator<<(std::ostream& os, const Transform& s);

/**
 * @class TransformStamped
 * @brief Associates a transform with a timestamp.
 */
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
