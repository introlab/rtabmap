
/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MadgwickFilter.h"
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Reciprocal_of_the_square_root
static inline float invSqrt(float x)
{
  float xhalf = 0.5f * x;
  union
  {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  /* The next line can be repeated any number of times to increase accuracy */
  u.x = u.x * (1.5f - xhalf * u.x * u.x);
  return u.x;
}

template<typename T>
static inline void normalizeVectorOpt(T& vx, T& vy, T& vz)
{
  T recipNorm = invSqrt (vx * vx + vy * vy + vz * vz);
  vx *= recipNorm;
  vy *= recipNorm;
  vz *= recipNorm;
}

template<typename T>
static inline void normalizeQuaternion(T& q0, T& q1, T& q2, T& q3)
{
  T recipNorm = invSqrt (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

static inline void rotateAndScaleVector(
    float q0, float q1, float q2, float q3,
    float _2dx, float _2dy, float _2dz,
    float& rx, float& ry, float& rz) {

  // result is half as long as input
  rx = _2dx * (0.5f - q2 * q2 - q3 * q3)
     + _2dy * (q0 * q3 + q1 * q2)
     + _2dz * (q1 * q3 - q0 * q2);
  ry = _2dx * (q1 * q2 - q0 * q3)
     + _2dy * (0.5f - q1 * q1 - q3 * q3)
     + _2dz * (q0 * q1 + q2 * q3);
  rz = _2dx * (q0 * q2 + q1 * q3)
     + _2dy * (q2 * q3 - q0 * q1)
     + _2dz * (0.5f - q1 * q1 - q2 * q2);
}

static inline void orientationChangeFromGyro(
    float q0, float q1, float q2, float q3,
    float gx, float gy, float gz,
    float& qDot1, float& qDot2, float& qDot3, float& qDot4)
{
  // Rate of change of quaternion from gyroscope
  // See EQ 12
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
}

static inline void addGradientDescentStep(
    float q0, float q1, float q2, float q3,
    float _2dx, float _2dy, float _2dz,
    float mx, float my, float mz,
    float& s0, float& s1, float& s2, float& s3)
{
  float f0, f1, f2;

  // Gradient decent algorithm corrective step
  // EQ 15, 21
  rotateAndScaleVector(q0,q1,q2,q3, _2dx, _2dy, _2dz, f0, f1, f2);

  f0 -= mx;
  f1 -= my;
  f2 -= mz;


  // EQ 22, 34
  // Jt * f
  s0 += (_2dy * q3 - _2dz * q2) * f0
      + (-_2dx * q3 + _2dz * q1) * f1
      + (_2dx * q2 - _2dy * q1) * f2;
  s1 += (_2dy * q2 + _2dz * q3) * f0
      + (_2dx * q2 - 2.0f * _2dy * q1 + _2dz * q0) * f1
      + (_2dx * q3 - _2dy * q0 - 2.0f * _2dz * q1) * f2;
  s2 += (-2.0f * _2dx * q2 + _2dy * q1 - _2dz * q0) * f0
      + (_2dx * q1 + _2dz * q3) * f1
      + (_2dx * q0 + _2dy * q3 - 2.0f * _2dz * q2) * f2;
  s3 += (-2.0f * _2dx * q3 + _2dy * q0 + _2dz * q1) * f0
      + (-_2dx * q0 - 2.0f * _2dy * q3 + _2dz * q2) * f1
      + (_2dx * q1 + _2dy * q2) * f2;
}

template<typename T>
static inline void crossProduct(
      T ax, T ay, T az,
      T bx, T by, T bz,
      T& rx, T& ry, T& rz) {
  rx = ay*bz - az*by;
  ry = az*bx - ax*bz;
  rz = ax*by - ay*bx;
}

template<typename T>
static inline T normalizeVector(T& vx, T& vy, T& vz) {
  T norm = sqrt(vx*vx + vy*vy + vz*vz);
  T inv = 1.0 / norm;
  vx *= inv;
  vy *= inv;
  vz *= inv;
  return norm;

}

static inline bool computeOrientation(
  Eigen::Vector3f A,
  Eigen::Vector3f E,
  Eigen::Quaternionf& orientation) {

  float Hx, Hy, Hz;
  float Mx, My, Mz;
  float normH;

  // A: pointing up
  float Ax = A[0], Ay = A[1], Az = A[2];

  // E: pointing down/north
  float Ex = E[0], Ey = E[1], Ez = E[2];

  // H: vector horizontal, pointing east
  // H = E x A
  crossProduct(Ex, Ey, Ez, Ax, Ay, Az, Hx, Hy, Hz);

  // normalize H
  normH = normalizeVector(Hx, Hy, Hz);
  if (normH < 1E-7) {
	// device is close to free fall (or in space?), or close to
	// magnetic north pole.
	// mag in T => Threshold 1E-7, typical values are  > 1E-5.
	return false;
  }

  // normalize A
  normalizeVector(Ax, Ay, Az);

  // M: vector horizontal, pointing north
  // M = A x H
  crossProduct(Ax, Ay, Az, Hx, Hy, Hz, Mx, My, Mz);

  // Create matrix for basis transformation
  Eigen::Matrix3f R;

	//case WorldFrame::ENU:
	  // vector space world W:
	  // Basis: bwx (1,0,0) east, bwy (0,1,0) north, bwz (0,0,1) up
	  // vector space local L:
	  // Basis: H, M , A
	  // W(1,0,0) => L(H)
	  // W(0,1,0) => L(M)
	  // W(0,0,1) => L(A)

	  // R: Transform Matrix local => world equals basis of L, because basis of W is I
	  R(0,0) = Hx;     R(0,1) = Mx;     R(0,2) = Ax;
	  R(1,0) = Hy;     R(1,1) = My;     R(1,2) = Ay;
	  R(2,0) = Hz;     R(2,1) = Mz;     R(2,2) = Az;

  // Matrix.getRotation assumes vector rotation, but we're using
  // coordinate systems. Thus negate rotation angle (inverse).
  Eigen::Quaternionf q(R);
  orientation = q.inverse();
  return true;
}


static inline bool computeOrientation(
  Eigen::Vector3f A,
  Eigen::Quaternionf& orientation) {

  // This implementation could be optimized regarding speed.

  // magnetic Field E must not be parallel to A,
  // choose an arbitrary orthogonal vector
  Eigen::Vector3f E;
  if (fabs(A[2]) > 0.1) {
  	  E[0] = 0.0;
  	  E[1] = A[2];
  	  E[2] = -A[1];
  } else if (fabs(A[0]) > 0.1 || fabs(A[1]) > 0.1) {
	  E[0] = A[1];
	  E[1] = A[0];
	  E[2] = 0.0;
  } else {
	  // free fall
	  return false;
  }

  return computeOrientation(A, E, orientation);
}

MadgwickFilter::MadgwickFilter(const ParametersMap & parameters) :
	IMUFilter(parameters),
	q0(1.0), q1(0.0), q2(0.0), q3(0.0),
	w_bx_(0.0), w_by_(0.0), w_bz_(0.0),
	initialized_(false),
	gain_ (Parameters::defaultImuFilterMadgwickGain()),
	zeta_ (Parameters::defaultImuFilterMadgwickZeta())
{
	parseParameters(parameters);
}

void MadgwickFilter::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kImuFilterMadgwickGain(), gain_);
	Parameters::parse(parameters, Parameters::kImuFilterMadgwickZeta(), zeta_);
}

/**
 * Gain of the filter. Higher values lead to faster convergence but
 * more noise. Lower values lead to slower convergence but smoother signal. [0.0, 1.0]
 */
void MadgwickFilter::setAlgorithmGain(double gain)
{
	gain_ = gain;
}

/**
 * Gyro drift gain (approx. rad/s). [-1.0, 1.0]
 */
void MadgwickFilter::setDriftBiasGain(double zeta)
{
	zeta_ = zeta;
}

void MadgwickFilter::getOrientation(double & qx, double & qy, double & qz, double & qw) const
{
	qx = this->q1;
	qy = this->q2;
	qz = this->q3;
	qw = this->q0;

	// perform precise normalization of the output, using 1/sqrt()
	// instead of the fast invSqrt() approximation. Without this,
	// TF2 complains that the quaternion is not normalized.
	double recipNorm = 1 / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
	qx *= recipNorm;
	qy *= recipNorm;
	qz *= recipNorm;
	qw *= recipNorm;
}

void MadgwickFilter::reset(double qx, double qy, double qz, double qw)
{
	this->q0 = qw;
	this->q1 = qx;
	this->q2 = qy;
	this->q3 = qz;

	w_bx_ = 0;
	w_by_ = 0;
	w_bz_ = 0;

	initialized_ = false;
}

void MadgwickFilter::updateImpl(
		double gx, double gy, double gz,
		double ax, double ay, double az,
		double dt)
{
  if(!initialized_)
  {
	  Eigen::Quaternionf orientation;
	  Eigen::Vector3f A;
	  A[0] = ax;
	  A[1] = ay;
	  A[2] = az;
	if(computeOrientation(A,orientation))
	{
		reset(orientation.x(), orientation.y(), orientation.z(), orientation.w());
		initialized_ = true;
	}
	return;
  }

  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  // Rate of change of quaternion from gyroscope
  orientationChangeFromGyro (q0, q1, q2, q3, gx, gy, gz, qDot1, qDot2, qDot3, qDot4);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
	// Normalise accelerometer measurement
	  normalizeVectorOpt(ax, ay, az);

	// Gradient decent algorithm corrective step
	s0 = 0.0;  s1 = 0.0;  s2 = 0.0;  s3 = 0.0;
    //case WorldFrame::ENU:
	// Gravity: [0, 0, 1]
	addGradientDescentStep(q0, q1, q2, q3, 0.0, 0.0, 2.0, ax, ay, az, s0, s1, s2, s3);

	normalizeQuaternion(s0, s1, s2, s3);

	// Apply feedback step
	qDot1 -= gain_ * s0;
	qDot2 -= gain_ * s1;
	qDot3 -= gain_ * s2;
	qDot4 -= gain_ * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  if(dt <= 0.0)
  {
	  UWARN("dt=%f <=0.0, orientation will not be updated!", dt);
	  return;
  }
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  //printf("%fs %f %f %f %f\n", dt, q0, q1, q2, q3);

  // Normalise quaternion
  normalizeQuaternion (q0, q1, q2, q3);
}

}
