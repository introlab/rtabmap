/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ComplementaryFilter.h"
#include <rtabmap/utilite/ULogger.h>
#include <cstdio>
#include <cmath>
#include <iostream>

namespace rtabmap {

const double ComplementaryFilter::kGravity = 9.81;
const double ComplementaryFilter::gamma_ = 0.01;
// Bias estimation steady state thresholds
const double ComplementaryFilter::kAngularVelocityThreshold = 0.2;
const double ComplementaryFilter::kAccelerationThreshold = 0.1;
const double ComplementaryFilter::kDeltaAngularVelocityThreshold = 0.01;

ComplementaryFilter::ComplementaryFilter(const ParametersMap & parameters):
	IMUFilter(parameters),
    gain_acc_(Parameters::defaultImuFilterComplementaryGainAcc()),
    bias_alpha_(Parameters::defaultImuFilterComplementaryBiasAlpha()),
    do_bias_estimation_(Parameters::defaultImuFilterComplementaryDoBiasEstimation()),
    do_adaptive_gain_(Parameters::defaultImuFilterComplementaryDoAdpativeGain()),
    initialized_(false),
    steady_state_(false),
    q0_(1), q1_(0), q2_(0), q3_(0),
    wx_prev_(0), wy_prev_(0), wz_prev_(0),
    wx_bias_(0), wy_bias_(0), wz_bias_(0)
{
	parseParameters(parameters);
}

void ComplementaryFilter::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kImuFilterComplementaryGainAcc(), gain_acc_);
	Parameters::parse(parameters, Parameters::kImuFilterComplementaryBiasAlpha(), bias_alpha_);
	Parameters::parse(parameters, Parameters::kImuFilterComplementaryDoBiasEstimation(), do_bias_estimation_);
	Parameters::parse(parameters, Parameters::kImuFilterComplementaryDoAdpativeGain(), do_adaptive_gain_);
}

void ComplementaryFilter::setDoBiasEstimation(bool do_bias_estimation)
{
  do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::getDoBiasEstimation() const
{
  return do_bias_estimation_;
}

void ComplementaryFilter::setDoAdaptiveGain(bool do_adaptive_gain)
{
  do_adaptive_gain_ = do_adaptive_gain;
}

bool ComplementaryFilter::getDoAdaptiveGain() const
{
  return do_adaptive_gain_;
}

bool ComplementaryFilter::setGainAcc(double gain)
{
  if (gain >= 0 && gain <= 1.0)
  {
    gain_acc_ = gain;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getGainAcc() const
{
  return gain_acc_;
}

bool ComplementaryFilter::getSteadyState() const
{
  return steady_state_;
}

bool ComplementaryFilter::setBiasAlpha(double bias_alpha)
{
  if (bias_alpha >= 0 && bias_alpha <= 1.0)
  {
    bias_alpha_ = bias_alpha;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getBiasAlpha() const
{
  return bias_alpha_;
}

void ComplementaryFilter::reset(
    double qx, double qy, double qz, double qw)
{
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(qw, qx, qy, qz, q0_, q1_, q2_, q3_);

  wx_prev_=0;
  wy_prev_=0;
  wz_prev_=0;
  wx_bias_=0;
  wy_bias_=0;
  wz_bias_=0;
  initialized_ = false;
  steady_state_= false;
}


double ComplementaryFilter::getAngularVelocityBiasX() const
{
  return wx_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasY() const
{
  return wy_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasZ() const
{
  return wz_bias_;
}

void ComplementaryFilter::updateImpl(
		double gx, double gy, double gz,
		double ax, double ay, double az,
	    double dt)
{
  if (!initialized_)
  {
    // First time - ignore prediction:
    getMeasurement(ax, ay, az,
                   q0_, q1_, q2_, q3_);
    initialized_ = true;
    return;
  }

  if(dt <= 0.0)
  {
    UWARN("dt=%f <=0.0, orientation will not be updated!", dt);
    return;
  }

  // Bias estimation.
  if (do_bias_estimation_)
    updateBiases(ax, ay, az, gx, gy, gz);

  // Prediction.
  double q0_pred, q1_pred, q2_pred, q3_pred;
  getPrediction(gx, gy, gz, dt,
                q0_pred, q1_pred, q2_pred, q3_pred);

  // Correction (from acc):
  // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
  getAccCorrection(ax, ay, az,
                   q0_pred, q1_pred, q2_pred, q3_pred,
                   dq0_acc, dq1_acc, dq2_acc, dq3_acc);

  double gain;
  if (do_adaptive_gain_)
  {
    gain = getAdaptiveGain(gain_acc_, ax, ay, az);

  }
  else
  {
    gain = gain_acc_;

  }

  scaleQuaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

  quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
                           dq0_acc, dq1_acc, dq2_acc, dq3_acc,
                           q0_, q1_, q2_, q3_);

  normalizeQuaternion(q0_, q1_, q2_, q3_);
}

bool ComplementaryFilter::checkState(double ax, double ay, double az,
                                     double wx, double wy, double wz) const
{
  double acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
    return false;

  if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
    return false;

  if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
      fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
      fabs(wz - wz_bias_) > kAngularVelocityThreshold)
    return false;

  return true;
}

void ComplementaryFilter::updateBiases(double ax, double ay, double az,
                                       double wx, double wy, double wz)
{
  steady_state_ = checkState(ax, ay, az, wx, wy, wz);

  if (steady_state_)
  {
    wx_bias_ += bias_alpha_ * (wx - wx_bias_);
    wy_bias_ += bias_alpha_ * (wy - wy_bias_);
    wz_bias_ += bias_alpha_ * (wz - wz_bias_);
  }

  wx_prev_ = wx;
  wy_prev_ = wy;
  wz_prev_ = wz;
}

void ComplementaryFilter::getPrediction(
    double wx, double wy, double wz, double dt,
    double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const
{
  double wx_unb = wx - wx_bias_;
  double wy_unb = wy - wy_bias_;
  double wz_unb = wz - wz_bias_;

  q0_pred = q0_ + 0.5*dt*( wx_unb*q1_ + wy_unb*q2_ + wz_unb*q3_);
  q1_pred = q1_ + 0.5*dt*(-wx_unb*q0_ - wy_unb*q3_ + wz_unb*q2_);
  q2_pred = q2_ + 0.5*dt*( wx_unb*q3_ - wy_unb*q0_ - wz_unb*q1_);
  q3_pred = q3_ + 0.5*dt*(-wx_unb*q2_ + wy_unb*q1_ - wz_unb*q0_);

  normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred);
}

void ComplementaryFilter::getMeasurement(
    double ax, double ay, double az,
    double mx, double my, double mz,
    double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  double q0_acc, q1_acc, q2_acc, q3_acc;

  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);
  if (az >=0)
    {
      q0_acc =  sqrt((az + 1) * 0.5);
      q1_acc = -ay/(2.0 * q0_acc);
      q2_acc =  ax/(2.0 * q0_acc);
      q3_acc = 0;
    }
    else
    {
      double X = sqrt((1 - az) * 0.5);
      q0_acc = -ay/(2.0 * X);
      q1_acc = X;
      q2_acc = 0;
      q3_acc = ax/(2.0 * X);
    }

  // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
  // frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  double lx = (q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx +
      2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz;
  double ly = 2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc +
      q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz;

  // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame. q1_mag and q2_mag are defined as 0.
	double gamma = lx*lx + ly*ly;
	double beta = sqrt(gamma + lx*sqrt(gamma));
  double q0_mag = beta / (sqrt(2.0 * gamma));
  double q3_mag = ly / (sqrt(2.0) * beta);

  // The quaternion multiplication between q_acc and q_mag represents the
  // quaternion, orientation of the Global frame wrt the local frame.
  // q = q_acc times q_mag
  quaternionMultiplication(q0_acc, q1_acc, q2_acc, q3_acc,
                           q0_mag, 0, 0, q3_mag,
                           q0_meas, q1_meas, q2_meas, q3_meas );
  //q0_meas = q0_acc*q0_mag;
  //q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
  //q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
  //q3_meas = q0_acc*q3_mag;
}


void ComplementaryFilter::getMeasurement(
    double ax, double ay, double az,
    double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.

  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

  if (az >=0)
  {
    q0_meas =  sqrt((az + 1) * 0.5);
    q1_meas = -ay/(2.0 * q0_meas);
    q2_meas =  ax/(2.0 * q0_meas);
    q3_meas = 0;
  }
  else
  {
    double X = sqrt((1 - az) * 0.5);
    q0_meas = -ay/(2.0 * X);
    q1_meas = X;
    q2_meas = 0;
    q3_meas = ax/(2.0 * X);
  }
}

void ComplementaryFilter::getAccCorrection(
  double ax, double ay, double az,
  double p0, double p1, double p2, double p3,
  double& dq0, double& dq1, double& dq2, double& dq3)
{
  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

  // Acceleration reading rotated into the world frame by the inverse predicted
  // quaternion (predicted gravity):
  double gx, gy, gz;
  rotateVectorByQuaternion(ax, ay, az,
                           p0, -p1, -p2, -p3,
                           gx, gy, gz);

  // Delta quaternion that rotates the predicted gravity into the real gravity:
  dq0 =  sqrt((gz + 1) * 0.5);
  dq1 = -gy/(2.0 * dq0);
  dq2 =  gx/(2.0 * dq0);
  dq3 =  0.0;
}

void ComplementaryFilter::getOrientation(
    double& qx, double& qy, double& qz, double& qw) const
{
  // Return the inverse of the state (state is fixed wrt body).
  invertQuaternion(q0_, q1_, q2_, q3_, qw,qx,qy,qz);
}

double ComplementaryFilter::getAdaptiveGain(double alpha, double ax, double ay, double az)
{
  double a_mag = sqrt(ax*ax + ay*ay + az*az);
  double error = fabs(a_mag - kGravity)/kGravity;
  double factor;
  double error1 = 0.1;
  double error2 = 0.2;
  double m = 1.0/(error1 - error2);
  double b = 1.0 - m*error1;
  if (error < error1)
    factor = 1.0;
  else if (error < error2)
    factor = m*error + b;
  else
    factor = 0.0;
  //printf("FACTOR: %f \n", factor);
  return factor*alpha;
}

void normalizeVector(double& x, double& y, double& z)
{
  double norm = sqrt(x*x + y*y + z*z);

  x /= norm;
  y /= norm;
  z /= norm;
}

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void invertQuaternion(
  double q0, double q1, double q2, double q3,
  double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv)
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}

void scaleQuaternion(
  double gain,
  double& dq0, double& dq1, double& dq2, double& dq3)
{
	if (dq0 < 0.0)//0.9
  {
    // Slerp (Spherical linear interpolation):
    double angle = acos(dq0);
    double A = sin(angle*(1.0 - gain))/sin(angle);
    double B = sin(angle * gain)/sin(angle);
    dq0 = A + B * dq0;
    dq1 = B * dq1;
    dq2 = B * dq2;
    dq3 = B * dq3;
  }
  else
  {
    // Lerp (Linear interpolation):
    dq0 = (1.0 - gain) + gain * dq0;
    dq1 = gain * dq1;
    dq2 = gain * dq2;
    dq3 = gain * dq3;
  }

  normalizeQuaternion(dq0, dq1, dq2, dq3);
}

void quaternionMultiplication(
  double p0, double p1, double p2, double p3,
  double q0, double q1, double q2, double q3,
  double& r0, double& r1, double& r2, double& r3)
{
  // r = p q
  r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
  r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void rotateVectorByQuaternion(
  double x, double y, double z,
  double q0, double q1, double q2, double q3,
  double& vx, double& vy, double& vz)
{
  vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
  vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
  vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}


}  // namespace rtabmap

