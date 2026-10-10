/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef LIDARCAMERACALIBRATION_CALIBRATIONPROBLEM_H_
#define LIDARCAMERACALIBRATION_CALIBRATIONPROBLEM_H_

#include <rtabmap/core/Transform.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

#include <functional>
#include <vector>

// What makes a lidar point an edge point, in order of precedence.
enum EdgeType
{
	kEdgeDepth = 0,      // in front of a depth discontinuity
	kEdgeCrease = 1,     // where the surface's orientation changes (e.g., wall and floor)
	kEdgeIntensity = 2   // where the surface's reflectance changes
};

// One node of the database: its image, its lidar scan, and the lidar's edges.
struct Frame
{
	int id;
	cv::Mat gray;          // as stored, matching the camera model
	cv::Mat edges;         // CV_8U: the image's edges (Canny), non-zero on an edge
	cv::Mat edgeDistance;  // CV_32F: distance (pixels) to the image's nearest edge
	cv::Mat edgeScore;     // CV_32F: 1 on the image's edges, decaying with the distance to them
	cv::Mat K;             // CV_64F 3x3
	rtabmap::Transform scanToCam;  // camera pose in the scan frame, before correction
	cv::Mat cloud;         // Nx4 CV_32F, scan frame: x y z intensity (0 without intensity)
	cv::Mat normals;       // Mx6 CV_32F, scan frame: voxelized points and their normals (for creases)
	bool hasIntensity;
	// The robot's speeds (m/s, deg/s; negative if unknown): instantaneous, odometry's when the
	// node was added, and mean along odometry over the meanWindow (s) since the previous
	// node, which is when an assembled scan was taken.
	float linearSpeed = -1.0f, angularSpeed = -1.0f;
	float meanLinearSpeed = -1.0f, meanAngularSpeed = -1.0f, meanWindow = 0.0f;
	int meanPoses = 0;  // odometry steps the mean speed is from: more than 1 with intermediate nodes
	std::vector<cv::Point3f> edgePoints;  // depth and intensity edge points, scan frame
	std::vector<float> edgeWeights;
	std::vector<unsigned char> edgeTypes; // EdgeType of each edge point
};

// The correction of parameters p = tx ty tz (m), rx ry rz (deg), in the camera frame.
rtabmap::Transform correctionFrom(const double p[6]);
// The same, in double precision: rtabmap's Transform is float, too coarse for the small
// steps of numerical derivatives.
Eigen::Isometry3d correctionFromDouble(const double p[6]);

// The lidar edge points of some nodes, and how well a candidate correction C lines them
// up with their images' edges. This is all a solver sees of the calibration.
class CalibrationProblem
{
public:
	// The frames' edge points are read at each call, so they can be selected again after
	// the problem is made. minDepth: lidar points closer than this to the camera (m) are
	// ignored.
	CalibrationProblem(const std::vector<Frame> & frames, const std::vector<int> & nodes, float minDepth);

	const std::vector<Frame> & frames() const {return frames_;}
	const std::vector<int> & nodes() const {return nodes_;}
	float minDepth() const {return minDepth_;}

	// Calls visit(frame, point index, u, v) for each edge point that C projects in its
	// image, at full resolution.
	void project(const rtabmap::Transform & C,
			const std::function<void(const Frame &, size_t, float, float)> & visit) const;

	// What the direct solvers maximize: the weighted mean of the image edges' score under
	// the projected lidar edge points, in [0, 1]. A point out of its image counts as 0.
	double score(const rtabmap::Transform & C) const;
	int evaluations() const {return evaluations_;}

	// What the least-squares solvers minimize, per point: the distance (pixels) from
	// where p projects edge point i of f to f's nearest image edge, capped; the cap where
	// it does not project in the image. In double precision, for numerical derivatives.
	double edgeDistance(const Frame & f, size_t i, const double p[6], double cap) const;

private:
	const std::vector<Frame> & frames_;
	std::vector<int> nodes_;
	float minDepth_;
	mutable int evaluations_;
};

#endif /* LIDARCAMERACALIBRATION_CALIBRATIONPROBLEM_H_ */
