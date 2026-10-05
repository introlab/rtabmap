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

// Refines the transform between a camera and a lidar without a target, from a database
// in which nodes hold an image and a lidar scan taken together, by aligning the lidar's
// depth discontinuities with the image's edges (as in J. Levinson and S. Thrun,
// "Automatic Online Calibration of Cameras and Lasers", RSS 2013).
//
// The result is a correction X of the camera's mount, in the camera's body frame (x
// forward, y left, z up): the robot base -> camera body transform B becomes B * X. It is
// estimated for the nodes all together, so that a node's own errors (time sync, odometry
// over the scan) average out. Internally, the solvers search for the same correction in
// the camera's optical frame, C = R^-1 * X * R (R: the optical rotation), in which the
// lidar is projected: each node's camera local transform T becomes T * C.

#include "CalibrationProblem.h"
#include "CorrectionSolver.h"

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <stdio.h>
#include <vector>

using namespace rtabmap;

void showUsage(const char * exec)
{
	const std::vector<std::string> solvers = availableSolvers();
	printf("\nUsage:\n"
			"%s [Options] database.db\n"
			"  Refine the camera-lidar extrinsics of a database whose nodes have an image and\n"
			"  a lidar scan taken together, by aligning the lidar's edges (depth discontinuities,\n"
			"  creases and, if the scans have intensity, intensity edges) with the images' edges.\n"
			"  Prints a correction X of the camera's mount, in the camera's body frame (x forward,\n"
			"  y left, z up): the robot base -> camera transform B (e.g., base_link ->\n"
			"  camera_link) becomes B * X.\n"
			"Options:\n"
			"    --solver \"name\"   How the correction is searched for: \"simplex\" (OpenCV's\n"
			"                      Nelder-Mead, default), \"pattern\" (one parameter at a\n"
			"                      time, with decreasing steps), \"g2o\" or \"gtsam\" (least\n"
			"                      squares on the distances to the image edges, if rtabmap is\n"
			"                      built with them). Available in this build: %s.\n"
			"    --verbose         Also print the sensitivity: how much the score drops with\n"
			"                      the result off by 1 deg or 2 cm on each axis.\n"
			"    --translation     Also estimate the translation. Off by default: unless the\n"
			"                      scene is close compared to the lever arm between the\n"
			"                      sensors, it is not observable (check the split result).\n"
			"    --images \"dir\"    Save, for every node, its image with its edges (green) and\n"
			"                      the lidar edges projected (depth: red, crease: blue,\n"
			"                      intensity: yellow), and the lidar's intensity and surface\n"
			"                      orientation (yellow: facing the camera, red: edge on),\n"
			"                      before and after the correction, in this directory.\n"
			"    --decimation #    Image decimation at which the lidar's depth discontinuities\n"
			"                      are found, so that the projected scan is dense (default 4).\n"
			"    --jump #.#        Relative depth jump for a lidar point to be on a\n"
			"                      discontinuity, as a fraction of its depth: 0.15 (default)\n"
			"                      means a neighbor at least 15% of this point's depth farther\n"
			"                      than where this point's surface would continue.\n"
			"    --intensity_jump #.#  Relative intensity change for a lidar point to be on an\n"
			"                      intensity edge, as a fraction: 0.4 (default) means a\n"
			"                      neighbor at least 1.4 times brighter or darker.\n"
			"    --intensity_weight #.#  Weight of intensity edges relative to depth\n"
			"                      discontinuities (default 1).\n"
			"    --crease_angle #.#  Also use creases: lidar points where the surface normals\n"
			"                      differ by this angle (deg), e.g., between a wall and the\n"
			"                      floor (default 45, 0 disables).\n"
			"    --crease_voxel #.#  Voxel size (m) of the scans on which normals are\n"
			"                      computed, smoother than at full resolution (default 0.1).\n"
			"    --no_intensity    Use only depth discontinuities, even if the scans have\n"
			"                      intensity.\n"
			"    --sigma #.#       Fall off (pixels) of the image edges' score (default 3).\n"
			"    --min_depth #.#   Ignore lidar points closer to the camera (m, default 0.5).\n"
			"    --initial_rotation #.# #.# #.#  Start from this correction (roll, pitch, yaw in\n"
			"                      deg, in the camera's body frame) instead of none, e.g., to\n"
			"                      see from how far the result is found again (default 0 0 0).\n"
			"    --max_angular_speed #.#  Skip the nodes rotating faster than this (deg/s, mean\n"
			"                      since the previous node, over which an assembled scan is\n"
			"                      taken, from odometry; default 0: all).\n"
			"    --max_linear_speed #.#  Skip the nodes moving faster than this (m/s; default\n"
			"                      0: all).\n"
			"    --voxel #.#       Voxel filter the scans first (m, default 0: as stored), to\n"
			"                      compare the result at several lidar densities.\n"
			"\n", exec, uJoin(std::list<std::string>(solvers.begin(), solvers.end()), ", ").c_str());
	exit(1);
}

void computeEdgeMaps(Frame & f, float sigma)
{
	cv::Mat blurred;
	cv::GaussianBlur(f.gray, blurred, cv::Size(5, 5), 1.5);
	cv::Canny(blurred, f.edges, 40, 100);
	cv::distanceTransform(f.edges == 0, f.edgeDistance, cv::DIST_L2, 3);
	// Smooth enough for a local search to follow, highest on the edges.
	cv::exp(-f.edgeDistance / sigma, f.edgeScore);
}

// The scan as seen by the camera at scanToCam * C, at the image's resolution divided by
// decimation, where the projected scan is dense.
struct CellMaps
{
	cv::Mat depth;         // CV_32F, the nearest point's depth, 0 where none
	cv::Mat index;         // CV_32S, the nearest point's index in the cloud, -1 where none
	cv::Mat logIntensity;  // CV_32F, empty if not computed
	cv::Mat normals;       // CV_32FC3, in the camera frame, not normalized, zero where unknown; empty if not computed
};

CellMaps computeCellMaps(const Frame & f, const Transform & C, int decimation, bool withIntensity,
		bool withNormals, float normalsVoxel, float minDepth)
{
	CellMaps maps;
	const int w = f.gray.cols / decimation, h = f.gray.rows / decimation;
	const double fx = f.K.at<double>(0,0) / decimation, fy = f.K.at<double>(1,1) / decimation;
	const double cx = f.K.at<double>(0,2) / decimation, cy = f.K.at<double>(1,2) / decimation;
	const Transform scanInCam = (f.scanToCam * C).inverse();

	// Nearest point per pixel, and which one it is.
	cv::Mat depth(h, w, CV_32F, cv::Scalar(0));
	cv::Mat index(h, w, CV_32S, cv::Scalar(-1));
	for(int i = 0; i < f.cloud.rows; ++i)
	{
		const float * p = f.cloud.ptr<float>(i);
		const cv::Point3f pc = util3d::transformPoint(cv::Point3f(p[0], p[1], p[2]), scanInCam);
		if(pc.z < minDepth)
		{
			continue;
		}
		const int u = int(fx * pc.x / pc.z + cx), v = int(fy * pc.y / pc.z + cy);
		if(u < 0 || v < 0 || u >= w || v >= h)
		{
			continue;
		}
		float & d = depth.at<float>(v, u);
		if(d == 0 || pc.z < d)
		{
			d = pc.z;
			index.at<int>(v, u) = i;
		}
	}

	// Log-intensity, so that a change is relative and intensity's fall off with range
	// matters less. A cell's is the mean over all the points of the surface it sees
	// (within 5% of its nearest point's depth), not its nearest point's alone: the
	// beams of a lidar do not return the same intensity from the same surface, and a
	// node's scan assembles several sweeps, so neighboring cells seen by different beams
	// would otherwise differ, making false edges along the beams' traces. Then median
	// filtered against what speckle remains.
	cv::Mat logIntensity;
	if(withIntensity && f.hasIntensity)
	{
		cv::Mat sum(h, w, CV_32F, cv::Scalar(0));
		cv::Mat count(h, w, CV_32S, cv::Scalar(0));
		for(int i = 0; i < f.cloud.rows; ++i)
		{
			const float * p = f.cloud.ptr<float>(i);
			const cv::Point3f pc = util3d::transformPoint(cv::Point3f(p[0], p[1], p[2]), scanInCam);
			if(pc.z < minDepth)
			{
				continue;
			}
			const int u = int(fx * pc.x / pc.z + cx), v = int(fy * pc.y / pc.z + cy);
			if(u < 0 || v < 0 || u >= w || v >= h || pc.z > depth.at<float>(v, u) * 1.05f)
			{
				continue;
			}
			sum.at<float>(v, u) += std::log(1.0f + std::max(0.0f, p[3]));
			++count.at<int>(v, u);
		}
		logIntensity = cv::Mat(h, w, CV_32F, cv::Scalar(0));
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				const int n = count.at<int>(v, u);
				if(n > 0)
				{
					logIntensity.at<float>(v, u) = sum.at<float>(v, u) / n;
				}
			}
		}
		cv::medianBlur(logIntensity, logIntensity, 3);
	}

	// Surface normals, in the camera frame, turned toward it: a cell's is the mean over
	// the (voxelized) points of the surface it sees, as for intensity. A voxel stands for
	// the surface over its whole size, which can span several cells: its normal is spread
	// over the cells it covers, else most cells would have none where the voxels are
	// larger than the cells, and creases (which need all the neighbors' normals) be missed.
	cv::Mat cellNormals;  // CV_32FC3, zero where unknown
	if(withNormals && !f.normals.empty())
	{
		const Eigen::Matrix3f rotation = scanInCam.toEigen3f().linear();
		cellNormals = cv::Mat(h, w, CV_32FC3, cv::Scalar(0, 0, 0));
		for(int i = 0; i < f.normals.rows; ++i)
		{
			const float * p = f.normals.ptr<float>(i);
			const cv::Point3f pc = util3d::transformPoint(cv::Point3f(p[0], p[1], p[2]), scanInCam);
			if(pc.z < minDepth || !std::isfinite(p[3]))
			{
				continue;
			}
			Eigen::Vector3f n = rotation * Eigen::Vector3f(p[3], p[4], p[5]);
			if(n.dot(Eigen::Vector3f(pc.x, pc.y, pc.z)) > 0.0f)
			{
				n = -n;  // toward the camera
			}
			const double pu = fx * pc.x / pc.z + cx, pv = fy * pc.y / pc.z + cy;
			const double halfSize = 0.5 * normalsVoxel * fx / pc.z;  // in cells
			for(int v = std::max(0, int(std::floor(pv - halfSize))); v <= std::min(h - 1, int(std::floor(pv + halfSize))); ++v)
			{
				for(int u = std::max(0, int(std::floor(pu - halfSize))); u <= std::min(w - 1, int(std::floor(pu + halfSize))); ++u)
				{
					const float d = depth.at<float>(v, u);
					if(d == 0 || pc.z > d * 1.05f || pc.z < d * 0.95f)
					{
						continue;  // not the surface this cell sees
					}
					cellNormals.at<cv::Vec3f>(v, u) += cv::Vec3f(n.x(), n.y(), n.z());
				}
			}
		}
	}

	maps.depth = depth;
	maps.index = index;
	maps.logIntensity = logIntensity;
	maps.normals = cellNormals;
	return maps;
}

// Edge points of a cell map located at the image's full resolution, as Canny locates edges:
// the map (1 or 3 channels) is interpolated and smoothed at full resolution, and an edge is
// where its change is the largest across it (non-maximum suppression along the direction of
// largest change of its channels, as for a color image), in the cells where cellWeight > 0
// (the cells found on an edge at cell resolution, and how strong). At cell resolution, an
// edge is a band of a few cells (any cell differing enough from a neighbor), and the
// nearest point of a cell is anywhere in it. These edges are on continuous surfaces (not
// depth discontinuities): the depth is interpolated where they are found (inverse depth,
// linear on a plane), and they are turned back into 3D. One point per cell, the strongest.
void addFullResolutionEdges(Frame & f, const Transform & C, int decimation, const cv::Mat & depth,
		const cv::Mat & map, const cv::Mat & cellWeight, unsigned char type)
{
	const int w = depth.cols, h = depth.rows;
	const int W = w * decimation, H = h * decimation;
	const double FX = f.K.at<double>(0,0), FY = f.K.at<double>(1,1);
	const double CX = f.K.at<double>(0,2), CY = f.K.at<double>(1,2);
	const Transform camToScan = f.scanToCam * C;
	cv::Mat smooth, dx, dy;
	cv::resize(map, smooth, cv::Size(W, H), 0, 0, cv::INTER_LINEAR);
	cv::GaussianBlur(smooth, smooth, cv::Size(0, 0), decimation / 2.0);
	cv::Sobel(smooth, dx, CV_32F, 1, 0, 3);
	cv::Sobel(smooth, dy, CV_32F, 0, 1, 3);
	const int channels = smooth.channels();
	// Largest rate of change and its direction (Di Zenzo).
	cv::Mat magnitude(H, W, CV_32F, cv::Scalar(0));
	cv::Mat direction(H, W, CV_32F, cv::Scalar(0));
	for(int y = 0; y < H; ++y)
	{
		const float * gx = dx.ptr<float>(y);
		const float * gy = dy.ptr<float>(y);
		for(int x = 0; x < W; ++x)
		{
			float gxx = 0, gyy = 0, gxy = 0;
			for(int c = 0; c < channels; ++c)
			{
				const float a = gx[x * channels + c], b = gy[x * channels + c];
				gxx += a * a; gyy += b * b; gxy += a * b;
			}
			magnitude.at<float>(y, x) = 0.5f * (gxx + gyy + std::sqrt((gxx - gyy) * (gxx - gyy) + 4.0f * gxy * gxy));
			direction.at<float>(y, x) = 0.5f * std::atan2(2.0f * gxy, gxx - gyy);
		}
	}
	std::vector<float> best(w * h, 0.0f);
	std::vector<cv::Point2f> bestPixel(w * h);
	for(int y = 1; y < H - 1; ++y)
	{
		for(int x = 1; x < W - 1; ++x)
		{
			const int u = x / decimation, v = y / decimation;
			if(cellWeight.at<float>(v, u) <= 0.0f)
			{
				continue;
			}
			const float m = magnitude.at<float>(y, x);
			const float a = direction.at<float>(y, x);
			const int sx = int(std::round(std::cos(a))), sy = int(std::round(std::sin(a)));
			if(m <= best[v * w + u] || m < magnitude.at<float>(y + sy, x + sx) || m < magnitude.at<float>(y - sy, x - sx))
			{
				continue;
			}
			// Sub-pixel position across the edge, from a parabola through the strengths: a
			// pixel's center would put all these points on the image's pixel centers when
			// projected with the correction they were selected with, where the image edge
			// score (interpolated between pixel centers) peaks, making that correction a
			// false optimum.
			const float mm = magnitude.at<float>(y - sy, x - sx), mp = magnitude.at<float>(y + sy, x + sx);
			const float den = mm - 2.0f * m + mp;
			const float offset = den < 0.0f ? std::max(-0.5f, std::min(0.5f, 0.5f * (mm - mp) / den)) : 0.0f;
			best[v * w + u] = m;
			bestPixel[v * w + u] = cv::Point2f(x + offset * sx, y + offset * sy);
		}
	}
	for(int v = 0; v < h; ++v)
	{
		for(int u = 0; u < w; ++u)
		{
			if(best[v * w + u] <= 0.0f)
			{
				continue;
			}
			const cv::Point2f & px = bestPixel[v * w + u];
			// Bilinear inverse depth between the 4 nearest cell centers, all on the same
			// surface (within 5% of each other's depth).
			const float cu = (px.x + 0.5f) / decimation - 0.5f, cv_ = (px.y + 0.5f) / decimation - 0.5f;
			const int u0 = std::max(0, std::min(w - 2, int(std::floor(cu)))), v0 = std::max(0, std::min(h - 2, int(std::floor(cv_))));
			const float au = std::max(0.0f, std::min(1.0f, cu - u0)), av = std::max(0.0f, std::min(1.0f, cv_ - v0));
			const float d00 = depth.at<float>(v0, u0), d01 = depth.at<float>(v0, u0 + 1);
			const float d10 = depth.at<float>(v0 + 1, u0), d11 = depth.at<float>(v0 + 1, u0 + 1);
			const float dMin = std::min(std::min(d00, d01), std::min(d10, d11)), dMax = std::max(std::max(d00, d01), std::max(d10, d11));
			if(dMin <= 0.0f || dMax > dMin * 1.05f)
			{
				continue;
			}
			const float inverse = (1 - av) * ((1 - au) / d00 + au / d01) + av * ((1 - au) / d10 + au / d11);
			const float z = 1.0f / inverse;
			const cv::Point3f pc(float((px.x - CX) / FX) * z, float((px.y - CY) / FY) * z, z);
			f.edgePoints.push_back(util3d::transformPoint(pc, camToScan));
			f.edgeWeights.push_back(cellWeight.at<float>(v, u));
			f.edgeTypes.push_back(type);
		}
	}
}

// The lidar's edges as seen by the camera at scanToCam * C: points in front of a depth
// discontinuity, found at a resolution where the projected scan is dense, then points on a
// crease or an intensity edge (a change of the surface's reflectance, e.g., paint or
// material, which the image is likely to show too, where there is no depth discontinuity),
// found at that resolution and located at the image's.
void selectEdgePoints(Frame & f, const Transform & C, int decimation, float jumpThreshold,
		float intensityJumpThreshold, float intensityWeight, float creaseAngle, float creaseVoxel, float minDepth)
{
	const CellMaps maps = computeCellMaps(f, C, decimation, intensityJumpThreshold > 0.0f, creaseAngle > 0.0f,
			creaseVoxel, minDepth);
	const cv::Mat & depth = maps.depth;
	const cv::Mat & index = maps.index;
	const cv::Mat & logIntensity = maps.logIntensity;
	const cv::Mat & cellNormals = maps.normals;
	const int w = depth.cols, h = depth.rows;
	const float logIntensityJump = std::log(1.0f + intensityJumpThreshold);

	f.edgePoints.clear();
	f.edgeWeights.clear();
	f.edgeTypes.clear();

	// Depth discontinuities, and which cells are (no crease or intensity edge there).
	cv::Mat depthEdge(h, w, CV_8U, cv::Scalar(0));
	for(int v = 1; v < h - 1; ++v)
	{
		for(int u = 1; u < w - 1; ++u)
		{
			const float d = depth.at<float>(v, u);
			if(d == 0)
			{
				continue;
			}
			// Largest relative jump to a farther neighbor, beyond where this point's surface
			// would continue: this point is in front of it. On a plane, inverse depth is
			// linear in the image, so the surface's continuation at a neighbor is predicted
			// from the opposite neighbor. A surface seen at a grazing angle (the floor ahead
			// of a low camera) has a steep depth gradient but follows its continuation: it
			// is not a discontinuity, whereas a background behind an object's edge is far
			// beyond the object's continuation.
			float jump = 0.0f;
			for(int dv = -1; dv <= 1; ++dv)
			{
				for(int du = -1; du <= 1; ++du)
				{
					if(du == 0 && dv == 0)
					{
						continue;
					}
					const float n = depth.at<float>(v + dv, u + du);
					const float o = depth.at<float>(v - dv, u - du);
					if(n <= 0 || o <= 0)
					{
						continue;  // no opposite neighbor: no prediction
					}
					const float predictedInverse = 2.0f / d - 1.0f / o;
					if(predictedInverse <= 0.0f)
					{
						continue;  // the surface recedes beyond the horizon there
					}
					jump = std::max(jump, (n - 1.0f / predictedInverse) / d);
				}
			}
			if(jump > jumpThreshold)
			{
				depthEdge.at<unsigned char>(v, u) = 1;
				const float * p = f.cloud.ptr<float>(index.at<int>(v, u));
				f.edgePoints.emplace_back(p[0], p[1], p[2]);
				f.edgeWeights.push_back(std::min(1.0f, jump));
				f.edgeTypes.push_back(kEdgeDepth);
			}
		}
	}

	// Creases: cells where the surface normals of two opposite neighbors differ by more than
	// creaseAngle, where the cell and all its neighbors have a normal, weighted by the angle.
	cv::Mat creaseWeight(h, w, CV_32F, cv::Scalar(0));
	if(creaseAngle > 0.0f && !cellNormals.empty())
	{
		static const int directions[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};  // (du, dv)
		cv::Mat unit(h, w, CV_32FC3, cv::Scalar(0, 0, 0));  // unit normals, zero where unknown
		cv::Mat known(h, w, CV_8U, cv::Scalar(0));
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				const cv::Vec3f & n = cellNormals.at<cv::Vec3f>(v, u);
				const double nn = cv::norm(n);
				if(nn > 0)
				{
					unit.at<cv::Vec3f>(v, u) = n / nn;
					known.at<unsigned char>(v, u) = 1;
				}
			}
		}
		for(int v = 1; v < h - 1; ++v)
		{
			for(int u = 1; u < w - 1; ++u)
			{
				bool complete = !depthEdge.at<unsigned char>(v, u);
				for(int dv = -1; dv <= 1 && complete; ++dv)
				{
					for(int du = -1; du <= 1 && complete; ++du)
					{
						complete = known.at<unsigned char>(v + dv, u + du) != 0;
					}
				}
				if(!complete)
				{
					continue;
				}
				float strongest = 0.0f;
				for(int k = 0; k < 4; ++k)
				{
					const int du = directions[k][0], dv = directions[k][1];
					const float c = unit.at<cv::Vec3f>(v + dv, u + du).dot(unit.at<cv::Vec3f>(v - dv, u - du));
					strongest = std::max(strongest, std::acos(std::max(-1.0f, std::min(1.0f, c))) * 180.0f / float(M_PI));
				}
				if(strongest > creaseAngle)
				{
					creaseWeight.at<float>(v, u) = std::min(1.0f, strongest / (2.0f * creaseAngle));
				}
			}
		}
		addFullResolutionEdges(f, C, decimation, depth, unit, creaseWeight, kEdgeCrease);
	}

	// Intensity edges: cells whose log-intensity differs from a neighbor's by more than
	// logIntensityJump, where all of them are known (no edge against a hole), and not
	// already on a depth discontinuity or a crease, weighted by the change.
	if(!logIntensity.empty())
	{
		cv::Mat intensityWeightMap(h, w, CV_32F, cv::Scalar(0));
		for(int v = 1; v < h - 1; ++v)
		{
			for(int u = 1; u < w - 1; ++u)
			{
				if(depth.at<float>(v, u) == 0 || depthEdge.at<unsigned char>(v, u) || creaseWeight.at<float>(v, u) > 0.0f)
				{
					continue;
				}
				bool complete = true;
				float change = 0.0f;
				const float center = logIntensity.at<float>(v, u);
				for(int dv = -1; dv <= 1 && complete; ++dv)
				{
					for(int du = -1; du <= 1; ++du)
					{
						if(depth.at<float>(v + dv, u + du) == 0)
						{
							complete = false;
							break;
						}
						change = std::max(change, std::fabs(logIntensity.at<float>(v + dv, u + du) - center));
					}
				}
				if(complete && change > logIntensityJump)
				{
					intensityWeightMap.at<float>(v, u) = intensityWeight * std::min(1.0f, change / (2.0f * logIntensityJump));
				}
			}
		}
		addFullResolutionEdges(f, C, decimation, depth, logIntensity, intensityWeightMap, kEdgeIntensity);
	}
}

// The node's id and speeds, in the image's top left corner, one per line.
void drawNodeInfo(cv::Mat & image, const Frame & f)
{
	std::vector<std::string> lines(1, uFormat("Node %d", f.id));
	if(f.meanAngularSpeed >= 0.0f)
	{
		lines.push_back(uFormat("mean over %.1f s (%d poses): %.2f m/s %.1f deg/s", f.meanWindow, f.meanPoses + 1, f.meanLinearSpeed, f.meanAngularSpeed));
	}
	if(f.angularSpeed >= 0.0f)
	{
		lines.push_back(uFormat("instant: %.2f m/s %.1f deg/s", f.linearSpeed, f.angularSpeed));
	}
	const double scale = 0.35 * image.cols / 640.0;
	const int lineHeight = std::max(10, int(30.0 * scale + 0.5));
	for(size_t i = 0; i < lines.size(); ++i)
	{
		const cv::Point origin(6, lineHeight * int(i + 1));
		cv::putText(image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
		cv::putText(image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
	}
}

void saveOverlay(const Frame & f, const Transform & C, float minDepth, const std::string & path)
{
	// The image darkened, its edges in green, the lidar edge points over them: depth
	// discontinuities in red, creases in blue, intensity edges in yellow.
	cv::Mat out;
	cv::cvtColor(f.gray / 2, out, cv::COLOR_GRAY2BGR);
	out.setTo(cv::Scalar(0, 200, 0), f.edges);
	const Transform scanInCam = (f.scanToCam * C).inverse();
	const double fx = f.K.at<double>(0,0), fy = f.K.at<double>(1,1);
	const double cx = f.K.at<double>(0,2), cy = f.K.at<double>(1,2);
	for(size_t i = 0; i < f.edgePoints.size(); ++i)
	{
		const cv::Point3f pc = util3d::transformPoint(f.edgePoints[i], scanInCam);
		if(pc.z >= minDepth)
		{
			// One pixel each, so that the image edges under them stay visible.
			const int u = int(fx * pc.x / pc.z + cx + 0.5), v = int(fy * pc.y / pc.z + cy + 0.5);
			if(u >= 0 && v >= 0 && u < out.cols && v < out.rows)
			{
				static const cv::Vec3b colors[3] = {cv::Vec3b(0, 0, 255), cv::Vec3b(255, 128, 0), cv::Vec3b(0, 255, 255)};
				out.at<cv::Vec3b>(v, u) = colors[f.edgeTypes[i]];
			}
		}
	}
	drawNodeInfo(out, f);
	cv::imwrite(path, out);
}

// The cell maps the lidar edges are found from, half transparent over the image, with the
// image's edges in green and the map's own in blue, from red (0) to yellow (1): the lidar's intensity (as used, log
// and median filtered, contrast stretched per image: red dark, yellow bright), and how the
// surface faces the camera (red: seen edge on, at a grazing angle, yellow: facing it).
void saveCellMapOverlays(const Frame & f, const Transform & C, int decimation, float normalsVoxel,
		float minDepth, const std::string & prefix, const std::string & suffix)
{
	const CellMaps maps = computeCellMaps(f, C, decimation, true, true, normalsVoxel, minDepth);
	const int w = maps.depth.cols, h = maps.depth.rows;
	const double fx = f.K.at<double>(0,0) / decimation, fy = f.K.at<double>(1,1) / decimation;
	const double cx = f.K.at<double>(0,2) / decimation, cy = f.K.at<double>(1,2) / decimation;

	// values: CV_32F in [0,1] per cell, negative where unknown.
	auto save = [&](const cv::Mat & values, const std::string & path)
	{
		cv::Mat cells(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat known(h, w, CV_8U, cv::Scalar(0));
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				const float x = values.at<float>(v, u);
				if(x >= 0.0f)
				{
					cells.at<cv::Vec3b>(v, u) = cv::Vec3b(0, (unsigned char)(255.0f * std::min(1.0f, x)), 255);
					known.at<unsigned char>(v, u) = 255;
				}
			}
		}
		cv::Mat out, up, upKnown, blended;
		cv::cvtColor(f.gray, out, cv::COLOR_GRAY2BGR);
		const cv::Rect roi(0, 0, w * decimation, h * decimation);
		cv::resize(cells, up, roi.size(), 0, 0, cv::INTER_NEAREST);
		cv::resize(known, upKnown, roi.size(), 0, 0, cv::INTER_NEAREST);
		cv::addWeighted(out(roi), 0.5, up, 0.5, 0.0, blended);
		blended.copyTo(out(roi), upKnown);
		out.setTo(cv::Scalar(0, 200, 0), f.edges);

		// The map's own edges, as the image's (Canny), in blue: smoothly upsampled so that
		// they do not follow the cells' outline, and not against holes.
		cv::Mat value8(h, w, CV_8U, cv::Scalar(0));
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				value8.at<unsigned char>(v, u) = (unsigned char)(255.0f * std::max(0.0f, std::min(1.0f, values.at<float>(v, u))));
			}
		}
		cv::Mat smooth, mapEdges, inside;
		cv::resize(value8, smooth, roi.size(), 0, 0, cv::INTER_LINEAR);
		cv::GaussianBlur(smooth, smooth, cv::Size(5, 5), 1.5);
		cv::Canny(smooth, mapEdges, 40, 100);
		cv::erode(upKnown, inside, cv::Mat(), cv::Point(-1, -1), decimation);
		mapEdges &= inside;
		out(roi).setTo(cv::Scalar(255, 0, 0), mapEdges);
		drawNodeInfo(out, f);
	cv::imwrite(path, out);
	};

	if(!maps.logIntensity.empty())
	{
		std::vector<float> known;
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				if(maps.depth.at<float>(v, u) > 0)
				{
					known.push_back(maps.logIntensity.at<float>(v, u));
				}
			}
		}
		if(!known.empty())
		{
			std::sort(known.begin(), known.end());
			const float low = known[known.size() * 2 / 100], high = known[known.size() * 98 / 100];
			cv::Mat values(h, w, CV_32F, cv::Scalar(-1));
			for(int v = 0; v < h; ++v)
			{
				for(int u = 0; u < w; ++u)
				{
					if(maps.depth.at<float>(v, u) > 0)
					{
						const float x = high > low ? (maps.logIntensity.at<float>(v, u) - low) / (high - low) : 0.5f;
						values.at<float>(v, u) = std::min(1.0f, std::max(0.0f, x));
					}
				}
			}
			save(values, prefix + "_intensity" + suffix);
		}
	}

	if(!maps.normals.empty())
	{
		cv::Mat values(h, w, CV_32F, cv::Scalar(-1));
		for(int v = 0; v < h; ++v)
		{
			for(int u = 0; u < w; ++u)
			{
				const cv::Vec3f & n = maps.normals.at<cv::Vec3f>(v, u);
				const double nn = cv::norm(n);
				if(nn > 0)
				{
					// Cosine between the normal and the ray to the cell: 1 facing, 0 edge on.
					const cv::Vec3f ray((u + 0.5 - cx) / fx, (v + 0.5 - cy) / fy, 1.0);
					values.at<float>(v, u) = float(std::fabs(n.dot(ray)) / (nn * cv::norm(ray)));
				}
			}
		}
		save(values, prefix + "_normals" + suffix);
	}
}

// The correction of the camera's mount X (camera body frame: x forward, y left, z up) for
// the solvers' parameters p (the correction C in the camera's optical frame), and back.
Transform mountCorrection(const double p[6])
{
	return CameraModel::opticalRotation() * correctionFrom(p) * CameraModel::opticalRotation().inverse();
}

void parametersFromMountCorrection(const Transform & X, double p[6])
{
	const Transform C = CameraModel::opticalRotation().inverse() * X * CameraModel::opticalRotation();
	float x, y, z, roll, pitch, yaw;
	C.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
	p[0] = x; p[1] = y; p[2] = z;
	p[3] = roll * 180.0 / M_PI; p[4] = pitch * 180.0 / M_PI; p[5] = yaw * 180.0 / M_PI;
}

void printCorrection(const char * label, const double p[6])
{
	float x, y, z, roll, pitch, yaw;
	mountCorrection(p).getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
	printf("%s xyz=(%.3f, %.3f, %.3f) m  rpy=(%.2f, %.2f, %.2f) deg\n", label, x, y, z,
			roll * 180.0f / float(M_PI), pitch * 180.0f / float(M_PI), yaw * 180.0f / float(M_PI));
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	if(argc < 2)
	{
		showUsage(argv[0]);
	}

	bool translation = false;
	bool verbose = false;
	std::string solverName = "simplex";
	std::string imagesDir;
	int decimation = 4;
	float jump = 0.15f;
	float intensityJump = 0.4f;
	float intensityWeight = 1.0f;
	float creaseAngle = 45.0f;
	float creaseVoxel = 0.1f;
	float sigma = 3.0f;
	float voxel = 0.0f;
	float minDepth = 0.5f;
	float maxAngularSpeed = 0.0f;
	float maxLinearSpeed = 0.0f;
	double initialRotation[3] = {0, 0, 0};
	for(int i = 1; i < argc - 1; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage(argv[0]);
		}
		else if(std::strcmp(argv[i], "--solver") == 0 && i + 1 < argc - 1)
		{
			solverName = argv[++i];
		}
		else if(std::strcmp(argv[i], "--verbose") == 0)
		{
			verbose = true;
		}
		else if(std::strcmp(argv[i], "--translation") == 0)
		{
			translation = true;
		}
		else if(std::strcmp(argv[i], "--images") == 0 && i + 1 < argc - 1)
		{
			imagesDir = argv[++i];
		}
		else if(std::strcmp(argv[i], "--decimation") == 0 && i + 1 < argc - 1)
		{
			decimation = std::max(1, uStr2Int(argv[++i]));
		}
		else if(std::strcmp(argv[i], "--jump") == 0 && i + 1 < argc - 1)
		{
			jump = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--intensity_jump") == 0 && i + 1 < argc - 1)
		{
			intensityJump = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--intensity_weight") == 0 && i + 1 < argc - 1)
		{
			intensityWeight = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--crease_angle") == 0 && i + 1 < argc - 1)
		{
			creaseAngle = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--crease_voxel") == 0 && i + 1 < argc - 1)
		{
			creaseVoxel = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--no_intensity") == 0)
		{
			intensityJump = 0.0f;
		}
		else if(std::strcmp(argv[i], "--sigma") == 0 && i + 1 < argc - 1)
		{
			sigma = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--voxel") == 0 && i + 1 < argc - 1)
		{
			voxel = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--min_depth") == 0 && i + 1 < argc - 1)
		{
			minDepth = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--max_angular_speed") == 0 && i + 1 < argc - 1)
		{
			maxAngularSpeed = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--max_linear_speed") == 0 && i + 1 < argc - 1)
		{
			maxLinearSpeed = uStr2Float(argv[++i]);
		}
		else if(std::strcmp(argv[i], "--initial_rotation") == 0 && i + 3 < argc - 1)
		{
			for(int k = 0; k < 3; ++k)
			{
				initialRotation[k] = uStr2Double(argv[++i]);
			}
		}
		else
		{
			printf("Unknown option \"%s\"\n", argv[i]);
			showUsage(argv[0]);
		}
	}
	const std::unique_ptr<CorrectionSolver> solver = createSolver(solverName, sigma);
	if(!solver)
	{
		printf("Unknown solver \"%s\".\n", solverName.c_str());
		showUsage(argv[0]);
	}
	{
		// The least-squares solvers' double precision correction must be correctionFrom()'s.
		const double t[6] = {0.01, -0.02, 0.03, 1.5, -2.5, 3.5};
		const double diff = (correctionFrom(t).toEigen3d().matrix() - correctionFromDouble(t).matrix()).cwiseAbs().maxCoeff();
		UASSERT_MSG(diff < 1e-5, uFormat("correctionFromDouble() differs from correctionFrom() by %g", diff).c_str());
		double back[6];
		parametersFromMountCorrection(mountCorrection(t), back);
		for(int k = 0; k < 6; ++k)
		{
			UASSERT_MSG(std::fabs(back[k] - t[k]) < 1e-4, "parametersFromMountCorrection() is not mountCorrection()'s inverse");
		}
	}
	const std::string databasePath = argv[argc - 1];
	if(std::string(databasePath).rfind("--", 0) == 0)
	{
		showUsage(argv[0]);
	}

	DBDriver * driver = DBDriver::create();
	if(!driver->openConnection(databasePath, false, /*readOnly=*/true))
	{
		printf("Cannot open database \"%s\".\n", databasePath.c_str());
		delete driver;
		return 1;
	}
	// All the nodes, with the intermediate ones (only odometry poses, recorded between the
	// nodes with data when the map was made with Rtabmap/CreateIntermediateNodes), for the
	// speeds; the nodes with data for the calibration.
	UTimer totalTimer;
	UTimer stepTimer;
	std::vector<std::pair<std::string, double> > times;  // step, s
	double normalsTime = 0.0;
	std::set<int> idSet;
	driver->getAllNodeIds(idSet, true);
	std::list<int> ids(idSet.begin(), idSet.end());
	std::list<Signature *> allSignatures;
	driver->loadSignatures(ids, allSignatures);
	struct OdometrySample
	{
		Transform pose;
		double stamp;
		bool intermediate;
	};
	std::map<int, OdometrySample> odometry;
	std::list<Signature *> signatures;
	int intermediateNodes = 0;
	for(Signature * s : allSignatures)
	{
		const bool intermediate = s->getWeight() == -1;
		odometry.insert(std::make_pair(s->id(), OdometrySample{s->getPose(), s->getStamp(), intermediate}));
		if(intermediate)
		{
			++intermediateNodes;
			delete s;
		}
		else
		{
			signatures.push_back(s);
		}
	}
	driver->loadNodeData(signatures, true, true, false, false);

	std::vector<Frame> frames;
	int skipped = 0;
	int tooFast = 0;
	int noVelocity = 0;
	for(Signature * s : signatures)
	{
		// The node's speeds. The instantaneous one is odometry's when the node was added,
		// at the end of its scan. An assembled scan is taken since the previous node: its
		// mean speed over that time is along the odometry poses from the previous node to
		// this one, through the intermediate nodes if any (else only the net motion between
		// both, less than the path if the robot went back and forth). While the robot
		// moves fast, an error in the sensors' time synchronization, motion within the
		// assembled scan, and motion blur or rolling shutter in the image move the edges more.
		float linearSpeed = -1.0f, angularSpeed = -1.0f;
		float meanLinearSpeed = -1.0f, meanAngularSpeed = -1.0f, meanWindow = 0.0f;
		int meanPoses = 0;
		const std::vector<float> & velocity = s->getVelocity();
		if(velocity.size() == 6)
		{
			linearSpeed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
			angularSpeed = std::sqrt(velocity[3]*velocity[3] + velocity[4]*velocity[4] + velocity[5]*velocity[5]) * 180.0f / float(M_PI);
		}
		std::map<int, OdometrySample>::const_iterator current = odometry.find(s->id());
		std::map<int, OdometrySample>::const_iterator start = current;
		while(start != odometry.begin())
		{
			--start;
			if(!start->second.intermediate)
			{
				break;
			}
		}
		if(start != current && !start->second.intermediate)
		{
			const double dt = current->second.stamp - start->second.stamp;
			if(dt > 0.0 && dt < 10.0)
			{
				double distance = 0.0, angle = 0.0;
				bool valid = true;
				for(std::map<int, OdometrySample>::const_iterator iter = start; iter != current && valid; ++iter)
				{
					const Transform & from = iter->second.pose;
					const Transform & to = std::next(iter)->second.pose;
					valid = !from.isNull() && !to.isNull();
					if(valid)
					{
						const Transform motion = from.inverse() * to;
						distance += motion.getNorm();
						angle += Eigen::AngleAxisf(motion.toEigen3f().linear()).angle();
						++meanPoses;
					}
				}
				if(valid)
				{
					meanLinearSpeed = distance / dt;
					meanAngularSpeed = angle * 180.0 / M_PI / dt;
					meanWindow = dt;
				}
			}
		}
		if(maxAngularSpeed > 0.0f || maxLinearSpeed > 0.0f)
		{
			// The mean speed over the scan, else the instantaneous one.
			const float linear = meanLinearSpeed >= 0.0f ? meanLinearSpeed : linearSpeed;
			const float angular = meanAngularSpeed >= 0.0f ? meanAngularSpeed : angularSpeed;
			if(angular < 0.0f)
			{
				++noVelocity;
			}
			else if((maxAngularSpeed > 0.0f && angular > maxAngularSpeed) || (maxLinearSpeed > 0.0f && linear > maxLinearSpeed))
			{
				++tooFast;
				delete s;
				continue;
			}
		}
		SensorData & data = s->sensorData();
		cv::Mat image, depth;
		LaserScan scan;
		data.uncompressData(&image, &depth, &scan);
		if(voxel > 0.0f && !scan.isEmpty())
		{
			scan = util3d::commonFiltering(scan, 1, 0.0f, 0.0f, voxel);
		}
		if(!image.empty() && !scan.isEmpty() && data.cameraModels().size() == 1 &&
		   data.cameraModels()[0].isValidForProjection())
		{
			const CameraModel & model = data.cameraModels()[0];
			Frame f;
			f.id = s->id();
			f.linearSpeed = linearSpeed;
			f.angularSpeed = angularSpeed;
			f.meanLinearSpeed = meanLinearSpeed;
			f.meanAngularSpeed = meanAngularSpeed;
			f.meanWindow = meanWindow;
			f.meanPoses = meanPoses;
			if(image.channels() == 3)
			{
				cv::cvtColor(image, f.gray, cv::COLOR_BGR2GRAY);
			}
			else
			{
				f.gray = image;
			}
			computeEdgeMaps(f, sigma);
			f.K = model.K().clone();
			f.scanToCam = scan.localTransform().inverse() * model.localTransform();
			// Scan frame, as stored.
			f.hasIntensity = scan.hasIntensity();
			f.cloud = cv::Mat(scan.size(), 4, CV_32F, cv::Scalar(0));
			for(int i = 0; i < scan.size(); ++i)
			{
				const float * p = scan.data().ptr<float>(0, i);
				f.cloud.at<float>(i, 0) = p[0];
				f.cloud.at<float>(i, 1) = p[1];
				f.cloud.at<float>(i, 2) = scan.is2d() ? 0.0f : p[2];
				if(f.hasIntensity)
				{
					f.cloud.at<float>(i, 3) = p[scan.getIntensityOffset()];
				}
			}
			if((creaseAngle > 0.0f || !imagesDir.empty()) && !scan.is2d())
			{
				// Normals on a voxelized copy: smoother than at full resolution, and much
				// faster. Oriented toward the lidar (the scan frame's origin). Also for the
				// images, which show them.
				UTimer normalsTimer;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				cloud->resize(f.cloud.rows);
				for(int i = 0; i < f.cloud.rows; ++i)
				{
					cloud->at(i) = pcl::PointXYZ(f.cloud.at<float>(i, 0), f.cloud.at<float>(i, 1), f.cloud.at<float>(i, 2));
				}
				if(creaseVoxel > 0.0f)
				{
					cloud = util3d::voxelize(cloud, creaseVoxel);
				}
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, 20);
				f.normals = cv::Mat(cloud->size(), 6, CV_32F);
				for(size_t i = 0; i < cloud->size(); ++i)
				{
					float * n = f.normals.ptr<float>(i);
					n[0] = cloud->at(i).x; n[1] = cloud->at(i).y; n[2] = cloud->at(i).z;
					n[3] = normals->at(i).normal_x; n[4] = normals->at(i).normal_y; n[5] = normals->at(i).normal_z;
				}
				normalsTime += normalsTimer.elapsed();
			}
			frames.push_back(f);
		}
		else
		{
			++skipped;
		}
		delete s;
	}
	driver->closeConnection(false);
	delete driver;
	times.push_back(std::make_pair(std::string("Loading the nodes (image edges, scans)"), stepTimer.ticks() - normalsTime));
	if(normalsTime > 0.0)
	{
		times.push_back(std::make_pair(std::string("Normals of the scans"), normalsTime));
	}
	size_t points = 0;
	for(const Frame & f : frames)
	{
		points += f.cloud.rows;
	}
	printf("%d nodes with an image (one camera) and a lidar scan, %d skipped. %d lidar points per node on average%s (%.1f s).\n",
			(int)frames.size(), skipped, frames.empty() ? 0 : int(points / frames.size()),
			voxel > 0.0f ? uFormat(" after a %g m voxel filter", voxel).c_str() : "", totalTimer.elapsed());
	if(intermediateNodes)
	{
		printf("%d intermediate nodes (odometry poses only), for the nodes' mean speeds.\n", intermediateNodes);
	}
	if(maxAngularSpeed > 0.0f || maxLinearSpeed > 0.0f)
	{
		printf("%d nodes moving too fast skipped (rotating faster than %s, moving faster than %s)%s.\n", tooFast,
				maxAngularSpeed > 0.0f ? uFormat("%g deg/s", maxAngularSpeed).c_str() : "any",
				maxLinearSpeed > 0.0f ? uFormat("%g m/s", maxLinearSpeed).c_str() : "any",
				noVelocity ? uFormat(", %d nodes without a speed kept", noVelocity).c_str() : "");
	}
	if(frames.size() < 2)
	{
		printf("Not enough nodes.\n");
		return 1;
	}

	std::vector<int> all, even, odd;
	for(size_t i = 0; i < frames.size(); ++i)
	{
		all.push_back(i);
		(i % 2 ? odd : even).push_back(i);
	}

	const CalibrationProblem problem(frames, all, minDepth);
	printf("Solver: %s\n", solver->name());

	// Discontinuities as seen with the current extrinsics (or with the initial rotation
	// given), then again with the result.
	double initial[6] = {0, 0, 0, 0, 0, 0};
	parametersFromMountCorrection(Transform(0, 0, 0, initialRotation[0] * M_PI / 180.0,
			initialRotation[1] * M_PI / 180.0, initialRotation[2] * M_PI / 180.0), initial);
	if(initialRotation[0] != 0 || initialRotation[1] != 0 || initialRotation[2] != 0)
	{
		printCorrection("Starting from:", initial);
	}
	double p[6];
	std::copy(initial, initial + 6, p);
	for(int pass = 0; pass < 2; ++pass)
	{
		size_t points = 0;
		size_t byType[3] = {0, 0, 0};
		stepTimer.start();
		for(Frame & f : frames)
		{
			selectEdgePoints(f, correctionFrom(p), decimation, jump, intensityJump, intensityWeight, creaseAngle, creaseVoxel, minDepth);
			points += f.edgePoints.size();
			for(unsigned char t : f.edgeTypes)
			{
				++byType[t];
			}
		}
		const double selectionTime = stepTimer.ticks();
		const double before = problem.score(correctionFrom(p));
		const int evaluations = problem.evaluations();
		solver->solve(problem, translation, p);
		const double solverTime = stepTimer.ticks();
		times.push_back(std::make_pair(uFormat("Pass %d: lidar edge points", pass + 1), selectionTime));
		times.push_back(std::make_pair(uFormat("Pass %d: solver", pass + 1), solverTime));
		printf("Pass %d: %d lidar edge points (depth %d, crease %d, intensity %d), score %.4f -> %.4f (%d score evaluations, %.1f s + %.1f s)\n",
				pass + 1, (int)points, (int)byType[kEdgeDepth], (int)byType[kEdgeCrease], (int)byType[kEdgeIntensity],
				before, problem.score(correctionFrom(p)), problem.evaluations() - evaluations - 1, selectionTime, solverTime);
	}

	{
		// How often each kind of lidar edge lands on an image edge once corrected: what
		// each brings, and how much of it is noise.
		size_t near[3] = {0, 0, 0}, total[3] = {0, 0, 0};
		for(const Frame & f : frames)
		{
			for(unsigned char t : f.edgeTypes)
			{
				++total[t];
			}
		}
		problem.project(correctionFrom(p), [&](const Frame & f, size_t i, float u, float v) {
			if(f.edgeDistance.at<float>(int(v + 0.5f), int(u + 0.5f)) <= 2.0f)
			{
				++near[f.edgeTypes[i]];
			}
		});
		printf("\nLidar edge points within 2 pixels of an image edge, once corrected: depth %.0f%%, crease %.0f%%, intensity %.0f%%\n",
				total[kEdgeDepth] ? 100.0 * near[kEdgeDepth] / total[kEdgeDepth] : 0.0,
				total[kEdgeCrease] ? 100.0 * near[kEdgeCrease] / total[kEdgeCrease] : 0.0,
				total[kEdgeIntensity] ? 100.0 * near[kEdgeIntensity] / total[kEdgeIntensity] : 0.0);
	}

	printf("\nConsistency, each half of the nodes on its own:\n");
	stepTimer.start();
	for(int h = 0; h < 2; ++h)
	{
		double q[6];
		std::copy(initial, initial + 6, q);
		solver->solve(CalibrationProblem(frames, h ? odd : even, minDepth), translation, q);
		printCorrection(h ? "  odd nodes: " : "  even nodes:", q);
	}
	times.push_back(std::make_pair(std::string("Halves of the nodes"), stepTimer.ticks()));

	if(verbose)
	{
		// Sensitivity: how much the score drops with the result off by 1 deg or 2 cm along
		// or about each of the camera's body axes (mean of both directions). A large drop:
		// the data determines that axis well; almost none: it is not observable from it.
		const double best = problem.score(correctionFrom(p));
		const char * names[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
		const double probe[6] = {0.02, 0.02, 0.02, 1.0, 1.0, 1.0};
		double drop[6];
		for(int k = 0; k < 6; ++k)
		{
			double sum = 0.0;
			for(int sign = 0; sign < 2; ++sign)
			{
				double delta[6] = {0, 0, 0, 0, 0, 0};
				delta[k] = sign ? -probe[k] : probe[k];
				const Transform moved = mountCorrection(p) * Transform(delta[0], delta[1], delta[2],
						delta[3] * M_PI / 180.0, delta[4] * M_PI / 180.0, delta[5] * M_PI / 180.0);
				double q[6];
				parametersFromMountCorrection(moved, q);
				sum += problem.score(correctionFrom(q));
			}
			drop[k] = best > 0.0 ? 100.0 * (best - sum / 2.0) / best : 0.0;
		}
		printf("\nSensitivity (score drop with the result off by 1 deg or 2 cm; little: not observable):\n"
				"  %s %.1f%%  %s %.1f%%  %s %.1f%%  |  %s %.1f%%  %s %.1f%%  %s %.1f%%\n",
				names[3], drop[3], names[4], drop[4], names[5], drop[5], names[0], drop[0], names[1], drop[1], names[2], drop[2]);
		times.push_back(std::make_pair(std::string("Sensitivity"), stepTimer.ticks()));
	}

	if(!imagesDir.empty())
	{
		UDirectory::makeDir(imagesDir);
		for(size_t i = 0; i < frames.size(); ++i)
		{
			const std::string prefix = imagesDir + "/" + uFormat("%04d", frames[i].id);
			// Before: from where the search started (with the initial rotation, if any).
			saveOverlay(frames[i], correctionFrom(initial), minDepth, prefix + "_edges_1_before.png");
			saveOverlay(frames[i], correctionFrom(p), minDepth, prefix + "_edges_2_after.png");
			saveCellMapOverlays(frames[i], correctionFrom(initial), decimation, creaseVoxel, minDepth, prefix, "_1_before.png");
			saveCellMapOverlays(frames[i], correctionFrom(p), decimation, creaseVoxel, minDepth, prefix, "_2_after.png");
		}
		printf("\nImage overlays saved to %s\n", imagesDir.c_str());
		times.push_back(std::make_pair(std::string("Images"), stepTimer.ticks()));
	}

	printf("\n");
	const Transform X = mountCorrection(p);
	{
		float x, y, z, roll, pitch, yaw;
		X.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		const Eigen::Quaternionf q = X.getQuaternionf();
		printf("Correction of the camera's mount X, in the camera's body frame (x forward, y left, z up):\n"
				"    xyz (m): %.6f %.6f %.6f\n"
				"    roll pitch yaw (rad): %.6f %.6f %.6f  (deg: %.4f %.4f %.4f)\n"
				"    quaternion (x y z w): %.6f %.6f %.6f %.6f\n\n",
				x, y, z, roll, pitch, yaw, roll * 180.0f / float(M_PI), pitch * 180.0f / float(M_PI), yaw * 180.0f / float(M_PI),
				q.x(), q.y(), q.z(), q.w());
	}
	if(initialRotation[0] != 0 || initialRotation[1] != 0 || initialRotation[2] != 0)
	{
		// As if the database's camera transform were off by the initial rotation: what
		// undoes it, to see that the result was found again from there.
		const Transform X0(0, 0, 0, initialRotation[0] * M_PI / 180.0, initialRotation[1] * M_PI / 180.0, initialRotation[2] * M_PI / 180.0);
		float x, y, z, roll, pitch, yaw;
		(X0.inverse() * X).getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		printf("    relative to the initial rotation (%g, %g, %g deg): rpy (deg): %.4f %.4f %.4f\n\n",
				initialRotation[0], initialRotation[1], initialRotation[2],
				roll * 180.0f / float(M_PI), pitch * 180.0f / float(M_PI), yaw * 180.0f / float(M_PI));
	}
	printf("The robot base -> camera transform B (e.g., base_link -> camera_link) becomes B * X.\n"
			"Or insert X after B in TF: base_link -[B]-> camera_link_measured -[X]-> camera_link,\n"
			"the rest unchanged.\n");

	printf("\nTime:\n");
	for(const std::pair<std::string, double> & t : times)
	{
		printf("  %-40s %7.1f s\n", t.first.c_str(), t.second);
	}
	printf("  %-40s %7.1f s\n", "Total", totalTimer.elapsed());
	return 0;
}
