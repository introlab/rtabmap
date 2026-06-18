/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <set>

#include <rtabmap/core/optimizer/OptimizerCeres.h>

#ifdef RTABMAP_CERES
#include <ceres/ceres.h>

#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#include <ceres/manifold.h>
#else
#include <ceres/local_parameterization.h>
#endif

#include "ceres/pose_graph_2d/types.h"
#include "ceres/pose_graph_2d/pose_graph_2d_error_term.h"
#include "ceres/pose_graph_2d/angle_manifold.h"
#include "ceres/pose_graph_3d/types.h"
#include "ceres/pose_graph_3d/pose_graph_3d_error_term.h"
#include "ceres/bundle/BAProblem.h"
#include "ceres/bundle/snavely_reprojection_error.h"
#include "ceres/bundle/snavely_stereo_reprojection_error.h"
#include "ceres/bundle/between_cameras_error.h"
#include "ceres/bundle/planar_constraint_error.h"

#if not(CERES_VERSION_MAJOR > 1 || (CERES_VERSION_MAJOR == 1 && CERES_VERSION_MINOR >= 12))
#include "ceres/pose_graph_3d/eigen_quaternion_manifold.h"
#endif

#endif

namespace rtabmap {
namespace {

#ifdef RTABMAP_CERES
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
inline void SetCeresProblemManifold(ceres::Problem& problem, double* params,
                                    ceres::Manifold* manifold) {
  problem.SetManifold(params, manifold);
#else
inline void SetCeresProblemManifold(
    ceres::Problem& problem, double* params,
    ceres::LocalParameterization* parameterization) {
  problem.SetParameterization(params, parameterization);
#endif
}
#endif

}  // namespace

bool OptimizerCeres::available()
{
#ifdef RTABMAP_CERES
	return true;
#else
	return false;
#endif
}

OptimizerCeres::OptimizerCeres(const ParametersMap & parameters) :
		Optimizer(parameters),
		pixelVariance_(Parameters::defaultOptimizerPixelVariance()),
		disparityVariance_(Parameters::defaultOptimizerDisparityVariance()),
		robustKernelDelta_(Parameters::defaultOptimizerRobustKernelDelta()),
		baseline_(Parameters::defaultOptimizerBaseline())
{
	parseParameters(parameters);
}

void OptimizerCeres::parseParameters(const ParametersMap & parameters)
{
	Optimizer::parseParameters(parameters);
	Parameters::parse(parameters, Parameters::kOptimizerPixelVariance(),     pixelVariance_);
	Parameters::parse(parameters, Parameters::kOptimizerDisparityVariance(), disparityVariance_);
	Parameters::parse(parameters, Parameters::kOptimizerRobustKernelDelta(), robustKernelDelta_);
	Parameters::parse(parameters, Parameters::kOptimizerBaseline(),          baseline_);
	UASSERT(pixelVariance_ > 0.0);
	UASSERT(disparityVariance_ > 0.0);
	UASSERT(baseline_ >= 0.0);
}

std::map<int, Transform> OptimizerCeres::optimize(
		int rootId,
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & edgeConstraints,
		cv::Mat & outputCovariance,
		std::list<std::map<int, Transform> > * intermediateGraphes, // contains poses after tree init to last one before the end
		double * finalError,
		int * iterationsDone)
{
	outputCovariance = cv::Mat::eye(6,6,CV_64FC1);
	std::map<int, Transform> optimizedPoses;
#ifdef RTABMAP_CERES
	UDEBUG("Optimizing graph (pose=%d constraints=%d)...", (int)poses.size(), (int)edgeConstraints.size());
	if(edgeConstraints.size()>=1 && poses.size()>=2 && iterations() > 0)
	{
		//Build problem
		ceres::Problem problem;
		std::map<int, ceres::examples::Pose2d> poses2d;
		ceres::examples::MapOfPoses poses3d;

		UDEBUG("fill poses to Ceres...");
		if(isSlam2d())
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					UASSERT(!iter->second.isNull());
					ceres::examples::Pose2d p;
					p.x = iter->second.x();
					p.y = iter->second.y();
					p.yaw_radians = ceres::examples::NormalizeAngle(iter->second.theta());
					poses2d.insert(std::make_pair(iter->first, p));
				}
			}
		}
		else
		{
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					UASSERT(!iter->second.isNull());
					ceres::examples::Pose3d p;
					p.p.x() = iter->second.x();
					p.p.y() = iter->second.y();
					p.p.z() = iter->second.z();
					p.q = iter->second.getQuaterniond();
					poses3d.insert(std::make_pair(iter->first, p));
				}
			}

		}

		ceres::LossFunction* loss_function = NULL;
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
		ceres::Manifold* angle_local_manifold = NULL;
		ceres::Manifold* quaternion_local_manifold = NULL;
#else
		ceres::LocalParameterization* angle_local_manifold = NULL;
		ceres::LocalParameterization* quaternion_local_manifold = NULL;
#endif

		for(std::multimap<int, Link>::const_iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
		{
			int id1 = iter->second.from();
			int id2 = iter->second.to();

			if(id1 != id2 && id1 > 0 && id2 > 0)
			{
				UASSERT(poses.find(id1) != poses.end() && poses.find(id2) != poses.end());

				if(isSlam2d())
				{
					Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
					if(!isCovarianceIgnored())
					{
						information(0,0) = iter->second.infMatrix().at<double>(0,0); // x-x
						information(0,1) = iter->second.infMatrix().at<double>(0,1); // x-y
						information(0,2) = iter->second.infMatrix().at<double>(0,5); // x-theta
						information(1,0) = iter->second.infMatrix().at<double>(1,0); // y-x
						information(1,1) = iter->second.infMatrix().at<double>(1,1); // y-y
						information(1,2) = iter->second.infMatrix().at<double>(1,5); // y-theta
						information(2,0) = iter->second.infMatrix().at<double>(5,0); // theta-x
						information(2,1) = iter->second.infMatrix().at<double>(5,1); // theta-y
						information(2,2) = iter->second.infMatrix().at<double>(5,5); // theta-theta
					}

					float yaw_radians = ceres::examples::NormalizeAngle(iter->second.transform().theta());
					const Eigen::Matrix3d sqrt_information = information.llt().matrixU();

					// Ceres will take ownership of the pointer.
					ceres::CostFunction* cost_function = ceres::examples::PoseGraph2dErrorTerm::Create(
							iter->second.transform().x(),
							iter->second.transform().y(),
							yaw_radians,
							sqrt_information);

					std::map<int, ceres::examples::Pose2d>::iterator pose_begin_iter = poses2d.find(id1);
					std::map<int, ceres::examples::Pose2d>::iterator pose_end_iter = poses2d.find(id2);

					problem.AddResidualBlock(
						cost_function, loss_function,
						&pose_begin_iter->second.x,	&pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
						&pose_end_iter->second.x, &pose_end_iter->second.y, &pose_end_iter->second.yaw_radians);

					if(angle_local_manifold == NULL)
					{
						angle_local_manifold = ceres::examples::AngleManifold::Create();
					}
					SetCeresProblemManifold(problem, &pose_begin_iter->second.yaw_radians, angle_local_manifold);
					SetCeresProblemManifold(problem, &pose_end_iter->second.yaw_radians, angle_local_manifold);
				}
				else
				{
					ceres::examples::MapOfPoses::iterator pose_begin_iter = poses3d.find(id1);
					ceres::examples::MapOfPoses::iterator pose_end_iter = poses3d.find(id2);
					ceres::examples::Constraint3d constraint;
					Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
					if(!isCovarianceIgnored())
					{
						memcpy(information.data(), iter->second.infMatrix().data, iter->second.infMatrix().total()*sizeof(double));
					}

					ceres::examples::Pose3d t;
					t.p.x() = iter->second.transform().x();
					t.p.y() = iter->second.transform().y();
					t.p.z() = iter->second.transform().z();
					t.q = iter->second.transform().getQuaterniond();

					const Eigen::Matrix<double, 6, 6> sqrt_information = information.llt().matrixU();
					// Ceres will take ownership of the pointer.
					ceres::CostFunction* cost_function = ceres::examples::PoseGraph3dErrorTerm::Create(t, sqrt_information);
					problem.AddResidualBlock(cost_function, loss_function,
											  pose_begin_iter->second.p.data(), pose_begin_iter->second.q.coeffs().data(),
											  pose_end_iter->second.p.data(), pose_end_iter->second.q.coeffs().data());
					if(quaternion_local_manifold == NULL)
					{
#if CERES_VERSION_MAJOR >= 3 || \
    (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
						quaternion_local_manifold = new ceres::EigenQuaternionManifold;
#else
						quaternion_local_manifold = new ceres::EigenQuaternionParameterization;
#endif
					}
					SetCeresProblemManifold(problem, pose_begin_iter->second.q.coeffs().data(), quaternion_local_manifold);
					SetCeresProblemManifold(problem, pose_end_iter->second.q.coeffs().data(), quaternion_local_manifold);
				}
			}
			//else // not supporting pose prior and landmarks
		}

		if(isSlam2d())
		{
			// The pose graph optimization problem has three DOFs that are not fully
			// constrained. This is typically referred to as gauge freedom. You can apply
			// a rigid body transformation to all the nodes and the optimization problem
			// will still have the exact same cost. The Levenberg-Marquardt algorithm has
			// internal damping which mitigate this issue, but it is better to properly
			// constrain the gauge freedom. This can be done by setting one of the poses
			// as constant so the optimizer cannot change it.
			std::map<int, ceres::examples::Pose2d>::iterator pose_start_iter = rootId>0?poses2d.find(rootId):poses2d.begin();
			UASSERT(pose_start_iter != poses2d.end());
			problem.SetParameterBlockConstant(&pose_start_iter->second.x);
			problem.SetParameterBlockConstant(&pose_start_iter->second.y);
			problem.SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
		}
		else
		{
			// The pose graph optimization problem has six DOFs that are not fully
			// constrained. This is typically referred to as gauge freedom. You can apply
			// a rigid body transformation to all the nodes and the optimization problem
			// will still have the exact same cost. The Levenberg-Marquardt algorithm has
			// internal damping which mitigates this issue, but it is better to properly
			// constrain the gauge freedom. This can be done by setting one of the poses
			// as constant so the optimizer cannot change it.
			ceres::examples::MapOfPoses::iterator pose_start_iter = rootId>0?poses3d.find(rootId):poses3d.begin();
			UASSERT(pose_start_iter != poses3d.end());
			problem.SetParameterBlockConstant(pose_start_iter->second.p.data());
			problem.SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
		}

		UINFO("Ceres optimizing begin (iterations=%d)", iterations());

		ceres::Solver::Options options;
		// SPARSE_NORMAL_CHOLESKY is the standard linear solver for
		// pose-graph SLAM: the Hessian is a sparse symmetric positive
		// definite matrix over pose blocks, which Cholesky factors
		// directly and robustly.
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
		options.max_num_iterations = iterations();
		options.function_tolerance = this->epsilon();
		ceres::Solver::Summary summary;
		UTimer timer;
		ceres::Solve(options, &problem, &summary);
		if(ULogger::level() == ULogger::kDebug)
		{
			UDEBUG("Ceres Report:");
			std::cout << summary.FullReport() << '\n';
		}
		if(!summary.IsSolutionUsable())
		{
			UWARN("ceres: Could not find a usable solution, aborting optimization!");
			return optimizedPoses;
		}

		if(finalError)
		{
			*finalError = summary.final_cost;
		}
		if(iterationsDone)
		{
			*iterationsDone = summary.iterations.size();
		}
		UINFO("Ceres optimizing end (%d iterations done, error=%f, time = %f s)", (int)summary.iterations.size(), summary.final_cost, timer.ticks());


		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first > 0)
			{
				if(isSlam2d())
				{
					const std::map<int, ceres::examples::Pose2d>::iterator & pter = poses2d.find(iter->first);
					float roll, pitch, yaw;
					iter->second.getEulerAngles(roll, pitch, yaw);

					Transform newPose(pter->second.x, pter->second.y, iter->second.z(), roll, pitch, pter->second.yaw_radians);

					UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));
				}
				else
				{
					const std::map<int, ceres::examples::Pose3d, std::less<int>,
								   Eigen::aligned_allocator<std::pair<const int, ceres::examples::Pose3d> > >::
						iterator& pter = poses3d.find(iter->first);

					Transform newPose(pter->second.p.x(), pter->second.p.y(), pter->second.p.z(), pter->second.q.x(), pter->second.q.y(), pter->second.q.z(), pter->second.q.w());

					UASSERT_MSG(!newPose.isNull(), uFormat("Optimized pose %d is null!?!?", iter->first).c_str());
					optimizedPoses.insert(std::pair<int, Transform>(iter->first, newPose));

				}
			}
		}

		// Ceres doesn't compute marginals...
	}
	else if(poses.size() == 1 || iterations() <= 0)
	{
		optimizedPoses = poses;
	}
	else
	{
		UWARN("This method should be called at least with 1 pose!");
	}
	UDEBUG("Optimizing graph...end!");
#else
	UERROR("Not built with Ceres support!");
#endif
	return optimizedPoses;
}

std::map<int, Transform> OptimizerCeres::optimizeBA(
		int rootId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & links,
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences, // <ID words, IDs frames + keypoint/Disparity>)
		std::set<int> * outliers)
{
#ifdef RTABMAP_CERES
	// run sba optimization

	std::map<int, Transform> poses(posesIn.lower_bound(1), posesIn.end());

	ceres::BAProblem baProblem;

	// Multi-camera support: each (pose, camera-in-rig) pair becomes its
	// own parameter block. Total camera blocks = sum over poses of rig
	// size. Single-cam rigs (the common case) reduce to 1 block per pose.
	int totalCameras = 0;
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
		UASSERT(iterModel != models.end() && !iterModel->second.empty());
		totalCameras += static_cast<int>(iterModel->second.size());
	}

	baProblem.num_cameras_ = totalCameras;
	baProblem.num_points_ = points3DMap.size();
	baProblem.num_observations_ = 0;
	for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter=wordReferences.begin();
		iter!=wordReferences.end();
		++iter)
	{
		baProblem.num_observations_ += iter->second.size();
	}

	baProblem.point_index_ = new int[baProblem.num_observations_];
	baProblem.camera_index_ = new int[baProblem.num_observations_];
	baProblem.observations_ = new double[4 * baProblem.num_observations_];
	baProblem.cameras_ = new double[6 * baProblem.num_cameras_];
	baProblem.points_ = new double[3 * baProblem.num_points_];

	// Each camera is a set of 6 parameters: R and t. The rotation R is
	// specified as a Rodrigues' vector. The map is keyed on
	// (poseId, camIdx-in-rig) so multi-camera observations can look up
	// the correct vertex.
	int oi=0;
	int camIndex=0;
	std::map<std::pair<int,int>, int> camIdxByKey;
	for(std::map<int, Transform>::const_iterator iter=poses.begin();
		iter!=poses.end();
		++iter)
	{
		std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
		UASSERT(iterModel != models.end());

		for(size_t c = 0; c < iterModel->second.size(); ++c)
		{
			const CameraModel & m = iterModel->second[c];
			UASSERT(m.isValidForProjection());

			const Transform t = (iter->second * m.localTransform()).inverse();
			cv::Mat R = (cv::Mat_<double>(3,3) <<
					(double)t.r11(), (double)t.r12(), (double)t.r13(),
					(double)t.r21(), (double)t.r22(), (double)t.r23(),
					(double)t.r31(), (double)t.r32(), (double)t.r33());

			cv::Mat rvec(1,3, CV_64FC1);
			cv::Rodrigues(R, rvec);

			UASSERT(oi+6 <= baProblem.num_cameras_*6);

			baProblem.cameras_[oi++] = rvec.at<double>(0,0);
			baProblem.cameras_[oi++] = rvec.at<double>(0,1);
			baProblem.cameras_[oi++] = rvec.at<double>(0,2);
			baProblem.cameras_[oi++] = t.x();
			baProblem.cameras_[oi++] = t.y();
			baProblem.cameras_[oi++] = t.z();

			camIdxByKey.insert(std::make_pair(std::make_pair(iter->first, (int)c), camIndex++));
		}
	}
	UASSERT(oi == baProblem.num_cameras_*6);

	oi=0;
	int pointIndex=0;
	std::map<int, int> pointIdToIndex;
	for(std::map<int, cv::Point3f>::const_iterator kter = points3DMap.begin(); kter!=points3DMap.end(); ++kter)
	{
		UASSERT(oi+3 <= baProblem.num_points_*3);

		baProblem.points_[oi++] = kter->second.x;
		baProblem.points_[oi++] = kter->second.y;
		baProblem.points_[oi++] = kter->second.z;

		pointIdToIndex.insert(std::make_pair(kter->first, pointIndex++));
	}
	UASSERT(oi == baProblem.num_points_*3);

	// Per-observation stereo metadata. For mono observations
	// observed_disparity[i] = 0 and baseline_fx[i] = 0 -- the second-loop
	// branch picks the mono cost function in that case.
	std::vector<double> observed_disparity(baProblem.num_observations_, 0.0);
	std::vector<double> baseline_fx(baProblem.num_observations_, 0.0);

	oi = 0;
	for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter=wordReferences.begin();
		iter!=wordReferences.end();
		++iter)
	{
		for(std::map<int, FeatureBA>::const_iterator jter=iter->second.begin();
			jter!=iter->second.end();
			++jter)
		{
			const int poseId = jter->first;
			const int camIdx = jter->second.cameraIndex;
			std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(poseId);
			UASSERT(iterModel != models.end());
			UASSERT(camIdx >= 0 && camIdx < (int)iterModel->second.size());
			const CameraModel & m = iterModel->second[camIdx];
			UASSERT(m.isValidForProjection());

			std::map<std::pair<int,int>, int>::const_iterator camIt =
					camIdxByKey.find(std::make_pair(poseId, camIdx));
			UASSERT(camIt != camIdxByKey.end());

			baProblem.camera_index_[oi] = camIt->second;
			baProblem.point_index_[oi] = pointIdToIndex.at(iter->first);
			baProblem.observations_[4*oi] = jter->second.kpt.pt.x - m.cx();
			baProblem.observations_[4*oi+1] = jter->second.kpt.pt.y - m.cy();
			baProblem.observations_[4*oi+2] = m.fx();
			baProblem.observations_[4*oi+3] = m.fy();

			// Stereo path: if a baseline is encoded in the camera model
			// (Tx<0, the rtabmap convention) AND we have a finite positive
			// depth from the observation, derive the observed disparity
			// and cache baseline*fx for the cost function. For RGB-D /
			// mono-with-depth (Tx==0) we fall back on the configurable
			// Optimizer/Baseline -- a "fake baseline" that lets BA treat
			// depth observations as stereo disparity. depth==0 or
			// effective baseline==0 -> mono observation; the second loop
			// will pick SnavelyReprojectionError instead.
			const double Tx    = m.Tx();
			const double fx    = m.fx();
			const double depth = jter->second.depth;
			const double baseline = Tx < 0.0 ? (-Tx / fx) : baseline_;
			if(baseline > 0.0 && uIsFinite(depth) && depth > 0.0)
			{
				baseline_fx[oi]        = baseline * fx;
				observed_disparity[oi] = baseline_fx[oi] / depth;
			}
			++oi;
		}
	}
	UASSERT(oi == baProblem.num_observations_);

	// Build problem
	const double* observations = baProblem.observations();
	// Create residuals for each observation in the bundle adjustment problem. The
	// parameters for cameras and points are added automatically.
	ceres::Problem problem;

	// Per-axis weighting mirrors g2o's stereo information matrix:
	// 1/pixelVariance on u/v, 1/disparityVariance on disparity. Pure
	// loop-invariant, hoisted out.
	const double inv_sigma_uv = 1.0 / std::sqrt(pixelVariance_);
	const double inv_sigma_d  = 1.0 / std::sqrt(disparityVariance_);
	int monoObsCount   = 0;
	int stereoObsCount = 0;
	for (int i = 0; i < baProblem.num_observations(); ++i) {
		const double u  = observations[4 * i];
		const double v  = observations[4 * i + 1];
		const double fx = observations[4 * i + 2];
		const double fy = observations[4 * i + 3];

		ceres::CostFunction* cost_function = 0;
		if(baseline_fx[i] > 0.0 && observed_disparity[i] > 0.0)
		{
			// Stereo (3 residuals: u, v, disparity). The disparity channel
			// pins z relative to the observing camera, which collapses the
			// mono BA gauge from 7 DOF to 6 (scale becomes observable).
			cost_function = ceres::SnavelyStereoReprojectionError::Create(
					u, v, observed_disparity[i], fx, fy, baseline_fx[i],
					inv_sigma_uv, inv_sigma_d);
			++stereoObsCount;
		}
		else
		{
			// Mono (2 residuals: u, v).
			cost_function = ceres::SnavelyReprojectionError::Create(u, v, fx, fy);
			++monoObsCount;
		}
		// Pass nullptr when robustKernelDelta_ <= 0 -- Ceres treats that as
		// identity (no kernel). A new loss instance per block is required:
		// Ceres takes ownership and deletes each.
		ceres::LossFunction* loss_function =
				robustKernelDelta_ > 0.0 ? new ceres::HuberLoss(robustKernelDelta_) : nullptr;
		problem.AddResidualBlock(cost_function,
								 loss_function,
								 baProblem.mutable_camera_for_observation(i),
								 baProblem.mutable_point_for_observation(i));
	}
	UDEBUG("Ceres BA: %d mono + %d stereo observations", monoObsCount, stereoObsCount);

	// Pose-graph constraints (kNeighbor / etc.) between cameras. Same role
	// as the EdgeSBACam edges in OptimizerG2O and the BetweenFactor<Pose3>
	// factors in OptimizerGTSAM -- folds the relative-pose chain into the
	// BA cost so chains pulled by odometry don't drift freely.
	int linkObsCount = 0;
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() <= 0 || link.to() <= 0 || link.from() == link.to())
		{
			continue;
		}
		std::map<std::pair<int,int>, int>::const_iterator itA =
				camIdxByKey.find(std::make_pair(link.from(), 0));
		std::map<std::pair<int,int>, int>::const_iterator itB =
				camIdxByKey.find(std::make_pair(link.to(),   0));
		if(itA == camIdxByKey.end() || itB == camIdxByKey.end())
		{
			continue;
		}
		// Convert link from body-to-body into camera-to-camera using the
		// localTransforms of both endpoints (same idiom as g2o / GTSAM).
		const Transform camLink = models.at(link.from())[0].localTransform().inverse() *
		                          link.transform() *
		                          models.at(link.to())[0].localTransform();

		// Decompose camLink (cam_b in cam_a's frame) into (angle-axis, translation).
		const cv::Mat R = (cv::Mat_<double>(3, 3) <<
				(double)camLink.r11(), (double)camLink.r12(), (double)camLink.r13(),
				(double)camLink.r21(), (double)camLink.r22(), (double)camLink.r23(),
				(double)camLink.r31(), (double)camLink.r32(), (double)camLink.r33());
		cv::Mat rvec(1, 3, CV_64FC1);
		cv::Rodrigues(R, rvec);
		const Eigen::Vector3d aa_meas(rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2));
		const Eigen::Vector3d t_meas(camLink.x(), camLink.y(), camLink.z());

		// Information matrix from the link covariance. rtabmap stores it as
		// [linear|angular] -- same axis order as our residual [t; rot], so
		// no block swap is needed (unlike the GTSAM path which expects
		// [angular|linear]). cv::Mat is row-major; copying into Eigen
		// (column-major) effectively transposes, which is a no-op for the
		// symmetric info matrix.
		Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
		if(!isCovarianceIgnored())
		{
			memcpy(information.data(), link.infMatrix().data, link.infMatrix().total()*sizeof(double));
		}
		// Whitening matrix: we want sqrt_info such that
		// sqrt_info^T * sqrt_info = info. LLT gives info = L*L^T with L lower
		// triangular, so the upper-triangular U = L^T (matrixU()) satisfies
		// U^T*U = info. Then ||U*r||² = r^T*info*r as desired.
		const Eigen::Matrix<double, 6, 6> sqrt_info =
				information.llt().matrixU();

		ceres::CostFunction * cost = ceres::BetweenCamerasError::Create(t_meas, aa_meas, sqrt_info);
		// No robust kernel on between-camera constraints (matches g2o/GTSAM:
		// only projection edges carry the Huber kernel in BA).
		problem.AddResidualBlock(cost,
				nullptr,
				baProblem.cameras_ + itA->second * 6,
				baProblem.cameras_ + itB->second * 6);
		++linkObsCount;
	}
	if(linkObsCount > 0)
	{
		UDEBUG("Ceres BA: %d pose-graph links", linkObsCount);
	}

	// Multi-camera rigid edges: for each pose with >1 cameras in its rig,
	// constrain cam 0 -> cam i with a high-info BetweenCamerasError (same
	// idiom as g2o's Identity*1e7 edge and GTSAM's high-info BetweenFactor
	// inside their multicam loops). The measurement is the constant
	// cam0->cami transform derived from the localTransforms.
	int rigEdgeCount = 0;
	for(std::map<int, std::vector<CameraModel> >::const_iterator iter=models.begin(); iter!=models.end(); ++iter)
	{
		if(!uContains(poses, iter->first))
		{
			continue;
		}
		if(iter->second.size() < 2)
		{
			continue;
		}
		std::map<std::pair<int,int>, int>::const_iterator cam0It =
				camIdxByKey.find(std::make_pair(iter->first, 0));
		if(cam0It == camIdxByKey.end())
		{
			continue;
		}
		const Transform & lt0 = iter->second[0].localTransform();
		// Tight info matrix (matches g2o's 9999999 diagonal).
		Eigen::Matrix<double, 6, 6> rigInfo = Eigen::Matrix<double, 6, 6>::Identity() * 9999999.0;
		const Eigen::Matrix<double, 6, 6> rigSqrtInfo = rigInfo.llt().matrixU();
		for(size_t c = 1; c < iter->second.size(); ++c)
		{
			std::map<std::pair<int,int>, int>::const_iterator camCIt =
					camIdxByKey.find(std::make_pair(iter->first, (int)c));
			if(camCIt == camIdxByKey.end())
			{
				continue;
			}
			const Transform camLink = lt0.inverse() * iter->second[c].localTransform();
			const cv::Mat R = (cv::Mat_<double>(3, 3) <<
					(double)camLink.r11(), (double)camLink.r12(), (double)camLink.r13(),
					(double)camLink.r21(), (double)camLink.r22(), (double)camLink.r23(),
					(double)camLink.r31(), (double)camLink.r32(), (double)camLink.r33());
			cv::Mat rvec(1, 3, CV_64FC1);
			cv::Rodrigues(R, rvec);
			const Eigen::Vector3d aa(rvec.at<double>(0,0), rvec.at<double>(0,1), rvec.at<double>(0,2));
			const Eigen::Vector3d tm(camLink.x(), camLink.y(), camLink.z());
			ceres::CostFunction * cost = ceres::BetweenCamerasError::Create(tm, aa, rigSqrtInfo);
			problem.AddResidualBlock(cost,
					nullptr,
					baProblem.cameras_ + cam0It->second * 6,
					baProblem.cameras_ + camCIt->second * 6);
			++rigEdgeCount;
		}
	}
	if(rigEdgeCount > 0)
	{
		UDEBUG("Ceres BA: %d multi-cam rigid edges", rigEdgeCount);
	}

	// 2D / planar BA mode: lock each non-root pose's primary (cam 0)
	// vertex to its initial body-z (lateral motion + yaw stay free).
	// Other cameras of a multi-cam rig follow via the rigid edges above.
	// Root pose's cam 0 is fixed entirely so the gauge has no remaining
	// z-DOF. Mirrors the g2o EdgeSBACamPrior path.
	if(isSlam2d())
	{
		const double sqrtInfo = std::sqrt(1e9);  // matches g2o pinfo(2,2) = 1e9
		int planarObsCount = 0;
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<std::pair<int,int>, int>::const_iterator camIt =
					camIdxByKey.find(std::make_pair(iter->first, 0));
			if(camIt == camIdxByKey.end())
			{
				continue;
			}
			double * cam_block = baProblem.cameras_ + camIt->second * 6;
			const bool fixNode = (rootId >= 0 && iter->first == rootId) ||
			                     (rootId <  0 && iter->first != -rootId);
			if(fixNode)
			{
				problem.SetParameterBlockConstant(cam_block);
				continue;
			}
			// Unary planar constraint on the BODY z (the camera vertex is in
			// world-to-camera; we extract body z by composing with the
			// inverse localTransform inside the cost function).
			std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
			if(iterModel == models.end() || iterModel->second.empty())
			{
				continue;
			}
			const Transform & localTransform = iterModel->second[0].localTransform();
			const Eigen::Matrix3d R_bc = (Eigen::Matrix3d() <<
					(double)localTransform.r11(), (double)localTransform.r12(), (double)localTransform.r13(),
					(double)localTransform.r21(), (double)localTransform.r22(), (double)localTransform.r23(),
					(double)localTransform.r31(), (double)localTransform.r32(), (double)localTransform.r33()).finished();
			const Eigen::Vector3d t_bc(localTransform.x(), localTransform.y(), localTransform.z());

			ceres::CostFunction * planar = ceres::PlanarConstraintError::Create(
					R_bc, t_bc, iter->second.z(), sqrtInfo);
			problem.AddResidualBlock(planar, nullptr, cam_block);
			++planarObsCount;
		}
		if(planarObsCount > 0)
		{
			UDEBUG("Ceres BA: %d planar-constraint blocks (2D mode)", planarObsCount);
		}
	}

	// SBA
	// Make Ceres automatically detect the bundle structure. Note that the
	// standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
	// for standard bundle adjustment problems.
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.max_num_iterations = iterations();
	//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.function_tolerance  = this->epsilon();
	// Force the LM to run to max_num_iterations rather than stopping early
	// on Ceres' default parameter/gradient tolerances. Matters in particular
	// for high-weight unary constraints (e.g. the 2D planar lock at sqrt(1e9))
	// whose residual can stall the parameter step below the default 1e-8
	// before the constraint is fully satisfied.
	options.parameter_tolerance = 0.0;
	options.gradient_tolerance  = 0.0;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	if(ULogger::level() == ULogger::kDebug)
	{
		UDEBUG("Ceres report:");
		std::cout << summary.FullReport() << "\n";
	}
	if(!summary.IsSolutionUsable())
	{
		UWARN("ceres: Could not find a usable solution, aborting optimization!");
		return poses;
	}

	//update poses (read back from cam 0 of each rig -- the other cameras
	//are rigidly constrained to it).
	std::map<int, Transform> newPoses = poses;
	for(std::map<int, Transform>::iterator iter=newPoses.begin(); iter!=newPoses.end(); ++iter)
	{
		std::map<std::pair<int,int>, int>::const_iterator camIt =
				camIdxByKey.find(std::make_pair(iter->first, 0));
		if(camIt == camIdxByKey.end())
		{
			continue;
		}
		const int oi = camIt->second * 6;
		cv::Mat rvec = (cv::Mat_<double>(1,3) <<
				baProblem.cameras_[oi], baProblem.cameras_[oi+1], baProblem.cameras_[oi+2]);

		cv::Mat R;
		cv::Rodrigues(rvec, R);
		Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), baProblem.cameras_[oi+3],
				    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), baProblem.cameras_[oi+4],
				    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), baProblem.cameras_[oi+5]);

		if(this->isSlam2d())
		{
			// Body pose recovered by BA (with the planar constraint that locks
			// each non-root body z to its initial value). If the constraint held,
			// z is already within ~mm of the initial; snap it back exactly.
			// Otherwise the optimizer's planar lock didn't bite -- fall back to
			// projecting the BA delta onto SE(2) and applying it to the initial.
			Transform body = (models.at(iter->first)[0].localTransform() * t).inverse();
			if(std::fabs(body.z() - iter->second.z()) < 0.001f)
			{
				body.z() = iter->second.z();
				iter->second = body;
			}
			else
			{
				UWARN("Planar constraints didn't work!? original pose (%d), pose %s -> %s. Falling back to old approach.",
						iter->first,
						iter->second.prettyPrint().c_str(),
						body.prettyPrint().c_str());
				const Transform delta = iter->second.inverse() * body;
				iter->second = iter->second * delta.to3DoF();
			}
		}
		else
		{
			iter->second = (models.at(iter->first)[0].localTransform() * t).inverse();
		}

	}

	//update 3D points
	oi = 0;
	for(std::map<int, cv::Point3f>::iterator kter = points3DMap.begin(); kter!=points3DMap.end(); ++kter)
	{
		kter->second.x = baProblem.points_[oi++];
		kter->second.y = baProblem.points_[oi++];
		kter->second.z = baProblem.points_[oi++];
	}

	return newPoses;

#else
	UERROR("RTAB-Map is not built with ceres!");
	return std::map<int, Transform>();
#endif
}

} /* namespace rtabmap */
