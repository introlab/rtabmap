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
#include <ceres/local_parameterization.h>
#include "ceres/pose_graph_2d/types.h"
#include "ceres/pose_graph_2d/pose_graph_2d_error_term.h"
#include "ceres/pose_graph_2d/angle_local_parameterization.h"
#include "ceres/pose_graph_3d/types.h"
#include "ceres/pose_graph_3d/pose_graph_3d_error_term.h"
#include "ceres/bundle/BAProblem.h"
#include "ceres/bundle/snavely_reprojection_error.h"

#if not(CERES_VERSION_MAJOR > 1 || (CERES_VERSION_MAJOR == 1 && CERES_VERSION_MINOR >= 12))
#include "ceres/pose_graph_3d/eigen_quaternion_parameterization.h"
#endif

#endif

namespace rtabmap {

bool OptimizerCeres::available()
{
#ifdef RTABMAP_CERES
	return true;
#else
	return false;
#endif
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
		ceres::LocalParameterization* angle_local_parameterization = NULL;
		ceres::LocalParameterization* quaternion_local_parameterization = NULL;

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
					const Eigen::Matrix3d sqrt_information = information.llt().matrixL();

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

					if(angle_local_parameterization == NULL)
					{
						angle_local_parameterization = ceres::examples::AngleLocalParameterization::Create();
					}
					problem.SetParameterization(&pose_begin_iter->second.yaw_radians, angle_local_parameterization);
					problem.SetParameterization(&pose_end_iter->second.yaw_radians, angle_local_parameterization);
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

					const Eigen::Matrix<double, 6, 6> sqrt_information = information.llt().matrixL();
					// Ceres will take ownership of the pointer.
					ceres::CostFunction* cost_function = ceres::examples::PoseGraph3dErrorTerm::Create(t, sqrt_information);
					problem.AddResidualBlock(cost_function, loss_function,
											  pose_begin_iter->second.p.data(), pose_begin_iter->second.q.coeffs().data(),
											  pose_end_iter->second.p.data(), pose_end_iter->second.q.coeffs().data());
					if(quaternion_local_parameterization == NULL)
					{
						quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
					}
					problem.SetParameterization(pose_begin_iter->second.q.coeffs().data(), quaternion_local_parameterization);
					problem.SetParameterization(pose_end_iter->second.q.coeffs().data(), quaternion_local_parameterization);
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
		options.linear_solver_type = ceres::ITERATIVE_SCHUR;
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

	baProblem.num_cameras_ = poses.size();
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

	// Each camera is a set of 6 parameters: R and t. The rotation R is specified as a Rodrigues' vector.
	int oi=0;
	int camIndex=0;
	std::map<int, int> camIdToIndex;
	for(std::map<int, Transform>::const_iterator iter=poses.begin();
		iter!=poses.end();
		++iter)
	{
		// Get camera model
		std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(iter->first);
		UASSERT(iterModel != models.end());
		if(iterModel->second.size() != 1)
		{
			UERROR("Multi-camera BA not implemented for Ceres, only single camera.");
			return std::map<int, Transform>();
		}
		UASSERT(iterModel->second[0].isValidForProjection());

		const Transform & t = (iter->second * iterModel->second[0].localTransform()).inverse();
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

		camIdToIndex.insert(std::make_pair(iter->first, camIndex++));
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

	oi = 0;
	for(std::map<int, std::map<int, FeatureBA> >::const_iterator iter=wordReferences.begin();
		iter!=wordReferences.end();
		++iter)
	{
		for(std::map<int, FeatureBA>::const_iterator jter=iter->second.begin();
			jter!=iter->second.end();
			++jter)
		{
			std::map<int, std::vector<CameraModel> >::const_iterator iterModel = models.find(jter->first);
			UASSERT(iterModel != models.end());
			if(iterModel->second.size() != 1)
			{
				UERROR("Multi-camera BA not implemented for Ceres, only single camera.");
				return std::map<int, Transform>();
			}
			UASSERT(iterModel->second[0].isValidForProjection());

			baProblem.camera_index_[oi] = camIdToIndex.at(jter->first);
			baProblem.point_index_[oi] = pointIdToIndex.at(iter->first);
			baProblem.observations_[4*oi] = jter->second.kpt.pt.x - iterModel->second[0].cx();
			baProblem.observations_[4*oi+1] = jter->second.kpt.pt.y - iterModel->second[0].cy();
			baProblem.observations_[4*oi+2] = iterModel->second[0].fx();
			baProblem.observations_[4*oi+3] = iterModel->second[0].fy();
			++oi;
		}
	}
	UASSERT(oi == baProblem.num_observations_);

	// Build problem
	const double* observations = baProblem.observations();
	// Create residuals for each observation in the bundle adjustment problem. The
	// parameters for cameras and points are added automatically.
	ceres::Problem problem;

	for (int i = 0; i < baProblem.num_observations(); ++i) {
		// Each Residual block takes a point and a camera as input and outputs a 2
		// dimensional residual. Internally, the cost function stores the observed
		// image location and compares the reprojection against the observation.
		ceres::CostFunction* cost_function =
			ceres::SnavelyReprojectionError::Create(
					observations[4 * i],      //u
					observations[4 * i + 1],  //v
					observations[4 * i + 2],  //fx
					observations[4 * i + 3]); //fy
		ceres::LossFunction* loss_function = new ceres::HuberLoss(8.0);
		problem.AddResidualBlock(cost_function,
								 loss_function,
								 baProblem.mutable_camera_for_observation(i),
								 baProblem.mutable_point_for_observation(i));
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
	options.function_tolerance = this->epsilon();
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

	//update poses
	std::map<int, Transform> newPoses = poses;
	oi=0;
	for(std::map<int, Transform>::iterator iter=newPoses.begin(); iter!=newPoses.end(); ++iter)
	{
		cv::Mat rvec = (cv::Mat_<double>(1,3) <<
				baProblem.cameras_[oi], baProblem.cameras_[oi+1], baProblem.cameras_[oi+2]);

		cv::Mat R;
		cv::Rodrigues(rvec, R);
		Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), baProblem.cameras_[oi+3],
				    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), baProblem.cameras_[oi+4],
				    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), baProblem.cameras_[oi+5]);

		oi+=6;

		if(this->isSlam2d())
		{
			t = (models.at(iter->first)[0].localTransform() * t).inverse();
			t = iter->second.inverse() * t;
			iter->second *= t.to3DoF();
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
