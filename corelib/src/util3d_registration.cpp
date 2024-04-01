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

#include "rtabmap/core/util3d_registration.h"

#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

namespace rtabmap
{

namespace util3d
{

// Get transform from cloud2 to cloud1
Transform transformFromXYZCorrespondencesSVD(
	const pcl::PointCloud<pcl::PointXYZ> & cloud1,
	const pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;

	// Perform the alignment
	Eigen::Matrix4f matrix;
	svd.estimateRigidTransformation(cloud1, cloud2, matrix);
	return Transform::fromEigen4f(matrix);
}

// Get transform from cloud2 to cloud1
Transform transformFromXYZCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud1,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud2,
		double inlierThreshold,
		int iterations,
		int refineIterations,
		double refineSigma,
		std::vector<int> * inliersOut,
		cv::Mat * covariance)
{
	//NOTE: this method is a mix of two methods:
	//  - getRemainingCorrespondences() in pcl/registration/impl/correspondence_rejection_sample_consensus.hpp
	//  - refineModel() in pcl/sample_consensus/sac.h

	if(covariance)
	{
		*covariance = cv::Mat::eye(6,6,CV_64FC1);
	}
	Transform transform;
	if(cloud1->size() >=3 && cloud1->size() == cloud2->size())
	{
		// RANSAC
		UDEBUG("iterations=%d inlierThreshold=%f", iterations, inlierThreshold);
		std::vector<int> source_indices (cloud2->size());
		std::vector<int> target_indices (cloud1->size());

		// Copy the query-match indices
		for (int i = 0; i < (int)cloud1->size(); ++i)
		{
			source_indices[i] = i;
			target_indices[i] = i;
		}

		// From the set of correspondences found, attempt to remove outliers
		// Create the registration model
		pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model;
		model.reset(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloud2, source_indices));
		// Pass the target_indices
		model->setInputTarget (cloud1, target_indices);
		// Create a RANSAC model
		pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, inlierThreshold);
		sac.setMaxIterations(iterations);

		// Compute the set of inliers
		if(sac.computeModel())
		{
			std::vector<int> inliers;
			Eigen::VectorXf model_coefficients;

			sac.getInliers(inliers);
			sac.getModelCoefficients (model_coefficients);

			if (refineIterations>0)
			{
				double error_threshold = inlierThreshold;
				int refine_iterations = 0;
				bool inlier_changed = false, oscillating = false;
				std::vector<int> new_inliers, prev_inliers = inliers;
				std::vector<size_t> inliers_sizes;
				Eigen::VectorXf new_model_coefficients = model_coefficients;
				do
				{
					// Optimize the model coefficients
					model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
					inliers_sizes.push_back (prev_inliers.size ());

					// Select the new inliers based on the optimized coefficients and new threshold
					model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);
					UDEBUG("RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.",
							(int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

					if (new_inliers.empty ())
					{
						++refine_iterations;
						if (refine_iterations >= refineIterations)
						{
							break;
						}
						continue;
					}

					// Estimate the variance and the new threshold
					double variance = model->computeVariance ();
					error_threshold = std::min (inlierThreshold, refineSigma * sqrt(variance));

					UDEBUG ("RANSAC refineModel: New estimated error threshold: %f (variance=%f) on iteration %d out of %d.",
						  error_threshold, variance, refine_iterations, refineIterations);
					inlier_changed = false;
					std::swap (prev_inliers, new_inliers);

					// If the number of inliers changed, then we are still optimizing
					if (new_inliers.size () != prev_inliers.size ())
					{
						// Check if the number of inliers is oscillating in between two values
						if (inliers_sizes.size () >= 4)
						{
							if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
							inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
							{
								oscillating = true;
								break;
							}
						}
						inlier_changed = true;
						continue;
					}

					// Check the values of the inlier set
					for (size_t i = 0; i < prev_inliers.size (); ++i)
					{
						// If the value of the inliers changed, then we are still optimizing
						if (prev_inliers[i] != new_inliers[i])
						{
							inlier_changed = true;
							break;
						}
					}
				}
				while (inlier_changed && ++refine_iterations < refineIterations);

				// If the new set of inliers is empty, we didn't do a good job refining
				if (new_inliers.empty ())
				{
					UWARN ("RANSAC refineModel: Refinement failed: got an empty set of inliers!");
				}

				if (oscillating)
				{
					UDEBUG("RANSAC refineModel: Detected oscillations in the model refinement.");
				}

				std::swap (inliers, new_inliers);
				model_coefficients = new_model_coefficients;
			}

			if (inliers.size() >= 3)
			{
				if(inliersOut)
				{
					*inliersOut = inliers;
				}
				if(covariance)
				{
					double variance =  model->computeVariance();
					UASSERT(uIsFinite(variance));
					*covariance *= variance;
				}

				// get best transformation
				Eigen::Matrix4f bestTransformation;
				bestTransformation.row (0) = model_coefficients.segment<4>(0);
				bestTransformation.row (1) = model_coefficients.segment<4>(4);
				bestTransformation.row (2) = model_coefficients.segment<4>(8);
				bestTransformation.row (3) = model_coefficients.segment<4>(12);

				transform = Transform::fromEigen4f(bestTransformation);
				UDEBUG("RANSAC inliers=%d/%d tf=%s", (int)inliers.size(), (int)cloud1->size(), transform.prettyPrint().c_str());

				return transform.inverse(); // inverse to get actual pose transform (not correspondences transform)
			}
			else
			{
				UDEBUG("RANSAC: Model with inliers < 3");
			}
		}
		else
		{
			UDEBUG("RANSAC: Failed to find model");
		}
	}
	else
	{
		UDEBUG("Not enough points to compute the transform");
	}
	return Transform();
}

template<typename PointNormalT>
void computeVarianceAndCorrespondencesImpl(
		const typename pcl::PointCloud<PointNormalT>::ConstPtr & cloudA,
		const typename pcl::PointCloud<PointNormalT>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double maxCorrespondenceAngle,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	variance = 1;
	correspondencesOut = 0;
	typename pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>::Ptr est;
	est.reset(new pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT>);
	const typename pcl::PointCloud<PointNormalT>::ConstPtr & target = cloudA->size()>cloudB->size()?cloudA:cloudB;
	const typename pcl::PointCloud<PointNormalT>::ConstPtr & source = cloudA->size()>cloudB->size()?cloudB:cloudA;
	est->setInputTarget(target);
	est->setInputSource(source);
	pcl::Correspondences correspondences;
	est->determineReciprocalCorrespondences(correspondences, maxCorrespondenceDistance);

	if(correspondences.size())
	{
		std::vector<double> distances(correspondences.size());
		correspondencesOut = 0;
		for(unsigned int i=0; i<correspondences.size(); ++i)
		{
			distances[i] = correspondences[i].distance;
			if(maxCorrespondenceAngle <= 0.0)
			{
				++correspondencesOut;
			}
			else
			{
				Eigen::Vector4f v1(
						target->at(correspondences[i].index_match).normal_x,
						target->at(correspondences[i].index_match).normal_y,
						target->at(correspondences[i].index_match).normal_z,
						0);
				Eigen::Vector4f v2(
						source->at(correspondences[i].index_query).normal_x,
						source->at(correspondences[i].index_query).normal_y,
						source->at(correspondences[i].index_query).normal_z,
						0);
				float angle = pcl::getAngle3D(v1, v2);
				if(angle < maxCorrespondenceAngle)
				{
					++correspondencesOut;
				}
			}
		}
		if(correspondencesOut)
		{
			distances.resize(correspondencesOut);

			//variance
			std::sort(distances.begin (), distances.end ());
			double median_error_sqr = distances[distances.size () >> 1];
			variance = (2.1981 * median_error_sqr);
		}
	}
}

void computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double maxCorrespondenceAngle,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	computeVarianceAndCorrespondencesImpl<pcl::PointNormal>(cloudA, cloudB, maxCorrespondenceDistance, maxCorrespondenceAngle, variance, correspondencesOut, reciprocal);
}

void computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double maxCorrespondenceAngle,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	computeVarianceAndCorrespondencesImpl<pcl::PointXYZINormal>(cloudA, cloudB, maxCorrespondenceDistance, maxCorrespondenceAngle, variance, correspondencesOut, reciprocal);
}

template<typename PointT>
void computeVarianceAndCorrespondencesImpl(
		const typename pcl::PointCloud<PointT>::ConstPtr & cloudA,
		const typename pcl::PointCloud<PointT>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	variance = 1;
	correspondencesOut = 0;
	typename pcl::registration::CorrespondenceEstimation<PointT, PointT>::Ptr est;
	est.reset(new pcl::registration::CorrespondenceEstimation<PointT, PointT>);
	est->setInputTarget(cloudA->size()>cloudB->size()?cloudA:cloudB);
	est->setInputSource(cloudA->size()>cloudB->size()?cloudB:cloudA);
	pcl::Correspondences correspondences;
	est->determineReciprocalCorrespondences(correspondences, maxCorrespondenceDistance);

	if(correspondences.size()>=3)
	{
		std::vector<double> distances(correspondences.size());
		for(unsigned int i=0; i<correspondences.size(); ++i)
		{
			distances[i] = correspondences[i].distance;
		}

		//variance
		std::sort(distances.begin (), distances.end ());
		double median_error_sqr = distances[distances.size () >> 1];
		variance = (2.1981 * median_error_sqr);
	}

	correspondencesOut = (int)correspondences.size();
}

void computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	computeVarianceAndCorrespondencesImpl<pcl::PointXYZ>(cloudA, cloudB, maxCorrespondenceDistance, variance, correspondencesOut, reciprocal);
}

void computeVarianceAndCorrespondences(
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloudA,
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloudB,
		double maxCorrespondenceDistance,
		double & variance,
		int & correspondencesOut,
		bool reciprocal)
{
	computeVarianceAndCorrespondencesImpl<pcl::PointXYZI>(cloudA, cloudB, maxCorrespondenceDistance, variance, correspondencesOut, reciprocal);
}

// return transform from source to target (All points must be finite!!!)
template<typename PointT>
Transform icpImpl(const typename pcl::PointCloud<PointT>::ConstPtr & cloud_source,
			  const typename pcl::PointCloud<PointT>::ConstPtr & cloud_target,
			  double maxCorrespondenceDistance,
			  int maximumIterations,
			  bool & hasConverged,
			  pcl::PointCloud<PointT> & cloud_source_registered,
			  float epsilon,
			  bool icp2D)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	// Set the input source and target
	icp.setInputTarget (cloud_target);
	icp.setInputSource (cloud_source);

	if(icp2D)
	{
		typename pcl::registration::TransformationEstimation2D<PointT, PointT>::Ptr est;
		est.reset(new pcl::registration::TransformationEstimation2D<PointT, PointT>);
		icp.setTransformationEstimation(est);
	}

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maximumIterations);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (epsilon*epsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	icp.align (cloud_source_registered);
	hasConverged = icp.hasConverged();
	return Transform::fromEigen4f(icp.getFinalTransformation());
}

// return transform from source to target (All points must be finite!!!)
Transform icp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_source,
			  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_target,
			  double maxCorrespondenceDistance,
			  int maximumIterations,
			  bool & hasConverged,
			  pcl::PointCloud<pcl::PointXYZ> & cloud_source_registered,
			  float epsilon,
			  bool icp2D)
{
	return icpImpl(cloud_source, cloud_target, maxCorrespondenceDistance, maximumIterations, hasConverged, cloud_source_registered, epsilon, icp2D);
}

// return transform from source to target (All points must be finite!!!)
Transform icp(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_source,
			  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_target,
			  double maxCorrespondenceDistance,
			  int maximumIterations,
			  bool & hasConverged,
			  pcl::PointCloud<pcl::PointXYZI> & cloud_source_registered,
			  float epsilon,
			  bool icp2D)
{
	return icpImpl(cloud_source, cloud_target, maxCorrespondenceDistance, maximumIterations, hasConverged, cloud_source_registered, epsilon, icp2D);
}

// return transform from source to target (All points/normals must be finite!!!)
template<typename PointNormalT>
Transform icpPointToPlaneImpl(
		const typename pcl::PointCloud<PointNormalT>::ConstPtr & cloud_source,
		const typename pcl::PointCloud<PointNormalT>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<PointNormalT> & cloud_source_registered,
		float epsilon,
		bool icp2D)
{
	pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
	// Set the input source and target
	icp.setInputTarget (cloud_target);
	icp.setInputSource (cloud_source);

	typename pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT>::Ptr est;
	est.reset(new pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT, PointNormalT>);
	icp.setTransformationEstimation(est);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (maximumIterations);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (epsilon*epsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);
	//icp.setRANSACOutlierRejectionThreshold(maxCorrespondenceDistance);

	// Perform the alignment
	icp.align (cloud_source_registered);
	hasConverged = icp.hasConverged();
	Transform t = Transform::fromEigen4f(icp.getFinalTransformation());

	if(icp2D)
	{
		// FIXME probably an estimation approach already 2D like in icp() version above exists.
		t = t.to3DoF();
	}

	return t;
}

// return transform from source to target (All points/normals must be finite!!!)
Transform icpPointToPlane(
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointNormal> & cloud_source_registered,
		float epsilon,
		bool icp2D)
{
	return icpPointToPlaneImpl(cloud_source, cloud_target, maxCorrespondenceDistance, maximumIterations, hasConverged, cloud_source_registered, epsilon, icp2D);
}
// return transform from source to target (All points/normals must be finite!!!)
Transform icpPointToPlane(
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloud_source,
		const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr & cloud_target,
		double maxCorrespondenceDistance,
		int maximumIterations,
		bool & hasConverged,
		pcl::PointCloud<pcl::PointXYZINormal> & cloud_source_registered,
		float epsilon,
		bool icp2D)
{
	return icpPointToPlaneImpl(cloud_source, cloud_target, maxCorrespondenceDistance, maximumIterations, hasConverged, cloud_source_registered, epsilon, icp2D);
}

}

}
