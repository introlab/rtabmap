/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_SRC_ICP_CCCORELIB_H_
#define CORELIB_SRC_ICP_CCCORELIB_H_

#include <CCCoreLib/RegistrationTools.h>

namespace rtabmap {

rtabmap::Transform icpCC(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & fromCloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr & toCloud,
		int maxIterations = 150,
		double minRMSDecrease = 0.00001,
		bool force3DoF = false,
		bool force4DoF = false,
		int samplingLimit = 50000,
		double finalOverlapRatio = 0.85,
		bool filterOutFarthestPoints = false,
		double maxFinalRMS = 0.2,
		float * finalRMS = 0,
		std::string * errorMsg = 0)
{
	UDEBUG("maxIterations=%d", maxIterations);
	UDEBUG("minRMSDecrease=%f", minRMSDecrease);
	UDEBUG("samplingLimit=%d", samplingLimit);
	UDEBUG("finalOverlapRatio=%f", finalOverlapRatio);
	UDEBUG("filterOutFarthestPoints=%s", filterOutFarthestPoints?"true":"false");
	UDEBUG("force 3DoF=%s 4DoF=%s", force3DoF?"true":"false", force4DoF?"true":"false");
	UDEBUG("maxFinalRMS=%f", maxFinalRMS);

	rtabmap::Transform icpTransformation;

	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
	CCCoreLib::PointProjectionTools::Transformation transform;
	CCCoreLib::ICPRegistrationTools::Parameters params;
	{
		if(minRMSDecrease > 0.0)
		{
			params.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
			params.minRMSDecrease = minRMSDecrease; //! The minimum error (RMS) reduction between two consecutive steps to continue process (ignored if convType is not MAX_ERROR_CONVERGENCE)
		}
		else
		{
			params.convType = CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE;
			params.nbMaxIterations = maxIterations; //! The maximum number of iteration (ignored if convType is not MAX_ITER_CONVERGENCE)
		}
		params.adjustScale = false;             //! Whether to release the scale parameter during the registration procedure or not
		params.filterOutFarthestPoints = filterOutFarthestPoints; //! If true, the algorithm will automatically ignore farthest points from the reference, for better convergence
		params.samplingLimit = samplingLimit; //! Maximum number of points per cloud (they are randomly resampled below this limit otherwise)
		params.finalOverlapRatio = finalOverlapRatio;  //! Theoretical overlap ratio (at each iteration, only this percentage (between 0 and 1) will be used for registration
		params.modelWeights = nullptr;          //! Weights for model points (i.e. only if the model entity is a cloud) (optional)
		params.dataWeights = nullptr;           //! Weights for data points (optional)
		params.transformationFilters = force3DoF?33:force4DoF?1:0;  //! Filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
		params.maxThreadCount = 0;              //! Maximum number of threads to use (0 = max)
	}

	double finalError = 0.0;
	unsigned finalPointCount = 0;

	CCCoreLib::PointCloud toPointCloud = CCCoreLib::PointCloud();
	CCCoreLib::PointCloud fromPointCloud = CCCoreLib::PointCloud();

	fromPointCloud.reserve(fromCloud->points.size());
	for(uint nIndex=0; nIndex < fromCloud->points.size(); nIndex++)
	{
		CCVector3 P;
		P.x = fromCloud->points[nIndex].x;
		P.y = fromCloud->points[nIndex].y;
		P.z = force3DoF?0:fromCloud->points[nIndex].z;
		fromPointCloud.addPoint(P);
	}
	toPointCloud.reserve(toCloud->points.size());
	for(uint nIndex=0; nIndex < toCloud->points.size(); nIndex++)
	{
		CCVector3 P;
		P.x = toCloud->points[nIndex].x;
		P.y = toCloud->points[nIndex].y;
		P.z = force3DoF?0:toCloud->points[nIndex].z;
		toPointCloud.addPoint(P);
	}

	UDEBUG("CCCoreLib: start ICP");
	result = CCCoreLib::ICPRegistrationTools::Register(
			&fromPointCloud,
			nullptr,
			&toPointCloud,
			params,
			transform,
			finalError,
			finalPointCount);
	UDEBUG("CCCoreLib: ICP done!");

	UDEBUG("CC ICP result: %d", result);
	UDEBUG("CC Final error: %f . Finall Pointcount: %d", finalError, finalPointCount);
	UDEBUG("CC ICP success Trans: %f %f %f", transform.T.x,transform.T.y,transform.T.z);

	if(finalRMS)
	{
		*finalRMS = (float)finalError;
	}

	if(result != 1)
	{
		std::string msg = uFormat("CCCoreLib has failed: Rejecting transform as result %d !=1", result);
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}

		icpTransformation.setNull();
		return icpTransformation;
	}
	else if(!transform.R.isValid())
	{
		std::string msg = uFormat("CCCoreLib has failed: Rotation matrix is invalid");
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}
		icpTransformation.setNull();
		return icpTransformation;
	}
	//CC transform to EIgen4f
	Eigen::Matrix4f matrix;
	matrix.setIdentity();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			matrix(i,j)=transform.R.getValue(i,j);
		}
	}
	for(int i=0;i<3;i++)
	{
		matrix(i,3)=transform.T[i];
	}

	icpTransformation = rtabmap::Transform::fromEigen4f(matrix);
	icpTransformation = icpTransformation.inverse();
	UDEBUG("CC ICP result: %s", icpTransformation.prettyPrint().c_str());

	if(finalError > maxFinalRMS)
	{
		std::string msg = uFormat("CCCoreLib has failed: Rejecting transform as RMS %f > %f (%s) ", finalError, maxFinalRMS, rtabmap::Parameters::kIcpCCMaxFinalRMS().c_str());
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}
		icpTransformation.setNull();
	}
	return icpTransformation;
}

}

#endif /* CORELIB_SRC_ICP_CCCORELIB_H_ */
