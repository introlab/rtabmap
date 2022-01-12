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

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <map>
#include <list>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

class FeatureBA
{
public:
	FeatureBA(const cv::KeyPoint & kptIn, const float & depthIn = 0.0f, const cv::Mat & descriptorIn = cv::Mat()):
		kpt(kptIn),
		depth(depthIn),
		descriptor(descriptorIn)
	{}
	cv::KeyPoint kpt;
	float depth;
	cv::Mat descriptor;
};

////////////////////////////////////////////
// Graph optimizers
////////////////////////////////////////////
class RTABMAP_EXP Optimizer
{
public:
	enum Type {
		kTypeUndef = -1,
		kTypeTORO = 0,
		kTypeG2O = 1,
		kTypeGTSAM = 2,
		kTypeCeres = 3,
		kTypeCVSBA = 4
	};
	static bool isAvailable(Optimizer::Type type);
	static Optimizer * create(const ParametersMap & parameters);
	static Optimizer * create(Optimizer::Type type, const ParametersMap & parameters = ParametersMap());

	// Get connected poses and constraints from a set of links
	void getConnectedGraph(
			int fromId,
			const std::map<int, Transform> & posesIn,
			const std::multimap<int, Link> & linksIn,
			std::map<int, Transform> & posesOut,
			std::multimap<int, Link> & linksOut,
			bool adjustPosesWithConstraints = true) const;

public:
	virtual ~Optimizer() {}

	virtual Type type() const = 0;

	// getters
	int iterations() const {return iterations_;}
	bool isSlam2d() const {return slam2d_;}
	bool isCovarianceIgnored() const {return covarianceIgnored_;}
	double epsilon() const {return epsilon_;}
	bool isRobust() const {return robust_;}
	bool priorsIgnored() const {return priorsIgnored_;}
	bool landmarksIgnored() const {return landmarksIgnored_;}
	float gravitySigma() const {return gravitySigma_;}

	// setters
	void setIterations(int iterations) {iterations_ = iterations;}
	void setSlam2d(bool enabled) {slam2d_ = enabled;}
	void setCovarianceIgnored(bool enabled) {covarianceIgnored_ = enabled;}
	void setEpsilon(double epsilon) {epsilon_ = epsilon;}
	void setRobust(bool enabled) {robust_ = enabled;}
	void setPriorsIgnored(bool enabled) {priorsIgnored_ = enabled;}
	void setLandmarksIgnored(bool enabled) {landmarksIgnored_ = enabled;}
	void setGravitySigma(float value) {gravitySigma_ = value;}

	virtual void parseParameters(const ParametersMap & parameters);

	std::map<int, Transform> optimizeIncremental(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints,
			std::list<std::map<int, Transform> > * intermediateGraphes = 0,
			double * finalError = 0,
			int * iterationsDone = 0);

	std::map<int, Transform> optimize(
				int rootId,
				const std::map<int, Transform> & poses,
				const std::multimap<int, Link> & constraints,
				std::list<std::map<int, Transform> > * intermediateGraphes = 0,
				double * finalError = 0,
				int * iterationsDone = 0);

	// inherited classes should implement one of these methods
	virtual std::map<int, Transform> optimize(
				int rootId,
				const std::map<int, Transform> & poses,
				const std::multimap<int, Link> & constraints,
				cv::Mat & outputCovariance,
				std::list<std::map<int, Transform> > * intermediateGraphes = 0,
				double * finalError = 0,
				int * iterationsDone = 0);
	virtual std::map<int, Transform> optimizeBA(
			int rootId, // if negative, all other poses are fixed
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, CameraModel> & models, // in case of stereo, Tx should be set
			std::map<int, cv::Point3f> & points3DMap,
			const std::map<int, std::map<int, FeatureBA> > & wordReferences, // <ID words, IDs frames + keypoint/depth/descriptor>
			std::set<int> * outliers = 0);

	std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			std::map<int, cv::Point3f> & points3DMap,
			std::map<int, std::map<int, FeatureBA> > & wordReferences, // <ID words, IDs frames + keypoint/depth/descriptor>
			bool rematchFeatures = false);

	std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			bool rematchFeatures = false);

	Transform optimizeBA(
			const Link & link,
			const CameraModel & model,
			std::map<int, cv::Point3f> & points3DMap,
			const std::map<int, std::map<int, FeatureBA> > & wordReferences,
			std::set<int> * outliers = 0);

	void computeBACorrespondences(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, Signature> & signatures,
			std::map<int, cv::Point3f> & points3DMap,
			std::map<int, std::map<int, FeatureBA > > & wordReferences, // <ID words, IDs frames + keypoint/depth/descriptor>
			bool rematchFeatures = false);

protected:
	Optimizer(
			int iterations         = Parameters::defaultOptimizerIterations(),
			bool slam2d            = Parameters::defaultRegForce3DoF(),
			bool covarianceIgnored = Parameters::defaultOptimizerVarianceIgnored(),
			double epsilon         = Parameters::defaultOptimizerEpsilon(),
			bool robust            = Parameters::defaultOptimizerRobust(),
			bool priorsIgnored     = Parameters::defaultOptimizerPriorsIgnored(),
			bool landmarksIgnored  = Parameters::defaultOptimizerLandmarksIgnored(),
			float gravitySigma     = Parameters::defaultOptimizerGravitySigma());
	Optimizer(const ParametersMap & parameters);

private:
	int iterations_;
	bool slam2d_;
	bool covarianceIgnored_;
	double epsilon_;
	bool robust_;
	bool priorsIgnored_;
	bool landmarksIgnored_;
	float gravitySigma_;
};

} /* namespace rtabmap */
#endif /* OPTIMIZER_H_ */
