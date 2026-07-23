#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d_transforms.h>

#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <set>
#include <string>
#include <vector>

namespace {

bool near(const cv::Point3f & a, const cv::Point3f & b, float tolerance)
{
	return std::fabs(a.x-b.x) <= tolerance &&
			std::fabs(a.y-b.y) <= tolerance &&
			std::fabs(a.z-b.z) <= tolerance;
}

rtabmap::FeatureBA observation(
		const cv::Point3f & point,
		const rtabmap::Transform & pose,
		const rtabmap::CameraModel & model,
		float offsetU = 0.0f,
		float offsetV = 0.0f)
{
	const rtabmap::Transform cameraPose = pose * model.localTransform();
	const cv::Point3f pointInCamera = rtabmap::util3d::transformPoint(point, cameraPose.inverse());
	float u = 0.0f;
	float v = 0.0f;
	model.reproject(pointInCamera.x, pointInCamera.y, pointInCamera.z, u, v);
	return rtabmap::FeatureBA(cv::KeyPoint(u+offsetU, v+offsetV, 1.0f));
}

int fail(const std::string & message)
{
	std::cerr << "test_optimizer_g2o_ba_outliers: " << message << std::endl;
	return 1;
}

std::string formatOutliers(const rtabmap::BAOutliers & outliers)
{
	std::ostringstream stream;
	for(rtabmap::BAOutliers::const_iterator iter=outliers.begin(); iter!=outliers.end(); ++iter)
	{
		stream << iter->first << ":";
		for(std::set<int>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
		{
			stream << " " << *jter;
		}
		stream << "; ";
	}
	return stream.str();
}

} // namespace

int main()
{
	using namespace rtabmap;

	const CameraModel model(100.0, 100.0, 320.0, 240.0, CameraModel::opticalRotation(), 0.0, cv::Size(640, 480));
	std::map<int, Transform> poses;
	for(int id=1; id<=10; ++id)
	{
		poses.insert(std::make_pair(id, Transform(0.0f, 0.05f*(id-1), 0.0f, 0.0f, 0.0f, 0.0f)));
	}
	poses.at(10) = poses.at(4);

	std::map<int, std::vector<CameraModel> > models;
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		models.insert(std::make_pair(iter->first, std::vector<CameraModel>(1, model)));
	}

	const int partiallyRejectedWordId = -42;
	const int fullyRejectedWordId = 7;
	const cv::Point3f partialTruth(5.0f, -0.1f, 0.1f);
	const cv::Point3f partialInitial(5.0f, -0.2f, 0.1f);
	const cv::Point3f fullyRejectedTruth(5.0f, 0.0f, 0.0f);
	const cv::Point3f fullyRejectedInitial(5.0f, -0.5f, 0.0f);

	std::map<int, cv::Point3f> points3DMap;
	points3DMap.insert(std::make_pair(partiallyRejectedWordId, partialInitial));
	points3DMap.insert(std::make_pair(fullyRejectedWordId, fullyRejectedInitial));

	std::map<int, std::map<int, FeatureBA> > wordReferences;
	for(int id=1; id<=9; ++id)
	{
		if(id != 4)
		{
			wordReferences[partiallyRejectedWordId].insert(std::make_pair(id, observation(partialTruth, poses.at(id), model)));
		}
	}
	wordReferences[partiallyRejectedWordId].insert(std::make_pair(4, observation(partialTruth, poses.at(4), model, 80.0f, -80.0f)));
	wordReferences[fullyRejectedWordId].insert(std::make_pair(1, observation(fullyRejectedTruth, poses.at(1), model, 100.0f)));
	wordReferences[fullyRejectedWordId].insert(std::make_pair(2, observation(fullyRejectedTruth, poses.at(2), model, -100.0f)));

	std::multimap<int, Link> links;
	cv::Mat information = cv::Mat::eye(6, 6, CV_64FC1);
	information *= 1000000.0;
	links.insert(std::make_pair(4, Link(4, 10, Link::kNeighbor, Transform::getIdentity(), information)));

	ParametersMap parameters;
	parameters.insert(std::make_pair(Parameters::kg2oRobustKernelDelta(), "8"));
	OptimizerG2O optimizer(parameters);
	optimizer.setIterations(10);
	BAOutliers outliers;
	const std::map<int, Transform> optimizedPoses = optimizer.optimizeBA(
			-10, poses, links, models, points3DMap, wordReferences, &outliers);

	if(optimizedPoses.size() != poses.size())
	{
		return fail("bundle adjustment did not return all poses");
	}
	BAOutliers::const_iterator partialOutliers = outliers.find(partiallyRejectedWordId);
	BAOutliers::const_iterator fullyRejectedOutliers = outliers.find(fullyRejectedWordId);
	if(outliers.size() != 2 ||
			partialOutliers == outliers.end() ||
			partialOutliers->second.size() != 1 ||
			partialOutliers->second.count(4) != 1 ||
			fullyRejectedOutliers == outliers.end() ||
			fullyRejectedOutliers->second.size() != 2 ||
			fullyRejectedOutliers->second.count(1) != 1 ||
			fullyRejectedOutliers->second.count(2) != 1)
	{
		return fail("rejected observations do not match their word and pose IDs: " + formatOutliers(outliers));
	}
	if(!near(points3DMap.at(partiallyRejectedWordId), partialTruth, 0.01f))
	{
		return fail("a landmark with active observations was restored instead of optimized");
	}
	if(!near(points3DMap.at(fullyRejectedWordId), fullyRejectedInitial, 0.0001f))
	{
		return fail("a landmark with only rejected observations was not restored");
	}
	return 0;
}
