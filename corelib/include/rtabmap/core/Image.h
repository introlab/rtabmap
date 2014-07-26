/*
 * Image.h
 *
 *  Created on: 2012-12-08
 *      Author: mathieu
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Features2d.h>

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
class Image
{
public:
	Image(const cv::Mat & image = cv::Mat(),
		  int id = 0,
		  const cv::Mat & descriptors = cv::Mat(),
		  Feature2D::Type featureType = Feature2D::kFeatureUndef,
		  const std::vector<cv::KeyPoint> & keypoints = std::vector<cv::KeyPoint>()) :
		_image(image),
		_id(id),
		_descriptors(descriptors),
		_featureType(featureType),
		_keypoints(keypoints),
		_fx(0.0f),
		_fy(0.0f),
		_cx(0.0f),
		_cy(0.0f),
		_localTransform(Transform::getIdentity())
	{
	}

	// Metric constructor
	Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0) :
		_image(image),
		_id(id),
		_featureType(Feature2D::kFeatureUndef),
		_depth(depth),
		_fx(fx),
		_fy(fy),
		_cx(cx),
		_cy(cy),
		_pose(pose),
		_localTransform(localTransform)
	{
	}

	// Metric constructor + 2d depth
	Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  const cv::Mat & depth2d,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0) :
		_image(image),
		_id(id),
		_featureType(Feature2D::kFeatureUndef),
		_depth(depth),
		_depth2d(depth2d),
		_fx(fx),
		_fy(fy),
		_cx(cx),
		_cy(cy),
		_pose(pose),
		_localTransform(localTransform)
	{
	}

	virtual ~Image() {}

	bool empty() const {return _image.empty() && _descriptors.empty() && _keypoints.size() == 0;}
	const cv::Mat & image() const {return _image;}
	int id() const {return _id;};
	const cv::Mat & descriptors() const {return _descriptors;}
	Feature2D::Type featureType() const {return _featureType;}
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	void setDescriptors(const cv::Mat & descriptors, Feature2D::Type featureType) {_descriptors = descriptors; _featureType=featureType;}
	void setKeypoints(const std::vector<cv::KeyPoint> & keypoints) {_keypoints = keypoints;}

	bool isMetric() const {return !_depth.empty() || _fx != 0.0f || _fy != 0.0f || !_pose.isNull();}
	void setPose(const Transform & pose) {_pose = pose;}
	const cv::Mat & depth() const {return _depth;}
	const cv::Mat & depth2d() const {return _depth2d;}
	float depthFx() const {return _fx;}
	float depthFy() const {return _fy;}
	float depthCx() const {return _cx;}
	float depthCy() const {return _cy;}
	const Transform & pose() const {return _pose;}
	const Transform & localTransform() const {return _localTransform;}

private:
	cv::Mat _image;
	int _id;
	cv::Mat _descriptors;
	Feature2D::Type _featureType;
	std::vector<cv::KeyPoint> _keypoints;

	// Metric stuff
	cv::Mat _depth;
	cv::Mat _depth2d;
	float _fx;
	float _fy;
	float _cx;
	float _cy;
	Transform _pose;
	Transform _localTransform;
};

}


#endif /* IMAGE_H_ */
