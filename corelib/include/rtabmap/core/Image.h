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
		  const std::vector<cv::KeyPoint> & keypoints = std::vector<cv::KeyPoint>()) :
		_image(image),
		_id(id),
		_descriptors(descriptors),
		_keypoints(keypoints),
		_depthConstant(0.0f),
		_localTransform(Transform::getIdentity())
	{
	}

	// Metric constructor
	Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  float depthConstant,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0) :
		_image(image),
		_id(id),
		_depth(depth),
		_depthConstant(depthConstant),
		_pose(pose),
		_localTransform(localTransform)
	{
	}

	// Metric constructor + 2d depth
	Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  const cv::Mat & depth2d,
		  float depthConstant,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0) :
		_image(image),
		_id(id),
		_depth(depth),
		_depth2d(depth2d),
		_depthConstant(depthConstant),
		_pose(pose),
		_localTransform(localTransform)
	{
	}

	virtual ~Image() {}

	bool empty() const {return _image.empty() && _descriptors.empty() && _keypoints.size() == 0;}
	const cv::Mat & image() const {return _image;}
	int id() const {return _id;};
	const cv::Mat & descriptors() const {return _descriptors;}
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	void setDescriptors(const cv::Mat & descriptors) {_descriptors = descriptors;}
	void setKeypoints(const std::vector<cv::KeyPoint> & keypoints) {_keypoints = keypoints;}

	bool isMetric() const {return !_depth.empty() || _depthConstant != 0.0f || !_pose.isNull();}
	void setPose(const Transform & pose) {_pose = pose;}
	const cv::Mat & depth() const {return _depth;}
	const cv::Mat & depth2d() const {return _depth2d;}
	float depthConstant() const {return _depthConstant;}
	const Transform & pose() const {return _pose;}
	const Transform & localTransform() const {return _localTransform;}

private:
	cv::Mat _image;
	int _id;
	cv::Mat _descriptors;
	std::vector<cv::KeyPoint> _keypoints;

	// Metric stuff
	cv::Mat _depth;
	cv::Mat _depth2d;
	float _depthConstant;
	Transform _pose;
	Transform _localTransform;
};

}


#endif /* IMAGE_H_ */
