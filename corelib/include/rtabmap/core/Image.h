/*
 * Image.h
 *
 *  Created on: 2012-12-08
 *      Author: mathieu
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

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
		_keypoints(keypoints)
	{
	}

	bool empty() const {return _image.empty() && _descriptors.empty() && _keypoints.size() == 0;}
	const cv::Mat & image() const {return _image;}
	int id() const {return _id;};
	const cv::Mat & descriptors() const {return _descriptors;}
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}

private:
	cv::Mat _image;
	int _id;
	cv::Mat _descriptors;
	std::vector<cv::KeyPoint> _keypoints;
};

}


#endif /* IMAGE_H_ */
