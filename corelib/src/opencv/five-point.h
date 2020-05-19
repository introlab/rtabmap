/*
 * five-point.h
 *
 *  Created on: May 18, 2020
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_OPENCV_FIVE_POINT_H_
#define CORELIB_SRC_OPENCV_FIVE_POINT_H_

namespace cv3
{

cv::Mat findEssentialMat( cv::InputArray _points1, cv::InputArray _points2, cv::InputArray _cameraMatrix,
                          int method, double prob, double threshold, cv::OutputArray _mask = cv::noArray());

int recoverPose( cv::InputArray E, cv::InputArray _points1, cv::InputArray _points2,
                 cv::InputArray _cameraMatrix, cv::OutputArray _R, cv::OutputArray _t, double distanceThresh,
                 cv::InputOutputArray _mask, cv::OutputArray triangulatedPoints);
}


#endif /* CORELIB_SRC_OPENCV_FIVE_POINT_H_ */
