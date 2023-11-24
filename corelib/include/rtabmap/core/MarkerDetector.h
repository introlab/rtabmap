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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_MARKERDETECTOR_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_MARKERDETECTOR_H_

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/CameraModel.h>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_ARUCO
#include <opencv2/aruco.hpp>
#endif

namespace rtabmap {

typedef std::map<int, Transform> MapIdPose;

class MarkerInfo {
public:
    MarkerInfo(int id, float length, Transform pose) :
        id_(id),
        length_(length),
        pose_(pose)
    {}
    int id() const {return id_;}
    float length() const {return length_;}
    const Transform & pose() const {return pose_;}
private:
    int id_;
    float length_;
    Transform pose_;
};

class RTABMAP_CORE_EXPORT MarkerDetector {
    
public:
	MarkerDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~MarkerDetector();
	void parseParameters(const ParametersMap & parameters);

	// Use the other detect(), in which the returned map contains the length of each marker detected.
    RTABMAP_DEPRECATED
    MapIdPose detect(const cv::Mat & image,
			const CameraModel & model,
			const cv::Mat & depth = cv::Mat(),
			float * estimatedMarkerLength = 0,
			cv::Mat * imageWithDetections = 0);

    std::map<int, MarkerInfo> detect(const cv::Mat & image,
		    const std::vector<CameraModel> & models,
		    const cv::Mat & depth = cv::Mat(),
		    const std::map<int, float> & markerLengths = std::map<int, float>(),
		    cv::Mat * imageWithDetections = 0);

    std::map<int, MarkerInfo> detect(const cv::Mat & image,
		    const CameraModel & model,
		    const cv::Mat & depth = cv::Mat(),
		    const std::map<int, float> & markerLengths = std::map<int, float>(),
		    cv::Mat * imageWithDetections = 0);

private:
#ifdef HAVE_OPENCV_ARUCO
	cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
	float markerLength_;
	float maxDepthError_;
	float maxRange_;
	float minRange_;
	int dictionaryId_;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
#endif
};

} /* namespace rtabmap */

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_MARKERDETECTOR_H_ */
