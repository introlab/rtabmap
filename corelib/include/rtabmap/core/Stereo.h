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

#ifndef STEREO_H_
#define STEREO_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

class RTABMAP_CORE_EXPORT Stereo {
public:
	static Stereo * create(const ParametersMap & parameters = ParametersMap());

public:
	Stereo(const ParametersMap & parameters = ParametersMap());
	virtual ~Stereo() {}

	virtual void parseParameters(const ParametersMap & parameters);
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;

	cv::Size winSize() const {return cv::Size(winWidth_, winHeight_);}
	int iterations() const   {return iterations_;}
	int maxLevel() const     {return maxLevel_;}
	float minDisparity() const {return minDisparity_;}
	float maxDisparity() const {return maxDisparity_;}
	bool winSSD() const      {return winSSD_;}

private:
	int winWidth_;
	int winHeight_;
	int iterations_;
	int maxLevel_;
	float minDisparity_;
	float maxDisparity_;
	bool winSSD_;
};

class RTABMAP_CORE_EXPORT StereoOpticalFlow : public Stereo {
public:
	StereoOpticalFlow(const ParametersMap & parameters = ParametersMap());
	virtual ~StereoOpticalFlow() {}

	virtual void parseParameters(const ParametersMap & parameters);
	virtual std::vector<cv::Point2f> computeCorrespondences(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage,
			const std::vector<cv::Point2f> & leftCorners,
			std::vector<unsigned char> & status) const;

	float epsilon() const {return epsilon_;}

private:
	float epsilon_;
};

} /* namespace rtabmap */

#endif /* STEREO_H_ */
