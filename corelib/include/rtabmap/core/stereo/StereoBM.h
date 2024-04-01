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

#ifndef STEREOBM_H_
#define STEREOBM_H_

#include <rtabmap/core/StereoDense.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

class RTABMAP_CORE_EXPORT StereoBM : public StereoDense {
public:
	StereoBM(int blockSize, int numDisparities);
	StereoBM(const ParametersMap & parameters = ParametersMap());
	virtual ~StereoBM() {}

	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat computeDisparity(
			const cv::Mat & leftImage,
			const cv::Mat & rightImage) const;

private:
	int blockSize_;         //15
	int minDisparity_;      //0
	int numDisparities_;    //64
	int preFilterSize_;     //9
	int preFilterCap_;      //31
	int uniquenessRatio_;   //15
	int textureThreshold_;  //10
	int speckleWindowSize_; //100
	int speckleRange_;      //4
	int disp12MaxDiff_;     //-1
};

} /* namespace rtabmap */

#endif /* STEREOBM_H_ */
