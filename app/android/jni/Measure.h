/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef MEASURE_H_
#define MEASURE_H_

#include <opencv2/opencv.hpp>

class Measure
{
public:
	Measure(
			const cv::Point3f & pt1,
			const cv::Point3f & pt2,
			const cv::Vec3f & n1,
			const cv::Vec3f & n2) :
				pt1_(pt1),
				pt2_(pt2),
				n1_(n1),
				n2_(n2)
	{
		length_ = cv::norm(pt2_ - pt1_);
	}
	virtual ~Measure() {}

	float length() const {return length_;}
	const cv::Point3f & pt1() const {return pt1_;}
	const cv::Point3f & pt2() const {return pt2_;}
	const cv::Vec3f & n1() const {return n1_;}
	const cv::Vec3f & n2() const {return n2_;}

private:
	cv::Point3f pt1_;
	cv::Point3f pt2_;
	cv::Vec3f n1_;
	cv::Vec3f n2_;
	float length_;
};


#endif /* MEASURE_H_ */
