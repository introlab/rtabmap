/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KEYPOINTDETECTOR_H_
#define KEYPOINTDETECTOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

class VWDictionary;

class RTABMAP_EXP KeypointDetector
{
public:
	virtual ~KeypointDetector() {}
	std::list<cv::KeyPoint> generateKeypoints(const IplImage * image);
	virtual void parseParameters(const ParametersMap & parameters);
	unsigned int getWordsPerImageTarget() const {return _wordsPerImageTarget;}
	double getAdaptiveResponseThr() const {return _adaptiveResponseThr;}
	virtual double getMinimumResponseThr() const = 0;
	bool isUsingAdaptiveResponseThr() const {return _usingAdaptiveResponseThr;}
	void setRoi(const std::string & roi);
protected:
	KeypointDetector(const ParametersMap & parameters = ParametersMap());
	void setAdaptiveResponseThr(float adaptiveResponseThr) {_adaptiveResponseThr = adaptiveResponseThr;}
private:
	virtual std::list<cv::KeyPoint> _generateKeypoints(const IplImage * image, const cv::Rect & roi) const = 0;
	cv::Rect computeRoi(const IplImage * image) const;
private:
	unsigned int _wordsPerImageTarget;
	bool _usingAdaptiveResponseThr;
	double _adaptiveResponseThr;
	std::vector<float> _roiRatios; // size 4
};

//SURFDetector
class RTABMAP_EXP SURFDetector : public KeypointDetector
{
public:
	SURFDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~SURFDetector();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual double getMinimumResponseThr() const {return _surf.hessianThreshold;};
private:
	virtual std::list<cv::KeyPoint> _generateKeypoints(const IplImage * image, const cv::Rect & roi) const;
private:
	cv::SURF _surf;
	bool _gpuVersion;
	bool _upright;
};

//SIFTDetector
class RTABMAP_EXP SIFTDetector : public KeypointDetector
{
public:
	SIFTDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~SIFTDetector();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual double getMinimumResponseThr() const {return _detectorParams.threshold;};
private:
	virtual std::list<cv::KeyPoint> _generateKeypoints(const IplImage * image, const cv::Rect & roi) const;
private:
	cv::SIFT::CommonParams _commonParams;
	cv::SIFT::DetectorParams _detectorParams;
};

//StarDetector
class RTABMAP_EXP StarDetector : public KeypointDetector
{
public:
	StarDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~StarDetector();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual double getMinimumResponseThr() const {return (double)_star.responseThreshold;};
private:
	virtual std::list<cv::KeyPoint> _generateKeypoints(const IplImage * image, const cv::Rect & roi) const;
private:
	cv::StarDetector _star;
};

}

#endif /* KEYPOINTDETECTOR_H_ */
