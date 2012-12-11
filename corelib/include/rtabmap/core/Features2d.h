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

#ifndef KEYPOINTDESCRIPTOR_H_
#define KEYPOINTDESCRIPTOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include "rtabmap/core/Parameters.h"

namespace rtabmap {

/////////////////////
// KeypointDescriptor
/////////////////////
class RTABMAP_EXP KeypointDescriptor {
public:
	enum DescriptorType {kDescriptorSurf, kDescriptorSift, kDescriptorUndef};

public:
	virtual ~KeypointDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const = 0;

protected:
	KeypointDescriptor(const ParametersMap & parameters = ParametersMap());
};

//SURFDescriptor
class RTABMAP_EXP SURFDescriptor : public KeypointDescriptor
{
public:
	SURFDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~SURFDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	double _hessianThreshold;
	int _nOctaves;
	int _nOctaveLayers;
	bool _extended;
	bool _upright;

	bool _gpuVersion;
};

//SIFTDescriptor
class RTABMAP_EXP SIFTDescriptor : public KeypointDescriptor
{
public:
	SIFTDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~SIFTDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int _nfeatures;
	int _nOctaveLayers;
	double _contrastThreshold;
	double _edgeThreshold;
	double _sigma;
};


/////////////////////
// KeypointDetector
/////////////////////
class RTABMAP_EXP KeypointDetector
{
public:
	enum DetectorType {kDetectorSurf, kDetectorSift, kDetectorUndef};

public:
	virtual ~KeypointDetector() {}
	std::vector<cv::KeyPoint> generateKeypoints(const cv::Mat & image);
	virtual void parseParameters(const ParametersMap & parameters);
	unsigned int getWordsPerImageTarget() const {return _wordsPerImageTarget;}
	void setRoi(const std::string & roi);
	cv::Rect computeRoi(const cv::Mat & image) const;
protected:
	KeypointDetector(const ParametersMap & parameters = ParametersMap());
private:
	virtual std::vector<cv::KeyPoint> _generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const = 0;
private:
	unsigned int _wordsPerImageTarget;
	std::vector<float> _roiRatios; // size 4
};

//SURFDetector
class RTABMAP_EXP SURFDetector : public KeypointDetector
{
public:
	SURFDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~SURFDetector();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::vector<cv::KeyPoint> _generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const;
private:
	double _hessianThreshold;
	int _nOctaves;
	int _nOctaveLayers;
	bool _extended;
	bool _upright;

	bool _gpuVersion;
};

//SIFTDetector
class RTABMAP_EXP SIFTDetector : public KeypointDetector
{
public:
	SIFTDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~SIFTDetector();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::vector<cv::KeyPoint> _generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const;
private:
	int _nfeatures;
	int _nOctaveLayers;
	double _contrastThreshold;
	double _edgeThreshold;
	double _sigma;
};

}

#endif /* KEYPOINTDESCRIPTOR_H_ */
