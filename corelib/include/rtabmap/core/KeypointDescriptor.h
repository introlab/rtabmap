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

class RTABMAP_EXP KeypointDescriptor {
public:
	enum DescriptorType {kDescriptorSurf, kDescriptorSift, kDescriptorBrief, kDescriptorColor, kDescriptorHue, kDescriptorUndef};

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

//BRIEFDescriptor
class RTABMAP_EXP BRIEFDescriptor : public KeypointDescriptor
{
public:
	BRIEFDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~BRIEFDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int _size;
};

//MinMax ColorDescriptor
class RTABMAP_EXP ColorDescriptor : public KeypointDescriptor
{
public:
	ColorDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~ColorDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;
protected:
	void getCircularROI(int R, std::vector<int> & RxV) const;
};

//MinMax HueDescriptor
class RTABMAP_EXP HueDescriptor : public ColorDescriptor
{
public:
	HueDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~HueDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;
private:
	// assuming that rgb values are normalized [0,1]
	float rgb2hue(float r, float g, float b) const;

	// assuming that rgb values are normalized [0,1]
	inline float rgb2saturation(float r, float g, float b) const
	{
		float min = r;
		min<g?min=g:min;
		min<b?min=b:min;
		float eps = 0.00001f;

		return 1-(3*min)/(r+g+b+eps);
	}

	// assuming that rgb values are normalized [0,1]
	inline float rgb2intensity(float r, float g, float b) const
	{
		return (r+g+b)/3;
	}
};

}

#endif /* KEYPOINTDESCRIPTOR_H_ */
