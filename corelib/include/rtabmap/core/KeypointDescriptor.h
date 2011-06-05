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
	virtual ~KeypointDescriptor();

	std::list<std::vector<float> > generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;
	void setChildDescriptor(KeypointDescriptor * childDescriptor);
	virtual void parseParameters(const ParametersMap & parameters);

protected:
	KeypointDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	const KeypointDescriptor * getChildDescriptor() const {return _childDescriptor;}

private:
	virtual std::list<std::vector<float> > _generateDescriptors(
		const IplImage * image,
		const std::list<cv::KeyPoint> & keypoints) const = 0;

private:
	KeypointDescriptor * _childDescriptor;
};

//SURFDescriptor
class RTABMAP_EXP SURFDescriptor : public KeypointDescriptor
{
public:
	SURFDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	virtual ~SURFDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::list<std::vector<float> > _generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;

private:
	cv::SURF _surf;
	bool _gpuVersion;
	bool _upright;
};

//SIFTDescriptor
class RTABMAP_EXP SIFTDescriptor : public KeypointDescriptor
{
public:
	SIFTDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	virtual ~SIFTDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::list<std::vector<float> > _generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;

private:
	cv::SIFT::CommonParams _commonParams;
	cv::SIFT::DescriptorParams _descriptorParams;
};

//LaplacianDescriptor
class RTABMAP_EXP LaplacianDescriptor : public KeypointDescriptor
{
public:
	LaplacianDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	virtual ~LaplacianDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::list<std::vector<float> > _generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;
};

//MinMax ColorDescriptor
class RTABMAP_EXP ColorDescriptor : public KeypointDescriptor
{
public:
	ColorDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	virtual ~ColorDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
protected:
	void getCircularROI(int R, std::vector<int> & RxV) const;
private:
	virtual std::list<std::vector<float> > _generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;
};

//MinMax HueDescriptor
class RTABMAP_EXP HueDescriptor : public ColorDescriptor
{
public:
	HueDescriptor(const ParametersMap & parameters = ParametersMap(), KeypointDescriptor * childDescriptor = 0);
	virtual ~HueDescriptor();
	virtual void parseParameters(const ParametersMap & parameters);
private:
	virtual std::list<std::vector<float> > _generateDescriptors(const IplImage * image, const std::list<cv::KeyPoint> & keypoints) const;

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
