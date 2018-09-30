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
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/camera/CameraRGBDImages.h>

namespace rtabmap
{

bool CameraRGBDImages::available()
{
	return true;
}

CameraRGBDImages::CameraRGBDImages(
		const std::string & pathRGBImages,
		const std::string & pathDepthImages,
		float depthScaleFactor,
		float imageRate,
		const Transform & localTransform) :
		CameraImages(pathRGBImages, imageRate, localTransform)
{
	UASSERT(depthScaleFactor >= 1.0);
	cameraDepth_.setPath(pathDepthImages);
	cameraDepth_.setDepth(true, depthScaleFactor);
}

CameraRGBDImages::~CameraRGBDImages()
{
}

bool CameraRGBDImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	bool success = false;
	if(CameraImages::init(calibrationFolder, cameraName) && cameraDepth_.init())
	{
		if(this->imagesCount() == cameraDepth_.imagesCount())
		{
			success = true;
		}
		else
		{
			UERROR("Cameras don't have the same number of images (%d vs %d)",
					this->imagesCount(), cameraDepth_.imagesCount());
		}
	}

	return success;
}

bool CameraRGBDImages::isCalibrated() const
{
	return this->cameraModel().isValidForProjection();
}

std::string CameraRGBDImages::getSerial() const
{
	return this->cameraModel().name();
}

SensorData CameraRGBDImages::captureImage(CameraInfo * info)
{
	SensorData data;

	SensorData rgb, depth;
	rgb = CameraImages::captureImage(info);
	if(!rgb.imageRaw().empty())
	{
		depth = cameraDepth_.takeImage();
		if(!depth.depthRaw().empty())
		{
			data = SensorData(rgb.imageRaw(), depth.depthRaw(), rgb.cameraModels(), rgb.id(), rgb.stamp());
			data.setGroundTruth(rgb.groundTruth());
		}
	}
	return data;
}


} // namespace rtabmap
