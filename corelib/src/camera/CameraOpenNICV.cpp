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

#include <rtabmap/core/camera/CameraOpenNICV.h>
#include <rtabmap/utilite/UTimer.h>
#if CV_MAJOR_VERSION > 3
#include <opencv2/videoio/videoio_c.h>
#endif

namespace rtabmap
{

bool CameraOpenNICV::available()
{
	return cv::getBuildInformation().find("OpenNI:                      YES") != std::string::npos;
}

CameraOpenNICV::CameraOpenNICV(bool asus, float imageRate, const rtabmap::Transform & localTransform) :
	Camera(imageRate, localTransform),
	_asus(asus),
	_depthFocal(0.0f)
{

}

CameraOpenNICV::~CameraOpenNICV()
{
	_capture.release();
}

bool CameraOpenNICV::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	if(_capture.isOpened())
	{
		_capture.release();
	}

	ULOGGER_DEBUG("Camera::init()");
	_capture.open( _asus?CV_CAP_OPENNI_ASUS:CV_CAP_OPENNI );
	if(_capture.isOpened())
	{
		_capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
		_depthFocal = _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
		// Print some avalible device settings.
		UINFO("Depth generator output mode:");
		UINFO("FRAME_WIDTH        %f", _capture.get( CV_CAP_PROP_FRAME_WIDTH ));
		UINFO("FRAME_HEIGHT       %f", _capture.get( CV_CAP_PROP_FRAME_HEIGHT ));
		UINFO("FRAME_MAX_DEPTH    %f mm", _capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ));
		UINFO("BASELINE           %f mm", _capture.get( CV_CAP_PROP_OPENNI_BASELINE ));
		UINFO("FPS                %f", _capture.get( CV_CAP_PROP_FPS ));
		UINFO("Focal              %f", _capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ));
		UINFO("REGISTRATION       %f", _capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ));
		if(_capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) == 0.0)
		{
			UERROR("Depth registration is not activated on this device!");
		}
		if( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
		{
			UINFO("Image generator output mode:");
			UINFO("FRAME_WIDTH    %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ));
			UINFO("FRAME_HEIGHT   %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ));
			UINFO("FPS            %f", _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ));
		}
		else
		{
			UERROR("Camera: Device doesn't contain image generator.");
			_capture.release();
			return false;
		}
	}
	else
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	return true;
}

bool CameraOpenNICV::isCalibrated() const
{
	return true;
}

SensorData CameraOpenNICV::captureImage(CameraInfo * info)
{
	SensorData data;
	if(_capture.isOpened())
	{
		_capture.grab();
		cv::Mat depth, rgb;
		_capture.retrieve(depth, CV_CAP_OPENNI_DEPTH_MAP );
		_capture.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE );

		depth = depth.clone();
		rgb = rgb.clone();

		UASSERT(_depthFocal>0.0f);
		if(!rgb.empty() && !depth.empty())
		{
			CameraModel model(
					_depthFocal, //fx
					_depthFocal, //fy
					float(rgb.cols/2) - 0.5f,  //cx
					float(rgb.rows/2) - 0.5f,  //cy
					this->getLocalTransform(),
					0,
					rgb.size());
			data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}
	return data;
}

} // namespace rtabmap
