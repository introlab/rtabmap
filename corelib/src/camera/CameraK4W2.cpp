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

#include <rtabmap/core/camera/CameraK4W2.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_K4W2
#include <Kinect.h>
#endif

namespace rtabmap
{

#ifdef RTABMAP_K4W2
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
#endif

bool CameraK4W2::available()
{
#ifdef RTABMAP_K4W2
	return true;
#else
	return false;
#endif
}

CameraK4W2::CameraK4W2(
	int deviceId,
	Type type,
	float imageRate,
	const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_K4W2
	,
	type_(type),
	pKinectSensor_(NULL),
	pCoordinateMapper_(NULL),
	pDepthCoordinates_(new DepthSpacePoint[cColorWidth * cColorHeight]),
	pColorCoordinates_(new ColorSpacePoint[cDepthWidth * cDepthHeight]),
	pMultiSourceFrameReader_(NULL),
	pColorRGBX_(new RGBQUAD[cColorWidth * cColorHeight]),
	hMSEvent(NULL)
#endif
{
}

CameraK4W2::~CameraK4W2()
{
#ifdef RTABMAP_K4W2
	if (pDepthCoordinates_)
	{
		delete[] pDepthCoordinates_;
		pDepthCoordinates_ = NULL;
	}

	if (pColorCoordinates_)
	{
		delete[] pColorCoordinates_;
		pColorCoordinates_ = NULL;
	}

	if (pColorRGBX_)
	{
		delete[] pColorRGBX_;
		pColorRGBX_ = NULL;
	}

	close();
#endif
}

void CameraK4W2::close()
{
#ifdef RTABMAP_K4W2
	if (pMultiSourceFrameReader_)
	{
		pMultiSourceFrameReader_->UnsubscribeMultiSourceFrameArrived(hMSEvent);
		CloseHandle((HANDLE)hMSEvent);
		hMSEvent = NULL;
	}

	// done with frame reader
	SafeRelease(pMultiSourceFrameReader_);

	// done with coordinate mapper
	SafeRelease(pCoordinateMapper_);

	// close the Kinect Sensor
	if (pKinectSensor_)
	{
		pKinectSensor_->Close();
	}

	SafeRelease(pKinectSensor_);

	colorCameraModel_ = CameraModel();
#endif
}

bool CameraK4W2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_K4W2
	HRESULT hr;

	close();

	hr = GetDefaultKinectSensor(&pKinectSensor_);
	if (FAILED(hr))
	{
		return false;
	}

	if (pKinectSensor_)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		hr = pKinectSensor_->Open();

		if (SUCCEEDED(hr))
		{
			hr = pKinectSensor_->get_CoordinateMapper(&pCoordinateMapper_);

			if (SUCCEEDED(hr))
			{
				hr = pKinectSensor_->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
					&pMultiSourceFrameReader_);

				if (SUCCEEDED(hr))
				{
					hr = pMultiSourceFrameReader_->SubscribeMultiSourceFrameArrived(&hMSEvent);
				}
			}
		}
	}

	if (!pKinectSensor_ || FAILED(hr))
	{
		UERROR("No ready Kinect found!");
		close();
		return false;
	}

	// to query camera parameters, we should wait a little
	uSleep(3000);

	// initialize color calibration if not set yet
	CameraIntrinsics intrinsics;
	hr = pCoordinateMapper_->GetDepthCameraIntrinsics(&intrinsics);
	if (SUCCEEDED(hr) && intrinsics.FocalLengthX > 0.0f)
	{
		// guess color intrinsics by comparing two reprojections
		CameraModel depthModel(
			intrinsics.FocalLengthX,
			intrinsics.FocalLengthY,
			intrinsics.PrincipalPointX,
			intrinsics.PrincipalPointY);

		cv::Mat fakeDepth = cv::Mat::ones(cDepthHeight, cDepthWidth, CV_16UC1) * 1000;
		hr = pCoordinateMapper_->MapDepthFrameToColorSpace(cDepthWidth * cDepthHeight, (UINT16*)fakeDepth.data, cDepthWidth * cDepthHeight, pColorCoordinates_);
		if (SUCCEEDED(hr))
		{
			int firstIndex = -1;
			int lastIndex = -1;
			for (int depthIndex = 0; depthIndex < (cDepthWidth*cDepthHeight); ++depthIndex)
			{
				ColorSpacePoint p = pColorCoordinates_[depthIndex];
				// Values that are negative infinity means it is an invalid color to depth mapping so we
				// skip processing for this pixel
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					if (firstIndex == -1)
					{
						firstIndex = depthIndex;
					}
					lastIndex = depthIndex;
				}
			}

			UASSERT(firstIndex >= 0 && lastIndex >= 0);
			float fx, fy, cx, cy;
			float x1, y1, z1, x2, y2, z2;
			depthModel.project(firstIndex - (firstIndex / cDepthWidth)*cDepthWidth, firstIndex / cDepthWidth, 1.0f, x1, y1, z1);
			depthModel.project(lastIndex - (lastIndex / cDepthWidth)*cDepthWidth, lastIndex / cDepthWidth, 1.0f, x2, y2, z2);
			ColorSpacePoint uv1 = pColorCoordinates_[firstIndex];
			ColorSpacePoint uv2 = pColorCoordinates_[lastIndex];
			fx = ((uv1.X - uv2.X)*z1*z2) / (x1*z2 - x2*z1);
			cx = uv1.X - (x1 / z1) * fx;
			fy = ((uv1.Y - uv2.Y)*z1*z2) / (y1*z2 - y2*z1);
			cy = uv1.Y - (y1 / z1) * fy;

			colorCameraModel_ = CameraModel(
				fx,
				fy,
				float(cColorWidth) - cx,
				cy,
				this->getLocalTransform(),
				0,
				cv::Size(cColorWidth, cColorHeight));
		}
	}

	if (!colorCameraModel_.isValidForProjection())
	{
		UERROR("Failed to get camera parameters! Is the camera connected? Try restarting the camera again or use kTypeColor2DepthSD.");
		close();
		return false;
	}

	std::string serial = getSerial();
	if (!serial.empty())
	{
		UINFO("Running kinect device \"%s\"", serial.c_str());
	}

	return true;
#else
	UERROR("CameraK4W2: RTAB-Map is not built with Kinect for Windows 2 SDK support!");
	return false;
#endif
}

bool CameraK4W2::isCalibrated() const
{
	return true;
}

std::string CameraK4W2::getSerial() const
{
#ifdef RTABMAP_K4W2
	if (pKinectSensor_)
	{
		wchar_t uid[255] = { 0 };
		// It seems to fail every time!?
		HRESULT hr = pKinectSensor_->get_UniqueKinectId(255, uid);
		if (SUCCEEDED(hr))
		{
			std::wstring ws(uid);
			return std::string(ws.begin(), ws.end());
		}
	}
#endif
	return "";
}

SensorData CameraK4W2::captureImage(CameraInfo * info)
{
	SensorData data;

#ifdef RTABMAP_K4W2

	if (!pMultiSourceFrameReader_)
	{
		return data;
	}

	HRESULT hr;

	//now check for frame events
	HANDLE handles[] = { reinterpret_cast<HANDLE>(hMSEvent) };

	double t = UTimer::now();
	while((UTimer::now()-t < 5.0) && WaitForMultipleObjects(_countof(handles), handles, false, 5000) == WAIT_OBJECT_0)
	{
		IMultiSourceFrameArrivedEventArgs* pArgs = NULL;

		hr = pMultiSourceFrameReader_->GetMultiSourceFrameArrivedEventData(hMSEvent, &pArgs);
		if (SUCCEEDED(hr))
		{
			IMultiSourceFrameReference * pFrameRef = NULL;
			hr = pArgs->get_FrameReference(&pFrameRef);
			if (SUCCEEDED(hr))
			{
				IMultiSourceFrame* pMultiSourceFrame = NULL;
				IDepthFrame* pDepthFrame = NULL;
				IColorFrame* pColorFrame = NULL;

				hr = pFrameRef->AcquireFrame(&pMultiSourceFrame);
				if (FAILED(hr))
				{
					UERROR("Failed getting latest frame.");
				}

				IDepthFrameReference* pDepthFrameReference = NULL;
				hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
				if (SUCCEEDED(hr))
				{
					hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
				}
				SafeRelease(pDepthFrameReference);

				IColorFrameReference* pColorFrameReference = NULL;
				hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
				if (SUCCEEDED(hr))
				{
					hr = pColorFrameReference->AcquireFrame(&pColorFrame);
				}
				SafeRelease(pColorFrameReference);

				if (pDepthFrame && pColorFrame)
				{
					IFrameDescription* pDepthFrameDescription = NULL;
					int nDepthWidth = 0;
					int nDepthHeight = 0;
					UINT nDepthBufferSize = 0;
					UINT16 *pDepthBuffer = NULL;

					IFrameDescription* pColorFrameDescription = NULL;
					int nColorWidth = 0;
					int nColorHeight = 0;
					ColorImageFormat imageFormat = ColorImageFormat_None;
					UINT nColorBufferSize = 0;
					RGBQUAD *pColorBuffer = NULL;

					// get depth frame data
					if (SUCCEEDED(hr))
						hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
					if (SUCCEEDED(hr))
						hr = pDepthFrameDescription->get_Width(&nDepthWidth);
					if (SUCCEEDED(hr))
						hr = pDepthFrameDescription->get_Height(&nDepthHeight);
					if (SUCCEEDED(hr))
						hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);

					// get color frame data
					if (SUCCEEDED(hr))
						hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
					if (SUCCEEDED(hr))
						hr = pColorFrameDescription->get_Width(&nColorWidth);
					if (SUCCEEDED(hr))
						hr = pColorFrameDescription->get_Height(&nColorHeight);
					if (SUCCEEDED(hr))
						hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
					if (SUCCEEDED(hr))
					{
						if (imageFormat == ColorImageFormat_Bgra)
						{
							hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
						}
						else if (pColorRGBX_)
						{
							pColorBuffer = pColorRGBX_;
							nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
							hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
						}
						else
						{
							hr = E_FAIL;
						}
					}

					if(SUCCEEDED(hr))
					{
						//ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
						//	pColorBuffer, nColorWidth, nColorHeight,
						//	pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);

						// Make sure we've received valid data
						if (pCoordinateMapper_ &&
							pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
							pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight))
						{
							if (type_ == kTypeColor2DepthSD)
							{
								HRESULT hr = pCoordinateMapper_->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, pDepthCoordinates_);
								if (SUCCEEDED(hr))
								{
									cv::Mat depth = cv::Mat::zeros(nDepthHeight, nDepthWidth, CV_16UC1);
									cv::Mat imageColorRegistered = cv::Mat::zeros(nDepthHeight, nDepthWidth, CV_8UC3);
									// loop over output pixels
									for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
									{
										DepthSpacePoint p = pDepthCoordinates_[colorIndex];
										// Values that are negative infinity means it is an invalid color to depth mapping so we
										// skip processing for this pixel
										if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
										{
											// To avoid black lines caused by rounding pixel values, we should set 4 pixels
											// At the same do mirror
											int pixel_x_l, pixel_y_l, pixel_x_h, pixel_y_h;
											pixel_x_l = nDepthWidth - static_cast<int>(p.X);
											pixel_y_l = static_cast<int>(p.Y);
											pixel_x_h = pixel_x_l - 1;
											pixel_y_h = pixel_y_l + 1;

											const RGBQUAD* pSrc = pColorBuffer + colorIndex;
											if ((pixel_x_l >= 0 && pixel_x_l < nDepthWidth) && (pixel_y_l >= 0 && pixel_y_l < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_l, pixel_x_l);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_l, pixel_x_l) = *(pDepthBuffer + nDepthWidth - pixel_x_l + pixel_y_l*nDepthWidth);
											}
											if ((pixel_x_l >= 0 && pixel_x_l < nDepthWidth) && (pixel_y_h >= 0 && pixel_y_h < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_h, pixel_x_l);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_h, pixel_x_l) = *(pDepthBuffer + nDepthWidth - pixel_x_l + pixel_y_h*nDepthWidth);
											}
											if ((pixel_x_h >= 0 && pixel_x_h < nDepthWidth) && (pixel_y_l >= 0 && pixel_y_l < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_l, pixel_x_h);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_l, pixel_x_h) = *(pDepthBuffer + nDepthWidth - pixel_x_h + pixel_y_l*nDepthWidth);
											}
											if ((pixel_x_h >= 0 && pixel_x_h < nDepthWidth) && (pixel_y_h >= 0 && pixel_y_h < nDepthHeight))
											{
												unsigned char *  ptr = imageColorRegistered.ptr<unsigned char>(pixel_y_h, pixel_x_h);
												ptr[0] = pSrc->rgbBlue;
												ptr[1] = pSrc->rgbGreen;
												ptr[2] = pSrc->rgbRed;
												depth.at<unsigned short>(pixel_y_h, pixel_x_h) = *(pDepthBuffer + nDepthWidth - pixel_x_h + pixel_y_h*nDepthWidth);
											}
										}
									}

									CameraIntrinsics intrinsics;
									pCoordinateMapper_->GetDepthCameraIntrinsics(&intrinsics);
									CameraModel model(
										intrinsics.FocalLengthX,
										intrinsics.FocalLengthY,
										intrinsics.PrincipalPointX,
										intrinsics.PrincipalPointY,
										this->getLocalTransform(),
										0,
										depth.size());
									data = SensorData(imageColorRegistered, depth, model, this->getNextSeqID(), UTimer::now());
								}
								else
								{
									UERROR("Failed color to depth registration!");
								}
							}
							else //depthToColor
							{
								HRESULT hr = pCoordinateMapper_->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, pColorCoordinates_);
								if (SUCCEEDED(hr))
								{
									cv::Mat depthSource(nDepthHeight, nDepthWidth, CV_16UC1, pDepthBuffer);
									cv::Mat depthRegistered = cv::Mat::zeros(
										type_ == kTypeDepth2ColorSD ? nColorHeight/2 : nColorHeight, 
										type_ == kTypeDepth2ColorSD ? nColorWidth/2 : nColorWidth,
										CV_16UC1);
									cv::Mat imageColor;
									if(type_ == kTypeDepth2ColorSD)
									{
										cv::Mat tmp;
										cv::resize(cv::Mat(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer), tmp, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
										cv::cvtColor(tmp, imageColor, CV_BGRA2BGR);
									}
									else
									{
										cv::cvtColor(cv::Mat(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer), imageColor, CV_BGRA2BGR);
									}
									// loop over output pixels
									for (int depthIndex = 0; depthIndex < (nDepthWidth*nDepthHeight); ++depthIndex)
									{
										ColorSpacePoint p = pColorCoordinates_[depthIndex];
										// Values that are negative infinity means it is an invalid color to depth mapping so we
										// skip processing for this pixel
										if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
										{
											if (type_ == kTypeDepth2ColorSD)
											{
												p.X /= 2.0f;
												p.Y /= 2.0f;
											}
											const unsigned short & depth_value = depthSource.at<unsigned short>(0, depthIndex);
											int pixel_x_l, pixel_y_l, pixel_x_h, pixel_y_h;
											// get the coordinate on image plane.
											pixel_x_l = depthRegistered.cols - p.X; // flip depth
											pixel_y_l = p.Y;
											pixel_x_h = pixel_x_l - 1;
											pixel_y_h = pixel_y_l + 1;

											if (pixel_x_l >= 0 && pixel_x_l < depthRegistered.cols &&
												pixel_y_l>0 && pixel_y_l < depthRegistered.rows && // ignore first line
												depth_value)
											{
												unsigned short & depthPixel = depthRegistered.at<unsigned short>(pixel_y_l, pixel_x_l);
												if (depthPixel == 0 || depthPixel > depth_value)
												{
													depthPixel = depth_value;
												}
											}
											if (pixel_x_h >= 0 && pixel_x_h < depthRegistered.cols &&
												pixel_y_h>0 && pixel_y_h < depthRegistered.rows && // ignore first line
												depth_value)
											{
												unsigned short & depthPixel = depthRegistered.at<unsigned short>(pixel_y_h, pixel_x_h);
												if (depthPixel == 0 || depthPixel > depth_value)
												{
													depthPixel = depth_value;
												}
											}
										}
									}

									CameraModel model = colorCameraModel_;
									if (type_ == kTypeDepth2ColorSD)
									{
										model = model.scaled(0.5);
									}
									util2d::fillRegisteredDepthHoles(depthRegistered, true, true, type_ == kTypeDepth2ColorHD);
									depthRegistered = rtabmap::util2d::fillDepthHoles(depthRegistered, 1);
									cv::flip(imageColor, imageColor, 1);
									data = SensorData(imageColor, depthRegistered, model, this->getNextSeqID(), UTimer::now());
								}
								else
								{
									UERROR("Failed depth to color registration!");
								}
							}
						}
					}

					SafeRelease(pDepthFrameDescription);
					SafeRelease(pColorFrameDescription);
				}

				pFrameRef->Release();

				SafeRelease(pDepthFrame);
				SafeRelease(pColorFrame);
				SafeRelease(pMultiSourceFrame);
			}	
			pArgs->Release();
		}
		if (!data.imageRaw().empty())
		{
			break;
		}
	}
#else
	UERROR("CameraK4W2: RTAB-Map is not built with Kinect for Windows 2 SDK support!");
#endif
	return data;
}

} // namespace rtabmap
