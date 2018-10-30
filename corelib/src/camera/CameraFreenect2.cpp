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

#include <rtabmap/core/camera/CameraFreenect2.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_FREENECT2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#endif

namespace rtabmap
{

bool CameraFreenect2::available()
{
#ifdef RTABMAP_FREENECT2
	return true;
#else
	return false;
#endif
}

CameraFreenect2::CameraFreenect2(
	int deviceId, 
	Type type, 
	float imageRate, 
	const Transform & localTransform, 
	float minDepth,
	float maxDepth,
	bool bilateralFiltering,
	bool edgeAwareFiltering,
	bool noiseFiltering,
	const std::string & pipelineName) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_FREENECT2
        ,
		deviceId_(deviceId),
		type_(type),
		freenect2_(0),
		dev_(0),
		listener_(0),
		reg_(0),
		minKinect2Depth_(minDepth),
		maxKinect2Depth_(maxDepth),
		bilateralFiltering_(bilateralFiltering),
		edgeAwareFiltering_(edgeAwareFiltering),
		noiseFiltering_(noiseFiltering),
		pipelineName_(pipelineName)
#endif
{
#ifdef RTABMAP_FREENECT2
	UASSERT(minKinect2Depth_ < maxKinect2Depth_ && minKinect2Depth_>0 && maxKinect2Depth_>0 && maxKinect2Depth_<=65.535f);
	freenect2_ = new libfreenect2::Freenect2();
	switch(type_)
	{
	case kTypeColorIR:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir);
		break;
	case kTypeIRDepth:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		break;
	case kTypeColor2DepthSD:
	case kTypeDepth2ColorHD:
	case kTypeDepth2ColorSD:
	default:
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color  | libfreenect2::Frame::Depth);
		break;
	}
#endif
}

CameraFreenect2::~CameraFreenect2()
{
#ifdef RTABMAP_FREENECT2
	UDEBUG("");
	if(dev_)
	{
		dev_->stop();
		dev_->close();
		//deleted in freenect2_ destructor (Freeenect2Impl::clearDevices())
	}
	delete listener_;
	delete reg_;
	delete freenect2_;
	UDEBUG("");
#endif
}

#ifdef RTABMAP_FREENECT2
libfreenect2::PacketPipeline *createPacketPipelineByName(const std::string & name)
{
	std::string availablePipelines;
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
	availablePipelines += "gl ";
	if (name == "gl")
	{
		UINFO("Using 'gl' pipeline.");
		return new libfreenect2::OpenGLPacketPipeline();
	}
#endif
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
	availablePipelines += "cuda cudakde ";
	if (name == "cuda")
	{
		UINFO("Using 'cuda' pipeline.");
		return new libfreenect2::CudaPacketPipeline();
	}
	if (name == "cudakde")
	{
		UINFO("Using 'cudakde' pipeline.");
		return new libfreenect2::CudaKdePacketPipeline();
	}
#endif
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
	availablePipelines += "cl clkde ";
	if (name == "cl")
	{
		UINFO("Using 'cl' pipeline.");
		return new libfreenect2::OpenCLPacketPipeline();
	}
	if (name == "clkde")
	{
		UINFO("Using 'clkde' pipeline.");
		return new libfreenect2::OpenCLKdePacketPipeline();
	}
#endif
	availablePipelines += "cpu";
	if (name == "cpu")
	{
		UINFO("Using 'cpu' pipeline.");
		return new libfreenect2::CpuPacketPipeline();
	}

	if (!name.empty())
	{
		UERROR("'%s' pipeline is not available. Available pipelines are: \"%s\". Default one is used instead (first one in the list).", 
			name.c_str(), availablePipelines.c_str());
	}

	// create default pipeline
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
	UINFO("Using 'gl' pipeline.");
	return new libfreenect2::OpenGLPacketPipeline();
#elif defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
	UINFO("Using 'cuda' pipeline.");
	return new libfreenect2::CudaPacketPipeline();
#elif defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
	UINFO("Using 'cl' pipeline.");
	return new libfreenect2::OpenCLPacketPipeline();
#else
	UINFO("Using 'cpu' pipeline.");
	return new libfreenect2::CpuPacketPipeline();
#endif
}
#endif

bool CameraFreenect2::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_FREENECT2
	if(dev_)
	{
		dev_->stop();
		dev_->close();
		dev_ = 0; //deleted in freenect2_ destructor (Freeenect2Impl::clearDevices())
	}

	if(reg_)
	{
		delete reg_;
		reg_ = 0;
	}

	libfreenect2::PacketPipeline * pipeline = createPacketPipelineByName(pipelineName_);

	if(deviceId_ <= 0)
	{
		UDEBUG("Opening default device...");
		dev_ = freenect2_->openDefaultDevice(pipeline);
		pipeline = 0;// pipeline deleted in dev_ (Freenect2DeviceImpl::~Freenect2DeviceImpl())
	}
	else
	{
		UDEBUG("Opening device ID=%d...", deviceId_);
		dev_ = freenect2_->openDevice(deviceId_, pipeline);
		pipeline = 0;// pipeline deleted in dev_ (Freenect2DeviceImpl::~Freenect2DeviceImpl())
	}

	if(dev_)
	{
		//default
		//MinDepth(0.5f),
		//MaxDepth(4.5f),
		//EnableBilateralFilter(true),
		//EnableEdgeAwareFilter(true)
		libfreenect2::Freenect2Device::Config config;
		config.EnableBilateralFilter = bilateralFiltering_;
		config.EnableEdgeAwareFilter = edgeAwareFiltering_;
		config.MinDepth = minKinect2Depth_;
		config.MaxDepth = maxKinect2Depth_;
		dev_->setConfiguration(config);

		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);

		dev_->start();

		UINFO("CameraFreenect2: device serial: %s", dev_->getSerialNumber().c_str());
		UINFO("CameraFreenect2: device firmware: %s", dev_->getFirmwareVersion().c_str());

		//default registration params
		libfreenect2::Freenect2Device::IrCameraParams depthParams = dev_->getIrCameraParams();
		libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev_->getColorCameraParams();
		reg_ = new libfreenect2::Registration(depthParams, colorParams);

		// look for calibration files
		stereoModel_ = StereoCameraModel();
		if(!calibrationFolder.empty())
		{
			std::string calibrationName = dev_->getSerialNumber();
			if(!cameraName.empty())
			{
				calibrationName = cameraName;
			}
			stereoModel_.setName(calibrationName, "depth", "rgb");
			if(!stereoModel_.load(calibrationFolder, calibrationName, false))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration "
					"is used. Note that from version 0.11.10, calibration suffixes for Freenect2 driver have "
					"changed from \"_left\"->\"_depth\" and \"_right\"->\"_rgb\". You can safely rename "
					"the calibration files to avoid recalibrating.",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
					  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());

				if(type_==kTypeColor2DepthSD)
				{
					UWARN("Freenect2: When using custom calibration file, type "
						  "kTypeColor2DepthSD is not supported. kTypeDepth2ColorSD is used instead...");
					type_ = kTypeDepth2ColorSD;
				}

				// downscale color image by 2
				cv::Mat colorP = stereoModel_.right().P();
				cv::Size colorSize = stereoModel_.right().imageSize();
				if(type_ == kTypeDepth2ColorSD)
				{
					colorP.at<double>(0,0)/=2.0f; //fx
					colorP.at<double>(1,1)/=2.0f; //fy
					colorP.at<double>(0,2)/=2.0f; //cx
					colorP.at<double>(1,2)/=2.0f; //cy
					colorSize.width/=2;
					colorSize.height/=2;
				}
				cv::Mat depthP = stereoModel_.left().P();
				cv::Size depthSize = stereoModel_.left().imageSize();
				float ratioY = float(colorSize.height)/float(depthSize.height);
				float ratioX = float(colorSize.width)/float(depthSize.width);
				depthP.at<double>(0,0)*=ratioX; //fx
				depthP.at<double>(1,1)*=ratioY; //fy
				depthP.at<double>(0,2)*=ratioX; //cx
				depthP.at<double>(1,2)*=ratioY; //cy
				depthSize.width*=ratioX;
				depthSize.height*=ratioY;
				const CameraModel & l = stereoModel_.left();
				const CameraModel & r = stereoModel_.right();
				stereoModel_ = StereoCameraModel(stereoModel_.name(),
						depthSize, l.K_raw(), l.D_raw(), l.R(), depthP,
						colorSize, r.K_raw(), r.D_raw(), r.R(), colorP,
						stereoModel_.R(), stereoModel_.T(), stereoModel_.E(), stereoModel_.F());
				stereoModel_.initRectificationMap();
			}
		}

		return true;
	}
	else
	{
		UERROR("CameraFreenect2: no device connected or failure opening the default one! Note that rtabmap should link on libusb of libfreenect2. "
					"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
	}
#else
	UERROR("CameraFreenect2: RTAB-Map is not built with Freenect2 support!");
#endif
	return false;
}

bool CameraFreenect2::isCalibrated() const
{
	return true;
}

std::string CameraFreenect2::getSerial() const
{
#ifdef RTABMAP_FREENECT2
	if(dev_)
	{
		return dev_->getSerialNumber();
	}
#endif
	return "";
}

SensorData CameraFreenect2::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_FREENECT2
	if(dev_ && listener_)
	{
		libfreenect2::FrameMap frames;
#ifndef LIBFREENECT2_THREADING_STDLIB
		UDEBUG("Waiting for new frames... If it is stalled here, rtabmap should link on libusb of libfreenect2. "
				"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		listener_->waitForNewFrame(frames);
#else
		if(!listener_->waitForNewFrame(frames, 1000))
		{
			UWARN("CameraFreenect2: Failed to get frames! rtabmap should link on libusb of libfreenect2. "
					"Tip, before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		}
		else
#endif
		{
			double stamp = UTimer::now();
			libfreenect2::Frame *rgbFrame = 0;
			libfreenect2::Frame *irFrame = 0;
			libfreenect2::Frame *depthFrame = 0;

			switch(type_)
			{
			case kTypeColorIR: //used for calibration
				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
				irFrame = uValue(frames, libfreenect2::Frame::Ir, (libfreenect2::Frame*)0);
				break;
			case kTypeIRDepth:
				irFrame = uValue(frames, libfreenect2::Frame::Ir, (libfreenect2::Frame*)0);
				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
				break;
			case kTypeColor2DepthSD:
			case kTypeDepth2ColorSD:
			case kTypeDepth2ColorHD:
			case kTypeDepth2ColorHD2:
			default:
				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
				break;
			}

			cv::Mat rgb, depth;
			float fx=0,fy=0,cx=0,cy=0;
			if(irFrame && depthFrame)
			{
				cv::Mat irMat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data);
				//convert to gray scaled
				float maxIr_ = 0x7FFF;
				float minIr_ = 0x0;
				const float factor = 255.0f / float((maxIr_ - minIr_));
				rgb = cv::Mat(irMat.rows, irMat.cols, CV_8UC1);
				for(int i=0; i<irMat.rows; ++i)
				{
					for(int j=0; j<irMat.cols; ++j)
					{
						rgb.at<unsigned char>(i, j) = (unsigned char)std::min(float(std::max(irMat.at<float>(i,j) - minIr_, 0.0f)) * factor, 255.0f);
					}
				}

				cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);

				cv::flip(rgb, rgb, 1);
				cv::flip(depth, depth, 1);

				if(stereoModel_.isValidForRectification())
				{
					//rectify
					rgb = stereoModel_.left().rectifyImage(rgb);
					depth = stereoModel_.left().rectifyDepth(depth);
					fx = stereoModel_.left().fx();
					fy = stereoModel_.left().fy();
					cx = stereoModel_.left().cx();
					cy = stereoModel_.left().cy();
				}
				else
				{
					libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
					fx = params.fx;
					fy = params.fy;
					cx = params.cx;
					cy = params.cy;
				}
			}
			else
			{
				//rgb + ir or rgb + depth
				if(stereoModel_.isValidForRectification())
				{
					cv::Mat rgbMatC4((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
					cv::Mat rgbMat; // rtabmap uses 3 channels RGB
					#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

					 cv::cvtColor(rgbMatC4, rgbMat, CV_RGBA2BGR);

					#else 

					cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR);

					#endif 
					cv::flip(rgbMat, rgb, 1);

					//rectify color
					rgb = stereoModel_.right().rectifyImage(rgb);
					if(irFrame)
					{
						//rectify IR
						cv::Mat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);
						depth = stereoModel_.left().rectifyImage(depth);
					}
					else
					{
						//rectify depth
						cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);

						//depth = stereoModel_.left().rectifyImage(depth, 0); // ~0.5/4 ms but is more noisy
						depth = stereoModel_.left().rectifyDepth(depth);      // ~16/25 ms

						bool registered = true;
						if(registered)
						{
							depth = util2d::registerDepth(
									depth,
									stereoModel_.left().P().colRange(0,3).rowRange(0,3), //scaled depth K
									depth.size(),
									stereoModel_.right().P().colRange(0,3).rowRange(0,3), //scaled color K
									stereoModel_.stereoTransform());
							util2d::fillRegisteredDepthHoles(depth, true, false);
							fx = stereoModel_.right().fx();
							fy = stereoModel_.right().fy();
							cx = stereoModel_.right().cx();
							cy = stereoModel_.right().cy();
						}
						else
						{
							fx = stereoModel_.left().fx();
							fy = stereoModel_.left().fy();
							cx = stereoModel_.left().cx();
							cy = stereoModel_.left().cy();
						}
					}
				}
				else
				{
					//use data from libfreenect2
					if(irFrame)
					{
						cv::Mat rgbMatC4((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
						cv::Mat rgbMat; // rtabmap uses 3 channels RGB
						#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

						 cv::cvtColor(rgbMatC4, rgbMat, CV_RGB2BGR); 

						#else 

						cv::cvtColor(rgbMatC4, rgbMat, CV_BGRA2BGR); 

						#endif 
						cv::flip(rgbMat, rgb, 1);

						cv::Mat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data).convertTo(depth, CV_16U, 1);
						cv::flip(depth, depth, 1);
					}
					else
					{
						//registration of the depth
						UASSERT(reg_!=0);

						float maxDepth = maxKinect2Depth_*1000.0f;
						float minDepth =  minKinect2Depth_*1000.0f;
						if(type_ == kTypeColor2DepthSD || type_ == kTypeDepth2ColorHD)
						{
							cv::Mat rgbMatBGRA;
							libfreenect2::Frame depthUndistorted(512, 424, 4);
							libfreenect2::Frame rgbRegistered(512, 424, 4);

							// do it before registration
							if(noiseFiltering_)
							{
								cv::Mat depthMat = cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data);
								for(int dx=0; dx<depthMat.cols; ++dx)
								{
									bool onEdgeX = dx==depthMat.cols-1;
									for(int dy=0; dy<depthMat.rows; ++dy)
									{
										bool onEdge = onEdgeX || dy==depthMat.rows-1;
										float z = 0.0f;
										float & dz = depthMat.at<float>(dy,dx);
										if(dz>=minDepth && dz <= maxDepth)
										{
											z = dz;
											if(noiseFiltering_ && !onEdge)
											{
												z=0;
												const float & dz1 = depthMat.at<float>(dy,dx+1);
												const float & dz2 = depthMat.at<float>(dy+1,dx);
												const float & dz3 = depthMat.at<float>(dy+1,dx+1);
												if( dz1>=minDepth && dz1 <= maxDepth &&
													dz2>=minDepth && dz2 <= maxDepth &&
													dz3>=minDepth && dz3 <= maxDepth)
												{
													float avg = (dz + dz1 + dz2 + dz3) / 4.0f;
													float thres = 0.01f*avg;
											
													if( fabs(dz-avg) < thres &&
														fabs(dz1-avg) < thres &&
														fabs(dz2-avg) < thres &&
														fabs(dz3-avg) < thres)
													{
														z = dz;
													}
												}
											}
										}
										dz = z; 
									}
								}
							}
	
							libfreenect2::Frame bidDepth(1920, 1082, 4); // HD
							reg_->apply(rgbFrame, depthFrame, &depthUndistorted, &rgbRegistered, true, &bidDepth);
	
							cv::Mat depthMat;
							if(type_ == kTypeColor2DepthSD)
							{						
								rgbMatBGRA = cv::Mat((int)rgbRegistered.height, (int)rgbRegistered.width, CV_8UC4, rgbRegistered.data);
								depthMat = cv::Mat((int)depthUndistorted.height, (int)depthUndistorted.width, CV_32FC1, depthUndistorted.data);
						
								//use IR params
								libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
								fx = params.fx;
								fy = params.fy;
								cx = params.cx;
								cy = params.cy;
							}
							else
							{
								rgbMatBGRA = cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
								depthMat = cv::Mat((int)bidDepth.height, (int)bidDepth.width, CV_32FC1, bidDepth.data);
								depthMat = depthMat(cv::Range(1, 1081), cv::Range::all());
								
								//use color params
								libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
								fx = params.fx;
								fy = params.fy;
								cx = params.cx;
								cy = params.cy;
							}

							//filter max depth and flip
							depth = cv::Mat(depthMat.size(), CV_16UC1);
							for(int dx=0; dx<depthMat.cols; ++dx)
							{
								for(int dy=0; dy<depthMat.rows; ++dy)
								{
									unsigned short z = 0;
									const float & dz = depthMat.at<float>(dy,dx);
									if(dz>=minDepth && dz <= maxDepth)
									{
										z = (unsigned short)dz;
									}
									depth.at<unsigned short>(dy,(depthMat.cols-1)-dx) = z;  //flip
								}
							}
	
							// rtabmap uses 3 channels RGB
							#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

							 cv::cvtColor(rgbMatBGRA, rgb, CV_RGBA2BGR); 

							#else 

							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR); 

							#endif 
							cv::flip(rgb, rgb, 1);
						}
						else //register depth to color (OLD WAY)
						{
							UASSERT(type_ == kTypeDepth2ColorSD || type_ == kTypeDepth2ColorHD2);
							cv::Mat rgbMatBGRA = cv::Mat((int)rgbFrame->height, (int)rgbFrame->width, CV_8UC4, rgbFrame->data);
							if(type_ == kTypeDepth2ColorSD)
							{
								cv::Mat tmp;
								cv::resize(rgbMatBGRA, tmp, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
								rgbMatBGRA = tmp;
							}
							// rtabmap uses 3 channels RGB
							#ifdef LIBFREENECT2_WITH_TEGRAJPEG_SUPPORT 

							 cv::cvtColor(rgbMatBGRA, rgb, CV_RGBA2BGR); 

							#else 

							cv::cvtColor(rgbMatBGRA, rgb, CV_BGRA2BGR); 

							#endif 
							cv::flip(rgb, rgb, 1);	

							cv::Mat depthFrameMat = cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data);
							depth = cv::Mat::zeros(rgbMatBGRA.rows, rgbMatBGRA.cols, CV_16U);
							for(int dx=0; dx<depthFrameMat.cols-1; ++dx)
							{
								for(int dy=0; dy<depthFrameMat.rows-1; ++dy)
								{
									float dz = depthFrameMat.at<float>(dy,dx);
									if(dz>=minDepth && dz<=maxDepth)
									{
										bool goodDepth = true;
										if(noiseFiltering_)
										{
											goodDepth = false;
											float dz1 = depthFrameMat.at<float>(dy,dx+1);
											float dz2 = depthFrameMat.at<float>(dy+1,dx);
											float dz3 = depthFrameMat.at<float>(dy+1,dx+1);
											if(dz1>=minDepth && dz1 <= maxDepth &&
											   dz2>=minDepth && dz2 <= maxDepth &&
											   dz3>=minDepth && dz3 <= maxDepth)
											{
												float avg = (dz + dz1 + dz2 + dz3) / 4.0f;
												float thres = 0.01 * avg;
												if( fabs(dz-avg) < thres &&
													fabs(dz1-avg) < thres &&
													fabs(dz2-avg) < thres &&
													fabs(dz3-avg) < thres)
												{
													goodDepth = true;
												}
											}
										}
										if(goodDepth)
										{
											float cx=-1,cy=-1;
											reg_->apply(dx, dy, dz, cx, cy);
											if(type_ == kTypeDepth2ColorSD)
											{
												cx/=2.0f;
												cy/=2.0f;
											}
											int rcx = cvRound(cx);
											int rcy = cvRound(cy);
											if(uIsInBounds(rcx, 0, depth.cols) && uIsInBounds(rcy, 0, depth.rows))
											{
												unsigned short & zReg = depth.at<unsigned short>(rcy, rcx);
												if(zReg == 0 || zReg > (unsigned short)dz)
												{
													zReg = (unsigned short)dz;
												}
											}
										}
									}
								}
							}
							util2d::fillRegisteredDepthHoles(depth, true, true, type_==kTypeDepth2ColorHD2);
							util2d::fillRegisteredDepthHoles(depth, type_==kTypeDepth2ColorSD, type_==kTypeDepth2ColorHD2);//second pass
							cv::flip(depth, depth, 1);
							libfreenect2::Freenect2Device::ColorCameraParams params = dev_->getColorCameraParams();
							fx = params.fx*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							fy = params.fy*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							cx = params.cx*(type_==kTypeDepth2ColorSD?0.5:1.0f);
							cy = params.cy*(type_==kTypeDepth2ColorSD?0.5:1.0f);
						}
					}
				}
			}

			CameraModel model;
			if(fx && fy)
			{
				model=CameraModel(
						fx, //fx
						fy, //fy
						cx,  //cx
						cy, // cy
						this->getLocalTransform(),
						0,
						rgb.size());
			}
			data = SensorData(rgb, depth, model, this->getNextSeqID(), stamp);

			listener_->release(frames);
		}
	}
#else
	UERROR("CameraFreenect2: RTAB-Map is not built with Freenect2 support!");
#endif
	return data;
}

} // namespace rtabmap
