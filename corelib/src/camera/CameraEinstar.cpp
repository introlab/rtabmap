/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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


#include <rtabmap/core/camera/CameraEinstar.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef RTABMAP_EINSTAR
#include <einstar/calibration.hpp>
#include <einstar/depth.hpp>
#include <einstar/device.hpp>
#include <einstar/exposure.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#endif

namespace rtabmap
{

#ifdef RTABMAP_EINSTAR
namespace
{
const int kWidth = 1280;
const int kHeight = 1024;
const size_t kMaxPending = 8;
}

struct CameraEinstar::Impl
{
	struct Group
	{
		cv::Mat ir[2];
		cv::Mat color; // raw Bayer
	};

	std::unique_ptr<einstar::StereoDepth> depth;
	std::unique_ptr<einstar::Device> device;
	CameraModel colorModel;
	std::string serial;

	std::mutex mutex;
	std::condition_variable cv;
	std::map<uint32_t, Group> pending;
	std::deque<Group> ready;
	cv::Mat lastColor;

	std::vector<uint32_t> replayIds;
	size_t replayIndex = 0;
	// The first frame sets after the trigger starts are overexposed (observed),
	// as the vendor's CEinScan10::discardFrames suggests.
	int setsToDiscard = 2;
	einstar::AutoExposure autoExposure;
	uint32_t irExposureUs = einstar::ScanSettings().ir_exposure_us;

	// Bring-up aid: with EINSTAR_RECORD_DIR set, live frame sets are saved
	// like einstar-probe captures (replayable) with a per-frame log.
	std::string recordDir;
	std::ofstream recordLog;
	uint32_t recordSeq = 0;

	void onFrame(einstar::Frame && f)
	{
		std::optional<unsigned> cam = f.info.camera_index();
		if(!cam)
		{
			return;
		}
		cv::Mat img = cv::Mat(kHeight, kWidth, CV_8UC1, f.data.data()).clone();
		std::lock_guard<std::mutex> lock(mutex);
		if(*cam == 2)
		{
			lastColor = img;
			return;
		}
		Group & g = pending[f.info.frame_id];
		g.ir[*cam] = img;
		if(!g.ir[0].empty() && !g.ir[1].empty())
		{
			g.color = lastColor;
			if(setsToDiscard > 0)
			{
				--setsToDiscard;
				pending.erase(f.info.frame_id);
				return;
			}
			ready.push_back(std::move(g));
			pending.erase(f.info.frame_id);
			if(ready.size() > 2)
			{
				ready.pop_front(); // keep latency low
			}
			cv.notify_one();
		}
		while(pending.size() > kMaxPending)
		{
			pending.erase(pending.begin());
		}
	}

	bool nextLive(Group & out, int timeoutMs)
	{
		std::unique_lock<std::mutex> lock(mutex);
		if(!cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]{return !ready.empty();}))
		{
			return false;
		}
		out = std::move(ready.front());
		ready.pop_front();
		return true;
	}

	bool nextReplay(const std::string & dir, Group & out)
	{
		while(replayIndex < replayIds.size())
		{
			uint32_t id = replayIds[replayIndex++];
			Group g;
			for(unsigned c=0; c<3; ++c)
			{
				char name[64];
				snprintf(name, sizeof(name), "frame_%06u_cam%u.pgm", id, c);
				cv::Mat m = cv::imread(dir + "/" + name, cv::IMREAD_GRAYSCALE);
				if(c < 2)
				{
					g.ir[c] = m;
				}
				else if(!m.empty())
				{
					lastColor = m;
				}
			}
			g.color = lastColor;
			if(!g.ir[0].empty() && !g.ir[1].empty())
			{
				out = std::move(g);
				return true;
			}
		}
		return false;
	}
};
#endif

bool CameraEinstar::available()
{
#ifdef RTABMAP_EINSTAR
	return true;
#else
	return false;
#endif
}

CameraEinstar::CameraEinstar(
		const std::string & calibrationPath,
		const std::string & replayDir,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	calibrationPath_(calibrationPath),
	replayDir_(replayDir),
	minDepthMm_(130.0f),
	maxDepthMm_(800.0f)
{
}

CameraEinstar::~CameraEinstar()
{
#ifdef RTABMAP_EINSTAR
	if(impl_.get() && impl_->device.get())
	{
		impl_->device->stop_scanning();
		impl_->device->lights_off();
		impl_->device->stop_stream();
	}
#endif
}

void CameraEinstar::setDepthRange(float minDepthMm, float maxDepthMm)
{
	minDepthMm_ = minDepthMm;
	maxDepthMm_ = maxDepthMm;
}

bool CameraEinstar::init(const std::string &, const std::string &)
{
#ifdef RTABMAP_EINSTAR
	impl_.reset(new Impl());
	const bool live = replayDir_.empty();
	if(live)
	{
		// Writes are needed to start the scan trigger; libeinstar only allows
		// volatile settings (no flash writes or reboot).
		einstar::OpenOptions options;
		options.allow_writes = true;
		einstar::Result<std::unique_ptr<einstar::Device> > dev = einstar::Device::open(std::nullopt, options);
		if(!dev)
		{
			UERROR("Einstar: %s", dev.error().message.c_str());
			return false;
		}
		impl_->device = std::move(dev.value());
		impl_->serial = impl_->device->serial();
		if(const char * dir = std::getenv("EINSTAR_RECORD_DIR"))
		{
			impl_->recordDir = dir;
			UDirectory::makeDir(impl_->recordDir);
			impl_->recordLog.open(impl_->recordDir + "/record.tsv");
			impl_->recordLog << "seq\tir_exposure_us\tdepth_coverage\n";
			UWARN("Einstar: recording frame sets to %s", dir);
		}
	}

	std::optional<einstar::CameraModel> model;
	if(calibrationPath_.empty() && live)
	{
		einstar::Result<einstar::CameraModel> fromDevice = impl_->device->read_calibration();
		if(!fromDevice)
		{
			UERROR("Einstar: cannot read calibration from the scanner: %s", fromDevice.error().message.c_str());
			return false;
		}
		model = fromDevice.value();
	}
	else
	{
		einstar::CalibrationResult calib = einstar::load_calibration(calibrationPath_);
		if(!calib.model)
		{
			UERROR("Einstar: cannot load calibration \"%s\": %s", calibrationPath_.c_str(), einstar::to_string(*calib.error));
			return false;
		}
		model = calib.model;
	}
	einstar::DepthParams params;
	params.min_depth_mm = minDepthMm_;
	params.max_depth_mm = maxDepthMm_;
	impl_->depth.reset(new einstar::StereoDepth(*model, cv::Size(kWidth, kHeight), params));
	// The calibration frame is upside down relative to the scanner's natural
	// orientation (how the colour stream arrives), so images are rotated by
	// 180 degrees for output: the principal point mirrors and the optical
	// frame turns about its z axis.
	cv::Mat K = impl_->depth->color_K().clone();
	K.at<double>(0, 2) = kWidth - 1 - K.at<double>(0, 2);
	K.at<double>(1, 2) = kHeight - 1 - K.at<double>(1, 2);
	cv::Mat P = cv::Mat::zeros(3, 4, CV_64FC1);
	K.copyTo(P.colRange(0, 3));
	impl_->colorModel = CameraModel(
			"einstar_color",
			cv::Size(kWidth, kHeight),
			K,
			cv::Mat::zeros(1, 5, CV_64FC1),
			cv::Mat::eye(3, 3, CV_64FC1),
			P,
			this->getLocalTransform() * Transform(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0));
	UINFO("Einstar: calibration loaded, IR baseline %.1f mm", impl_->depth->baseline_mm());

	if(!live)
	{
		UDirectory dir(replayDir_, "pgm");
		std::set<uint32_t> ids;
		for(std::string f = dir.getNextFileName(); !f.empty(); f = dir.getNextFileName())
		{
			unsigned id, cam;
			if(sscanf(f.c_str(), "frame_%u_cam%u.pgm", &id, &cam) == 2)
			{
				ids.insert(id);
			}
		}
		impl_->replayIds.assign(ids.begin(), ids.end());
		impl_->serial = "replay";
		UINFO("Einstar: replaying %d frames from %s", (int)impl_->replayIds.size(), replayDir_.c_str());
		return !impl_->replayIds.empty();
	}

	Impl * impl = impl_.get();
	einstar::Status s = impl_->device->start_stream(
			[impl](einstar::Frame && f){impl->onFrame(std::move(f));},
			(size_t)kWidth * kHeight);
	if(!s)
	{
		UERROR("Einstar: %s", s.error().message.c_str());
		return false;
	}
	s = impl_->device->apply(einstar::ScanSettings());
	if(!s)
	{
		UERROR("Einstar: cannot apply scan settings: %s", s.error().message.c_str());
		impl_->device->stop_stream();
		return false;
	}
	s = impl_->device->start_scanning();
	if(!s)
	{
		UERROR("Einstar: cannot start scanning: %s", s.error().message.c_str());
		impl_->device->stop_stream();
		return false;
	}
	return true;
#else
	UERROR("CameraEinstar: RTAB-Map is not built with Einstar support!");
	return false;
#endif
}

bool CameraEinstar::isCalibrated() const
{
#ifdef RTABMAP_EINSTAR
	return impl_.get() && impl_->depth.get();
#else
	return false;
#endif
}

std::string CameraEinstar::getSerial() const
{
#ifdef RTABMAP_EINSTAR
	if(impl_.get())
	{
		return impl_->serial;
	}
#endif
	return "";
}

SensorData CameraEinstar::captureImage(SensorCaptureInfo * info)
{
	SensorData data;
#ifdef RTABMAP_EINSTAR
	if(!impl_.get() || !impl_->depth.get())
	{
		UERROR("Einstar: not initialized");
		return data;
	}
	UTimer timer;
	Impl::Group g;
	bool ok = replayDir_.empty() ? impl_->nextLive(g, 1000) : impl_->nextReplay(replayDir_, g);
	if(!ok)
	{
		return data;
	}
	if(g.color.empty())
	{
		// Depth is registered to the colour camera, so wait for a colour frame.
		UWARN("Einstar: no colour frame received yet, skipping IR pair.");
		return data;
	}
	// g.ir is indexed by stream camera; map it to the calibration IR cameras
	// in calibration orientation.
	cv::Mat irLeft, irRight;
	for(unsigned stream = 0; stream < 2; ++stream)
	{
		(einstar::calibration_index_for_stream(stream) == einstar::kIrLeftCamera ? irLeft : irRight) =
				einstar::to_calibration_orientation(g.ir[stream], stream);
	}
	const uint32_t exposureUs = impl_->irExposureUs; // in effect when this set was captured (approximately)
	if(impl_->device.get())
	{
		// Keep the speckle unsaturated; adjust from this thread, not the stream thread.
		std::optional<uint32_t> next = impl_->autoExposure.update(
				einstar::ByteView(g.ir[0].data, g.ir[0].total()),
				einstar::ByteView(g.ir[1].data, g.ir[1].total()),
				impl_->irExposureUs);
		if(next && impl_->device->set_exposure_us(0, *next) && impl_->device->set_exposure_us(1, *next))
		{
			impl_->irExposureUs = *next;
		}
	}
	cv::Mat depthMm = impl_->depth->depth_in_color(irLeft, irRight);
	if(!impl_->recordDir.empty())
	{
		const uint32_t seq = impl_->recordSeq++;
		const cv::Mat * images[3] = {&g.ir[0], &g.ir[1], &g.color};
		for(unsigned c=0; c<3; ++c)
		{
			char name[64];
			snprintf(name, sizeof(name), "/frame_%06u_cam%u.pgm", seq, c);
			cv::imwrite(impl_->recordDir + name, *images[c]);
		}
		impl_->recordLog << seq << '\t' << exposureUs << '\t'
				<< (double)cv::countNonZero(depthMm) / depthMm.total() << '\n';
		impl_->recordLog.flush();
	}
	cv::Mat depth16;
	depthMm.convertTo(depth16, CV_16UC1);
	cv::Mat rgb = impl_->depth->undistort_color(einstar::color_in_calibration_orientation(g.color));
	// Present upright (see colorModel).
	cv::rotate(depth16, depth16, cv::ROTATE_180);
	cv::rotate(rgb, rgb, cv::ROTATE_180);
	data = SensorData(rgb, depth16, impl_->colorModel, this->getNextSeqID(), UTimer::now());
	if(info)
	{
		info->timeCapture = timer.ticks();
	}
#else
	UERROR("CameraEinstar: RTAB-Map is not built with Einstar support!");
#endif
	return data;
}

} // namespace rtabmap
