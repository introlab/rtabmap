/*
Copyright (c) 2010-2022, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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
#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_

// Should be first on windows to avoid "WinSock.h has already been included" error
#include <pcl/io/vlp_grabber.h>
#include <boost/version.hpp>
#include <boost/asio.hpp>

#include <rtabmap/core/Lidar.h>
#include <rtabmap/utilite/USemaphore.h>

#include <thread>
#include <atomic>

// On Apple, closing pcl::HDLGrabber's UDP socket while its read thread is blocked
// in receive_from returns EBADF, which pcl::HDLGrabber::readPacketsFromSocket()
// rethrows uncaught in its own thread, aborting the process on shutdown. There we
// run our own single-threaded UDP receive loop and own the socket lifecycle.
// Other platforms use the base pcl::VLPGrabber path (which also avoids needing
// boost::asio::io_context, absent from the Boost 1.65 on e.g. Ubuntu Bionic).
// Comment this out to force the base path everywhere (e.g. A/B testing on macOS).
#if defined(__APPLE__)
#define RTABMAP_VLP16_USE_CUSTOM_SOCKET
#endif

namespace rtabmap {

struct PointXYZIT {
	float x;
	float y;
	float z;
	float i;
	float t;
};

class RTABMAP_CORE_EXPORT LidarVLP16 :public Lidar, public pcl::VLPGrabber {
public:
	LidarVLP16(
			const std::string& pcapFile,
			bool organized = false,
			bool stampLast = true,
			float frameRate = 0.0f,
			Transform localTransform = Transform::getIdentity());
	LidarVLP16(
			const boost::asio::ip::address& ipAddress,
			const std::uint16_t port = 2368,
			bool organized = false,
			bool useHostTime = true,
			bool stampLast = true,
			float frameRate = 0.0f,
			Transform localTransform = Transform::getIdentity());
	virtual ~LidarVLP16();

	SensorData takeScan(SensorCaptureInfo * info = 0) {return takeData(info);}

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "") override;
	virtual std::string getSerial() const override {return getName();}

	void setOrganized(bool enable);

#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
	// Overridden only on Apple: in network mode we run our own UDP receive loop
	// instead of pcl::HDLGrabber's, because on macOS/BSD closing the grabber's
	// socket while its read thread is blocked in receive_from returns EBADF, a
	// code that pcl::HDLGrabber::readPacketsFromSocket() rethrows uncaught (in
	// its own thread), aborting the process on shutdown. Owning the socket lets
	// us tear it down with the non-throwing receive_from overload. Like
	// pcl::HDLGrabber, we bind to the given LOCAL interface address (to scope
	// reception on a multi-homed host) and fall back to all interfaces (0.0.0.0)
	// when it is unspecified or not a local address. In PCAP mode we delegate to
	// the base grabber. On other platforms we inherit pcl::VLPGrabber directly.
	virtual void start() override;
	virtual void stop() override;
	virtual bool isRunning() const override;
#endif

private:
	void buildTimings(bool dualMode);
    virtual void toPointClouds (HDLDataPacket *dataPacket) override;
#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
    void readPackets();
#endif

protected:
    virtual SensorData captureData(SensorCaptureInfo * info = 0) override;

private:
    // timing offset lookup table
    std::vector< std::vector<float> > timingOffsets_;
    bool timingOffsetsDualMode_;
    double startSweepTime_;
    double startSweepTimeHost_;
    bool organized_;
    bool useHostTime_;
    bool stampLast_;
    SensorData lastScan_;
    std::vector<std::vector<PointXYZIT> > accumulatedScans_;
    USemaphore scanReady_;
    UMutex lastScanMutex_;

    // Own UDP receive path (network mode only).
    bool networkMode_;
    boost::asio::ip::address ipAddress_;
    std::uint16_t port_;
    bool receivedScan_;         // set once the first scan is received
#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
    boost::asio::io_context ioContext_;
    boost::asio::ip::udp::socket * socket_;
    std::thread * readThread_;
    std::atomic<bool> terminate_;
#endif
};

} /* namespace rtabmap */

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_ */
