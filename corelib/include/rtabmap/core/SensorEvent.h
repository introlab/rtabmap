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

#pragma once

#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/utilite/UEvent.h>
#include "rtabmap/core/SensorData.h"

namespace rtabmap
{

/**
 * @class SensorEvent
 * @brief Event class for sensor data communication in RTAB-Map's event system
 * 
 * SensorEvent is an event class that extends UEvent and is used to communicate
 * sensor data (images, depth, laser scans, etc.) through RTAB-Map's event-driven
 * architecture. It is typically posted by sensor capture threads and handled by
 * event handlers (e.g., RtabmapThread, DataRecorder, CalibrationDialog).
 * 
 * The class supports two event codes:
 * - **kCodeData**: Indicates that new sensor data is available
 * - **kCodeNoMoreImages**: Indicates that the sensor stream has ended (end of file, camera disconnected, etc.)
 * 
 * SensorEvent carries:
 * - **SensorData**: The actual sensor data (images, depth, scans, camera models, etc.)
 * - **SensorCaptureInfo**: Metadata about the capture (camera name, timing information, odometry pose, etc.)
 * 
 * @note This class is part of RTAB-Map's event system. Handlers should check the event code
 *       and class name before processing the event.
 * 
 * @see UEvent
 * @see SensorData
 * @see SensorCaptureInfo
 * @see UEventsHandler
 * @see UEventsManager
 */
class SensorEvent :
	public UEvent
{
public:
	/**
	 * @enum Code
	 * @brief Event codes for SensorEvent
	 */
	enum Code {
		kCodeData,          ///< Event code indicating new sensor data is available
		kCodeNoMoreImages   ///< Event code indicating the sensor stream has ended
	};

public:
	/**
	 * @brief Constructor from a single image
	 * 
	 * Creates a SensorEvent with kCodeData containing a single image.
	 * This is a convenience constructor for simple image-only sensor data.
	 * 
	 * @param image The image to include in the sensor data
	 * @param seq Sequence number (default: 0)
	 * @param stamp Timestamp in seconds (default: 0.0)
	 * @param cameraName Optional camera name identifier (default: empty)
	 */
	SensorEvent(const cv::Mat & image, int seq=0, double stamp = 0.0, const std::string & cameraName = std::string()) :
		UEvent(kCodeData),
		data_(image, seq, stamp)
	{
		sensorCaptureInfo_.cameraName = cameraName;
	}

	/**
	 * @brief Constructor for end-of-stream event
	 * 
	 * Creates a SensorEvent with kCodeNoMoreImages to signal that the sensor
	 * stream has ended (e.g., end of file, camera disconnected).
	 * 
	 * @note This constructor does not include any sensor data.
	 */
	SensorEvent() :
		UEvent(kCodeNoMoreImages)
	{
	}

	/**
	 * @brief Constructor from SensorData
	 * 
	 * Creates a SensorEvent with kCodeData containing the provided SensorData.
	 * 
	 * @param data The sensor data to include in the event
	 */
	SensorEvent(const SensorData & data) :
		UEvent(kCodeData),
		data_(data)
	{
	}

	/**
	 * @brief Constructor from SensorData with camera name
	 * 
	 * Creates a SensorEvent with kCodeData containing the provided SensorData
	 * and camera name.
	 * 
	 * @param data The sensor data to include in the event
	 * @param cameraName Camera name identifier
	 */
	SensorEvent(const SensorData & data, const std::string & cameraName) :
		UEvent(kCodeData),
		data_(data)
	{
		sensorCaptureInfo_.cameraName = cameraName;
	}
	
	/**
	 * @brief Constructor from SensorData with full capture info
	 * 
	 * Creates a SensorEvent with kCodeData containing the provided SensorData
	 * and complete sensor capture information (camera name, timing, odometry, etc.).
	 * 
	 * @param data The sensor data to include in the event
	 * @param sensorCaptureInfo Complete sensor capture information
	 */
	SensorEvent(const SensorData & data, const SensorCaptureInfo & sensorCaptureInfo) :
		UEvent(kCodeData),
		data_(data),
		sensorCaptureInfo_(sensorCaptureInfo)
	{
	}

	/**
	 * @brief Returns the sensor data contained in this event
	 * 
	 * Returns the SensorData object containing images, depth, laser scans,
	 * camera models, and other sensor information.
	 * 
	 * @return Const reference to the sensor data
	 */
	const SensorData & data() const {return data_;}
	
	/**
	 * @brief Returns the camera name
	 * 
	 * Returns the camera name identifier from the sensor capture info.
	 * 
	 * @return Const reference to the camera name string
	 */
	const std::string & cameraName() const {return sensorCaptureInfo_.cameraName;}
	
	/**
	 * @brief Returns the sensor capture information
	 * 
	 * Returns the complete sensor capture information including camera name,
	 * timing information, odometry pose, covariance, and other metadata.
	 * 
	 * @return Const reference to the sensor capture info
	 */
	const SensorCaptureInfo & info() const {return sensorCaptureInfo_;}

	/**
	 * @brief Virtual destructor
	 */
	virtual ~SensorEvent() {}
	
	/**
	 * @brief Returns the class name for event identification
	 * 
	 * Returns "SensorEvent" to allow event handlers to identify this event type.
	 * 
	 * @return Class name string "SensorEvent"
	 */
	virtual std::string getClassName() const {return std::string("SensorEvent");}

private:
	SensorData data_; ///< Sensor data (images, depth, scans, camera models, etc.)
	SensorCaptureInfo sensorCaptureInfo_; ///< Sensor capture metadata (camera name, timing, odometry, etc.)
};

/**
 * @deprecated Use SensorEvent instead
 * @brief Backward compatibility typedef
 * 
 * CameraEvent is deprecated. Use SensorEvent instead.
 */
RTABMAP_DEPRECATED typedef SensorEvent CameraEvent;

} // namespace rtabmap
