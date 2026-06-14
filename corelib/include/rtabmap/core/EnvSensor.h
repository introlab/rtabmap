/*
Copyright (c) 2010-2018, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_ENVSENSOR_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_ENVSENSOR_H_

#include <map>

namespace rtabmap {

/**
 * @class EnvSensor
 * @brief Single environmental measurement (type, value, timestamp).
 *
 * Built-in @ref Type values cover common phone/robot sensors; @ref kCustomSensor1
 * through @ref kCustomSensor9 are reserved for application-specific channels.
 * Units depend on the type (see @ref Type).
 *
 * Samples are stored per node in @ref SensorData via @ref EnvSensors (one entry
 * per type). Persisted in the database with node records (@ref DBDriver).
 *
 * @see SensorData::setEnvSensors()
 * @see SensorData::addEnvSensor()
 * @see SensorData::envSensors()
 */
class EnvSensor
{
public:
	/**
	 * @brief Environmental sensor channel identifier.
	 *
	 * Built-in types use fixed units; custom types (≥ @ref kCustomSensor1) have
	 * application-defined meaning and units.
	 */
	enum Type {
		kUndefined = 0,              ///< Uninitialized / unknown channel.
		kWifiSignalStrength,         ///< Wi‑Fi signal strength (dBm).
		kAmbientTemperature,         ///< Ambient temperature (°C).
		kAmbientAirPressure,         ///< Ambient air pressure (hPa).
		kAmbientLight,               ///< Ambient illuminance (lx).
		kAmbientRelativeHumidity,    ///< Relative humidity (%).

		kCustomSensor1 = 100,        ///< User-defined sensor slot 1.
		kCustomSensor2,
		kCustomSensor3,
		kCustomSensor4,
		kCustomSensor5,
		kCustomSensor6,
		kCustomSensor7,
		kCustomSensor8,
		kCustomSensor9
	};

public:
	/** @brief Default constructor: @ref kUndefined type, zero value and stamp. */
	EnvSensor() :
		type_(kUndefined),
		value_(0.0),
		stamp_(0.0)
	{}

	/**
	 * @brief Constructs a reading with the given type, value, and optional stamp.
	 * @param type Sensor channel (@ref Type).
	 * @param value Measurement in the units for @p type.
	 * @param stamp Timestamp in seconds (0 if unknown).
	 */
	EnvSensor(const Type & type, const double & value,const double & stamp = 0) :
		type_(type),
		value_(value),
		stamp_(stamp)
	{}

	virtual ~EnvSensor() {}

	/** @return Sensor channel. */
	const Type & type() const {return type_;}
	/** @return Measurement value (units depend on @ref type()). */
	const double & value() const {return value_;}
	/** @return Timestamp in seconds. */
	const double & stamp() const {return stamp_;}

private:
	Type type_;
	double value_;
	double stamp_;
};

/** @brief Map of environmental readings keyed by @ref EnvSensor::Type (at most one per type). */
typedef std::map<EnvSensor::Type, EnvSensor> EnvSensors;

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_ENVSENSOR_H_ */
