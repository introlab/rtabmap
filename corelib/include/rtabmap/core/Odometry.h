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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

namespace rtabmap {

class OdometryInfo;
class ParticleFilter;

/**
 * @class Odometry
 * @brief Abstract base class for visual, lidar and visual-inertial odometry backends.
 *
 * Odometry estimates the incremental motion between consecutive @ref SensorData frames.
 * Concrete implementations override @ref computeTransform(); the public @ref process()
 * pipeline handles IMU caching, optional motion guesses, filtering (Kalman or particle),
 * image decimation, deskewing and pose integration.
 *
 * Use @ref create() to instantiate a backend from @ref Parameters::kOdomStrategy().
 *
 * @see OdometryThread
 * @see OdometryInfo
 */
class RTABMAP_CORE_EXPORT Odometry
{
public:
	/** @brief Odometry backend selected by @ref Parameters::kOdomStrategy(). */
	enum Type {
		kTypeUndef = -1,    /**< Undefined / invalid type. */
		kTypeF2M = 0,       /**< Frame-to-map (default). */
		kTypeF2F = 1,       /**< Frame-to-frame. */
		kTypeFovis = 2,     /**< FOVIS stereo visual odometry. */
		kTypeViso2 = 3,     /**< libviso2. */
		kTypeDVO = 4,       /**< Dense visual odometry. */
		kTypeORBSLAM = 5,   /**< ORB-SLAM 2/3. */
		kTypeOkvis = 6,     /**< OKVIS. */
		kTypeLOAM = 7,      /**< LOAM lidar odometry. */
		kTypeMSCKF = 8,     /**< MSCKF visual-inertial. */
		kTypeVINSFusion = 9,/**< VINS-Fusion. */
		kTypeOpenVINS = 10, /**< OpenVINS. */
		kTypeFLOAM = 11,    /**< FLOAM lidar odometry. */
		kTypeOpen3D = 12,   /**< Open3D RGB-D odometry. */
		kTypeCuVSLAM = 13,  /**< cuVSLAM. */
		kTypeLIOSAM = 14    /**< LIO-SAM. */
	};

	/**
	 * @brief Creates an odometry instance from @ref Parameters::kOdomStrategy() in @p parameters.
	 * @param parameters RTAB-Map parameters (odometry strategy and related options).
	 * @return New odometry object (caller owns the pointer). Falls back to @ref kTypeF2M if the type is unknown.
	 */
	static Odometry * create(const ParametersMap & parameters = ParametersMap());
	/**
	 * @brief Creates an odometry instance of a given @p type.
	 * @param type In/out odometry type; updated to @ref kTypeF2M if @p type is unknown.
	 * @param parameters RTAB-Map parameters passed to the concrete backend.
	 */
	static Odometry * create(Type & type, const ParametersMap & parameters = ParametersMap());

	virtual ~Odometry();
	/**
	 * @brief Processes a sensor frame and updates the integrated pose.
	 * @param data Input sensor data (must have @c id() >= 0). May be modified in place (decompression, deskewing).
	 * @param info Optional output statistics and debug data.
	 * @return Updated integrated pose (@ref getPose()) after the frame is processed,
	 *         or a null transform if odometry is lost. The incremental transform is
	 *         available in @ref OdometryInfo::transform when @p info is provided.
	 */
	Transform process(SensorData & data, OdometryInfo * info = 0);
	/**
	 * @brief Processes a sensor frame with an external motion guess.
	 * @param data Input sensor data.
	 * @param guess Optional prior on the incremental transform (used by the backend when supported).
	 * @param info Optional output statistics and debug data.
	 */
	Transform process(SensorData & data, const Transform & guess, OdometryInfo * info = 0);
	/**
	 * @brief Resets internal state and sets the initial pose.
	 * @param initialPose Starting pose (must not be null). Z/roll/pitch may be cleared if @ref Parameters::kRegForce3DoF() is enabled.
	 */
	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	/** @return Concrete odometry backend type. */
	virtual Odometry::Type getType() = 0;
	/** @return True if the backend can process unrectified camera images. */
	virtual bool canProcessRawImages() const {return false;}
	/** @return True if the backend processes IMU asynchronously outside @ref process(). */
	virtual bool canProcessAsyncIMU() const {return false;}

	/** @return Current integrated odometry pose. */
	const Transform & getPose() const {return _pose;}
	/** @return True if @ref OdometryInfo debug/statistics fields are filled in @ref process(). */
	bool isInfoDataFilled() const {return _fillInfoData;}
	/** @deprecated Use @ref getVelocityGuess() instead. */
	RTABMAP_DEPRECATED const Transform & previousVelocityTransform() const;
	/** @return Last estimated velocity used for motion guessing (may be null). */
	const Transform & getVelocityGuess() const {return velocityGuess_;}
	/** @return Timestamp of the previously processed frame. */
	double previousStamp() const {return previousStamp_;}
	/** @return Number of frames processed since the last @ref reset(). */
	unsigned int framesProcessed() const {return framesProcessed_;}
	/** @return True if input images are already rectified (see @ref Parameters::kRtabmapImagesAlreadyRectified()). */
	bool imagesAlreadyRectified() const {return _imagesAlreadyRectified;}

protected:
	/** @return IMU orientations cached from recent frames (stamp → transform). */
	const std::map<double, Transform> & imus() const {return imus_;}

	/** @brief Constructs the base odometry state from RTAB-Map parameters. */
	Odometry(const rtabmap::ParametersMap & parameters);

private:
	/**
	 * @brief Computes the incremental transform for one frame (implemented by subclasses).
	 * @param data Sensor data for this frame (may already be decimated or deskewed).
	 * @param guess Motion prior from the base class or the caller.
	 * @param info Optional debug/statistics output.
	 * @return Incremental transform, or null if tracking failed.
	 */
	virtual Transform computeTransform(SensorData & data, const Transform & guess = Transform(), OdometryInfo * info = 0) = 0;

	void initKalmanFilter(const Transform & initialPose = Transform::getIdentity(), float vx=0.0f, float vy=0.0f, float vz=0.0f, float vroll=0.0f, float vpitch=0.0f, float vyaw=0.0f);
	void predictKalmanFilter(float dt, float * vx=0, float * vy=0, float * vz=0, float * vroll=0, float * vpitch=0, float * vyaw=0);
	void updateKalmanFilter(float & vx, float & vy, float & vz, float & vroll, float & vpitch, float & vyaw);

private:
	int _resetCountdown;
	bool _force3DoF;
	bool _holonomic;
	bool guessFromMotion_;
	float guessSmoothingDelay_;
	int _filteringStrategy;
	int _particleSize;
	float _particleNoiseT;
	float _particleLambdaT;
	float _particleNoiseR;
	float _particleLambdaR;
	bool _fillInfoData;
	float _kalmanProcessNoise;
	float _kalmanMeasurementNoise;
	unsigned int _imageDecimation;
	bool _alignWithGround;
	bool _publishRAMUsage;
	bool _imagesAlreadyRectified;
	bool _deskewing;
	Transform _pose;
	int _resetCurrentCount;
	double previousStamp_;
	std::list<std::pair<std::vector<float>, double> > previousVelocities_;
	Transform velocityGuess_;
	Transform imuLastTransform_;
	Transform previousGroundTruthPose_;
	float distanceTravelled_;
	unsigned int framesProcessed_;

	std::vector<ParticleFilter *> particleFilters_;
	cv::KalmanFilter kalmanFilter_;
	std::vector<StereoCameraModel> stereoModels_;
	std::vector<CameraModel> models_;
	std::map<double, Transform> imus_;
};

} /* namespace rtabmap */
#endif /* ODOMETRY_H_ */
