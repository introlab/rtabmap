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

#ifndef STEREOCAMERAMODEL_H_
#define STEREOCAMERAMODEL_H_

#include <rtabmap/core/CameraModel.h>

namespace rtabmap {

/**
 * @class StereoCameraModel
 * @brief A class representing a calibrated stereo camera system.
 *
 * This class encapsulates the calibration data and operations associated with a stereo camera setup,
 * including intrinsic and extrinsic parameters for both left and right cameras, stereo rectification, 
 * and methods for computing depth or disparity from stereo images.
 *
 * It relies internally on two `CameraModel` instances for the left and right cameras.
 *
 * Typical uses include:
 * - Stereo rectification
 * - Stereo disparity-to-depth conversion
 * - Saving and loading stereo camera calibration data
 * - Projecting or reprojecting points
 *
 * @see CameraModel
 */
class RTABMAP_CORE_EXPORT StereoCameraModel
{
public:
	/**
     * @brief Default constructor. Creates an empty stereo model with default suffixes ("left", "right").
     */
	StereoCameraModel() : leftSuffix_("left"), rightSuffix_("right") {}

	/**
	 * @brief Constructs a StereoCameraModel from detailed intrinsic and extrinsic parameters for both cameras.
	 * 
	 * Initializes the stereo camera model by specifying the calibration parameters for the left and right cameras,
	 * along with the stereo extrinsic parameters.
	 * 
	 * @param name Name identifier for the stereo camera.
	 * @param imageSize1 Image size (width, height) of the left camera.
	 * @param K1 Intrinsic camera matrix (3x3, CV_64FC1) for the left camera.
	 * @param D1 Distortion coefficients for the left camera.
	 * @param R1 Rectification matrix (3x3, CV_64FC1) for the left camera.
	 * @param P1 Projection matrix (3x4, CV_64FC1) for the left camera.
	 * @param imageSize2 Image size (width, height) of the right camera.
	 * @param K2 Intrinsic camera matrix (3x3, CV_64FC1) for the right camera.
	 * @param D2 Distortion coefficients for the right camera.
	 * @param R2 Rectification matrix (3x3, CV_64FC1) for the right camera.
	 * @param P2 Projection matrix (3x4, CV_64FC1) for the right camera.
	 * @param R Rotation matrix (3x3, CV_64FC1) representing the rotation from left to right camera coordinate system.
	 *          Can be empty if unknown.
	 * @param T Translation vector (3x1, CV_64FC1) representing the translation from left to right camera coordinate system.
	 *          Can be empty if unknown.
	 * @param E Essential matrix (3x3, CV_64FC1) encoding the stereo camera epipolar geometry.
	 *          Can be empty if unknown.
	 * @param F Fundamental matrix (3x3, CV_64FC1) encoding the stereo camera epipolar constraints.
	 *          Can be empty if unknown.
	 * @param localTransform The local transform associated with the stereo camera model.
	 * 
	 * @note All matrices must have correct sizes and types as specified.
	 *       The rectification and projection matrices (R1, P1, R2, P2) are used to define the stereo rectification parameters.
	 *       The rotation and translation (R, T) define the relative pose between the cameras.
	 */
	StereoCameraModel(
			const std::string & name,
			const cv::Size & imageSize1,
			const cv::Mat & K1, const cv::Mat & D1, const cv::Mat & R1, const cv::Mat & P1,
			const cv::Size & imageSize2,
			const cv::Mat & K2, const cv::Mat & D2, const cv::Mat & R2, const cv::Mat & P2,
			const cv::Mat & R, const cv::Mat & T, const cv::Mat & E, const cv::Mat & F,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));

	/**
	 * @brief Constructs a StereoCameraModel from two individual camera models and optional stereo extrinsic parameters.
	 *
	 * This constructor initializes the stereo camera model by assigning the provided left and right camera models.
	 * If the stereo extrinsics (`R`, `T`) are provided and valid, stereo rectification will be attempted—provided both
	 * cameras are valid for rectification and their image dimensions match.
	 *
	 * Each camera model will automatically have its name updated using the `name` parameter and default suffixes ("left", "right").
	 *
	 * @param name             The base name for the stereo camera model.
	 * @param leftCameraModel  The camera model representing the left camera.
	 * @param rightCameraModel The camera model representing the right camera.
	 * @param R                (Optional) Rotation matrix from left to right camera (3x3, CV_64FC1).
	 * @param T                (Optional) Translation vector from left to right camera (3x1, CV_64FC1).
	 * @param E                (Optional) Essential matrix between the two cameras (3x3, CV_64FC1).
	 * @param F                (Optional) Fundamental matrix between the two cameras (3x3, CV_64FC1).
	 *
	 * @throws UException if any of the provided matrices (`R`, `T`, `E`, `F`) are non-empty and not of the expected type/shape.
	 * @throws UException if `R` and `T` are provided but the camera models are not valid for rectification.
	 * 
	 * @note Stereo rectification is only attempted if both `R` and `T` are non-empty, the cameras are valid, and their image sizes match.
	 * @see updateStereoRectification()
	 */
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const cv::Mat & R = cv::Mat(),
			const cv::Mat & T = cv::Mat(),
			const cv::Mat & E = cv::Mat(),
			const cv::Mat & F = cv::Mat());
	
	/**
	 * @brief Constructs a StereoCameraModel from two camera models and an extrinsic Transform between them.
	 *
	 * This constructor sets up a stereo camera model using the given left and right camera models along with
	 * an optional 3D transform (`extrinsics`) representing the pose of the right camera relative to the left camera.
	 * 
	 * If a valid (non-null) transform is provided, the corresponding rotation and translation matrices are extracted
	 * and stored as the stereo extrinsic parameters. Stereo rectification will be attempted if both camera models
	 * are valid for rectification and their image sizes match.
	 *
	 * Each camera model will be renamed using the provided `name` and default suffixes ("left", "right").
	 *
	 * @param name         Base name for the stereo camera model.
	 * @param leftCameraModel  Camera model for the left camera.
	 * @param rightCameraModel Camera model for the right camera.
	 * @param extrinsics   (Optional) Transform from the left camera to the right camera. If null, no extrinsics are used.
	 *
	 * @throws UException if `extrinsics` is not null and either camera model is not valid for rectification.
	 * 
	 * @note Stereo rectification is performed only when `extrinsics` is valid and both camera models are rectifiable
	 *       with matching image dimensions.
	 * @see updateStereoRectification()
	 */
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const Transform & extrinsics);

	/**
     * @brief Minimal constructor using focal lengths and baseline only.
     */
	StereoCameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			const cv::Size & imageSize = cv::Size(0,0));
	/**
     * @brief Minimal constructor that also sets a name, required if we want to save it to a file.
     */
	StereoCameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			const cv::Size & imageSize = cv::Size(0,0));

	/**
     * @brief Destructor.
     */
	virtual ~StereoCameraModel() {}

	/**
     * @brief Returns true if both left and right models are valid for projection and the baseline is positive.
     */
	bool isValidForProjection() const {return left_.isValidForProjection() && right_.isValidForProjection() && baseline() > 0.0;}
	
	/**
     * @brief Returns true if both left and right models are valid for rectification.
     */
	bool isValidForRectification() const {return left_.isValidForRectification() && right_.isValidForRectification();}

	/**
     * @brief Initializes the rectification maps for both cameras.
     */
	void initRectificationMap() {left_.initRectificationMap(); right_.initRectificationMap();}

	/**
     * @brief Returns true if rectification maps are initialized.
     */
	bool isRectificationMapInitialized() const {return left_.isRectificationMapInitialized() && right_.isRectificationMapInitialized();}

	/**
     * @brief Sets the camera name and optional image suffixes for the left and right cameras.
     */
	void setName(const std::string & name, const std::string & leftSuffix = "left", const std::string & rightSuffix = "right");
	
	/**
     * @brief Gets the camera name.
     */
	const std::string & name() const {return name_;}

	/**
     * @brief Sets the image size for both left and right cameras.
     */
	void setImageSize(const cv::Size & size) {left_.setImageSize(size); right_.setImageSize(size);}

	/**
	 * @brief Loads stereo camera calibration data from disk.
	 *
	 * This method loads the intrinsic parameters for both the left and right cameras from files in the specified directory,
	 * using the provided camera name and internal suffixes. If `ignoreStereoTransform` is false, it also attempts to load
	 * the stereo extrinsic parameters (rotation, translation, essential, and fundamental matrices) from a YAML file.
	 *
	 * The stereo extrinsics are expected in the file:
	 * `directory/cameraName_pose.yaml`, following the ROS calibration format.
	 *
	 * @param directory The directory where the calibration files are located.
	 * @param cameraName The base name of the stereo camera (used to derive filenames).
	 * @param ignoreStereoTransform If true, skips loading stereo extrinsic parameters.
	 * @return true if loading is successful, false otherwise.
	 *
	 * @see save(), saveStereoTransform()
	 */
	bool load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform = true);

	/**
	 * @brief Saves stereo camera calibration data to disk.
	 *
	 * This method saves the intrinsic parameters of both left and right cameras to the specified directory.
	 * If `ignoreStereoTransform` is false, it also saves the stereo extrinsic parameters (rotation, translation,
	 * essential, and fundamental matrices) in a ROS-compatible YAML file named `cameraName_pose.yaml`.
	 *
	 * @param directory The directory where calibration files should be saved.
	 * @param ignoreStereoTransform If true, skips saving stereo extrinsic parameters.
	 * @return true if saving was successful, false otherwise.
	 *
	 * @see load(), saveStereoTransform()
	 */
	bool save(const std::string & directory, bool ignoreStereoTransform = true) const;

	/**
	 * @brief Saves stereo extrinsic parameters to a YAML file in ROS format.
	 *
	 * This method exports the stereo transform, including rotation, translation, essential, and fundamental matrices,
	 * into a YAML file named `cameraName_pose.yaml` located in the specified directory.
	 * The file format is compatible with ROS camera calibration tools.
	 *
	 * @param directory The target directory for saving the calibration file.
	 * @return true if saving was successful, false if required matrices are missing or invalid.
	 *
	 * @warning If extrinsics (`R_`, `T_`, `E_`, `F_`) are empty or invalid, nothing will be saved and a warning is printed.
	 *
	 * @see load(), save()
	 */
	bool saveStereoTransform(const std::string & directory) const;

	/**
	 * @brief Serializes the stereo camera model into a byte vector.
	 *
	 * This method serializes the left and right camera models along with the stereo extrinsic parameters
	 * (rotation matrix R_, translation vector T_, essential matrix E_, and fundamental matrix F_) into a
	 * contiguous byte array. The serialization format starts with a fixed-size integer header containing
	 * version info, stereo type, matrix sizes, and serialized data sizes, followed by the actual matrices and
	 * serialized camera data.
	 *
	 * The serialized data can later be restored using the corresponding `deserialize()` method.
	 *
	 * @return A vector of unsigned char containing the serialized stereo camera data.
	 */
	std::vector<unsigned char> serialize() const;

	/**
	 * @brief Deserializes stereo camera model data from a byte vector.
	 *
	 * This method wraps the pointer-based `deserialize()` and attempts to restore the stereo camera
	 * model from the given serialized byte vector.
	 *
	 * @param data The vector of bytes containing previously serialized stereo camera model data.
	 * @return The number of bytes read from the data if successful, 0 otherwise.
	 *
	 * @see deserialize(const unsigned char*, unsigned int)
	 */
	unsigned int deserialize(const std::vector<unsigned char>& data);

	/**
	 * @brief Deserializes stereo camera model data from a raw byte array.
	 *
	 * This method reconstructs the stereo camera model from the provided serialized data buffer.
	 * It expects the data format to match the one produced by `serialize()`, including a header with
	 * version info, matrix sizes, and data sizes, followed by the serialized extrinsic matrices and
	 * serialized left and right camera data.
	 *
	 * The method performs various sanity checks on data sizes and matrix dimensions and will fail if
	 * the data format or sizes are inconsistent.
	 *
	 * @param data Pointer to the raw serialized data buffer.
	 * @param dataSize Size in bytes of the data buffer.
	 * @return The number of bytes consumed during deserialization if successful, or 0 on failure.
	 *
	 * @warning The stereo camera model is reset to a default empty state before deserialization.
	 * @warning If the serialized data type is not stereo (type != 1), deserialization will fail.
	 *
	 * @see serialize()
	 */
	unsigned int deserialize(const unsigned char * data, unsigned int dataSize);

	/**
     * @brief Returns the stereo baseline in meters.
     */
	double baseline() const {return right_.fx()!=0.0 && left_.fx() != 0.0 ? left_.Tx() / left_.fx() - right_.Tx()/right_.fx():0.0;}

	/**
	 * @brief Computes the depth (Z coordinate) from a given disparity value.
	 *
	 * Uses the stereo camera model parameters to convert disparity to depth using the formula:
	 * \f[
	 *     \text{depth} = \frac{\text{baseline} \times f_x}{\text{disparity} + (c_{x_{right}} - c_{x_{left}})}
	 * \f]
	 * where \( f_x \) is the focal length of the left camera and \( c_x \) are principal points.
	 *
	 * @param disparity The disparity value (difference in pixel coordinates between left and right images).
	 * @return The computed depth in the same unit as the baseline (typically meters).
	 *         Returns 0 if disparity is zero or if the model is not valid for projection.
	 *
	 * @note This function requires the stereo camera to be valid for projection (i.e., calibrated and rectified).
	 */
	float computeDepth(float disparity) const;

	/**
	 * @brief Computes the disparity value from a given depth.
	 *
	 * Converts depth back to disparity using the inverse formula:
	 * \f[
	 *     \text{disparity} = \frac{\text{baseline} \times f_x}{\text{depth}} - (c_{x_{right}} - c_{x_{left}})
	 * \f]
	 *
	 * @param depth Depth value in the same unit as the baseline (typically meters).
	 * @return The computed disparity in pixels.
	 *         Returns 0 if depth is zero or if the model is not valid for projection.
	 *
	 * @note This function requires the stereo camera to be valid for projection (i.e., calibrated and rectified).
	 */
	float computeDisparity(float depth) const; // m

	/**
	 * @brief Computes the disparity value from a depth given in unsigned short format (millimeters).
	 *
	 * Converts depth expressed as an unsigned short (in millimeters) to disparity.
	 * The depth is first converted to meters before computing disparity using the formula:
	 * \f[
	 *     \text{disparity} = \frac{\text{baseline} \times f_x}{\text{depth (meters)}} - (c_{x_{right}} - c_{x_{left}})
	 * \f]
	 *
	 * @param depth Depth value in millimeters as an unsigned short.
	 * @return The computed disparity in pixels.
	 *         Returns 0 if depth is zero or if the model is not valid for projection.
	 *
	 * @note This function requires the stereo camera to be valid for projection (i.e., calibrated and rectified).
	 */
	float computeDisparity(unsigned short depth) const; // mm

	const cv::Mat & R() const {return R_;} ///< Stereo extrinsic rotation matrix.
	const cv::Mat & T() const {return T_;} ///< Stereo extrinsic translation vector.
	const cv::Mat & E() const {return E_;} ///< Essential matrix.
	const cv::Mat & F() const {return F_;} ///< Fundamental matrix

	/**
     * @brief Scales both cameras' calibration by a factor.
     */
	void scale(double scale);
	/**
     * @brief Applies region-of-interest (ROI) cropping to both cameras.
     */
	void roi(const cv::Rect & roi);

	/**
     * @brief Sets the local transform from left camera to robot base.
     */
	void setLocalTransform(const Transform & transform) {left_.setLocalTransform(transform);}

	/**
     * @brief Gets the local transform from left camera to robot base.
     */
	const Transform & localTransform() const {return left_.localTransform();}

	/**
     * @brief Returns the stereo transform (right camera relative to left).
     */
	Transform stereoTransform() const;

	/**
     * @brief Returns the left camera model.
     */
	const CameraModel & left() const {return left_;}
	/**
     * @brief Returns the right camera model.
     */
	const CameraModel & right() const {return right_;}

	/**
     * @brief Gets the suffix used for the left camera calibration file.
     */
	const std::string & getLeftSuffix() const {return leftSuffix_;}
	/**
     * @brief Gets the suffix used for the right camera calibration file.
     */
	const std::string & getRightSuffix() const {return rightSuffix_;}

private:
	void updateStereoRectification();

private:
	std::string leftSuffix_;   ///< Suffix for the left calibration file.
	std::string rightSuffix_;  ///< Suffix for the right calibration file.
	CameraModel left_;         ///< Left camera model.
	CameraModel right_;        ///< Right camera model.
	std::string name_;         ///< Model name or ID.

	cv::Mat R_; ///< Rotation matrix between cameras.
	cv::Mat T_; ///< Translation vector between cameras.
	cv::Mat E_; ///< Essential matrix.
	cv::Mat F_; ///< Fundamental matrix.
};

/**
 * @brief Outputs a textual representation of the StereoCameraModel to the given output stream.
 *
 * This operator prints the details of the stereo camera model including:
 * - The left camera parameters.
 * - The right camera parameters.
 * - The stereo extrinsic matrices: Rotation (R), Translation (T), Essential (E), and Fundamental (F).
 * - The baseline distance between the two cameras.
 *
 * @param os The output stream to write to.
 * @param model The StereoCameraModel instance to output.
 * @return A reference to the output stream after writing the model information.
 */
RTABMAP_CORE_EXPORT std::ostream& operator<<(std::ostream& os, const StereoCameraModel& model);

} // rtabmap

#endif /* STEREOCAMERAMODEL_H_ */
