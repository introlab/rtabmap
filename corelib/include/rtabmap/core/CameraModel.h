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

#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <opencv2/opencv.hpp>

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include "rtabmap/core/Transform.h"

namespace rtabmap {

/**
 * @class CameraModel
 * @brief Represents a pinhole camera model containing intrinsic and extrinsic 
 *        parameters, used for projection, rectification, and transformation.
 *
 * This class encapsulates camera calibration data, including intrinsic parameters (fx, fy, cx, cy),
 * distortion coefficients, rectification and projection matrices. It provides utility functions
 * for image rectification, projection from 2D to 3D, and vice versa.
 * 
 * This class supports the 4 to 14 parameters Radial Tangential distortion model (also called Plumb Bob or 
 * Brown-Conrady model) and 4 parameters Fish Eye model (also known as Equidistant model).
 * 
 * @see OpenCV's calib3d module for all supported camera models.
 */
class RTABMAP_CORE_EXPORT CameraModel
{
public:
	/**
	 * @brief Returns the default optical rotation to convert image coordinates to robot coordinates.
	 * 
	 * Image frame: x -> right, y -> down, z -> forward  
	 * Robot frame: x -> forward, y -> left, z -> up
	 * 
	 * @return Transform rotation matrix.
	 */
	static Transform opticalRotation() {return Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);}

public:

	/// Default constructor.
	CameraModel();
	
	/**
	 * @brief Constructor using full camera parameters.
	 * 
	 * @param name           Camera name or ID.
	 * @param imageSize      Size of the image (width x height).
	 * @param K              Intrinsic matrix (3x3).
	 * @param D              Distortion coefficients, 1xN matrix where N is between 4 and 14 parameters: 
	 *                       k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,tx,ty]]]]).
	 *                       To set Fish Eye / Equidistant model, it is implicitly used if we 
	 *                       provide 6 values like this: [k1,k2,0,0,k3,k4], where you only need to 
	 *                       fill "k" parameters.
	 * @param R              Rectification matrix (3x3).
	 * @param P              Projection matrix (3x4).
	 * @param localTransform Local transform to apply to the camera frame.
	 */
	CameraModel(
			const std::string & name,
			const cv::Size & imageSize,
			const cv::Mat & K,
			const cv::Mat & D,
			const cv::Mat & R,
			const cv::Mat & P,
			const Transform & localTransform = opticalRotation());

	/**
	 * @brief Minimal constructor using intrinsic parameters. This assumes the images are already rectified.
	 *
	 * @param fx             Focal length x.
	 * @param fy             Focal length y.
	 * @param cx             Principal point x.
	 * @param cy             Principal point y.
	 * @param localTransform Local transform to apply to the camera frame.
	 * @param Tx             Baseline * fx (optional). Mainly used in case of stereo pair.
	 * @param imageSize      Image size (optional).
	 */
	CameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = opticalRotation(),
			double Tx = 0.0f,
			const cv::Size & imageSize = cv::Size(0,0));
	
	/**
	 * @brief Minimal constructor with name for saving.
	 */
	CameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = opticalRotation(),
			double Tx = 0.0f,
			const cv::Size & imageSize = cv::Size(0,0));

	/// Destructor.
	virtual ~CameraModel() {}

	/**
	 * @brief Initializes the rectification maps used to undistort and rectify images.
	 *
	 * This function prepares the `mapX_` and `mapY_` lookup tables used for image rectification.
	 * It supports both standard radial-tangential distortion and fisheye/equidistant distortion models.
	 *
	 * - If the distortion model is **fisheye** (indicated by `D_.cols == 6`), it uses
	 *   `cv::fisheye::initUndistortRectifyMap()` to create the rectification maps. This requires OpenCV ≥ 2.4.10.
	 * - Otherwise, it uses the standard `cv::initUndistortRectifyMap()` for plumb bob or rational polynomial models.
	 *
	 * @pre The camera model must be valid for rectification:
	 *   - `imageSize_` must be non-zero.
	 *   - `D_` must be a 1-row matrix with an accepted number of columns (4, 5, 6, 8, 12, or 14).
	 *   - `R_` must be a 3x3 rectification matrix.
	 *   - `P_` must be a 3x4 projection matrix.
	 *
	 * @return `true` if the rectification maps were successfully initialized (`mapX_` and `mapY_` are not empty),
	 *         `false` otherwise.
	 *
	 * @see isRectificationMapInitialized(), rectifyImage(), rectifyDepth()
	 *
	 * @warning Requires OpenCV 2.4.10 or newer for fisheye support. If the version is older, fisheye rectification will not work.
	 */
	bool initRectificationMap();

	/**
	 * @brief Checks if the rectification map has been initialized.
	 * @return True if both mapX_ and mapY_ are initialized.
	 */
	bool isRectificationMapInitialized() const {return !mapX_.empty() && !mapY_.empty();}

	/**
	 * @brief Checks if the model is valid for 2D->3D projection.
	 */
	bool isValidForProjection() const {return fx()>0.0 && fy()>0.0 && cx()>0.0 && cy()>0.0;}
	/**
	 * @brief Checks if the model is valid for 3D->2D reprojection.
	 */
	bool isValidForReprojection() const {return fx()>0.0 && fy()>0.0 && cx()>0.0 && cy()>0.0 && imageWidth()>0 && imageHeight()>0;}
	/**
	 * @brief Checks if the model has sufficient data for image rectification.
	 */
	bool isValidForRectification() const
	{
		return imageSize_.width>0 &&
			   imageSize_.height>0 &&
			   !K_.empty() &&
			   !D_.empty() &&
			   !R_.empty() &&
			   !P_.empty();
	}

	/// Sets the camera name, used to set a camera name when saving to a file.
	void setName(const std::string & name) {name_=name;}
	/// Returns the camera name.
	const std::string & name() const {return name_;}

	/// Returns focal length in x.
	double fx() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(0,0):P_.at<double>(0,0);}
	/// Returns focal length in y.
	double fy() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(1,1):P_.at<double>(1,1);}
	/// Returns principal point x.
	double cx() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(0,2):P_.at<double>(0,2);}
	/// Returns principal point y.
	double cy() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(1,2):P_.at<double>(1,2);}
	/// Returns the x translation (usually fx * baseline in case of stereo, otherwise would be 0).
	double Tx() const {return P_.empty()?0.0:P_.at<double>(0,3);}

	/// Returns the raw intrinsic matrix (before rectification).
	cv::Mat K_raw() const {return K_;}
	/// Returns the raw distortion coefficients (before rectification).
	cv::Mat D_raw() const {return D_;} 
	/// Returns the rectified camera intrinsic matrix if the projection matrix P exists, otherwise returns the raw intrinsic matrix.
	cv::Mat K() const {return !P_.empty()?P_.colRange(0,3):K_;}
	/// Returns the rectified distortion coefficients (1x5 filled with zeros) if the projection matrix P exists, otherwise returns the raw distortion coefficients.
	cv::Mat D() const {return P_.empty()&&!D_.empty()?D_:cv::Mat::zeros(1,5,CV_64FC1);} // if P exists, return rectified version
	/// Returns the rectification matrix.
	cv::Mat R() const {return R_;}
	/// Returns the projection matrix.
	cv::Mat P() const {return P_;}

	/// Sets the local transform of the camera (base frame to optical frame).
	void setLocalTransform(const Transform & transform) {localTransform_ = transform;}
	/// Returns the local transform (base frame to optical frame).
	const Transform & localTransform() const {return localTransform_;}

	/**
	 * @brief Sets the image size of the camera model and updates the principal point if undefined.
	 *
	 * This function updates the internal image size (`imageSize_`) with the provided size.
	 * If the intrinsic matrices (`K_` or `P_`) are present and the principal point coordinates
	 * (`cx`, `cy`) are zero, they are set to the image center (`width/2 - 0.5`, `height/2 - 0.5`).
	 *
	 * This ensures the camera model remains valid and useful even when the calibration file
	 * has no principal point set or the image size is updated manually.
	 *
	 * @param size The new image size. It must be either both dimensions zero (clearing) or both positive.
	 *
	 * @pre `size.width > 0 && size.height > 0` or `size.width == 0 && size.height == 0`
	 * @post Updates the `imageSize_`, and adjusts `cx` and `cy` in `K_` and `P_` if they were initially zero.
	 *
	 * @warning If `K_` or `P_` are not initialized (`empty()`), no updates will be applied to them.
	 *
	 * @see imageSize(), imageWidth(), imageHeight()
	 */
	void setImageSize(const cv::Size & size);
	const cv::Size & imageSize() const {return imageSize_;}
	int imageWidth() const {return imageSize_.width;}
	int imageHeight() const {return imageSize_.height;}

	/**
	 * @brief Returns the horizontal field of view (FoV) in radians.
	 *
	 * The FoV is computed using the pinhole camera model as:
	 * \f[
	 * \text{FoV}_x = 2 \cdot \tan^{-1}\left(\frac{\text{image width}}{2 \cdot f_x}\right)
	 * \f]
	 *
	 * @return Horizontal field of view in radians. Returns 0.0 if image width or focal length is invalid.
	 */
	double fovX() const;

	/**
	 * @brief Returns the vertical field of view (FoV) in radians.
	 *
	 * The FoV is computed using the pinhole camera model as:
	 * \f[
	 * \text{FoV}_y = 2 \cdot \tan^{-1}\left(\frac{\text{image height}}{2 \cdot f_y}\right)
	 * \f]
	 *
	 * @return Vertical field of view in radians. Returns 0.0 if image height or focal length is invalid.
	 */
	double fovY() const;

	/**
	 * @brief Returns the horizontal field of view in degrees.
	 *
	 * Converts the result of `fovX()` from radians to degrees.
	 *
	 * @return Horizontal field of view in degrees. Returns 0.0 if the result is invalid.
	 */
	double horizontalFOV() const;
	
	/**
	 * @brief Returns the vertical field of view in degrees.
	 *
	 * Converts the result of `fovY()` from radians to degrees.
	 *
	 * @return Vertical field of view in degrees. Returns 0.0 if the result is invalid.
	 */
	double verticalFOV() const;

	/// Checks if the distortion model is fisheye (6 coefficients: k1,k2,0,0,k3,k4).
	bool isFisheye() const {return D_.cols == 6;}

	/**
	 * @brief Loads the camera model parameters from a YAML calibration file.
	 *
	 * This method attempts to read camera intrinsic/extrinsic parameters and image size from a YAML file,
	 * typically in the ROS calibration format. If the distortion model is "fisheye" or "equidistant", we expect
	 * 4 coefficients, which are converted to a 6-coefficient format for internal representation.
	 *
	 * Fields loaded (if present):  
	 * - `camera_name`  
	 * - `image_width` (pixels)
	 * - `image_height` (pixels)
	 * - `camera_matrix` (K, 3x3 double matrix)  
	 * - `distortion_coefficients` (D, 1xN double matrix)  
	 * - `distortion_model`  (string: name of the model)
	 * - `rectification_matrix` (R, 3x3 double matrix)  
	 * - `projection_matrix` (P, 3x4 double matrix)  
	 * - `local_transform` (camera pose w.r.t robot frame)
	 *
	 * On success, the internal matrices and settings of the camera model are updated. If the model
	 * is valid for rectification, the rectification map is initialized.
	 *
	 * @param filePath Absolute or relative path to the YAML file.
	 * @return True if the file was successfully loaded and parsed, false otherwise.
	 *
	 * @warning Logs warnings if any fields are missing. If file does not exist or parsing fails, returns false.
	 */
	bool load(const std::string & filePath);

	/**
	 * @brief Loads the camera model by constructing a file path from a directory and camera name.
	 *
	 * This is a convenience wrapper around `load(filePath)` that constructs the file path as:
	 * `directory + "/" + cameraName + ".yaml"`.
	 *
	 * @param directory Path to the folder containing the camera YAML file.
	 * @param cameraName Base name of the camera file (without extension).
	 * @return True if loading from the constructed path succeeds, false otherwise.
	 */
	bool load(const std::string & directory, const std::string & cameraName);

	/**
	 * @brief Saves the camera model parameters to a YAML calibration file in ROS format.
	 *
	 * The file will include the following fields if they are not empty:
	 * - `camera_name`
	 * - `image_width`
	 * - `image_height`
	 * - `camera_matrix` (K)
	 * - `distortion_coefficients` (D)
	 * - `distortion_model` (auto-detected based on number of distortion coefficients)
	 * - `rectification_matrix` (R)
	 * - `projection_matrix` (P)
	 * - `local_transform` (camera pose w.r.t robot frame)
	 *
	 * If the distortion matrix contains 6 coefficients (used for fisheye), it is converted
	 * to a standard 4-coefficient format for ROS compatibility.
	 *
	 * @param directory Path to the folder where the YAML file will be saved.
	 * @return True if saving was successful, false otherwise.
	 *
	 * @note If `name_` is empty, "camera.yaml" is used as the default filename.
	 * @warning Returns false and logs an error if none of the matrices are set.
	 */
	bool save(const std::string & directory) const;

	/**
	 * @brief Serializes the camera model to a binary format.
	 *
	 * The serialization includes the camera intrinsics (`K_`, `D_`), rectification matrix (`R_`), 
	 * projection matrix (`P_`), image size, and the local transform. The format is compact and suitable 
	 * for file storage or transmission over a network.
	 *
	 * Data layout:
	 * - Header (11 integers):
	 *   - [0-2]  RTAB-Map version (major, minor, patch)
	 *   - [3]    Camera type (0 = mono, 1=stereo)
	 *   - [4-5]  Image width, height
	 *   - [6-9]  Element counts for K, D, R, P matrices
	 *   - [10]   Size of localTransform (0 if null)
	 * - Data section (in order): raw memory blocks for K, D, R, P (`double` values), followed by `float` values for localTransform
	 *
	 * @return A byte vector containing the serialized data. The format is compatible with `deserialize()`.
	 *
	 * @note This is a custom binary format, not meant to be human-readable.
	 * @see deserialize(), StereoCameraModel
	 */
	std::vector<unsigned char> serialize() const;
	/**
	 * @brief Deserializes a camera model from a byte vector.
	 *
	 * This is a convenience wrapper around `deserialize(const unsigned char*, unsigned int)` 
	 * that takes a `std::vector<unsigned char>` instead of a raw buffer.
	 *
	 * @param data Byte vector containing data serialized by `serialize()`.
	 * @return The number of bytes successfully read and parsed. Returns 0 on failure.
	 *
	 * @see serialize(), deserialize(const unsigned char*, unsigned int)
	 */
	unsigned int deserialize(const std::vector<unsigned char>& data);

	/**
	 * @brief Deserializes a camera model from a raw byte buffer.
	 *
	 * Reads the camera intrinsics, distortion, rectification, projection matrices, image size, 
	 * and local transform from a serialized binary format previously created with `serialize()`.
	 *
	 * @param data Pointer to the binary data buffer.
	 * @param dataSize Size of the data buffer in bytes.
	 * @return The number of bytes successfully read. Returns 0 on error or if the format is invalid.
	 *
	 * @warning If the buffer format does not match the expected layout or version, an error is logged 
	 *          and the camera model remains in a default-initialized state.
	 *
	 * @note Assumes little-endian architecture and strict size/type matching. The serialized format 
	 *       must be created by `CameraModel::serialize()`. Non-mono camera types are not supported. 
	 *       See `StereoCameraModel` to serialize/deserialize stereo models.
	 *
	 * @see serialize()
	 */
	unsigned int deserialize(const unsigned char * data, unsigned int dataSize);

	/**
	 * @brief Returns a new camera model with all intrinsic parameters scaled by a given factor.
	 *
	 * This method scales the camera's intrinsic matrix (`K_`) and projection matrix (`P_`), as well as 
	 * the image size, by the given `scale` factor. The distortion and rectification matrices are left unchanged.
	 *
	 * Only valid camera models (i.e., those for which `isValidForProjection()` returns true) are scaled.
	 * If the model is invalid, a warning is issued and the original model is returned unchanged.
	 *
	 * @param scale Scaling factor (> 0). For example, use 0.5 to downscale or 2.0 to upscale.
	 * @return A scaled copy of the camera model with updated intrinsics and image size.
	 *
	 * @warning If the camera model is not valid for projection, the scale operation is ignored.
	 */
	CameraModel scaled(double scale) const;

	/**
	 * @brief Returns a new camera model adjusted for a given region of interest (ROI).
	 *
	 * This method shifts the principal point (`cx`, `cy`) in the intrinsic matrix (`K_`) and projection matrix (`P_`)
	 * by subtracting the ROI’s top-left `(x, y)` offset. The image size is also set to the ROI size.
	 *
	 * Only valid camera models (i.e., those for which `isValidForProjection()` returns true) can be adjusted.
	 * If the model is invalid, a warning is issued and the original model is returned unchanged.
	 *
	 * @param roi Region of interest defined as a rectangle (typically a subwindow of the full image).
	 * @return A new camera model adapted to the ROI with adjusted intrinsics and image size.
	 *
	 * @warning If the camera model is not valid for projection, the ROI operation is ignored.
	 */
	CameraModel roi(const cv::Rect & roi) const;

	/**
	 * @brief Rectifies a raw image using the precomputed rectification maps.
	 *
	 * This function applies geometric correction (rectification) to an image using the camera model's
	 * `mapX_` and `mapY_` rectification maps. It is typically used to correct lens distortion in images
	 * based on the calibration parameters.
	 *
	 * @param raw Input raw image (e.g., from camera). Must be a valid `cv::Mat`.
	 * @param interpolation Interpolation method to use. Typically `cv::INTER_LINEAR` or `cv::INTER_NEAREST`.
	 *
	 * @return Rectified image. If the rectification maps are not initialized, the function logs an error
	 *         and returns a clone of the original image.
	 *
	 * @pre `mapX_` and `mapY_` must be initialized using `initRectificationMap()`.
	 *
	 * @note Works for color and grayscale images of any valid type.
	 *
	 * @see initRectificationMap(), rectifyDepth()
	 */
	cv::Mat rectifyImage(const cv::Mat & raw, int interpolation = cv::INTER_LINEAR) const;

	/**
	 * @brief Rectifies a raw depth image using the precomputed rectification maps.
	 *
	 * This function applies geometric correction (rectification) to a 16-bit unsigned depth image.
	 * It performs a pixel-by-pixel bilinear interpolation, only if all neighboring pixels have valid
	 * (non-zero) depth values, and the variation among them is within 1% of their average.
	 *
	 * The method is optimized to avoid introducing noise in regions of high depth variance.
	 *
	 * @param raw Input raw depth image (`CV_16UC1`). Must contain 16-bit unsigned depth values.
	 *
	 * @return Rectified depth image. If the rectification maps are not initialized or the input
	 *         image is not of type `CV_16UC1`, the function logs an error and returns a clone of the input.
	 *
	 * @pre Input image must be of type `CV_16UC1`. `mapX_` and `mapY_` must be initialized.
	 *
	 * @note Inspired by the Kinect2 CPU depth registration implementation from:
	 *       https://github.com/code-iai/iai_kinect2
	 *
	 * @warning Invalid or noisy regions are skipped in interpolation to maintain depth consistency.
	 *
	 * @see initRectificationMap(), rectifyImage()
	 */
	cv::Mat rectifyDepth(const cv::Mat & raw) const;

	/**
	 * @brief Projects a 2D pixel and depth value into a 3D point in the camera coordinate frame (/camera_link).
	 *
	 * This function uses the camera's intrinsic parameters to compute the 3D point corresponding to the given
	 * 2D image coordinates and depth value.
	 *
	 * @param u Horizontal image coordinate (in pixels).
	 * @param v Vertical image coordinate (in pixels).
	 * @param depth Depth value at (u, v) in meters.
	 * @param[out] x Output X coordinate in 3D space.
	 * @param[out] y Output Y coordinate in 3D space.
	 * @param[out] z Output Z coordinate in 3D space (equals `depth`).
	 *
	 * @note If `depth <= 0`, the output (x, y, z) will be set to `NaN`.
	 *
	 * @see reproject()
	 */
	void project(float u, float v, float depth, float & x, float & y, float & z) const;
	
	/**
	 * @brief Reprojects a 3D point in the camera frame (/camera_link) into 2D image coordinates (floating-point).
	 *
	 * This function computes the image plane coordinates for a given 3D point using the camera's
	 * intrinsic parameters.
	 *
	 * @param x X coordinate in camera space.
	 * @param y Y coordinate in camera space.
	 * @param z Z coordinate in camera space (must be non-zero).
	 * @param[out] u Output horizontal image coordinate (float).
	 * @param[out] v Output vertical image coordinate (float).
	 *
	 * @pre `z != 0`
	 *
	 * @see project(), reproject(int&, int&)
	 */
	void reproject(float x, float y, float z, float & u, float & v) const;

	/**
	 * @brief Reprojects a 3D point in the camera frame (/camera_link) into 2D image coordinates (rounded to int).
	 *
	 * This version of `reproject()` returns integer pixel indices, computed from the 3D position.
	 *
	 * @param x X coordinate in camera space.
	 * @param y Y coordinate in camera space.
	 * @param z Z coordinate in camera space (must be non-zero).
	 * @param[out] u Output horizontal image coordinate (integer pixel).
	 * @param[out] v Output vertical image coordinate (integer pixel).
	 *
	 * @pre `z != 0`
	 *
	 * @see project(), reproject(float&, float&)
	 */
	void reproject(float x, float y, float z, int & u, int & v) const;

	/**
	 * @brief Checks if a given pixel coordinate lies within the image bounds.
	 *
	 * @param u Horizontal image coordinate (in pixels).
	 * @param v Vertical image coordinate (in pixels).
	 * @return `true` if the pixel is within the image dimensions, `false` otherwise.
	 *
	 * @note Inclusive lower bound, exclusive upper bound: `[0, width)`, `[0, height)`
	 */
	bool inFrame(int u, int v) const;

private:
	std::string name_;         ///< Camera name.
	cv::Size imageSize_;       ///< Image size.
	cv::Mat K_;                ///< Intrinsic matrix.
	cv::Mat D_;                ///< Distortion coefficients.
	cv::Mat R_;                ///< Rectification matrix.
	cv::Mat P_;                ///< Projection matrix.
	cv::Mat mapX_;             ///< Rectification map X.
	cv::Mat mapY_;             ///< Rectification map Y.
	Transform localTransform_; ///< Transform from camera to base link.
};

/**
 * @brief Stream operator for printing a camera model to an output stream.
 *
 * This function outputs the name, image size, and camera matrices (K, D, R, P)
 * along with the local transformation.
 *
 * Example output:
 * ```
 * Name: camera1
 * Size: 640x480
 * K= [fx,  0, cx;
 *      0, fy, cy;
 *      0,  0,  1]
 * D= [...]
 * R= [...]
 * P= [...]
 * LocalTransform= [...]
 * ```
 *
 * @param os Output stream.
 * @param model Camera model to print.
 * @return The modified output stream.
 */
RTABMAP_CORE_EXPORT std::ostream& operator<<(std::ostream& os, const CameraModel& model);

} /* namespace rtabmap */
#endif /* CAMERAMODEL_H_ */
