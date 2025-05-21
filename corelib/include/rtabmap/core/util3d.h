/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef UTIL3D_H_
#define UTIL3D_H_

#include "rtabmap/core/rtabmap_core_export.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/TextureMesh.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <opencv2/core/core.hpp>
#include <rtabmap/core/ProgressState.h>
#include <map>
#include <list>

namespace rtabmap
{

/** 
 * @brief This namespace contains 3D point cloud processing utilities.
 */
namespace util3d
{

/**
 * @brief Converts a PCL point cloud with RGBA information to an OpenCV RGB or BGR image.
 *
 * This function takes a structured point cloud (organized as height × width) and creates a corresponding
 * OpenCV `cv::Mat` image containing the RGB (or BGR) color values from the cloud.
 *
 * @param cloud The input organized point cloud of type `pcl::PointCloud<pcl::PointXYZRGBA>`.
 *              The cloud must be organized (i.e., `cloud.width` and `cloud.height` must be greater than 0).
 * @param bgrOrder If true, the output image will be in BGR format (OpenCV default); if false, it will be RGB.
 *
 * @return A `cv::Mat` of type `CV_8UC3` with the same width and height as the input point cloud,
 *         containing RGB or BGR color values depending on the `bgrOrder` flag.
 *
 * @note This function assumes that the point cloud is organized. If the cloud is unorganized, the behavior is undefined.
 * @see pcl::PointCloud, cv::Mat
 */
cv::Mat RTABMAP_CORE_EXPORT rgbFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		bool bgrOrder = true);

/**
 * @brief Generates a depth image from a PCL organized point cloud.
 *
 * This function converts a given organized point cloud of type `pcl::PointXYZRGBA` into a 
 * depth image (`cv::Mat`) in either 32-bit float or 16-bit unsigned format. The depth image 
 * contains the Z coordinate (depth) values of each point.
 *
 * @param[in] cloud The input organized point cloud (height x width), where each point contains XYZ and RGBA.
 * @param[in] depth16U If true, output depth will be in 16-bit unsigned integer (millimeters); otherwise, 32-bit float (meters).
 *
 * @return A depth image (`cv::Mat`) of the same resolution as the input cloud, with type `CV_16UC1` or `CV_32FC1`.
 *
 * @note The function assumes that the point cloud is organized (structured as an image).
 */
cv::Mat RTABMAP_CORE_EXPORT depthFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		bool depth16U = true);

/**
 * @brief Converts a PCL point cloud (with RGBA colors) into aligned RGB and depth OpenCV images.
 *
 * This function extracts RGB and depth information from a structured PCL point cloud
 * (organized point cloud with width and height) and fills the corresponding OpenCV matrices.
 *
 * @param[in] cloud The input organized point cloud containing RGBA data (structured as height x width).
 * @param[out] frameBGR The output color image (CV_8UC3). The channels are ordered as BGR if bgrOrder is true, otherwise RGB.
 * @param[out] frameDepth The output depth image (either CV_32FC1 for meters or CV_16UC1 for millimeters depending on depth16U).
 * @param[in] bgrOrder If true, store colors in BGR order. If false, store as RGB.
 * @param[in] depth16U If true, store depth as 16-bit unsigned integers (in millimeters), otherwise use 32-bit floats (in meters).
 *
 * @note The function assumes that the point cloud is organized. If the point cloud is not organized,
 *       the output matrices will not be valid.
 */
void RTABMAP_CORE_EXPORT rgbdFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		cv::Mat & rgb,
		cv::Mat & depth,
		bool bgrOrder = true,
		bool depth16U = true);

/**
 * @brief Projects a single depth pixel into 3D space.
 *
 * This function computes the 3D coordinates of a point given a depth image and camera intrinsic parameters.
 * It optionally applies smoothing and depth error compensation to improve the accuracy of the result.
 *
 * @param depthImage      The depth image (CV_16UC1 in millimeters or CV_32FC1 in meters).
 * @param x               The x coordinate (column index) of the pixel to project.
 * @param y               The y coordinate (row index) of the pixel to project.
 * @param cx              The principal point x-coordinate. If set to 0, it will default to image center.
 * @param cy              The principal point y-coordinate. If set to 0, it will default to image center.
 * @param fx              The focal length in x direction (in pixels).
 * @param fy              The focal length in y direction (in pixels).
 * @param smoothing       Whether to apply smoothing on the depth value at (x, y).
 * @param depthErrorRatio Ratio used to reject outlier depths when smoothing is enabled.
 *
 * @return A pcl::PointXYZ containing the 3D coordinates of the pixel in the depth image. If the depth value is invalid or <= 0, all coordinates are set to NaN.
 */
pcl::PointXYZ RTABMAP_CORE_EXPORT projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float depthErrorRatio = 0.02f);

/**
 * @brief Projects pixel coordinates to a normalized 3D ray in camera coordinates.
 *
 * This function computes the direction of a 3D ray from the camera origin
 * through a pixel in the image plane, assuming a pinhole camera model.
 * The returned ray is not scaled by depth; it has a fixed z component of 1.0.
 *
 * @param imageSize The size of the image (width, height).
 * @param x The x-coordinate of the pixel in the image.
 * @param y The y-coordinate of the pixel in the image.
 * @param cx The x-coordinate of the principal point. If zero or negative, defaults to (image width / 2) - 0.5.
 * @param cy The y-coordinate of the principal point. If zero or negative, defaults to (image height / 2) - 0.5.
 * @param fx The focal length in the x direction (pixels).
 * @param fy The focal length in the y direction (pixels).
 * @return A normalized Eigen::Vector3f representing the direction of the ray in camera coordinates.
 */
Eigen::Vector3f RTABMAP_CORE_EXPORT projectDepthTo3DRay(
		const cv::Size & imageSize,
		float x, float y,
		float cx, float cy,
		float fx, float fy);

/**
 * @brief Converts a depth image to a 3D point cloud.
 * 
 * This function uses the camera model's intrinsic parameters to convert each pixel in the depth image to a 3D point in the camera's coordinate system.
 * It handles the decimation of the image and ensures the resulting point cloud does not exceed the specified maximum and minimum depth values.
 *
 * @param[in] imageDepth The input depth image, which should be of type `CV_16UC1` or `CV_32FC1`.
 * @param[in] cx The optical center of the camera in the x-axis (usually the center of the image).
 * @param[in] cy The optical center of the camera in the y-axis (usually the center of the image).
 * @param[in] fx The focal length of the camera in the x-axis.
 * @param[in] fy The focal length of the camera in the y-axis.
 * @param[in] decimation Decimation factor, used to reduce the image resolution (use 0 for no decimation).
 * @param[in] maxDepth The maximum depth value to consider when creating the point cloud.
 * @param[in] minDepth The minimum depth value to consider when creating the point cloud.
 * @param[out] validIndices A pointer to a vector where the indices of valid points will be stored. If null, this is ignored.
 * 
 * @return A `pcl::PointCloud<pcl::PointXYZ>::Ptr` containing the 3D points corresponding to the depth image.
 * 
 * @deprecated
 * This function is deprecated and will be removed in future versions.
 * Use the `cloudFromDepth` function that accepts a `rtabmap::CameraModel` instead.
 */
RTABMAP_DEPRECATED pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);
/**
 * @brief Converts a depth image to a 3D point cloud using a camera model.
 * 
 * This function performs the conversion of a depth image to a point cloud using the provided `CameraModel` object.
 * It handles decimation, validates the depth values, and calculates the corresponding 3D coordinates in the camera's optical coordinate system.
 *
 * @param[in] imageDepthIn The input depth image, which should be of type `CV_16UC1` or `CV_32FC1`.
 * @param[in] model The camera model containing the intrinsic parameters (fx, fy, cx, cy).
 * @param[in] decimation The decimation factor for image resolution reduction (use 0 for no decimation).
 * @param[in] maxDepth The maximum depth value to consider when creating the point cloud.
 * @param[in] minDepth The minimum depth value to consider when creating the point cloud.
 * @param[out] validIndices A pointer to a vector to store indices of valid points in the point cloud. If null, it is ignored.
 *
 * @return A `pcl::PointCloud<pcl::PointXYZ>::Ptr` containing the 3D points corresponding to the depth image.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT cloudFromDepth(
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

/**
 * @brief Creates a point cloud from an RGB image and a depth image.
 * 
 * This function uses the RGB and depth images, along with camera intrinsic 
 * parameters, to generate a point cloud where each point contains RGB color 
 * and 3D spatial information. It also handles decimation and depth constraints 
 * (min/max depth) for efficient processing and memory management.
 * 
 * @param imageRgb The RGB image (e.g., in BGR format for OpenCV).
 * @param imageDepth The depth image (CV_16UC1 or CV_32FC1 format).
 * @param cx The x-coordinate of the camera's principal point (optical center).
 * @param cy The y-coordinate of the camera's principal point.
 * @param fx The focal length in x-direction (in pixels).
 * @param fy The focal length in y-direction (in pixels).
 * @param decimation The decimation factor for the image (negative value for decimation from RGB size).
 * @param maxDepth The maximum depth value to consider for valid points (set 0 to ignore).
 * @param minDepth The minimum depth value to consider for valid points.
 * @param validIndices A pointer to a vector that will store valid point indices (optional).
 * 
 * @return A shared pointer to a point cloud (pcl::PointCloud<pcl::PointXYZRGB>) containing RGB and 3D point data.
 * 
 * @deprecated
 * This function is deprecated and will be removed in future versions.
 * Use the version that accepts a `rtabmap::CameraModel` instead.
 */
RTABMAP_DEPRECATED pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

/**
 * @brief Creates a point cloud from an RGB image and a depth image using a CameraModel.
 * 
 * This function generates a point cloud where each point contains RGB color 
 * and 3D spatial information derived from the provided RGB and depth images, 
 * using intrinsic parameters provided in the CameraModel. It also applies 
 * decimation for efficiency, and depth constraints (min/max depth) to limit 
 * the points to a valid range.
 * 
 * @param imageRgb The RGB image (e.g., in BGR format for OpenCV).
 * @param imageDepthIn The depth image (CV_16UC1 or CV_32FC1 format).
 * @param model A CameraModel object that contains intrinsic camera parameters (fx, fy, cx, cy).
 * @param decimation The decimation factor for the image (negative value for decimation from RGB size).
 * @param maxDepth The maximum depth value to consider for valid points (set 0 to ignore).
 * @param minDepth The minimum depth value to consider for valid points.
 * @param validIndices A pointer to a vector that will store valid point indices (optional).
 * 
 * @return A shared pointer to a point cloud (pcl::PointCloud<pcl::PointXYZRGB>) containing RGB and 3D point data.
 * 
 * @note This version uses a CameraModel to encapsulate the camera parameters.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

/**
 * @brief Converts a disparity image to a 3D point cloud.
 * 
 * This function takes a disparity image and projects each disparity value to
 * a 3D point in space using the provided stereo camera model. The points are 
 * stored in a `pcl::PointCloud` object. The function also supports decimating 
 * the image for faster processing.
 * 
 * @param imageDisparity The input disparity image, which must be of type 
 *        `CV_32FC1` (floating-point) or `CV_16SC1` (16-bit signed short).
 * @param model The stereo camera model used to project disparity to 3D.
 * @param decimation The decimation factor for downsampling the image. It must be 
 *        greater than or equal to 1.
 * @param maxDepth The maximum depth for valid points in meters. Points with a 
 *        depth greater than this value will be discarded. A non-positive value 
 *        means no maximum depth constraint.
 * @param minDepth The minimum depth for valid points in meters. Points with a 
 *        depth less than this value will be discarded.
 * @param validIndices An optional vector that will be filled with the indices 
 *        of valid points in the cloud (those within the depth constraints). 
 *        If nullptr, the indices are not stored.
 * 
 * @return A shared pointer to a `pcl::PointCloud<pcl::PointXYZ>` containing the 
 *         3D points derived from the disparity image.
 * 
 * @note If the disparity image dimensions are not divisible by the decimation 
 *       factor, the decimation factor will be adjusted to the highest compatible 
 *       value.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT cloudFromDisparity(
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

/**
 * @brief Converts a disparity image and an RGB image to a 3D point cloud with color.
 * 
 * This function takes both a disparity image and an RGB image to create a 
 * colored 3D point cloud. Each RGB pixel is associated with a 3D point generated 
 * from the disparity value. The function also supports decimating the images for 
 * faster processing.
 * 
 * @param imageRgb The input RGB image, which must have 3 channels (color->BGR) or 
 *        1 channel (grayscale). The image is used to assign colors to the 3D points.
 * @param imageDisparity The input disparity image, which must be of type 
 *        `CV_32FC1` (floating-point) or `CV_16SC1` (16-bit signed short).
 * @param model The stereo camera model used to project disparity to 3D.
 * @param decimation The decimation factor for downsampling the images. It must 
 *        be greater than or equal to 1.
 * @param maxDepth The maximum depth for valid points in meters. Points with a 
 *        depth greater than this value will be discarded. A non-positive value 
 *        means no maximum depth constraint.
 * @param minDepth The minimum depth for valid points in meters. Points with a 
 *        depth less than this value will be discarded.
 * @param validIndices An optional vector that will be filled with the indices 
 *        of valid points in the cloud (those within the depth constraints). 
 *        If nullptr, the indices are not stored.
 * 
 * @return A shared pointer to a `pcl::PointCloud<pcl::PointXYZRGB>` containing 
 *         the 3D points with associated RGB color values.
 * 
 * @note If the disparity image dimensions are not divisible by the decimation 
 *       factor, the decimation factor will be adjusted to the highest compatible 
 *       value.
 * 
 * @warning The RGB image must have the same size as the disparity image.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0);

/**
 * @brief Converts a pair of stereo images (left and right) into a 3D point cloud with RGB color information.
 *
 * This function takes a pair of stereo images, computes the disparity map between the left and right images, 
 * and then converts the disparity map into a 3D point cloud where each point contains the 3D coordinates 
 * (x, y, z) and RGB color values from the corresponding pixel in the left image.
 *
 * The function supports both color and monochrome images and performs stereo rectification and disparity 
 * calculation internally. The resulting 3D point cloud is returned in the PCL format with RGB color for each 
 * point, using the `pcl::PointXYZRGB` type. Points outside the specified depth range are discarded.
 * 
 * @param[in] imageLeft The left stereo image (either grayscale or color). If color, the image is converted 
 *                      to grayscale internally for disparity calculation.
 * @param[in] imageRight The right stereo image (either grayscale or color). If color, the image is converted 
 *                       to grayscale internally for disparity calculation.
 * @param[in] model The stereo camera model that contains the parameters for projecting disparity values into 3D.
 * @param[in] decimation The decimation factor used to downsample the image and reduce computation time. 
 *                       It must be greater than or equal to 1.
 * @param[in] maxDepth The maximum allowable depth (z value). Points with a depth greater than this value 
 *                     will be discarded.
 * @param[in] minDepth The minimum allowable depth (z value). Points with a depth smaller than this value 
 *                     will be discarded.
 * @param[out] validIndices A pointer to a vector that will be filled with the indices of the valid points 
 *                           in the resulting point cloud. Can be set to `nullptr` if this information is not needed.
 * @param[in] parameters A map of additional parameters for disparity computation, used by the stereo disparity function.
 *
 * @return A pointer to a `pcl::PointCloud<pcl::PointXYZRGB>` containing the 3D points with RGB colors.
 *         The cloud is dense only in regions with valid disparity values and within the specified depth range.
 *
 * @note The function assumes that the input images are rectified and aligned to the same coordinate system.
 *       The disparity map is computed from the left and right mono images using the `disparityFromStereoImages` 
 *       utility function, which is provided by the `util2d` namespace.
 *
 * @warning The input images must have the same size, and the disparity computation assumes that the stereo 
 *          pair is well-calibrated.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		const StereoCameraModel & model,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & parameters = ParametersMap());

/**
 * @brief Generates a set of point clouds from sensor data.
 *
 * This function processes depth and image data from the provided `SensorData` object 
 * to generate point clouds. It supports both depth camera data (using a camera model) 
 * and stereo camera data (using disparity computation). It handles various preprocessing 
 * operations like decimation, applying regions of interest (ROIs), and transforming point clouds.
 *
 * The function creates a point cloud for each camera model (or stereo camera model) 
 * in the `SensorData` and returns them in a vector of point cloud pointers.
 *
 * @param sensorData A reference to the `SensorData` object that contains raw depth 
 *                   or image data, along with camera models.
 * @param decimation The decimation factor to downsample the data. A value of 0 means no decimation.
 *                   The decimation factor should be a factor of the image width and height.
 * @param maxDepth The maximum depth value to be considered in the generated point clouds.
 * @param minDepth The minimum depth value to be considered in the generated point clouds.
 * @param validIndices An optional vector to store valid point indices for each point cloud. 
 *                     If provided, the function will populate it with indices of valid points 
 *                     in each corresponding cloud.
 * @param stereoParameters A map of parameters for stereo image processing, used to compute disparity.
 * @param roiRatios A vector containing four float values representing the region of interest 
 *                  (ROI) in normalized coordinates (left, right, top, bottom). 
 *                  If not specified or set to [0 0 0 0], the entire image is used.
 * 
 * @return A vector of `pcl::PointCloud<pcl::PointXYZ>::Ptr` representing the generated point clouds 
 *         in base coordinate frame.
 * 
 * @note 
 * - The `sensorData` object must contain either depth data (for depth cameras) or image data 
 *   with a right image (for stereo cameras).
 * - The ROI ratios must be in the range [0.0f, 1.0f] and will be applied to both depth and image 
 *   data if provided.
 * - If the ROI cannot be divided evenly by the decimation factor, the function will ignore the ROI 
 *   and log an error.
 * - If the stereo data is provided, disparity will be computed from the left and right images, 
 *   and the resulting disparity image will be used to generate 3D point clouds.
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> RTABMAP_CORE_EXPORT cloudsFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<pcl::IndicesPtr> * validIndices = 0,
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

/**
 * @brief Generates a point cloud from sensor data.
 *
 * This function processes the provided sensor data, generates point clouds for each sensor, and
 * combines them into a single point cloud. It handles both single-camera and stereo data by 
 * calling the `cloudsFromSensorData` function. The resulting point cloud can be decimated 
 * according to the specified parameter and filtered based on depth constraints.
 * 
 * @param sensorData The sensor data containing depth and image information. This data is used 
 *        to generate the point clouds.
 * @param decimation The factor by which the generated point cloud should be decimated. A value of 
 *        1 means no decimation. The decimation factor should be a factor of the image width and height.
 * @param maxDepth The maximum depth allowed for valid points in the generated cloud.
 * @param minDepth The minimum depth allowed for valid points in the generated cloud.
 * @param validIndices Optional vector to store the indices of valid points in the generated point 
 *        cloud.
 * @param stereoParameters The stereo parameters that may be used when dealing with stereo camera 
 *        data.
 * @param roiRatios A vector containing four float values representing the region of interest (ROI) 
 *        ratios (left, right, top, bottom) to crop the sensor images before processing. The values 
 *        should be between 0 and 1. If not specified or set to [0 0 0 0], the entire image is used.
 * 
 * @return A pointer to a `pcl::PointCloud<pcl::PointXYZ>` containing the combined point cloud 
 *         generated from the sensor data. The point cloud is transformed in base coordinate frame.
 * 
 * @note The function will automatically handle the removal of NaN points from the cloud.
 *       If the `validIndices` pointer is provided, it will be filled with the indices of valid points.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT cloudFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

/**
 * @brief Generates a point cloud with RGB color data from sensor data.
 *
 * This function creates RGB point clouds from depth and color images obtained from a sensor.
 * It supports both single camera models and stereo camera models. The RGB and depth images 
 * are processed and converted into a 3D point cloud using the provided camera models.
 * Optionally, Region-of-Interest (ROI) ratios can be applied to the images to focus on specific areas.
 *
 * @param[in] sensorData The sensor data that contains the raw image and depth information.
 * @param[in] decimation The decimation factor to reduce the resolution of the point cloud.
 * @param[in] maxDepth The maximum depth to consider while generating the point cloud.
 * @param[in] minDepth The minimum depth to consider while generating the point cloud.
 * @param[out] validIndices A pointer to a vector of indices that indicate the valid points in the cloud. 
 *                          If nullptr, no indices will be returned.
 * @param[in] stereoParameters A map of parameters used for stereo vision processing (if stereo camera models are used).
 * @param[in] roiRatios A vector of 4 floats that represent the region of interest (ROI) ratios (left, right, top, bottom) 
 *                      for cropping the image. Each value should be between 0.0 and 1.0, representing the normalized 
 *                      position of the ROI. A value of 0.0 means no cropping, and a value of 1.0 means full cropping.
 *
 * @return A vector of point clouds containing the RGB point cloud data for each camera model. 
 *         Each point cloud is represented by pcl::PointCloud<pcl::PointXYZRGB>::Ptr. The point clouds 
 *         are transformed in base coordinate frame.
 *
 * @note If the sensor data does not contain both image and depth data, or if no camera models are available, 
 *       an empty vector will be returned.
 * 
 * @note If stereo camera models are used, the left and right images are processed for disparity computation.
 * 
 * @warning The function performs several assertions and checks, such as ensuring that the image and depth 
 *          images are divisible by the number of camera models, and that the ROI ratios are compatible with 
 *          the decimation factor.
 */
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RTABMAP_CORE_EXPORT cloudsRGBFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<pcl::IndicesPtr > * validIndices = 0,
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

/**
 * @brief Generates a point cloud of type pcl::PointXYZRGB from sensor data.
 *
 * This function processes sensor data (such as RGB images and depth images) to generate a point cloud
 * of type pcl::PointXYZRGB. It can handle stereo or multiple camera models, apply decimation, and 
 * filter points based on the maximum and minimum depth values. Optionally, it can also apply region-of-interest (ROI) ratios.
 *
 * @param[in] sensorData The sensor data containing raw RGB and depth images, as well as camera models.
 * @param[in] decimation The decimation factor to reduce the point cloud size. Default is 1.
 * @param[in] maxDepth The maximum depth value (points beyond this distance will be ignored).
 * @param[in] minDepth The minimum depth value (points closer than this distance will be ignored).
 * @param[out] validIndices A vector to store the indices of the valid points in the generated point cloud. 
 *                          It will be resized to the point cloud's size.
 * @param[in] stereoParameters A map of stereo camera parameters, used when dealing with stereo images.
 * @param[in] roiRatios A vector of four float values representing the region-of-interest (ROI) ratios for cropping the 
 *                      depth and RGB images (left right top bottom). If not specified or set to [0 0 0 0], the entire 
 *                      image is used.
 * 
 * @return A pointer to a pcl::PointCloud<pcl::PointXYZRGB> containing the generated point cloud.
 *         If multiple clouds are generated, they are merged into a single cloud. The validIndices vector 
 *         is populated if it is provided. The point cloud(s) is/are transformed in base coordinate frame.
 * 
 * @note If the sensor data is stereo, the function will use stereo processing to generate the point cloud. 
 *       If multiple camera models are provided, it generates and merges the point clouds from each camera.
 * @note The validIndices vector will be resized and populated if it is passed as an argument.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation = 1,
		float maxDepth = 0.0f,
		float minDepth = 0.0f,
		std::vector<int> * validIndices = 0,
		const ParametersMap & stereoParameters = ParametersMap(),
		const std::vector<float> & roiRatios = std::vector<float>()); // ignored for stereo

/**
 * @brief Converts the middle row of a depth image into a laser scan (point cloud) using camera intrinsics and a local transformation.
 *
 * This function projects each pixel from a depth image into 3D space using the camera intrinsics 
 * (focal lengths and principal point) and applies a transformation to each point in 3D space.
 * It filters out points that are outside the specified depth range (maxDepth, minDepth) and 
 * excludes any invalid points.
 *
 * @param depthImage The input depth image (single channel, 16-bit unsigned or 32-bit floating point).
 * @param fx The focal length in the x-axis (in pixels).
 * @param fy The focal length in the y-axis (in pixels).
 * @param cx The optical center in the x-axis (in pixels).
 * @param cy The optical center in the y-axis (in pixels).
 * @param maxDepth The maximum depth value (in meters). Points with depth larger than this will be discarded. If 0, no maximum depth filtering is applied.
 * @param minDepth The minimum depth value (in meters). Points with depth smaller than this will be discarded.
 * @param localTransform A transformation that will be applied to all points in the resulting point cloud. This can be an identity transformation if not needed.
 * 
 * @return pcl::PointCloud<pcl::PointXYZ> The resulting point cloud where each point represents a 3D coordinate projected from the depth image.
 *
 * @note Assumes the input depth image is either CV_16UC1 (16-bit unsigned integers) or CV_32FC1 (32-bit floating-point).
 * @note Assumes the camera is pointing parallel to ground (e.g., forward looking camera).
 */
pcl::PointCloud<pcl::PointXYZ> RTABMAP_CORE_EXPORT laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth = 0,
					float minDepth = 0,
					const Transform & localTransform = Transform::getIdentity());
/**
 * @brief Converts multiple depth images (e.g., from a stereo or multi-camera setup) into a single laser scan (point cloud).
 *
 * This function processes multiple depth images, one for each camera in a stereo or multi-camera setup.
 * It uses the camera models to project the depth values of the middle row to 3D space and combines them into a single point cloud.
 * Each camera's depth image is projected using the associated camera intrinsics, and the points are transformed
 * according to the camera's local transformation.
 *
 * @param depthImages The input depth images concatenated horizontally. Each camera's depth image is assumed to be of equal width and placed side by side.
 * @param cameraModels A vector of camera models, one for each depth image in the input. Each model contains the intrinsics and the local transformation for a specific camera.
 * @param maxDepth The maximum depth value (in meters). Points with depth larger than this will be discarded. If 0, no maximum depth filtering is applied.
 * @param minDepth The minimum depth value (in meters). Points with depth smaller than this will be discarded.
 * 
 * @return pcl::PointCloud<pcl::PointXYZ> The resulting point cloud where each point represents a 3D coordinate projected from the depth images.
 *
 * @note This function assumes that the number of camera models corresponds to the number of sub-images in the input depth images.
 * @note Each depth image is projected using the corresponding camera model in the `cameraModels` array.
 * @note Assumes the cameras are pointing parallel to ground (e.g., forward and backward looking cameras).
 */
pcl::PointCloud<pcl::PointXYZ> RTABMAP_CORE_EXPORT laserScanFromDepthImages(
		const cv::Mat & depthImages,
		const std::vector<CameraModel> & cameraModels,
		float maxDepth,
		float minDepth);

/**
 * @defgroup LaserScanFromPointCloud PCL to 3D LaserScan
 * @brief Converts various PCL point cloud types into a unified LaserScan format.
 * 
 * These functions extract laser scan data from 3D point clouds, optionally applying a transform and filtering invalid points.
 * Depending on the input type, output data may include position (x, y, z), intensity, color, and/or surface normals.
 *
 * ### Supported input types and corresponding output data:
 * - `pcl::PointXYZ` → (x, y, z) → LaserScan::kXYZ
 * - `pcl::PointXYZRGB` → (x, y, z, rgb) → LaserScan::kXYZRGB
 * - `pcl::PointXYZI` → (x, y, z, intensity) → LaserScan::kXYZI
 * - `pcl::PointNormal` → (x, y, z, nx, ny, nz) → LaserScan::kXYZNormal
 * - `pcl::PointXYZRGBNormal` → (x, y, z, rgb, nx, ny, nz) → LaserScan::kXYZRGBNormal
 * - `pcl::PointXYZINormal` → (x, y, z, intensity, nx, ny, nz) → LaserScan::kXYZINormal
 * - Combined point + normal cloud also supported.
 *
 * @param cloud The input PCL point cloud.
 * @param indices Optional subset of indices to use from the point cloud.
 * @param normals Optional normal cloud if not included in point type.
 * @param transform Optional rigid body transform to apply to the points.
 * @param filterNaNs Whether to ignore invalid or NaN points.
 * @return A LaserScan object containing scan data in appropriate format.
 *
 * @see laserScan2dFromPointCloud() for 2D-only extraction.
 */

 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZ` → (x, y, z) → LaserScan::kXYZ
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZ` → (x, y, z) → LaserScan::kXYZ
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointNormal` → (x, y, z, nx, ny, nz) → LaserScan::kXYZNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointNormal` → (x, y, z, nx, ny, nz) → LaserScan::kXYZNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointNormal` → (x, y, z, nx, ny, nz) → LaserScan::kXYZNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGB` → (x, y, z, rgb) → LaserScan::kXYZRGB
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGB` → (x, y, z, rgb) → LaserScan::kXYZRGB
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGB` → (x, y, z, intensity) → LaserScan::kXYZI
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZI` → (x, y, z, intensity) → LaserScan::kXYZI
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGBNormal` → (x, y, z, rgb, nx, ny, nz) → LaserScan::kXYZRGBNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGBNormal` → (x, y, z, rgb, nx, ny, nz) → LaserScan::kXYZRGBNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZRGBNormal` → (x, y, z, rgb, nx, ny, nz) → LaserScan::kXYZRGBNormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZINormal` → (x, y, z, intensity, nx, ny, nz) → LaserScan::kXYZINormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZINormal` → (x, y, z, intensity, nx, ny, nz) → LaserScan::kXYZINormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
 /**
  * @ingroup LaserScanFromPointCloud
  * @brief `pcl::PointXYZINormal` → (x, y, z, intensity, nx, ny, nz) → LaserScan::kXYZINormal
  */
LaserScan RTABMAP_CORE_EXPORT laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform = Transform(), bool filterNaNs = true);

 /**
  * @ingroup LaserScanFromPointCloud
  * @brief Convert `pcl::PCLPointCloud2` to rtabmap::LaserScan with all supported fields (see rtabmap::LaserScan::Format)
  */
template<typename PointCloud2T>
LaserScan laserScanFromPointCloud(const PointCloud2T & cloud, bool filterNaNs = true, bool is2D = false, const Transform & transform = Transform());

/**
 * @defgroup LaserScan2dFromPointCloud PCL to 2D LaserScan
 * @brief Converts various PCL point cloud types into a unified 2D LaserScan format.
 *
 * Returns only XY data, with optional intensity and/or normals.
 *
 * ### Output:
 * - `PointXYZ` → LaserScan::kXY
 * - `PointXYZI` → LaserScan::kXYI (x, y, intensity)
 * - `PointNormal` or `PointXYZ + Normal` → LaserScan::kXYNormal (x, y, nx, ny, nz)
 * - `PointXYZINormal` or `PointXYZI + Normal` → LaserScan::kXYINormal (x, y, inensity, nx, ny, nz)
 *
 * @param cloud The input cloud (2D assumed).
 * @param normals Optional normal cloud.
 * @param transform Optional transformation to apply.
 * @param filterNaNs Whether to ignore invalid points.
 * @return A 2D LaserScan.
 * 
 * @see laserScanFromPointCloud() for 3D extraction.
 */
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointXYZ` → LaserScan::kXY
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointXYZI` → LaserScan::kXYI (x, y, intensity)
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointNormal` → LaserScan::kXYNormal (x, y, nx, ny, nz)
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointXYZ + Normal` → LaserScan::kXYNormal (x, y, nx, ny, nz)
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointXYZINormal` → LaserScan::kXYINormal (x, y, inensity, nx, ny, nz)
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform = Transform(), bool filterNaNs = true);
/**
 * @ingroup LaserScan2dFromPointCloud
 * @brief `PointXYZI + Normal` → LaserScan::kXYINormal (x, y, inensity, nx, ny, nz)
 */
LaserScan RTABMAP_CORE_EXPORT laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform = Transform(), bool filterNaNs = true);

/** @defgroup LaserScanToPointCloud LaserScan to PCL
 *  @brief Functions to convert LaserScan data to various PCL point cloud formats.
 *
 * These functions convert an instance of rtabmap::LaserScan to
 * different PCL point cloud types, optionally applying a spatial transform.
 * Missing data such as RGB and intensity are handled with default values.
 * If the input LaserScan is 2D, as PCL doesn't have standard format that is just 2D, 
 * Z will be set to zero. If the input LaserScan doesn't have normals but the output does, 
 * the normals are set to zeros.
 * 
 * laserScanToPoint() functions convert a single point from the LaserScan at the specificied index.
 * @{
 */
/**
 * @brief Convert rtabmap::LaserScan to `pcl::PCLPointCloud2` with all supported fields (see rtabmap::LaserScan::Format)
 */
pcl::PCLPointCloud2::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloud2(const LaserScan & laserScan, const Transform & transform = Transform());
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloud(const LaserScan & laserScan, const Transform & transform = Transform());
pcl::PointCloud<pcl::PointNormal>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloudNormal(const LaserScan & laserScan, const Transform & transform = Transform());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloudRGB(const LaserScan & laserScan, const Transform & transform = Transform(), unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
pcl::PointCloud<pcl::PointXYZI>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloudI(const LaserScan & laserScan, const Transform & transform = Transform(), float intensity = 0.0f);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloudRGBNormal(const LaserScan & laserScan, const Transform & transform = Transform(), unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr RTABMAP_CORE_EXPORT laserScanToPointCloudINormal(const LaserScan & laserScan, const Transform & transform = Transform(), float intensity = 0.0f);


pcl::PointXYZ RTABMAP_CORE_EXPORT laserScanToPoint(const LaserScan & laserScan, int index);
pcl::PointNormal RTABMAP_CORE_EXPORT laserScanToPointNormal(const LaserScan & laserScan, int index);
pcl::PointXYZRGB RTABMAP_CORE_EXPORT laserScanToPointRGB(const LaserScan & laserScan, int index, unsigned char r = 100, unsigned char g = 100, unsigned char b = 100);
pcl::PointXYZI RTABMAP_CORE_EXPORT laserScanToPointI(const LaserScan & laserScan, int index, float intensity);
pcl::PointXYZRGBNormal RTABMAP_CORE_EXPORT laserScanToPointRGBNormal(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b);
pcl::PointXYZINormal RTABMAP_CORE_EXPORT laserScanToPointINormal(const LaserScan & laserScan, int index, float intensity);

/**@} */

/**
 * @brief Computes the minimum and maximum 3D points from a laser scan matrix.
 *
 * This function computes the minimum and maximum values along each of the X, Y, and Z
 * axes in the laser scan matrix. The laser scan is assumed to be a matrix of type
 * `CV_32FC2`, `CV_32FC3`, `CV_32FC4`, `CV_32FC5`, `CV_32FC6`, or `CV_32FC7`, where each row
 * represents a point in space. The matrix is expected to have 3 or more channels for 3D data,
 * but it can have additional channels (e.g., intensity, color, etc.).
 *
 * The Z-coordinate is only considered if the matrix has at least 3 channels (3D scan). If the
 * scan has fewer than 3 channels, the Z-coordinate is set to 0.
 *
 * @param laserScan The input laser scan data, which must be a matrix of type `CV_32FC2`, `CV_32FC3`,
 *                  `CV_32FC4`, `CV_32FC5`, `CV_32FC6`, or `CV_32FC7` (with at least 2 channels, and at least 3 channels for 3D).
 * @param min A reference to a `cv::Point3f` object where the minimum 3D point (X, Y, Z) will be stored.
 * @param max A reference to a `cv::Point3f` object where the maximum 3D point (X, Y, Z) will be stored.
 *
 * @throws exception if the input matrix is empty or has an invalid type.
 */
void RTABMAP_CORE_EXPORT getMinMax3D(const cv::Mat & laserScan, cv::Point3f & min, cv::Point3f & max);

/**
 * @brief Computes the minimum and maximum 3D points from a laser scan matrix and stores the results in pcl::PointXYZ.
 *
 * This function calls the `getMinMax3D` function that operates on `cv::Point3f` and converts
 * the results to `pcl::PointXYZ` format. It is useful when working with PCL data structures for 3D processing.
 *
 * @param laserScan The input laser scan data, which must be a matrix of type `CV_32FC2`, `CV_32FC3`,
 *                  `CV_32FC4`, `CV_32FC5`, `CV_32FC6`, or `CV_32FC7` (with at least 2 channels, and at least 3 channels for 3D).
 * @param min A reference to a `pcl::PointXYZ` object where the minimum 3D point (X, Y, Z) will be stored.
 * @param max A reference to a `pcl::PointXYZ` object where the maximum 3D point (X, Y, Z) will be stored.
 *
 * @throws exception if the input matrix is empty or has an invalid type.
 */
void RTABMAP_CORE_EXPORT getMinMax3D(const cv::Mat & laserScan, pcl::PointXYZ & min, pcl::PointXYZ & max);

/**
 * @brief Projects a 2D point from the left image and its disparity into 3D space.
 *
 * This function converts a 2D point from the left image and its corresponding disparity 
 * value into a 3D point in space using the provided stereo camera model.
 *
 * The conversion follows the formula:
 * \f[ Z = \frac{baseline \times f}{disparity + (cx1 - cx0)} \f]
 * Where:
 * - `baseline` is the distance between the left and right camera centers
 * - `f` is the focal length of the camera
 * - `cx1` and `cx0` are the x-coordinates of the principal points of the right and left cameras, respectively.
 *
 * @param pt The 2D point in the left image (in pixels).
 * @param disparity The disparity value for the corresponding point (in pixels).
 * @param model The stereo camera model containing the intrinsic parameters.
 *
 * @return A 3D point (x, y, z) in space corresponding to the input 2D point and disparity.
 * If the disparity is invalid or any required camera parameters are not set, it returns a point containing NaN values.
 *
 * @note This function assumes that the disparity value is positive, and that the baseline and focal lengths are also positive.
 */
cv::Point3f RTABMAP_CORE_EXPORT projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		const StereoCameraModel & model);

/**
 * @brief Projects a 2D point from the left image and the disparity map into 3D space.
 *
 * This function converts a 2D point from the left image and the disparity value from a disparity map 
 * into a 3D point in space using the provided stereo camera model.
 *
 * The function first retrieves the disparity value for the given 2D point from the disparity map. 
 * It then calls the other `projectDisparityTo3D` function to perform the conversion to 3D.
 *
 * @param pt The 2D point in the left image (in pixels).
 * @param disparity The disparity map (CV_32FC1 or CV_16SC1) from which the disparity value for the point is retrieved.
 * @param model The stereo camera model containing the intrinsic parameters.
 *
 * @return A 3D point (x, y, z) in space corresponding to the input 2D point and the disparity value.
 * If the point is outside the disparity map bounds or if the disparity value is invalid, it returns a point containing NaN values.
 *
 * @note This function checks that the input disparity matrix is not empty and has an appropriate type (either CV_32FC1 or CV_16SC1).
 */
cv::Point3f RTABMAP_CORE_EXPORT projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		const StereoCameraModel & model);

/**
 * @brief Register a point cloud (laser scan) to the camera's frame of reference and return a registered depth image.
 *
 * This function projects a laser scan (in /base_link coordinate system) into the camera frame of reference 
 * using the camera's intrinsic parameters and the camera's transform. The result is stored 
 * in a depth image (cv::Mat), where each pixel corresponds to the distance of the projected point in the camera frame.
 *
 * @param[in] imageSize The desired size of the output depth image (in pixels).
 * @param[in] cameraMatrixK The camera matrix (intrinsics), containing the focal lengths and principal points.
 * @param[in] laserScan A matrix of laser scan points. The type can be CV_32FC2 (2D), CV_32FC3, CV_32FC4, etc., 
 *                      where the points are assumed to be in the /base_link coordinate system.
 * @param[in] cameraTransform The transform from /base_link to /camera_link, used to adjust the point cloud 
 *                            to the camera's reference frame.
 *
 * @return A cv::Mat object representing the registered depth image, with each pixel corresponding to the 
 *         projected point's depth (z value) in the camera frame.
 *
 * @throws exception if the input cameraTransform is null, the laser scan is empty, or 
 *         the cameraMatrixK has incorrect dimensions.
 */
cv::Mat RTABMAP_CORE_EXPORT projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,  
		const cv::Mat & laserScan,
		const rtabmap::Transform & cameraTransform);

/**
 * @brief Register a point cloud (laser scan) to the camera's frame of reference and return a registered depth image.
 *
 * This function projects a laser scan (pcl::PointCloud) into the camera frame of reference using the 
 * camera's intrinsic parameters and the camera's transform. The result is stored in a depth 
 * image (cv::Mat), where each pixel corresponds to the distance of the projected point in the camera frame.
 *
 * @param[in] imageSize The desired size of the output depth image (in pixels).
 * @param[in] cameraMatrixK The camera matrix (intrinsics), containing the focal lengths and principal points.
 * @param[in] laserScan A pointer to a pcl::PointCloud<pcl::PointXYZ> object containing the laser scan points.
 *                      The points are assumed to be in the /base_link coordinate system.
 * @param[in] cameraTransform The transform from /base_link to /camera_link, used to adjust the point cloud 
 *                            to the camera's reference frame.
 *
 * @return A cv::Mat object representing the registered depth image, with each pixel corresponding to the 
 *         projected point's depth (z value) in the camera frame.
 *
 * @throws exception if the input cameraTransform is null, the laser scan is empty, or 
 *         the cameraMatrixK has incorrect dimensions.
 */
cv::Mat RTABMAP_CORE_EXPORT projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan,
		const rtabmap::Transform & cameraTransform);

/**
 * @brief Register a point cloud (laser scan) to the camera's frame of reference and return a registered depth image.
 *
 * This function projects a laser scan (pcl::PCLPointCloud2) into the camera frame of reference using the 
 * camera's intrinsic parameters and the camera's transform. The result is stored in a depth 
 * image (cv::Mat), where each pixel corresponds to the distance of the projected point in the camera frame.
 *
 * @param[in] imageSize The desired size of the output depth image (in pixels).
 * @param[in] cameraMatrixK The camera matrix (intrinsics), containing the focal lengths and principal points.
 * @param[in] laserScan A pointer to a pcl::PCLPointCloud2 object containing the laser scan points.
 *                      The points are assumed to be in the /base_link coordinate system.
 * @param[in] cameraTransform The transform from /base_link to /camera_link, used to adjust the point cloud 
 *                            to the camera's reference frame.
 *
 * @return A cv::Mat object representing the registered depth image, with each pixel corresponding to the 
 *         projected point's depth (z value) in the camera frame.
 *
 * @throws exception if the input cameraTransform is null, the laser scan is empty, or 
 *         the cameraMatrixK has incorrect dimensions.
 */
cv::Mat RTABMAP_CORE_EXPORT projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,                       
		const pcl::PCLPointCloud2::Ptr laserScan,
		const rtabmap::Transform & cameraTransform);

/**
 * @brief Fills holes (missing depth values) in a depth image by interpolating between non-zero values.
 *
 * This function iterates through a depth image and fills holes by interpolating between the surrounding
 * non-zero depth values. It works either vertically or horizontally, depending on the `verticalDirection`
 * parameter. The interpolation method fills the missing values based on the linear slope between two
 * valid depth values. Optionally, the interpolation can be extended to the border of the image if 
 * `fillToBorder` is set to `true`.
 *
 * @param registeredDepth A matrix representing the registered depth image, where missing depth values are assumed to be zero.
 *                        The matrix must be of type `CV_32FC1`.
 * @param verticalDirection If `true`, the holes are filled vertically (by column), otherwise the holes are filled horizontally (by row).
 * @param fillToBorder If `true`, the interpolation will fill the depth holes all the way to the border of the image (i.e., it will propagate the nearest valid depth to the edges of the image).
 *
 * @note The input `registeredDepth` matrix must be of type `CV_32FC1`, where depth values are stored as single-precision floating-point numbers.
 * @note With `fillToBorder` enabled, the current implementation only fills up to last pixel, i.e., first/last rows or columns are not filled.
 *
 * @warning This function modifies the input `registeredDepth` matrix in place.
 */
void RTABMAP_CORE_EXPORT fillProjectedCloudHoles(
		cv::Mat & depthRegistered,
		bool verticalDirection,
		bool fillToBorder);

/**
 * @brief Filters out points below a certain threshold in a depth image based on camera models.
 *
 * This function processes a depth image and filters out points below a specified threshold, which 
 * are assumed to belong to the floor. It does this by projecting the depth pixels into 3D space (base frame) 
 * using the camera models, and if the z-coordinate of the projected point is less than the threshold, 
 * the corresponding depth value is set to 0.
 *
 * @param depth The input depth image to be filtered. The matrix should contain depth values in 
 *              either `CV_16UC1` (unsigned short) or `CV_32FC1` (float) format.
 * @param cameraModels A vector of camera models used for reprojection. The camera models 
 *                     are used to convert the 2D image coordinates to 3D space.
 * @param threshold The z-threshold in meters, below which points are considered part of the floor 
 *                  and will be filtered out.
 * @param depthBelow A pointer to an optional output depth image where the filtered-out points 
 *                   (below the threshold) will be saved. If this is `nullptr`, no output is 
 *                   generated for filtered-out points.
 * 
 * @return A new depth image where points below the specified threshold are set to zero.
 * 
 * @note
 * - The function assumes that the depth image has valid depth values.
 * - If the input `depthBelow` is provided, it will contain the depth values for the points that 
 *   were considered as floor points.
 * - The function assumes that all camera models have the same resolution and are valid for reprojection.
 * 
 * @throws std::invalid_argument if the camera models are empty or invalid.
 */
cv::Mat RTABMAP_CORE_EXPORT filterFloor(
	const cv::Mat & depth,
	const std::vector<CameraModel> & cameraModels,
	float threshold,
	cv::Mat * depthBelow = 0);

/**
 * @brief Projects a 3D point cloud to the best camera (NodeID -> CameraIndex) for each point based on a policy.
 *
 * This function takes a point cloud of type `pcl::PointXYZRGBNormal` and projects each point to the best camera that sees the point.
 * The "best" camera is selected based on the distance, angle, and other parameters like Region of Interest (ROI) ratios and projection mask.
 *
 * @param cloud The input point cloud of type `pcl::PointXYZRGBNormal` containing 3D points.
 * @param cameraPoses A map of camera IDs to their respective poses (transformations).
 * @param cameraModels A map of camera IDs to their camera models (internal camera parameters).
 * @param maxDistance Maximum allowable distance from the camera for a point to be considered.
 * @param maxAngle Maximum allowable angle between the camera and the point normal for it to be considered.
 * @param roiRatios A vector of four floats defining the region of interest ratios for the camera image. See util2d::computeRoi() for format.
 * @param projMask A binary mask for projection, which will be checked to ensure the point lies within the mask.
 * @param distanceToCamPolicy If true, the distance to the camera is considered in the decision of the best camera, 
 *                            otherwise distance to center of the camera is used.
 * @param state A `ProgressState` object to provide feedback on progress or cancellation.
 *
 * @return A vector of pairs, where each pair contains a pair of camera ID and camera index, and a 2D UV coordinate.
 *         The UV coordinate represents the projection of each point onto the best camera.
 */
std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > RTABMAP_CORE_EXPORT projectCloudToCameras (
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance = 0.0f,
		float maxAngle = 0.0f,
		float maxDepthError = 0.0f,
		const std::vector<float> & roiRatios = std::vector<float>(),
		const cv::Mat & projMask = cv::Mat(),
		bool distanceToCamPolicy = false,
		const ProgressState * state = 0);
/**
 * @brief Projects a 3D point cloud to the best camera (NodeID -> CameraIndex) for each point based on a policy.
 *
 * This function takes a point cloud of type `pcl::PointXYZINormal` and projects each point to the best camera that sees the point.
 * The "best" camera is selected based on the distance, angle, and other parameters like Region of Interest (ROI) ratios and projection mask.
 *
 * @param cloud The input point cloud of type `pcl::PointXYZINormal` containing 3D points.
 * @param cameraPoses A map of camera IDs to their respective poses (transformations).
 * @param cameraModels A map of camera IDs to their camera models (internal camera parameters).
 * @param maxDistance Maximum allowable distance from the camera for a point to be considered.
 * @param maxAngle Maximum allowable angle between the camera and the point normal for it to be considered.
 * @param roiRatios A vector of four floats defining the region of interest ratios for the camera image. See util2d::computeRoi() for format.
 * @param projMask A binary mask for projection, which will be checked to ensure the point lies within the mask.
 * @param distanceToCamPolicy If true, the distance to the camera is considered in the decision of the best camera,
 *                            otherwise distance to center of the camera is used.
 * @param state A `ProgressState` object to provide feedback on progress or cancellation.
 *
 * @return A vector of pairs, where each pair contains a pair of camera ID and camera index, and a 2D UV coordinate.
 *         The UV coordinate represents the projection of each point onto the best camera.
 */
std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > RTABMAP_CORE_EXPORT projectCloudToCameras (
		const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance = 0.0f,
		float maxAngle = 0.0f,
		float maxDepthError = 0.0f,
		const std::vector<float> & roiRatios = std::vector<float>(),
		const cv::Mat & projMask = cv::Mat(),
		bool distanceToCamPolicy = false,
		const ProgressState * state = 0);

/**
 * @brief Checks if all coordinates of a 3D point are finite.
 *
 * This function verifies that the x, y, and z components of the given
 * `cv::Point3f` are all finite values (i.e., not NaN or infinite).
 *
 * @param pt The 3D point to check.
 * @return true if all components of the point are finite, false otherwise.
 */
bool RTABMAP_CORE_EXPORT isFinite(const cv::Point3f & pt);

/**
 * @brief Concatenates a list of PointXYZ point clouds into a single point cloud.
 *
 * This function takes a list of shared pointers to `pcl::PointCloud<pcl::PointXYZ>` objects,
 * and merges them into a single point cloud by appending the points of each input cloud.
 *
 * @param[in] clouds A list of pointers to PointXYZ point clouds to concatenate.
 * @return A new `pcl::PointCloud<pcl::PointXYZ>::Ptr` containing all the points from the input clouds.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds);
/**
 * @brief Concatenates a list of PointXYZRGB point clouds into a single point cloud.
 *
 * This function takes a list of shared pointers to `pcl::PointCloud<pcl::PointXYZRGB>` objects,
 * and merges them into a single point cloud by appending the points of each input cloud.
 *
 * @param[in] clouds A list of pointers to PointXYZRGB point clouds to concatenate.
 * @return A new `pcl::PointCloud<pcl::PointXYZRGB>::Ptr` containing all the points from the input clouds.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_CORE_EXPORT concatenateClouds(
		const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds);

/**
 * @brief Concatenates multiple sets of indices into a single index vector.
 *
 * This function takes a vector of shared pointers to PCL index vectors
 * and combines them into a single shared pointer containing all indices
 * in the same order as they appear in the input.
 *
 * @param indices A vector of `pcl::IndicesPtr` (shared pointers to index vectors).
 * @return A single `pcl::IndicesPtr` containing the concatenated indices.
 *
 * @note The output order preserves the original order of indices from each input set.
 *
 * @see concatenate(const pcl::IndicesPtr &, const pcl::IndicesPtr &)
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT concatenate(
		const std::vector<pcl::IndicesPtr> & indices);

/**
 * @brief Concatenates two sets of indices into one.
 *
 * This function creates a new index vector containing all indices from
 * `indicesA` followed by all indices from `indicesB`.
 *
 * @param indicesA The first set of indices to include.
 * @param indicesB The second set of indices to append.
 * @return A `pcl::IndicesPtr` containing the combined indices from both inputs.
 *
 * @note The input index vectors are not modified.
 *
 * @see concatenate(const std::vector<pcl::IndicesPtr> &)
 */
pcl::IndicesPtr RTABMAP_CORE_EXPORT concatenate(
		const pcl::IndicesPtr & indicesA,
		const pcl::IndicesPtr & indicesB);

/**
 * @brief Saves 3D word points to a PCD file, applying a transform to each point.
 *
 * This function takes a multimap of word identifiers and corresponding 3D PCL points,
 * applies the given transform to each point, and saves the resulting point cloud
 * to the specified PCD file.
 *
 * @param fileName       The path to the output PCD file.
 * @param words          A multimap containing word IDs and their associated pcl::PointXYZ coordinates.
 * @param transform      A transform to apply to each 3D point before saving.
 */
void RTABMAP_CORE_EXPORT savePCDWords(
		const std::string & fileName,
		const std::multimap<int, pcl::PointXYZ> & words,
		const Transform & transform = Transform::getIdentity());

/**
 * @brief Saves 3D word points (as OpenCV points) to a PCD file, applying a transform to each point.
 *
 * This function takes a multimap of word identifiers and corresponding 3D OpenCV points,
 * converts them to PCL format after applying the given transform, and saves them
 * as a point cloud to the specified PCD file.
 *
 * @param fileName       The path to the output PCD file.
 * @param words          A multimap containing word IDs and their associated cv::Point3f coordinates.
 * @param transform      A transform to apply to each 3D point before saving.
 */
void RTABMAP_CORE_EXPORT savePCDWords(
		const std::string & fileName,
		const std::multimap<int, cv::Point3f> & words,
		const Transform & transform = Transform::getIdentity());

/**
 * @brief Loads a KITTI-style Velodyne binary scan file into an OpenCV matrix.
 *
 * This function assumes the binary file contains a series of 4-float values per point
 * representing (X, Y, Z, Intensity). It loads the entire file into a `cv::Mat` with
 * type `CV_32FC4`.
 *
 * @param fileName Path to the .bin file.
 * @return A 1-row `cv::Mat` with 4 channels (XYZI), one column per point.
 */
cv::Mat RTABMAP_CORE_EXPORT loadBINScan(const std::string & fileName);
/**
 * @brief Loads a KITTI-style Velodyne binary scan and converts it to a PCL point cloud.
 *
 * Internally calls `loadScan()` and converts the result into a `pcl::PointCloud<pcl::PointXYZ>`.
 *
 * @param fileName Path to the .bin file.
 * @return Pointer to the loaded point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT loadBINCloud(const std::string & fileName);
/**
 * @brief Loads a KITTI-style Velodyne binary scan and converts it to a PCL point cloud.
 *
 * @param fileName Path to the .bin file.
 * @param dim Unused parameter.
 * @return Pointer to the loaded point cloud.
 * @deprecated This overload exists for compatibility but ignores the `dim` parameter. Use version without `dim` directly.
 */
RTABMAP_DEPRECATED pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT loadBINCloud(const std::string & fileName, int dim);

/**
 * @brief Loads a 3D scan from a file (.pcd, .ply, or .bin format).
 *
 * This function detects the file type based on the extension and loads the scan accordingly.
 * Binary .bin files are interpreted using the KITTI format (XYZI).
 * For .pcd or .ply files, a PCL point cloud is loaded and optionally interpreted as 2D if all Z values are 0.
 *
 * @param path Path to the scan file.
 * @return A `LaserScan` object containing the loaded scan data.
 */
LaserScan RTABMAP_CORE_EXPORT loadScan(const std::string & path);

/**
 * @brief Loads and optionally transforms/downsamples/voxelizes a point cloud.
 *
 * This function loads a point cloud from a .bin, .pcd, or .ply file. It can apply a transformation,
 * downsample using a step size, or filter using a voxel grid.
 *
 * @param path Path to the scan file.
 * @param transform Transformation to apply to the cloud (must not be null).
 * @param downsampleStep Step size to downsample (1 = no downsampling).
 * @param voxelSize Size of the voxel grid filter in meters (0 = no filtering).
 * @return Transformed and optionally filtered `pcl::PointCloud<pcl::PointXYZ>::Ptr`.
 * @deprecated Use loadScan() instead.
 */
RTABMAP_DEPRECATED pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_CORE_EXPORT loadCloud(
		const std::string & path,
		const Transform & transform = Transform::getIdentity(),
		int downsampleStep = 1,
		float voxelSize = 0.0f);

/**
 * @brief Lidar deskewing
 * @param input lidar, format should have time channel
 * @param input stamp of the lidar
 * @param velocity in base frame
 * @param velocity stamp at which it has been computed
 * @return lidar deskewed
 */
/**
 * @brief Deskews a laser scan based on the velocity transform and timestamp information.
 * 
 * This function takes an input `LaserScan` with a time channel (`kXYZIT` format), and deskews the scan 
 * based on the provided velocity transform. It assumes that the scan has a time channel with time 
 * information for each point relative to `inputStamp`. The deskewing process involves calculating the pose 
 * of the laser at each point in time and correcting the scan data accordingly.
 * 
 * @param[in] input The input LaserScan to be deskewed.
 * @param[in] inputStamp The timestamp of the input scan (e.g., epoch time).
 * @param[in] velocity The velocity transform (linear and angular velocities), in base frame.
 * 
 * @return A new `LaserScan` that has been deskewed based on the provided velocity and timestamps.
 * 
 * @note If the velocity is null or if the input scan does not have the correct format, an error will be logged.
 *       If the first and last timestamps are identical, deskewing cannot be performed, and an error is logged.
 * 
 * @warning This function assumes that the input `LaserScan` is in the `kXYZIT` format (with time data).
 *          If not, an error will be logged and an empty `LaserScan` will be returned.
 * 
 * @throws exception if velocity is null or the format is incorrect.
 * 
 */
LaserScan RTABMAP_CORE_EXPORT deskew(
		const LaserScan & input,
		double inputStamp,
		const rtabmap::Transform & velocity);

} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d.hpp"

#endif /* UTIL3D_H_ */
