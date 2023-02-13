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

#ifndef UTIL3D_SURFACE_H_
#define UTIL3D_SURFACE_H_

#include <rtabmap/core/rtabmap_core_export.h>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/ProgressState.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Version.h>
#include <set>
#include <list>

namespace rtabmap
{

class Memory;
class DBDriver;

namespace util3d
{

/**
 * @brief Given a set of polygons, create two indexes: polygons to neighbor polygons and vertices to polygons.
 *
 * @param polygons the polygons to be indexed.
 * @param cloudSize the size of the cloud of the corresponding mesh to polygons (must be at least as high as
 * the highest vertex value contained in the polygons).
 * @param neighborPolygons returned index from polygons to neighbor polygons (index size = polygons size).
 * @param vertexPolygons returned index from vertices to polygons (index size = cloudSize).
 */
void RTABMAP_CORE_EXPORT createPolygonIndexes(
		const std::vector<pcl::Vertices> & polygons,
		int cloudSize,
		std::vector<std::set<int> > & neighborPolygons,
		std::vector<std::set<int> > & vertexPolygons);

std::list<std::list<int> > RTABMAP_CORE_EXPORT clusterPolygons(
		const std::vector<std::set<int> > & neighborPolygons,
		int minClusterSize = 0);

std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		double angleTolerance,
		bool quad,
		int trianglePixelSize,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));
std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		double angleTolerance = M_PI/16,
		bool quad=true,
		int trianglePixelSize = 2,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));
std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT organizedFastMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		double angleTolerance = M_PI/16,
		bool quad=true,
		int trianglePixelSize = 2,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0));

void RTABMAP_CORE_EXPORT appendMesh(
		pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB);
void RTABMAP_CORE_EXPORT appendMesh(
		pcl::PointCloud<pcl::PointXYZRGB> & cloudA,
		std::vector<pcl::Vertices> & polygonsA,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloudB,
		const std::vector<pcl::Vertices> & polygonsB);

// return map from new to old polygon indices
std::vector<int> RTABMAP_CORE_EXPORT filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGBNormal> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons);
std::vector<int> RTABMAP_CORE_EXPORT filterNotUsedVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons);
std::vector<int> RTABMAP_CORE_EXPORT filterNaNPointsFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		pcl::PointCloud<pcl::PointXYZRGB> & outputCloud,
		std::vector<pcl::Vertices> & outputPolygons);

std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT filterCloseVerticesFromMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		const std::vector<pcl::Vertices> & polygons,
		float radius,
		float angle,
		bool keepLatestInRadius);

std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT filterInvalidPolygons(
		const std::vector<pcl::Vertices> & polygons);

pcl::PolygonMesh::Ptr RTABMAP_CORE_EXPORT createMesh(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloudWithNormals,
		float gp3SearchRadius = 0.025,
		float gp3Mu = 2.5,
		int gp3MaximumNearestNeighbors = 100,
		float gp3MaximumSurfaceAngle = M_PI/4,
		float gp3MinimumAngle = M_PI/18,
		float gp3MaximumAngle = 2*M_PI/3,
		bool gp3NormalConsistency = true);

pcl::TextureMesh::Ptr RTABMAP_CORE_EXPORT createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, CameraModel> & cameraModels,
		const std::map<int, cv::Mat> & cameraDepths,
		float maxDistance = 0.0f, // max camera distance to polygon to apply texture
		float maxDepthError = 0.0f, // maximum depth error between reprojected mesh and depth image to texture a face (-1=disabled, 0=edge length is used)
		float maxAngle = 0.0f, // maximum angle between camera and face (0=disabled)
		int minClusterSize = 50, // minimum size of polygons clusters textured
		const std::vector<float> & roiRatios = std::vector<float>(), // [left, right, top, bottom] region of interest (in ratios) of the image projected.
		const ProgressState * state = 0,
		std::vector<std::map<int, pcl::PointXY> > * vertexToPixels = 0, // For each point, we have a list of cameras with corresponding pixel in it. Beware that the camera ids don't correspond to pose ids, they are indexes from 0 to total camera models and texture's materials.
		bool distanceToCamPolicy = false);
pcl::TextureMesh::Ptr RTABMAP_CORE_EXPORT createTextureMesh(
		const pcl::PolygonMesh::Ptr & mesh,
		const std::map<int, Transform> & poses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		const std::map<int, cv::Mat> & cameraDepths,
		float maxDistance = 0.0f, // max camera distance to polygon to apply texture
		float maxDepthError = 0.0f, // maximum depth error between reprojected mesh and depth image to texture a face (-1=disabled, 0=edge length is used)
		float maxAngle = 0.0f, // maximum angle between camera and face (0=disabled)
		int minClusterSize = 50, // minimum size of polygons clusters textured
		const std::vector<float> & roiRatios = std::vector<float>(), // [left, right, top, bottom] region of interest (in ratios) of the image projected.
		const ProgressState * state = 0,
		std::vector<std::map<int, pcl::PointXY> > * vertexToPixels = 0, // For each point, we have a list of cameras with corresponding pixel in it. Beware that the camera ids don't correspond to pose ids, they are indexes from 0 to total camera models and texture's materials.
		bool distanceToCamPolicy = false);

/**
 * Remove not textured polygon clusters. If minClusterSize<0, only the largest cluster is kept.
 */
void RTABMAP_CORE_EXPORT cleanTextureMesh(
		pcl::TextureMesh & textureMesh,
		int minClusterSize);

pcl::TextureMesh::Ptr RTABMAP_CORE_EXPORT concatenateTextureMeshes(
		const std::list<pcl::TextureMesh::Ptr> & meshes);

void RTABMAP_CORE_EXPORT concatenateTextureMaterials(
		pcl::TextureMesh & mesh, const cv::Size & imageSize, int textureSize, int maxTextures, float & scale, std::vector<bool> * materialsKept=0);

std::vector<std::vector<RTABMAP_PCL_INDEX> > RTABMAP_CORE_EXPORT convertPolygonsFromPCL(
		const std::vector<pcl::Vertices> & polygons);
std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > RTABMAP_CORE_EXPORT convertPolygonsFromPCL(
		const std::vector<std::vector<pcl::Vertices> > & polygons);
std::vector<pcl::Vertices> RTABMAP_CORE_EXPORT convertPolygonsToPCL(
		const std::vector<std::vector<RTABMAP_PCL_INDEX> > & polygons);
std::vector<std::vector<pcl::Vertices> > RTABMAP_CORE_EXPORT convertPolygonsToPCL(
		const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & tex_polygons);

pcl::TextureMesh::Ptr RTABMAP_CORE_EXPORT assembleTextureMesh(
		const cv::Mat & cloudMat,
		const std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > & polygons,
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
		const std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > > & texCoords,
#else
		const std::vector<std::vector<Eigen::Vector2f> > & texCoords,
#endif
		cv::Mat & textures,
		bool mergeTextures = false);

pcl::PolygonMesh::Ptr RTABMAP_CORE_EXPORT assemblePolygonMesh(
		const cv::Mat & cloudMat,
		const std::vector<std::vector<RTABMAP_PCL_INDEX> > & polygons);

/**
 * Merge all textures in the mesh into "textureCount" textures of size "textureSize".
 * @return merged textures corresponding to new materials set in TextureMesh (height=textureSize, width=textureSize*materials)
 */
cv::Mat RTABMAP_CORE_EXPORT mergeTextures(
		pcl::TextureMesh & mesh,
		const std::map<int, cv::Mat> & images, // raw or compressed, can be empty if memory or dbDriver should be used
		const std::map<int, CameraModel> & calibrations, // Should match images
		const Memory * memory = 0,             // Should be set if images are not set
		const DBDriver * dbDriver = 0,         // Should be set if images and memory are not set
		int textureSize = 4096,
		int textureCount = 1,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels = std::vector<std::map<int, pcl::PointXY> >(), // needed for parameters below
		bool gainCompensation = true,
		float gainBeta = 10.0f,
		bool gainRGB = true,                 //Do gain compensation on each channel
		bool blending = true,
		int blendingDecimation = 0,          //0=auto depending on projected polygon size and texture size
		int brightnessContrastRatioLow = 0,  //0=disabled, values between 0 and 100
		int brightnessContrastRatioHigh = 0, //0=disabled, values between 0 and 100
		bool exposureFusion = false,         //Exposure fusion can be used only with OpenCV3
		const ProgressState * state = 0,
		unsigned char blankValue = 255,      //Gray value for blank polygons (without texture)
		std::map<int, std::map<int, cv::Vec4d> > * gains = 0, // <Camera ID, Camera Sub Index (multi-cameras), gains Gray-R-G-B>
		std::map<int, std::map<int, cv::Mat> > * blendingGains = 0, // <Camera ID, Camera Sub Index (multi-cameras), gains>
		std::pair<float, float> * contrastValues = 0); // Alpha/beta contrast values
cv::Mat RTABMAP_CORE_EXPORT mergeTextures(
		pcl::TextureMesh & mesh,
		const std::map<int, cv::Mat> & images, // raw or compressed, can be empty if memory or dbDriver should be used
		const std::map<int, std::vector<CameraModel> > & calibrations, // Should match images
		const Memory * memory = 0,             // Should be set if images are not set
		const DBDriver * dbDriver = 0,         // Should be set if images and memory are not set
		int textureSize = 4096,
		int textureCount = 1,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels = std::vector<std::map<int, pcl::PointXY> >(), // needed for parameters below
		bool gainCompensation = true,
		float gainBeta = 10.0f,
		bool gainRGB = true,                 //Do gain compensation on each channel
		bool blending = true,
		int blendingDecimation = 0,          //0=auto depending on projected polygon size and texture size
		int brightnessContrastRatioLow = 0,  //0=disabled, values between 0 and 100
		int brightnessContrastRatioHigh = 0, //0=disabled, values between 0 and 100
		bool exposureFusion = false,         //Exposure fusion can be used only with OpenCV3
		const ProgressState * state = 0,
		unsigned char blankValue = 255,      //Gray value for blank polygons (without texture)
		std::map<int, std::map<int, cv::Vec4d> > * gains = 0, // <Camera ID, Camera Sub Index (multi-cameras), gains Gray-R-G-B>
		std::map<int, std::map<int, cv::Mat> > * blendingGains = 0, // <Camera ID, Camera Sub Index (multi-cameras), gains>
		std::pair<float, float> * contrastValues = 0); // Alpha/beta contrast values

void RTABMAP_CORE_EXPORT fixTextureMeshForVisualization(pcl::TextureMesh & textureMesh);

// Use the same method with 22 parameters instead.
RTABMAP_DEPRECATED bool RTABMAP_CORE_EXPORT multiBandTexturing(
		const std::string & outputOBJPath,
		const pcl::PCLPointCloud2 & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const std::map<int, Transform> & cameraPoses,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels, // required output of util3d::createTextureMesh()
		const std::map<int, cv::Mat> & images,        // raw or compressed, can be empty if memory or dbDriver should be used
		const std::map<int, std::vector<CameraModel> > & cameraModels, // Should match images
		const Memory * memory = 0,                    // Should be set if images are not set
		const DBDriver * dbDriver = 0,                // Should be set if images and memory are not set
		int textureSize = 8192,
		const std::string & textureFormat = "jpg",    // png, jpg
		const std::map<int, std::map<int, cv::Vec4d> > & gains = std::map<int, std::map<int, cv::Vec4d> >(),       // optional output of util3d::mergeTextures()
		const std::map<int, std::map<int, cv::Mat> > & blendingGains = std::map<int, std::map<int, cv::Mat> >(),   // optional output of util3d::mergeTextures()
		const std::pair<float, float> & contrastValues = std::pair<float, float>(0,0),                             // optional output of util3d::mergeTextures()
		bool gainRGB = true);

/**
 * Texture mesh with AliceVision's multiband texturing approach. See also https://meshroom-manual.readthedocs.io/en/bibtex1/node-reference/nodes/Texturing.html.
 * @param outputOBJPath Output OBJ path
 * @param cloud input Cloud of the mesh.
 * @param polygons Input polygons of the mesh.
 * @param cameraPoses Poses of the cameras.
 * @param vertexToPixels Output from {@link #createTextureMesh()}.
 * @param images Images corresponding to cameraPoses, raw or compressed, can be empty if memory or dbDriver should be used.
 * @param cameraModels Camera calibrations corresponding to cameraPoses.
 * @param memory Should be set if images and dbDriver are not set.
 * @param dbDriver Should be set if images and memory are not set.
 * @param textureSize Output texture size 1024, 2048, 4096, 8192, 16384.
 * @param textureDownscale Downscaling to 4 or 8 will reduce the texture quality but speed up the computation time. Set Texture Downscale to 1 instead of 2 to get the maximum possible resolution with the resolution of your images. The output texture size will be divided by this value, e.g., with texture size of 8192 and downscale value of 2, the output will be 4096.
 * @param nbContrib number of contributions per frequency band for the multi-band blending (should be 4 values)
 * @param textureFormat Output texture format: "png" or "jpg".
 * @param gains Optional output of {@link #mergeTextures()}.
 * @param blendingGains Optional output of {@link #mergeTextures()}.
 * @param contrastValues Optional output of {@link #mergeTextures()}.
 * @param gainRGB Apply gain compensation on each RGB channels separately, otherwise it is apply equally to all channels.
 * @param unwrapMethod Method to unwrap input mesh if it does not have UV coordinates 0=Basic (> 600k faces) fast and simple. Can generate multiple atlases 2=LSCM (<= 600k faces): optimize space. Generates one atlas 1=ABF (<= 300k faces): optimize space and stretch. Generates one atlas.
 * @param fillHoles Fill Texture holes with plausible values True/False.
 * @param padding Texture edge padding size in pixel (0-100).
 * @param bestScoreThreshold 0.0 to disable filtering based on threshold to relative best score (0.0-1.0).
 * @param angleHardThreshold 0.0 to disable angle hard threshold filtering (0.0, 180.0).
 * @param forceVisibleByAllVertices Triangle visibility is based on the union of vertices visibility.
 */
bool RTABMAP_CORE_EXPORT multiBandTexturing(
		const std::string & outputOBJPath,
		const pcl::PCLPointCloud2 & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const std::map<int, Transform> & cameraPoses,
		const std::vector<std::map<int, pcl::PointXY> > & vertexToPixels,
		const std::map<int, cv::Mat> & images,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		const Memory * memory = 0,
		const DBDriver * dbDriver = 0,
		unsigned int textureSize = 8192,
		unsigned int textureDownscale = 2,
		const std::string & nbContrib = "1 5 10 0",
		const std::string & textureFormat = "jpg",
		const std::map<int, std::map<int, cv::Vec4d> > & gains = std::map<int, std::map<int, cv::Vec4d> >(),
		const std::map<int, std::map<int, cv::Mat> > & blendingGains = std::map<int, std::map<int, cv::Mat> >(),
		const std::pair<float, float> & contrastValues = std::pair<float, float>(0,0),
		bool gainRGB = true,
		unsigned int unwrapMethod = 0,
		bool fillHoles = false,
		unsigned int padding = 5,
		double bestScoreThreshold = 0.1,
		double angleHardThreshold = 90.0,
		bool forceVisibleByAllVertices = false);

cv::Mat RTABMAP_CORE_EXPORT computeNormals(
		const cv::Mat & laserScan,
		int searchK,
		float searchRadius);
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		int searchK = 20,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));

pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals2D(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK = 5,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeNormals2D(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK = 5,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeFastOrganizedNormals2D(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		int searchK = 5,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeFastOrganizedNormals2D(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		int searchK = 5,
		float searchRadius = 0.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));

pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float maxDepthChangeFactor = 0.02f,
		float normalSmoothingSize = 10.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));
pcl::PointCloud<pcl::Normal>::Ptr RTABMAP_CORE_EXPORT computeFastOrganizedNormals(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float maxDepthChangeFactor = 0.02f,
		float normalSmoothingSize = 10.0f,
		const Eigen::Vector3f & viewPoint = Eigen::Vector3f(0,0,0));

float RTABMAP_CORE_EXPORT computeNormalsComplexity(
		const LaserScan & scan,
		const Transform & t = Transform::getIdentity(),
		cv::Mat * pcaEigenVectors = 0,
		cv::Mat * pcaEigenValues = 0);
float RTABMAP_CORE_EXPORT computeNormalsComplexity(
		const pcl::PointCloud<pcl::Normal> & normals,
		const Transform & t = Transform::getIdentity(),
		bool is2d = false,
		cv::Mat * pcaEigenVectors = 0,
		cv::Mat * pcaEigenValues = 0);
float RTABMAP_CORE_EXPORT computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointNormal> & cloud,
		const Transform & t = Transform::getIdentity(),
		bool is2d = false,
		cv::Mat * pcaEigenVectors = 0,
		cv::Mat * pcaEigenValues = 0);
float RTABMAP_CORE_EXPORT computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const Transform & t = Transform::getIdentity(),
		bool is2d = false,
		cv::Mat * pcaEigenVectors = 0,
		cv::Mat * pcaEigenValues = 0);
float RTABMAP_CORE_EXPORT computeNormalsComplexity(
		const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const Transform & t = Transform::getIdentity(),
		bool is2d = false,
		cv::Mat * pcaEigenVectors = 0,
		cv::Mat * pcaEigenValues = 0);

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		float searchRadius = 0.0f,
		int polygonialOrder = 2,
		int upsamplingMethod = 0, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius = 0.0f,   // SAMPLE_LOCAL_PLANE
		float upsamplingStep = 0.0f,     // SAMPLE_LOCAL_PLANE
		int pointDensity = 0,            // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize = 1.0f,  // VOXEL_GRID_DILATION
		int dilationIterations = 0);     // VOXEL_GRID_DILATION
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RTABMAP_CORE_EXPORT mls(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float searchRadius = 0.0f,
		int polygonialOrder = 2,
		int upsamplingMethod = 0, // NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		float upsamplingRadius = 0.0f,   // SAMPLE_LOCAL_PLANE
		float upsamplingStep = 0.0f,     // SAMPLE_LOCAL_PLANE
		int pointDensity = 0,            // RANDOM_UNIFORM_DENSITY
		float dilationVoxelSize = 1.0f,  // VOXEL_GRID_DILATION
		int dilationIterations = 0);     // VOXEL_GRID_DILATION

// Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUp to 0.8f, otherwise set groundNormalsUp to 0.0f.
RTABMAP_DEPRECATED LaserScan RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		const LaserScan & scan,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp);
LaserScan RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		const LaserScan & scan,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0),
		float groundNormalsUp = 0.0f);
// Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUp to 0.8f, otherwise set groundNormalsUp to 0.0f.
RTABMAP_DEPRECATED void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp);
void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0),
		float groundNormalsUp = 0.0f);
// Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUp to 0.8f, otherwise set groundNormalsUp to 0.0f.
RTABMAP_DEPRECATED void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp);
void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0),
		float groundNormalsUp = 0.0f);
// Use version with groundNormalsUp as float. For forceGroundNormalsUp=true, set groundNormalsUp to 0.8f, otherwise set groundNormalsUp to 0.0f.
RTABMAP_DEPRECATED void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint,
		bool forceGroundNormalsUp);
void RTABMAP_CORE_EXPORT adjustNormalsToViewPoint(
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Eigen::Vector3f & viewpoint = Eigen::Vector3f(0,0,0),
		float groundNormalsUp = 0.0f);

void RTABMAP_CORE_EXPORT adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		float groundNormalsUp = 0.0f);
void RTABMAP_CORE_EXPORT adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		float groundNormalsUp = 0.0f);
void RTABMAP_CORE_EXPORT adjustNormalsToViewPoints(
		const std::map<int, Transform> & poses,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & rawCloud,
		const std::vector<int> & rawCameraIndices,
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		float groundNormalsUp = 0.0f);

void RTABMAP_CORE_EXPORT adjustNormalsToViewPoints(
		const std::map<int, Transform> & viewpoints,
		const LaserScan & rawScan,
		const std::vector<int> & viewpointIds,
		LaserScan & scan,
		float groundNormalsUp = 0.0f);

pcl::PolygonMesh::Ptr RTABMAP_CORE_EXPORT meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor);

template<typename pointT>
std::vector<pcl::Vertices> normalizePolygonsSide(
		const pcl::PointCloud<pointT> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const pcl::PointXYZ & viewPoint = pcl::PointXYZ(0,0,0));

template<typename pointRGBT>
void denseMeshPostProcessing(
		pcl::PolygonMeshPtr & mesh,
		float meshDecimationFactor = 0.0f, // value between 0 and 1, 0=disabled
		int maximumPolygons = 0,           // 0=disabled
		const typename pcl::PointCloud<pointRGBT>::Ptr & cloud = pcl::PointCloud<pointRGBT>::Ptr(), // A RGB point cloud used to transfer colors back to mesh (needed for parameters below)
		float transferColorRadius = 0.05f, // <0=disabled, 0=nearest color
		bool coloredOutput = true,         // Not used anymore, output is colored if transferColorRadius>=0
		bool cleanMesh = true,             // Remove polygons not colored (if coloredOutput is disabled, transferColorRadius is still used to clean the mesh)
		int minClusterSize = 50,           // Remove small polygon clusters after the mesh has been cleaned (0=disabled)
		ProgressState * progressState = 0);

/**
 * intersectRayTriangle(): find the 3D intersection of a ray with a triangle
 *     Input:   p = origin of the ray
 *              dir = direction of the ray
 *              v0 = point 0 of the triangle
 *              v1 = point 1 of the triangle
 *              v2 = point 2 of the triangle
 *     Output:  distance = distance from origin along ray direction
 *              normal = normal of the triangle (not normalized)
 *     Return: true = intersect in unique point inside the triangle
 *
 *     Intersection point can be computed with "I = p + dir*distance"
 *
 *  Copyright 2001 softSurfer, 2012 Dan Sunday
 *  This code may be freely used and modified for any purpose
 *  providing that this copyright notice is included with it.
 *  SoftSurfer makes no warranty for this code, and cannot be held
 *  liable for any real or imagined damage resulting from its use.
 *  Users of this code must verify correctness for their application.
 *
 *   Mathieu:  Adapted for PCL format
 */
bool RTABMAP_CORE_EXPORT intersectRayTriangle(
		const Eigen::Vector3f & p,
		const Eigen::Vector3f & dir,
		const Eigen::Vector3f & v0,
		const Eigen::Vector3f & v1,
		const Eigen::Vector3f & v2,
		float & distance,
		Eigen::Vector3f & normal);

template<typename PointT>
bool intersectRayMesh(
		const Eigen::Vector3f & origin,
		const Eigen::Vector3f & dir,
		const typename pcl::PointCloud<PointT> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		bool ignoreBackFaces,
		float & distance,
		Eigen::Vector3f & normal,
		int & index);


} // namespace util3d
} // namespace rtabmap

#include "rtabmap/core/impl/util3d_surface.hpp"

#endif /* UTIL3D_SURFACE_H_ */
