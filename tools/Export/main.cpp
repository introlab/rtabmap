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

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/common.h>
#include <pcl/surface/poisson.h>
#include <stdio.h>

#ifdef RTABMAP_PDAL
#include <rtabmap/core/PDALWriter.h>
#endif

#ifdef RTABMAP_LIBLAS
#include <rtabmap/core/LASWriter.h>
#endif

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-export [options] database.db\n"
			"  New version: we now have to explicitly set what we want to export to give a more fine-grained control.\n"
			"               At least one of these options is required:\n"
			"                   --cloud (old version used this by default)\n"
			"                   --mesh\n"
			"                   --poses\n"
			"                   --poses_camera\n"
			"                   --poses_scan\n"
			"                   --poses_gps\n"
			"                   --poses_gt\n"
			"                   --gps\n"
			"                   --images\n"
			"                   --images_id\n"
			"Options:\n"
			"    --output \"\"           Output name (default: name of the database is used).\n"
			"    --output_dir \"\"       Output directory (default: same directory than the database).\n"
			"    --ascii               Export PLY in ascii format.\n"
			"    --las                 Export cloud in LAS instead of PLY (PDAL or libLAS dependency required).\n"
			"    --cloud               Export assembled cloud.\n"
			"    --mesh                Export assembled mesh.\n"
			"    --texture             Texture the mesh. Used with --mesh option.\n"
			"    --texture_size  #     Texture size 1024, 2048, 4096, 8192, 16384 (default 8192).\n"
			"    --texture_count #     Maximum textures generated (default 1). Ignored by --multiband option (adjust --multiband_contrib instead).\n"
			"    --texture_range #     Maximum camera range for texturing a polygon (default 0 meters: no limit).\n"
			"    --texture_angle #     Maximum camera angle for texturing a polygon (default 0 deg: no limit).\n"
			"    --texture_depth_error # Maximum depth error between reprojected mesh and depth image to texture a face\n"
			"                                (-1=disabled, 0=edge length is used, default=0).\n"
			"    --texture_roi_ratios \"# # # #\" Region of interest from images to texture or to color scans. Format\n"
			"                                         is \"left right top bottom\" (e.g. \"0 0 0 0.1\" means 10%%\n"
			"                                         of the image bottom not used).\n"
			"    --texture_d2c         Distance to camera policy.\n"
			"    --texture_blur #      Motion blur threshold (default 0: disabled). Below this threshold, the image is\n"
			"                              considered blurred. 0 means disabled. 50 can be good default.\n"
			"    --cam_projection      Camera projection on assembled cloud and export node ID on each point (in PointSourceId field).\n"
			"    --cam_projection_keep_all  Keep not colored points from cameras (node ID will be 0 and color will be red).\n"
			"    --cam_projection_decimation  Decimate images before projecting the points.\n"
			"    --cam_projection_mask \"\"  File path for a mask. Format should be 8-bits grayscale. The mask should\n"
			"                                    cover all cameras in case multi-camera is used and have the same resolution.\n"
			"    --opt #               Optimization approach:\n"
			"                              0=Full Global Optimization (default)\n"
			"                              1=Iterative Global Optimization\n"
			"                              2=Use optimized poses already computed in the database instead\n"
			"                                of re-computing them (fallback to default if optimized poses don't exist).\n"
			"                              3=No optimization, use odometry poses directly.\n"
			"    --poses               Export optimized poses of the robot frame (e.g., base_link).\n"
			"    --poses_camera        Export optimized poses of the camera frame (e.g., optical frame).\n"
			"    --poses_scan          Export optimized poses of the scan frame.\n"
			"    --poses_gt            Export ground truth poses of the robot frame (e.g., base_link).\n"
			"    --poses_gps           Export GPS poses of the GPS frame in local coordinates.\n"
			"    --poses_format #      Format used for exported poses (default is 11):\n"
			"                              0=Raw 3x4 transformation matrix (r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz)\n"
			"                              1=RGBD-SLAM (in motion capture coordinate frame)\n"
			"                              2=KITTI (same as raw but in optical frame)\n"
			"                              3=TORO\n"
			"                              4=g2o\n"
			"                              10=RGBD-SLAM in ROS coordinate frame (stamp x y z qx qy qz qw)\n"
			"                              11=RGBD-SLAM in ROS coordinate frame + ID (stamp x y z qx qy qz qw id)\n"
			"    --gps #               Export GPS values of the GPS frame in world coordinates. Formats:\n"
			"                              0=Raw (stamp longitude latitude altitude error bearing)\n"
			"                              1=KML (Google Earth)\n"
			"    --images              Export images with stamp as file name.\n"
			"    --images_id           Export images with node id as file name.\n"
			"    --ba                  Do global bundle adjustment before assembling the clouds.\n"
			"    --gain          #     Gain compensation value (default 1, set 0 to disable).\n"
			"    --gain_gray           Do gain estimation compensation on gray channel only (default RGB channels).\n"
			"    --no_blending         Disable blending when texturing.\n"
			"    --no_clean            Disable cleaning colorless polygons.\n"
			"    --min_cluster   #     When meshing, filter clusters of polygons with size less than this\n"
			"                              threshold (default 200, -1 means keep only biggest contiguous surface).\n"
			"    --low_gain      #     Low brightness gain 0-100 (default 0).\n"
			"    --high_gain     #     High brightness gain 0-100 (default 10).\n"
			"    --multiband               Enable multiband texturing (AliceVision dependency required). Used with --texture option.\n"
			"    --multiband_downscale #   Downscaling reduce the texture quality but speed up the computation time (default 2).\n"
			"    --multiband_contrib \"# # # # \"  Number of contributions per frequency band for the\n"
			"                                          multi-band blending, should be 4 values! (default \"1 5 10 0\").\n"
			"    --multiband_unwrap #      Method to unwrap input mesh:\n"
			"                                  0=basic (default, >600k faces, fast)\n"
			"                                  1=ABF (<=300k faces, generate 1 atlas)\n"
			"                                  2=LSCM (<=600k faces, optimize space).\n"
			"    --multiband_fillholes     Fill Texture holes with plausible values.\n"
			"    --multiband_padding #     Texture edge padding size in pixel (0-100) (default 5).\n"
			"    --multiband_scorethr #    0 to disable filtering based on threshold to relative best score (0.0-1.0). (default 0.1).\n"
			"    --multiband_anglethr #    0 to disable angle hard threshold filtering (0.0, 180.0) (default 90.0).\n"
			"    --multiband_forcevisible  Triangle visibility is based on the union of vertices visibility.\n"
			"    --poisson_depth #     Set Poisson depth for mesh reconstruction.\n"
			"    --poisson_size  #     Set target polygon size when computing Poisson's depth for mesh reconstruction (default 0.03 m).\n"
			"    --max_polygons  #     Maximum polygons when creating a mesh (default 300000, set 0 for no limit).\n"
			"    --min_range     #     Minimum range of the created clouds (default 0 m).\n"
			"    --max_range     #     Maximum range of the created clouds (default 4 m, 0 m with --scan).\n"
			"    --decimation    #     Depth image decimation before creating the clouds (default 4, 1 with --scan).\n"
			"    --voxel         #     Voxel size of the created clouds (default 0.01 m, 0 m with --scan).\n"
			"    --ground_normals_up  #  Flip ground normals up if close to -z axis (default 0, 0=disabled, value should be >0 and <1, typical 0.9).\n"
			"    --noise_radius  #     Noise filtering search radius (default 0, 0=disabled).\n"
			"    --noise_k       #     Noise filtering minimum neighbors in search radius (default 5, 0=disabled).\n"
			"    --prop_radius_factor #  Proportional radius filter factor (default 0, 0=disabled). Start tuning from 0.01.\n"
			"    --prop_radius_scale  #  Proportional radius filter neighbor scale (default 2).\n"
			"    --random_samples #    Number of output samples using a random filter (default 0, 0=disabled).\n"
			"    --color_radius  #     Radius used to colorize polygons (default 0.05 m, 0 m with --scan). Set 0 for nearest color.\n"
			"    --scan                Use laser scan for the point cloud.\n"
			"    --save_in_db          Save resulting assembled point cloud or mesh in the database.\n"
			"    --xmin #              Minimum range on X axis to keep nodes to export.\n"
			"    --xmax #              Maximum range on X axis to keep nodes to export.\n"
			"    --ymin #              Minimum range on Y axis to keep nodes to export.\n"
			"    --ymax #              Maximum range on Y axis to keep nodes to export.\n"
			"    --zmin #              Minimum range on Z axis to keep nodes to export.\n"
			"    --zmax #              Maximum range on Z axis to keep nodes to export.\n"
			"    --density_radius #    Filter poses in a fixed radius (m) to keep only one to be exported in the assembled cloud.\n"
			"    --density_angle #     Filter poses up to angle (deg) in the --density_radius.\n"
			"    --filter_ceiling #    Filter points over a custom height (default 0 m, 0=disabled).\n"
			"    --filter_floor #      Filter points below a custom height (default 0 m, 0=disabled).\n"

			"\n%s", Parameters::showUsage());
	;
	exit(1);
}

class ConsoleProgessState : public ProgressState
{
	virtual bool callback(const std::string & msg) const
	{
		if(!msg.empty())
			printf("%s\n", msg.c_str());
		return true;
	}
};

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

	if(argc < 2)
	{
		showUsage();
	}

	bool binary = true;
	bool las = false;
	bool exportCloud = false;
	bool exportMesh = false;
	bool texture = false;
	bool ba = false;
	bool doGainCompensationRGB = true;
	float gainValue = 1;
	bool doBlending = true;
	bool doClean = true;
	int minCluster = 200;
	int poissonDepth = 0;
	float poissonSize = 0.03;
	int maxPolygons = 300000;
	int decimation = -1;
	float minRange = 0.0f;
	float maxRange = -1.0f;
	float voxelSize = -1.0f;
	float groundNormalsUp = 0.0f;
	float noiseRadius = 0.0f;
	int noiseMinNeighbors = 5;
	float proportionalRadiusFactor = 0.0f;
	float proportionalRadiusScale = 2.0f;
	int randomSamples = 0;
	int textureSize = 8192;
	int textureCount = 1;
	float textureRange = 0;
	float textureAngle = 0;
	float textureDepthError = 0;
	std::vector<float> textureRoiRatios;
	bool distanceToCamPolicy = false;
	int laplacianThr = 0;
	bool multiband = false;
	int multibandDownScale = 2;
	std::string multibandNbContrib = "1 5 10 0";
	int multibandUnwrap = 0;
	bool multibandFillHoles = false;
	int multibandPadding = 5;
	double multibandBestScoreThr = 0.1;
	double multibandAngleHardthr = 90;
	bool multibandForceVisible = false;
	float colorRadius = -1.0f;
	bool cloudFromScan = false;
	bool saveInDb = false;
	int lowBrightnessGain = 0;
	int highBrightnessGain = 10;
	bool camProjection = false;
	bool camProjectionKeepAll = false;
	int cameraProjDecimation = 1;
	std::string cameraProjMask;
	bool exportPoses = false;
	bool exportPosesCamera = false;
	bool exportPosesScan = false;
	bool exportPosesGt = false;
	bool exportPosesGps = false;
	int exportPosesFormat = 11;
	int exportGps = -1;
	bool exportImages = false;
	bool exportImagesId = false;
	int optimizationApproach = 0;
	std::string outputName;
	std::string outputDir;
	cv::Vec3f min, max;
	float densityRadius = 0.0f;
	float densityAngle = 0.0f;
	float filter_ceiling = 0.0f;
	float filter_floor = 0.0f;
	for(int i=1; i<argc; ++i)
	{
		if(std::strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
		else if(std::strcmp(argv[i], "--output") == 0)
		{
			++i;
			if(i<argc-1)
			{
				outputName = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--output_dir") == 0)
		{
			++i;
			if(i<argc-1)
			{
				outputDir = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--bin") == 0)
		{
			printf("No need to set --bin anymore, ply are now automatically exported in binary by default. Set --ascii to export as text.\n");
		}
		else if(std::strcmp(argv[i], "--ascii") == 0)
		{
			binary = false;
		}
		else if(std::strcmp(argv[i], "--las") == 0)
		{
#ifdef RTABMAP_PDAL
			las = true;
#elif defined(RTABMAP_LIBLAS)
			printf("\"--las\" option cannot be used with libLAS because the cloud has normals, build RTAB-Map with PDAL support to export in las with normals. Will export in PLY...\n");
#else
			printf("\"--las\" option cannot be used because RTAB-Map is not built with PDAL support. Will export in PLY...\n");
#endif

		}
		else if(std::strcmp(argv[i], "--cloud") == 0)
		{
			exportCloud = true;
		}
		else if(std::strcmp(argv[i], "--mesh") == 0)
		{
			exportMesh = true;
		}
		else if(std::strcmp(argv[i], "--texture") == 0)
		{
			texture = true;
		}
		else if(std::strcmp(argv[i], "--texture_size") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureSize = uStr2Int(argv[i]);
				UASSERT(textureSize%256==0);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_count") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureCount = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_range") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureRange = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_angle") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureAngle = uStr2Float(argv[i])*M_PI/180.0f;
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_depth_error") == 0)
		{
			++i;
			if(i<argc-1)
			{
				textureDepthError = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_roi_ratios") == 0)
		{
			++i;
			if(i<argc-1)
			{
				std::list<std::string> strValues = uSplit(argv[i], ' ');
				if(strValues.size() != 4)
				{
					printf("The number of values must be 4 (roi=\"%s\")\n", argv[i]);
					showUsage();
				}
				else
				{
					std::vector<float> tmpValues(4);
					unsigned int i=0;
					for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
					{
						tmpValues[i] = uStr2Float(*jter);
						++i;
					}

					if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
						tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
						tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
						tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
					{
						textureRoiRatios = tmpValues;
					}
					else
					{
						printf("The roi ratios are not valid (roi=\"%s\")\n", argv[i]);
						showUsage();
					}
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--texture_d2c") == 0)
		{
			distanceToCamPolicy = true;
		}
		else if(std::strcmp(argv[i], "--texture_blur") == 0)
		{
			++i;
			if(i<argc-1)
			{
				laplacianThr = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--cam_projection") == 0)
		{
			camProjection = true;
		}
		else if(std::strcmp(argv[i], "--cam_projection_keep_all") == 0)
		{
			camProjectionKeepAll = true;
		}
		else if(std::strcmp(argv[i], "--cam_projection_decimation") == 0)
		{
			++i;
			if(i<argc-1)
			{
				cameraProjDecimation = uStr2Int(argv[i]);
				if(cameraProjDecimation<1)
				{
					printf("--cam_projection_decimation cannot be <1! value=\"%s\"\n", argv[i]);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--cam_projection_mask") == 0)
		{
			++i;
			if(i<argc-1)
			{
				cameraProjMask = argv[i];
				if(!UFile::exists(cameraProjMask))
				{
					printf("--cam_projection_mask is set with a file not existing or don't have permissions to open it. Path=\"%s\"\n", argv[i]);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--poses") == 0)
		{
			exportPoses = true;
		}
		else if(std::strcmp(argv[i], "--poses_camera") == 0)
		{
			exportPosesCamera = true;
		}
		else if(std::strcmp(argv[i], "--poses_scan") == 0)
		{
			exportPosesScan = true;
		}
		else if(std::strcmp(argv[i], "--poses_gt") == 0)
		{
			exportPosesGt = true;
		}
		else if(std::strcmp(argv[i], "--poses_gps") == 0)
		{
			exportPosesGps = true;
		}
		else if(std::strcmp(argv[i], "--poses_format") == 0)
		{
			++i;
			if(i<argc-1)
			{
				exportPosesFormat = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--gps") == 0)
		{
			++i;
			if(i<argc-1)
			{
				exportGps = uStr2Int(argv[i]);
				if(exportGps<0 || exportGps>1)
				{
					printf("Wrong GPS format (%d), should be 0 or 1\n", exportGps);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--opt") == 0)
		{
			++i;
			if(i<argc-1)
			{
				optimizationApproach = uStr2Int(argv[i]);
				if(optimizationApproach<0 || optimizationApproach >3)
				{
					printf("Invalid --opt (%d)\n", optimizationApproach);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--images") == 0)
		{
			exportImages = true;
		}
		else if(std::strcmp(argv[i], "--images_id") == 0)
		{
			exportImages = true;
			exportImagesId = true;
		}
		else if(std::strcmp(argv[i], "--ba") == 0)
		{
			ba = true;
		}
		else if(std::strcmp(argv[i], "--gain_gray") == 0)
		{
			doGainCompensationRGB = false;
		}
		else if(std::strcmp(argv[i], "--gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				gainValue = uStr2Float(argv[i]);
				UASSERT(gainValue>=0.0f);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--no_blending") == 0)
		{
			doBlending = false;
		}
		else if(std::strcmp(argv[i], "--no_clean") == 0)
		{
			doClean = false;
		}
		else if(std::strcmp(argv[i], "--min_cluster") == 0)
		{
			++i;
			if(i<argc-1)
			{
				minCluster = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband") == 0)
		{
#ifdef RTABMAP_ALICE_VISION
			multiband = true;
#else
			printf("\"--multiband\" option cannot be used because RTAB-Map is not built with AliceVision support. Ignoring multiband...\n");
#endif
		}
		else if(std::strcmp(argv[i], "--multiband_fillholes") == 0)
		{
			multibandFillHoles = true;
		}
		else if(std::strcmp(argv[i], "--multiband_downscale") == 0)
		{
			++i;
			if(i<argc-1)
			{
				multibandDownScale = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband_contrib") == 0)
		{
			++i;
			if(i<argc-1)
			{
				if(uSplit(argv[i], ' ').size() != 4)
				{
					printf("--multiband_contrib has wrong format! value=\"%s\"\n", argv[i]);
					showUsage();
				}
				multibandNbContrib = argv[i];
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband_unwrap") == 0)
		{
			++i;
			if(i<argc-1)
			{
				multibandUnwrap = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband_padding") == 0)
		{
			++i;
			if(i<argc-1)
			{
				multibandPadding = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband_forcevisible") == 0)
		{
			multibandForceVisible = true;
		}
		else if(std::strcmp(argv[i], "--multiband_scorethr") == 0)
		{
			++i;
			if(i<argc-1)
			{
				multibandBestScoreThr = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--multiband_anglethr") == 0)
		{
			++i;
			if(i<argc-1)
			{
				multibandAngleHardthr = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--poisson_depth") == 0)
		{
			++i;
			if(i<argc-1)
			{
				poissonDepth = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--poisson_size") == 0)
		{
			++i;
			if(i<argc-1)
			{
				poissonSize = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--max_polygons") == 0)
		{
			++i;
			if(i<argc-1)
			{
				maxPolygons = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--min_range") == 0)
		{
			++i;
			if(i<argc-1)
			{
				minRange = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--max_range") == 0)
		{
			++i;
			if(i<argc-1)
			{
				maxRange = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--decimation") == 0)
		{
			++i;
			if(i<argc-1)
			{
				decimation = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--voxel") == 0)
		{
			++i;
			if(i<argc-1)
			{
				voxelSize = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--ground_normals_up") == 0)
		{
			++i;
			if(i<argc-1)
			{
				groundNormalsUp = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--noise_radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				noiseRadius = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--noise_k") == 0)
		{
			++i;
			if(i<argc-1)
			{
				noiseMinNeighbors = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--prop_radius_factor") == 0)
		{
			++i;
			if(i<argc-1)
			{
				proportionalRadiusFactor = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--prop_radius_scale") == 0)
		{
			++i;
			if(i<argc-1)
			{
				proportionalRadiusScale = uStr2Float(argv[i]);
				UASSERT_MSG(proportionalRadiusScale>=1.0f, "--prop_radius_scale should be >= 1.0");
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--random_samples") == 0)
		{
			++i;
			if(i<argc-1)
			{
				randomSamples = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--color_radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				colorRadius = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--scan") == 0)
		{
			cloudFromScan = true;
		}
		else if(std::strcmp(argv[i], "--save_in_db") == 0)
		{
			saveInDb = true;
		}
		else if(std::strcmp(argv[i], "--low_gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				lowBrightnessGain = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--high_gain") == 0)
		{
			++i;
			if(i<argc-1)
			{
				highBrightnessGain = uStr2Int(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--xmin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[0] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--xmax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[0] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--ymin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[1] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--ymax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[1] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--zmin") == 0)
		{
			++i;
			if(i<argc-1)
			{
				min[2] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--zmax") == 0)
		{
			++i;
			if(i<argc-1)
			{
				max[2] = uStr2Float(argv[i]);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--density_radius") == 0)
		{
			++i;
			if(i<argc-1)
			{
				densityRadius = uStr2Float(argv[i]);
				UASSERT(densityRadius>=0.0f);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--density_angle") == 0)
		{
			++i;
			if(i<argc-1)
			{
				densityAngle = uStr2Float(argv[i]);
				UASSERT(densityAngle>=0.0f);
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--filter_ceiling") == 0)
		{
			++i;
			if(i<argc-1)
			{
				filter_ceiling = uStr2Float(argv[i]);
				if(filter_floor!=0.0f && filter_ceiling != 0.0f && filter_ceiling<filter_floor)
				{
					printf("Option --filter_ceiling (%f) should be higher than --filter_floor option (%f)!\n", filter_ceiling, filter_floor);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
		else if(std::strcmp(argv[i], "--filter_floor") == 0)
		{
			++i;
			if(i<argc-1)
			{
				filter_floor = uStr2Float(argv[i]);
				if(filter_floor!=0.0f && filter_ceiling != 0.0f && filter_ceiling<filter_floor)
				{
					printf("Option --filter_ceiling (%f) should be higher than --filter_floor option (%f)!\n", filter_ceiling, filter_floor);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
		}
	}

	if(decimation < 1)
	{
		decimation = cloudFromScan?1:4;
	}
	if(maxRange < 0)
	{
		maxRange = cloudFromScan?0:4;
	}
	if(voxelSize < 0.0f)
	{
		voxelSize = cloudFromScan?0:0.01f;
	}
	if(colorRadius < 0.0f)
	{
		colorRadius = cloudFromScan?0:0.05f;
	}

	if(saveInDb)
	{
		if(multiband)
		{
			printf("Option --multiband is not supported with --save_in_db option, disabling multiband...\n");
			multiband = false;
		}
		if(textureCount>1)
		{
			printf("Option --texture_count > 1 is not supported with --save_in_db option, setting texture_count to 1...\n");
			textureCount = 1;
		}
	}

	if(!(exportCloud ||
		 exportMesh ||
		 exportImages ||
		 exportPoses ||
		 exportPosesScan ||
		 exportPosesCamera ||
		 exportPosesGt ||
		 exportPosesGps ||
		 exportGps>=0 ||
		 texture))
	{
		printf("Launching the tool without any required option(s) is deprecated. We will add --cloud to keep compatibilty with old behavior.\n");
		exportCloud = true;
	}

	if(texture && !exportMesh)
	{
		printf("To use --texture option, --mesh should be also enabled. Enabling --mesh.\n");
		exportMesh = true;
	}

	ParametersMap params = Parameters::parseArguments(argc, argv, false);

	std::string dbPath = argv[argc-1];

	if(!UFile::exists(dbPath))
	{
		UERROR("File \"%s\" doesn't exist!", dbPath.c_str());
		return -1;
	}

	// Get parameters
	ParametersMap parameters;
	DBDriver * driver = DBDriver::create();
	if(driver->openConnection(dbPath))
	{
		parameters = driver->getLastParameters();
		driver->closeConnection(false);
	}
	else
	{
		UERROR("Cannot open database %s!", dbPath.c_str());
		return -1;
	}
	delete driver;
	driver = 0;

	for(ParametersMap::iterator iter=params.begin(); iter!=params.end(); ++iter)
	{
		printf("Added custom parameter %s=%s\n",iter->first.c_str(), iter->second.c_str());
	}

	UTimer timer;

	printf("Opening database \"%s\"...\n", dbPath.c_str());
	uInsert(parameters, params);
	std::shared_ptr<DBDriver> dbDriver(DBDriver::create(parameters));
	if(!dbDriver->openConnection(dbPath))
	{
		printf("Failed to open database \"%s\"!\n", dbPath.c_str());
		return -1;
	}
	printf("Opening database \"%s\"... done (%fs).\n", dbPath.c_str(), timer.ticks());

	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> odomPoses;
	std::multimap<int, Link> links;
	dbDriver->getAllOdomPoses(odomPoses, true);
	dbDriver->getAllLinks(links, true, true);
	if(optimizationApproach == 3 || !(exportCloud || exportMesh || exportPoses || exportPosesCamera || exportPosesScan))
	{
		// Just use odometry poses when exporting only images
		optimizedPoses = odomPoses;
		if(optimizationApproach == 3)
		{
			printf("Loaded %d odometry poses from database.\n", (int)odomPoses.size());
		}
	}
	else
	{
		if(optimizationApproach == 2)
		{
			printf("Loading optimized poses from database...\n");
			optimizedPoses = dbDriver->loadOptimizedPoses();
			if(optimizedPoses.empty())
			{
				printf("The are no saved optimized poses in the database, we will do full global optimization instead.\n");
				optimizationApproach = 0;
			}
			else
			{
				printf("Loading optimized poses from database... done (%d optimized poses loaded).\n", (int)optimizedPoses.size());
			}
		}
		if(optimizationApproach <= 1)
		{
			std::string optimizationApproachStr = optimizationApproach==1?"Iterative global optimization":"Full global optimization";
			printf("Optimizing the map (%s)...\n", optimizationApproachStr.c_str());
			if(odomPoses.empty())
			{
				printf("The are no odometry poses!? Aborting...\n");
				return -1;
			}
			std::shared_ptr<Optimizer> optimizer(Optimizer::create(parameters));
			std::map<int, Transform> posesOut;
			std::multimap<int, Link> linksOut;
			UASSERT(odomPoses.lower_bound(1) != odomPoses.end());

			// Add landmarks if there are some
			// Marker priors parameters
			double markerPriorsLinearVariance = Parameters::defaultMarkerPriorsVarianceLinear();
			double markerPriorsAngularVariance = Parameters::defaultMarkerPriorsVarianceAngular();
			std::map<int, Transform> markerPriors;
			Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceLinear(), markerPriorsLinearVariance);
			UASSERT(markerPriorsLinearVariance>0.0f);
			Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceAngular(), markerPriorsAngularVariance);
			UASSERT(markerPriorsAngularVariance>0.0f);
			std::string markerPriorsStr;
			if(Parameters::parse(parameters, Parameters::kMarkerPriors(), markerPriorsStr))
			{
				std::list<std::string> strList = uSplit(markerPriorsStr, '|');
				for(std::list<std::string>::iterator iter=strList.begin(); iter!=strList.end(); ++iter)
				{
					std::string markerStr = *iter;
					while(!markerStr.empty() && !uIsDigit(markerStr[0]))
					{
						markerStr.erase(markerStr.begin());
					}
					if(!markerStr.empty())
					{
						std::string idStr = uSplitNumChar(markerStr).front();
						int id = uStr2Int(idStr);
						Transform prior = Transform::fromString(markerStr.substr(idStr.size()));
						if(!prior.isNull() && id>0)
						{
							markerPriors.insert(std::make_pair(-id, prior));
						}
						else
						{
							UERROR("Failed to parse element \"%s\" in parameter %s", markerStr.c_str(), Parameters::kMarkerPriors().c_str());
						}
					}
					else if(!iter->empty())
					{
						UERROR("Failed to parse parameter %s, value=\"%s\"", Parameters::kMarkerPriors().c_str(), iter->c_str());
					}
				}
			}
			for(std::multimap<int, rtabmap::Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
			{
				if(iter->second.type() == Link::kLandmark)
				{
					UASSERT(iter->second.from() > 0 && iter->second.to() < 0);
					int markerId = iter->second.to();
					if(odomPoses.find(iter->second.from()) != odomPoses.end() && odomPoses.find(markerId) == odomPoses.end())
					{
						odomPoses.insert(std::make_pair(markerId, odomPoses.at(iter->second.from())*iter->second.transform()));
						// add landmark priors if there are some
						if(markerPriors.find(markerId) != markerPriors.end())
						{
							cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
							infMatrix(cv::Range(0,3), cv::Range(0,3)) /= markerPriorsLinearVariance;
							infMatrix(cv::Range(3,6), cv::Range(3,6)) /= markerPriorsAngularVariance;
							links.insert(std::make_pair(markerId, Link(markerId, markerId, Link::kPosePrior, markerPriors.at(markerId), infMatrix)));
							printf("Added prior %d : %s (variance: lin=%f ang=%f)\n", markerId, markerPriors.at(markerId).prettyPrint().c_str(),
									markerPriorsLinearVariance, markerPriorsAngularVariance);
						}
					}
				}
			}


			optimizer->getConnectedGraph(odomPoses.lower_bound(1)->first, odomPoses, links, posesOut, linksOut);
			if(optimizationApproach == 1)
			{
				std::list<std::map<int, Transform> > intermediateGraphes;
				optimizedPoses = optimizer->optimize(odomPoses.lower_bound(1)->first, posesOut, linksOut, &intermediateGraphes);
			}
			else
			{
				optimizedPoses = optimizer->optimize(odomPoses.lower_bound(1)->first, posesOut, linksOut);
			}
			printf("Optimizing the map (%s)... done (%fs, poses=%d, links=%d).\n", optimizationApproachStr.c_str(), timer.ticks(), (int)optimizedPoses.size(), (int)linksOut.size());
		}

		if(optimizedPoses.empty())
		{
			printf("The optimized graph is empty!? Aborting...\n");
			return -1;
		}

		if(min[0] != max[0] || min[1] != max[1] || min[2] != max[2])
		{
			cv::Vec3f minP,maxP;
			graph::computeMinMax(optimizedPoses, minP, maxP);
			printf("Filtering poses (range: x=%.1f<->%.1f, y=%.1f<->%.1f, z=%.1f<->%.1f, map size=%.1f x %.1f x %.1f, map min/max: [%.1f, %.1f, %.1f] [%.1f, %.1f, %.1f])...\n",
					min[0],max[0],min[1],max[1],min[2],max[2],
					maxP[0]-minP[0],maxP[1]-minP[1],maxP[2]-minP[2],
					minP[0],minP[1],minP[2],maxP[0],maxP[1],maxP[2]);
			std::map<int, Transform> posesFiltered;
			for(std::map<int, Transform>::const_iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
			{
				bool ignore = false;
				if(min[0] != max[0] && (iter->second.x() < min[0] || iter->second.x() > max[0]))
				{
					ignore = true;
				}
				if(min[1] != max[1] && (iter->second.y() < min[1] || iter->second.y() > max[1]))
				{
					ignore = true;
				}
				if(min[2] != max[2] && (iter->second.z() < min[2] || iter->second.z() > max[2]))
				{
					ignore = true;
				}
				if(!ignore)
				{
					posesFiltered.insert(*iter);
				}
			}
			graph::computeMinMax(posesFiltered, minP, maxP);
			printf("Filtering poses... done! %d/%d remaining.\n", (int)posesFiltered.size(), (int)optimizedPoses.size());
			optimizedPoses = posesFiltered;
			if(optimizedPoses.empty())
			{
				printf("All poses filtered! Exiting.\n");
				return -1;
			}
		}

		if(ba)
		{
			printf("Global bundle adjustment...\n");
			// TODO: these conversions could be simplified
			UASSERT(optimizedPoses.lower_bound(1) != optimizedPoses.end());
			OptimizerG2O g2o(parameters);
			std::list<int> ids;
			for(std::map<int, Transform>::iterator iter=optimizedPoses.lower_bound(1); iter!=optimizedPoses.end(); ++iter)
			{
				ids.push_back(iter->first);
			}
			std::list<Signature *> signatures;
			dbDriver->loadSignatures(ids, signatures);
			std::map<int, Signature> nodes;
			for(std::list<Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
			{
				nodes.insert(std::make_pair((*iter)->id(), *(*iter)));
			}
			optimizedPoses = ((Optimizer*)&g2o)->optimizeBA(optimizedPoses.lower_bound(1)->first, optimizedPoses, links, nodes, true);
			printf("Global bundle adjustment... done (%fs).\n", timer.ticks());
		}
	}

	std::string outputDirectory = outputDir.empty()?UDirectory::getDir(dbPath):outputDir;
	if(!UDirectory::exists(outputDirectory))
	{
		UDirectory::makeDir(outputDirectory);
	}
	std::string baseName = outputName.empty()?uSplit(UFile::getName(dbPath), '.').front():outputName;

	// Construct the cloud
	if(exportCloud || exportMesh)
	{
		printf("Create and assemble the clouds...\n");
	}
	else if(exportImages || exportImagesId)
	{
		printf("Export images...\n");
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr assembledCloudI(new pcl::PointCloud<pcl::PointXYZI>);
	std::map<int, rtabmap::Transform> robotPoses;
	std::vector<std::map<int, rtabmap::Transform> > cameraPoses;
	std::map<int, rtabmap::Transform> scanPoses;
	std::map<int, rtabmap::Transform> gtPoses;
	std::map<int, rtabmap::Transform> gpsPoses;
	std::map<int, double> gpsStamps;
	GPS gpsOrigin;
	std::map<int, rtabmap::GPS> gpsValues;
	std::map<int, double> cameraStamps;
	std::map<int, std::vector<rtabmap::CameraModel> > cameraModels;
	std::map<int, cv::Mat> cameraDepths;
	int imagesExported = 0;
	std::vector<int> rawViewpointIndices;
	std::map<int, Transform> rawViewpoints;
	std::map<int, Transform> densityPoses;
	if(densityRadius && (exportCloud || exportMesh))
	{
		densityPoses = graph::radiusPosesFiltering(optimizedPoses, densityRadius, densityAngle*CV_PI/180.0f);
		printf("Keeping %d/%d poses after density filtering (--density_radius = %f --density_angle = %f).\n",
			(int)densityPoses.size(),
			(int)optimizedPoses.size(),
			densityRadius,
			densityAngle);
	}
	int processedNodes = 0;
	int lastPercent = 0;
	for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
	{
		if(iter->first<0)
		{
			// landmark, just add to list of poses
			robotPoses.insert(*iter);
			cameraStamps.insert(std::make_pair(iter->first, 0));
			continue;
		}
		
		Transform p, gt;
		int m;
		std::string l;
		GPS gps;
		std::vector<float> v;
		EnvSensors s;
		int weight = -1;
		double stamp = 0.0;
		dbDriver->getNodeInfo(iter->first, p, m, weight, l, stamp, gt, v, gps, s);

		SensorData data;
		bool loadImages = ((exportCloud || exportMesh) && (!cloudFromScan || texture || camProjection)) || exportImages;
		bool loadScan = ((exportCloud || exportMesh) && cloudFromScan) || exportPosesScan;
		if(loadImages || loadScan)
		{
			dbDriver->getNodeData(
				iter->first, 
				data, 
				loadImages, 
				loadScan,
				false,
				false);
		}

		// uncompress data
		std::vector<CameraModel> models;
		std::vector<StereoCameraModel> stereoModels;
		if(loadImages || exportPosesCamera)
		{
			dbDriver->getCalibration(iter->first, models, stereoModels);
		}

		if(exportCloud || exportMesh || exportImages)
		{
			bool densityFiltered = !densityPoses.empty() && densityPoses.find(iter->first) == densityPoses.end();
			cv::Mat rgb;
			cv::Mat depth;
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI;
			if(weight != -1)
			{
				if(!densityFiltered && cloudFromScan && (exportCloud || exportMesh))
				{
					LaserScan scan;
					data.uncompressData(exportImages?&rgb:0, (texture||exportImages)&&!data.depthOrRightCompressed().empty()?&depth:0, &scan);
					if(scan.empty())
					{
						printf("Node %d doesn't have scan data, empty cloud is created.\n", iter->first);
					}
					if(decimation>1 || minRange>0.0f || maxRange)
					{
						scan = util3d::commonFiltering(scan, decimation, minRange, maxRange);
					}
					if(scan.hasRGB())
					{
						cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
						if(noiseRadius>0.0f && noiseMinNeighbors>0)
						{
							indices = util3d::radiusFiltering(cloud, noiseRadius, noiseMinNeighbors);
						}
					}
					else
					{
						cloudI = util3d::laserScanToPointCloudI(scan, scan.localTransform());
						if(noiseRadius>0.0f && noiseMinNeighbors>0)
						{
							indices = util3d::radiusFiltering(cloudI, noiseRadius, noiseMinNeighbors);
						}
					}
				}
				else
				{
					data.uncompressData(&rgb, &depth);
					if(!densityFiltered && (exportCloud || exportMesh))
					{
						if(depth.empty())
						{
							printf("Node %d doesn't have depth or stereo data, empty cloud is "
									"created (if you want to create point cloud from scan, use --scan option).\n", iter->first);
						}
						cloud = util3d::cloudRGBFromSensorData(
								data,
								decimation,      // image decimation before creating the clouds
								maxRange,        // maximum depth of the cloud
								minRange,
								indices.get());
						if(noiseRadius>0.0f && noiseMinNeighbors>0)
						{
							indices = util3d::radiusFiltering(cloud, indices, noiseRadius, noiseMinNeighbors);
						}
					}
				}
			}

			if(exportImages && !rgb.empty())
			{
				std::string dirSuffix = (depth.type() != CV_16UC1 && depth.type() != CV_32FC1 && !depth.empty())?"left":"rgb";
				std::string dir = outputDirectory+"/"+baseName+"_"+dirSuffix;
				if(!UDirectory::exists(dir)) {
					UDirectory::makeDir(dir);
				}
				std::string outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f", stamp))+".jpg";
				cv::imwrite(outputPath, rgb);
				++imagesExported;
				if(!depth.empty())
				{
					std::string ext;
					cv::Mat depthExported = depth;
					if(depth.type() != CV_16UC1 && depth.type() != CV_32FC1)
					{
						ext = ".jpg";
						dir = outputDirectory+"/"+baseName+"_right";
					}
					else
					{
						ext = ".png";
						dir = outputDirectory+"/"+baseName+"_depth";
						if(depth.type() == CV_32FC1)
						{
							depthExported = rtabmap::util2d::cvtDepthFromFloat(depth);
						}
					}
					if(!UDirectory::exists(dir)) {
						UDirectory::makeDir(dir);
					}

					outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f", stamp))+ext;
					cv::imwrite(outputPath, depthExported);
				}

				// save calibration per image (calibration can change over time, e.g. camera has auto focus)
				for(size_t i=0; i<models.size(); ++i)
				{
					CameraModel model = models[i];
					std::string modelName = (exportImagesId?uNumber2Str(iter->first):uFormat("%f", stamp));
					if(models.size() > 1) {
						modelName += "_" + uNumber2Str((int)i);
					}
					model.setName(modelName);
					std::string dir = outputDirectory+"/"+baseName+"_calib";
					if(!UDirectory::exists(dir)) {
						UDirectory::makeDir(dir);
					}
					model.save(dir);
				}
				for(size_t i=0; i<stereoModels.size(); ++i)
				{
					StereoCameraModel model = stereoModels[i];
					std::string modelName = (exportImagesId?uNumber2Str(iter->first):uFormat("%f", stamp));
					if(stereoModels.size() > 1) {
						modelName += "_" + uNumber2Str((int)i);
					}
					model.setName(modelName, "left", "right");
					std::string dir = outputDirectory+"/"+baseName+"_calib";
					if(!UDirectory::exists(dir)) {
						UDirectory::makeDir(dir);
					}
					model.save(dir);
				}
			}

			if(exportCloud || exportMesh)
			{
				if(voxelSize>0.0f)
				{
					if(cloud.get() && !cloud->empty())
						cloud = rtabmap::util3d::voxelize(cloud, indices, voxelSize);
					else if(cloudI.get() && !cloudI->empty())
						cloudI = rtabmap::util3d::voxelize(cloudI, indices, voxelSize);
				}
				if(cloud.get() && !cloud->empty())
					cloud = rtabmap::util3d::transformPointCloud(cloud, iter->second);
				else if(cloudI.get() && !cloudI->empty())
					cloudI = rtabmap::util3d::transformPointCloud(cloudI, iter->second);

				if(filter_ceiling != 0.0 || filter_floor != 0.0f)
				{
					if(cloud.get() && !cloud->empty())
					{
						cloud = util3d::passThrough(cloud, "z", filter_floor!=0.0f?filter_floor:(float)std::numeric_limits<int>::min(), filter_ceiling!=0.0f?filter_ceiling:(float)std::numeric_limits<int>::max());
					}
					if(cloudI.get() && !cloudI->empty())
					{
						cloudI = util3d::passThrough(cloudI, "z", filter_floor!=0.0f?filter_floor:(float)std::numeric_limits<int>::min(), filter_ceiling!=0.0f?filter_ceiling:(float)std::numeric_limits<int>::max());
					}
				}

				if(cloudFromScan)
				{
					Transform lidarViewpoint = iter->second * data.laserScanRaw().localTransform();
					rawViewpoints.insert(std::make_pair(iter->first, lidarViewpoint));
				}
				else if(!models.empty() && !models[0].localTransform().isNull())
				{
					Transform cameraViewpoint = iter->second * models[0].localTransform(); // take the first camera
					rawViewpoints.insert(std::make_pair(iter->first, cameraViewpoint));
				}
				else if(!stereoModels.empty() && !stereoModels[0].localTransform().isNull())
				{
					Transform cameraViewpoint = iter->second * stereoModels[0].localTransform();
					rawViewpoints.insert(std::make_pair(iter->first, cameraViewpoint));
				}
				else
				{
					rawViewpoints.insert(*iter);
				}

				if(cloud.get() && !cloud->empty())
				{
					if(assembledCloud->empty())
					{
						*assembledCloud = *cloud;
					}
					else
					{
						*assembledCloud += *cloud;
					}
					rawViewpointIndices.resize(assembledCloud->size(), iter->first);
				}
				else if(cloudI.get() && !cloudI->empty())
				{
					if(assembledCloudI->empty())
					{
						*assembledCloudI = *cloudI;
					}
					else
					{
						*assembledCloudI += *cloudI;
					}
					rawViewpointIndices.resize(assembledCloudI->size(), iter->first);
				}
				if(texture && !depth.empty() && (depth.type() == CV_16UC1 || depth.type() == CV_32FC1))
				{
					cameraDepths.insert(std::make_pair(iter->first, depth));
				}
			}
		}

		if(models.empty())
		{
			for(size_t i=0; i<stereoModels.size(); ++i)
			{
				models.push_back(stereoModels[i].left());
			}
		}

		robotPoses.insert(std::make_pair(iter->first, iter->second));
		cameraStamps.insert(std::make_pair(iter->first, stamp));
		if(models.empty() && weight == -1 && !cameraModels.empty())
		{
			// For intermediate nodes, use latest models
			models = cameraModels.rbegin()->second;
		}
		if(!models.empty())
		{
			if(!data.imageCompressed().empty())
			{
				cameraModels.insert(std::make_pair(iter->first, models));
			}
			if(exportPosesCamera)
			{
				if(cameraPoses.empty())
				{
					cameraPoses.resize(models.size());
				}
				UASSERT_MSG(models.size() == cameraPoses.size(), "Not all nodes have same number of cameras to export camera poses.");
				for(size_t i=0; i<models.size(); ++i)
				{
					cameraPoses[i].insert(std::make_pair(iter->first, iter->second*models[i].localTransform()));
				}
			}
		}
		if(exportPosesScan && !data.laserScanCompressed().empty())
		{
			scanPoses.insert(std::make_pair(iter->first, iter->second*data.laserScanCompressed().localTransform()));
		}

		if(exportPosesGps || exportGps>=0)
		{
			if(gps.stamp() > 0.0)
			{
				if(exportPosesGps)
				{
					cv::Point3f p(0.0f,0.0f,0.0f);
					if(!gpsPoses.empty())
					{
						GeodeticCoords coords = gps.toGeodeticCoords();
						p = coords.toENU_WGS84(gpsOrigin.toGeodeticCoords());
					}
					else
					{
						gpsOrigin = gps;
					}
					Transform pose(p.x, p.y, p.z, 0.0f, 0.0f, (float)((-(gps.bearing()-90))*M_PI/180.0));
					gpsPoses.insert(std::make_pair(iter->first, pose));
				}
				if(exportGps>=0)
				{
					gpsValues.insert(std::make_pair(iter->first, gps));
				}
				gpsStamps.insert(std::make_pair(iter->first, gps.stamp()));
			}
		}

		if(exportPosesGt && !gt.isNull())
		{
			gtPoses.insert(std::make_pair(iter->first, gt));
		}

		if(optimizedPoses.size() >= 500)
		{
			++processedNodes;
			int percent = processedNodes*100/(int)optimizedPoses.size();
			if(percent != lastPercent)
			{
				printf("Processed %d/%d (%d%%) nodes...\n",
					processedNodes,
					(int)optimizedPoses.size(),
					percent);
				lastPercent = percent;
			}
		}
	}
	if(exportCloud || exportMesh)
	{
		printf("Create and assemble the clouds... done (%fs, %d points).\n", timer.ticks(), !assembledCloud->empty()?(int)assembledCloud->size():(int)assembledCloudI->size());
	}

	if(exportImages || exportImagesId)
	{
		printf("%d images exported!\n", imagesExported);
		if(!(exportCloud || exportMesh || exportPoses || exportPosesCamera || exportPosesScan)) {
			//images exported, early exit.
			return 0;
		}
	}

	ConsoleProgessState progressState;

	if(saveInDb)
	{
		driver = DBDriver::create();
		UASSERT(driver->openConnection(dbPath, false));
		Transform lastlocalizationPose;
		driver->loadOptimizedPoses(&lastlocalizationPose);
		//optimized poses have changed, reset 2d map
		driver->save2DMap(cv::Mat(), 0, 0, 0);
		driver->saveOptimizedPoses(robotPoses, lastlocalizationPose);
		cv::Vec3f vmin, vmax;
		graph::computeMinMax(robotPoses, vmin, vmax);
		printf("Saved %d poses to database! (min=[%f,%f,%f] max=[%f,%f,%f])\n",
			(int)robotPoses.size(),
			vmin[0], vmin[1], vmin[2],
			vmax[0], vmax[1], vmax[2]);
	}
	else
	{
		std::string posesExt = (exportPosesFormat==3?"toro":exportPosesFormat==4?"g2o":"txt");
		if(exportPoses)
		{
			std::string outputPath=outputDirectory+"/"+baseName+"_poses." + posesExt;
			if(robotPoses.begin() != robotPoses.lower_bound(1) && !(exportPosesFormat == 4 || exportPosesFormat == 11))
			{
				printf("Note that landmarks won't be exported because --poses_format is not 4 or 11 (currently using %d).\n", exportPosesFormat);
				rtabmap::graph::exportPoses(
					outputPath,
					exportPosesFormat,
					std::map<int, Transform>(robotPoses.lower_bound(1), robotPoses.end()),
					links,
					std::map<int, double>(cameraStamps.lower_bound(1), cameraStamps.end()));
			}
			else
			{
				rtabmap::graph::exportPoses(outputPath, exportPosesFormat, robotPoses, links, cameraStamps);
			}
			cv::Vec3f vmin, vmax;
			graph::computeMinMax(robotPoses, vmin, vmax);
			printf("%d poses exported to \"%s\". (min=[%f,%f,%f] max=[%f,%f,%f])\n",
				(int)robotPoses.size(),
				outputPath.c_str(),
				vmin[0], vmin[1], vmin[2],
				vmax[0], vmax[1], vmax[2]);
		}
		if(exportPosesCamera)
		{
			for(size_t i=0; i<cameraPoses.size(); ++i)
			{
				std::string outputPath;
				if(cameraPoses.size()==1)
					outputPath = outputDirectory+"/"+baseName+"_camera_poses." + posesExt;
				else
					outputPath = outputDirectory+"/"+baseName+"_camera_poses_"+uNumber2Str((int)i)+"." + posesExt;
				rtabmap::graph::exportPoses(outputPath, exportPosesFormat, cameraPoses[i], std::multimap<int, Link>(), cameraStamps);
				cv::Vec3f vmin, vmax;
				graph::computeMinMax(cameraPoses[i], vmin, vmax);
				printf("%d camera poses exported to \"%s\". (min=[%f,%f,%f] max=[%f,%f,%f])\n",
					(int)cameraPoses[i].size(),
					outputPath.c_str(),
					vmin[0], vmin[1], vmin[2],
					vmax[0], vmax[1], vmax[2]);
			}
		}
		if(exportPosesScan)
		{
			std::string outputPath=outputDirectory+"/"+baseName+"_scan_poses." + posesExt;
			rtabmap::graph::exportPoses(outputPath, exportPosesFormat, scanPoses, std::multimap<int, Link>(), cameraStamps);
			cv::Vec3f min, max;
			graph::computeMinMax(scanPoses, min, max);
			printf("%d scan poses exported to \"%s\". (min=[%f,%f,%f] max=[%f,%f,%f])\n",
					(int)scanPoses.size(),
					outputPath.c_str(),
					min[0], min[1], min[2],
					max[0], max[1], max[2]);
		}
		if(exportPosesGps)
		{
			std::string outputPath=outputDirectory+"/"+baseName+"_gps_poses." + posesExt;
			rtabmap::graph::exportPoses(outputPath, exportPosesFormat, gpsPoses, std::multimap<int, Link>(), gpsStamps);
			printf("%d GPS poses exported to \"%s\".\n",
					(int)gpsPoses.size(),
					outputPath.c_str());
		}
		if(exportPosesGt)
		{
			std::string outputPath=outputDirectory+"/"+baseName+"_gt_poses." + posesExt;
			rtabmap::graph::exportPoses(outputPath, exportPosesFormat, gtPoses, std::multimap<int, Link>(), cameraStamps);
			printf("%d scan poses exported to \"%s\".\n",
					(int)gtPoses.size(),
					outputPath.c_str());
		}
		if(exportGps>=0)
		{
			std::string outputPath=outputDirectory+"/"+baseName+"_gps." + (exportGps==0?"txt":"kml");
			rtabmap::graph::exportGPS(outputPath, gpsValues);
			printf("%d GPS values exported to \"%s\".\n",
					(int)gpsValues.size(),
					outputPath.c_str());
		}
	}

	if(!(exportCloud || exportMesh))
	{
		// poses exported, early exit
		return 0;
	}

	if(!assembledCloud->empty() || !assembledCloudI->empty())
	{
		if(proportionalRadiusFactor>0.0f && proportionalRadiusScale>=1.0f)
		{
			printf("Proportional radius filtering of the assembled cloud... (factor=%f scale=%f, %d points)\n", proportionalRadiusFactor, proportionalRadiusScale, !assembledCloud->empty()?(int)assembledCloud->size():(int)assembledCloudI->size());
			pcl::IndicesPtr indices;
			if(!assembledCloud->empty())
			{
				indices = util3d::proportionalRadiusFiltering(assembledCloud, rawViewpointIndices, rawViewpoints, proportionalRadiusFactor, proportionalRadiusScale);
				pcl::PointCloud<pcl::PointXYZRGB> tmp;
				pcl::copyPointCloud(*assembledCloud, *indices, tmp);
				*assembledCloud = tmp;
			}
			else if(!assembledCloudI->empty())
			{
				indices = util3d::proportionalRadiusFiltering(assembledCloudI, rawViewpointIndices, rawViewpoints, proportionalRadiusFactor, proportionalRadiusScale);
				pcl::PointCloud<pcl::PointXYZI> tmp;
				pcl::copyPointCloud(*assembledCloudI, *indices, tmp);
				*assembledCloudI = tmp;
			}
			if(indices.get())
			{
				std::vector<int> rawCameraIndicesTmp(indices->size());
				for (std::size_t i = 0; i < indices->size(); ++i)
					rawCameraIndicesTmp[i] = rawViewpointIndices[indices->at(i)];
				rawViewpointIndices = rawCameraIndicesTmp;
			}
			printf("Proportional radius filtering of the assembled cloud.... done! (%fs, %d points)\n", timer.ticks(), !assembledCloud->empty()?(int)assembledCloud->size():(int)assembledCloudI->size());
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr rawAssembledCloud(new pcl::PointCloud<pcl::PointXYZ>);
		if(voxelSize>0.0f)
		{
			printf("Voxel grid filtering of the assembled cloud... (voxel=%f, %d points)\n", voxelSize, !assembledCloud->empty()?(int)assembledCloud->size():(int)assembledCloudI->size());
			if(!assembledCloud->empty()) {
				pcl::copyPointCloud(*assembledCloud, *rawAssembledCloud); // used to adjust normal orientation
				assembledCloud = util3d::voxelize(assembledCloud, voxelSize);
			}
			else if(!assembledCloudI->empty()) {
				pcl::copyPointCloud(*assembledCloudI, *rawAssembledCloud); // used to adjust normal orientation
				assembledCloudI = util3d::voxelize(assembledCloudI, voxelSize);
			}
			printf("Voxel grid filtering of the assembled cloud.... done! (%fs, %d points)\n", timer.ticks(), !assembledCloud->empty()?(int)assembledCloud->size():(int)assembledCloudI->size());
		}

		printf("Computing normals of the assembled cloud... (k=20, %d points)\n", (int)assembledCloud->size()?(int)assembledCloud->size():(int)assembledCloudI->size());
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudToExport(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIToExport(new pcl::PointCloud<pcl::PointXYZINormal>);
		if(!assembledCloud->empty()) {
			pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(assembledCloud, 20, 0);
			UASSERT(assembledCloud->size() == normals->size());
			pcl::concatenateFields(*assembledCloud, *normals, *cloudToExport);
		}
		else if(!assembledCloudI->empty()) {
			pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(assembledCloudI, 20, 0);
			UASSERT(assembledCloudI->size() == normals->size());
			pcl::concatenateFields(*assembledCloudI, *normals, *cloudIToExport);
		}
		printf("Computing normals of the assembled cloud... done! (%fs, %d points)\n", timer.ticks(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
		assembledCloud->clear();
		assembledCloudI->clear();

		// adjust with point of views
		printf("Adjust normals to viewpoints of the assembled cloud... (%d points)\n", !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
		if(!rawAssembledCloud->empty()) {
			if(!cloudToExport->empty()) {
				util3d::adjustNormalsToViewPoints(
											rawViewpoints,
											rawAssembledCloud,
											rawViewpointIndices,
											cloudToExport,
											groundNormalsUp);
			}
			else if(!cloudIToExport->empty()) {
				util3d::adjustNormalsToViewPoints(
											rawViewpoints,
											rawAssembledCloud,
											rawViewpointIndices,
											cloudIToExport,
											groundNormalsUp);
			}
		}
		else if(!cloudToExport->empty()) {
			util3d::adjustNormalsToViewPoints(
										rawViewpoints,
										rawViewpointIndices,
										cloudToExport,
										groundNormalsUp);
		}
		else if(!cloudIToExport->empty()) {
			util3d::adjustNormalsToViewPoints(
										rawViewpoints,
										rawViewpointIndices,
										cloudIToExport,
										groundNormalsUp);
		}
		printf("Adjust normals to viewpoints of the assembled cloud... (%fs, %d points)\n", timer.ticks(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());

		if(randomSamples>0)
		{
			printf("Random samples filtering of the assembled cloud... (samples=%d, %d points)\n", randomSamples, !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
			if(!cloudToExport->empty())
			{
				cloudToExport = util3d::randomSampling(cloudToExport, randomSamples);
			}
			else if(!cloudIToExport->empty())
			{
				cloudIToExport = util3d::randomSampling(cloudIToExport, randomSamples);
			}
			printf("Random samples filtering of the assembled cloud.... done! (%fs, %d points)\n", timer.ticks(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
		}

		std::vector<int> pointToCamId;
		std::vector<float> pointToCamIntensity;
		if(camProjection && !robotPoses.empty())
		{
			printf("Camera projection... (cameras=%d)\n", (int)cameraModels.size());
			std::map<int, std::vector<rtabmap::CameraModel> > cameraModelsProj;
			if(cameraProjDecimation>1)
			{
				for(std::map<int, std::vector<rtabmap::CameraModel> >::iterator iter=cameraModels.begin();
						iter!=cameraModels.end();
						++iter)
				{
					std::vector<rtabmap::CameraModel> models;
					for(size_t i=0; i<iter->second.size(); ++i)
					{
						models.push_back(iter->second[i].scaled(1.0/double(cameraProjDecimation)));
					}
					cameraModelsProj.insert(std::make_pair(iter->first, models));
				}
			}
			else
			{
				cameraModelsProj = cameraModels;
			}

			if(exportImages)
			{
				printf("Camera projection... projecting cloud to individual cameras (--images option)\n");
				// projectCloudToCamera requires PCLPointCloud2
				pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
				if(!cloudToExport->empty())
				{
					pcl::toPCLPointCloud2(*cloudToExport, *cloud2);
				}
				else if(!cloudIToExport->empty())
				{
					pcl::toPCLPointCloud2(*cloudIToExport, *cloud2);
				}

				std::string dir = outputDirectory+"/"+baseName+"_depth_from_scan";
				if(!UDirectory::exists(dir)) {
					UDirectory::makeDir(dir);
				}

				for(std::map<int, std::vector<rtabmap::CameraModel> >::iterator iter=cameraModelsProj.begin();
					iter!=cameraModelsProj.end();
					++iter)
				{
					cv::Mat depth(iter->second.front().imageHeight(), iter->second.front().imageWidth()*iter->second.size(), CV_32FC1);
					for(size_t i=0; i<iter->second.size(); ++i)
					{
						cv::Mat subDepth = util3d::projectCloudToCamera(
								iter->second.at(i).imageSize(),
								iter->second.at(i).K(),
								cloud2,
								robotPoses.at(iter->first) * iter->second.at(i).localTransform());
						subDepth.copyTo(depth(cv::Range::all(), cv::Range(i*iter->second.front().imageWidth(), (i+1)*iter->second.front().imageWidth())));
					}

					depth = rtabmap::util2d::cvtDepthFromFloat(depth);
					std::string outputPath=dir+"/"+(exportImagesId?uNumber2Str(iter->first):uFormat("%f",cameraStamps.at(iter->first)))+".png";
					cv::imwrite(outputPath, depth);
				}
			}

			cv::Mat projMask;
			if(!cameraProjMask.empty())
			{
				projMask = cv::imread(cameraProjMask, cv::IMREAD_GRAYSCALE);
				if(cameraProjDecimation>1)
				{
					cv::Mat out = projMask;
					cv::resize(projMask, out, cv::Size(), 1.0f/float(cameraProjDecimation), 1.0f/float(cameraProjDecimation), cv::INTER_NEAREST);
					projMask = out;
				}
			}

			printf("Camera projection... projecting cloud to all cameras\n");
			pointToCamId.resize(!cloudToExport->empty()?cloudToExport->size():cloudIToExport->size());
			std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > pointToPixel;
			if(!cloudToExport->empty())
			{
				pointToPixel = util3d::projectCloudToCameras(
						*cloudToExport,
						robotPoses,
						cameraModelsProj,
						textureRange,
						textureAngle,
						textureRoiRatios,
						projMask,
						distanceToCamPolicy,
						&progressState);
			}
			else if(!cloudIToExport->empty())
			{
				pointToPixel = util3d::projectCloudToCameras(
						*cloudIToExport,
						robotPoses,
						cameraModelsProj,
						textureRange,
						textureAngle,
						textureRoiRatios,
						projMask,
						distanceToCamPolicy,
						&progressState);
				pointToCamIntensity.resize(pointToPixel.size());
			}

			printf("Camera projection... coloring the cloud\n");
			// color the cloud
			UASSERT(pointToPixel.empty() || pointToPixel.size() == pointToCamId.size());
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr assembledCloudValidPoints(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
			assembledCloudValidPoints->resize(pointToCamId.size());

			int imagesDone = 1;
			int coloredPoints = 0;
			for(std::map<int, rtabmap::Transform>::iterator iter=robotPoses.lower_bound(1); iter!=robotPoses.end(); ++iter)
			{
				int nodeID = iter->first;
				cv::Mat image;
				SensorData data;
				dbDriver->getNodeData(nodeID, data, true, false, false, false);
				if(!data.imageCompressed().empty())
				{
					data.uncompressDataConst(&image, 0);
				}
				if(!image.empty())
				{
					if(cameraProjDecimation>1)
					{
						image = util2d::decimate(image, cameraProjDecimation);
					}
				}
				UASSERT(cameraModelsProj.find(nodeID) != cameraModelsProj.end());
				int modelsSize = cameraModelsProj.at(nodeID).size();
				for(size_t i=0; i<pointToPixel.size(); ++i)
				{
					int cameraIndex = pointToPixel[i].first.second;
					if(nodeID == pointToPixel[i].first.first && cameraIndex>=0)
					{
						pcl::PointXYZRGBNormal pt;
						float intensity = 0;
						if(!cloudToExport->empty())
						{
							pt = cloudToExport->at(i);
						}
						else if(!cloudIToExport->empty())
						{
							pt.x = cloudIToExport->at(i).x;
							pt.y = cloudIToExport->at(i).y;
							pt.z = cloudIToExport->at(i).z;
							pt.normal_x = cloudIToExport->at(i).normal_x;
							pt.normal_y = cloudIToExport->at(i).normal_y;
							pt.normal_z = cloudIToExport->at(i).normal_z;
							intensity = cloudIToExport->at(i).intensity;
						}

						if(!image.empty())
						{
							int subImageWidth = image.cols / modelsSize;
							cv::Mat subImage = image(cv::Range::all(), cv::Range(cameraIndex*subImageWidth, (cameraIndex+1)*subImageWidth));

							int x = pointToPixel[i].second.x * (float)subImage.cols;
							int y = pointToPixel[i].second.y * (float)subImage.rows;
							UASSERT(x>=0 && x<subImage.cols);
							UASSERT(y>=0 && y<subImage.rows);

							if(subImage.type()==CV_8UC3)
							{
								cv::Vec3b bgr = subImage.at<cv::Vec3b>(y, x);
								pt.b = bgr[0];
								pt.g = bgr[1];
								pt.r = bgr[2];
							}
							else
							{
								UASSERT(subImage.type()==CV_8UC1);
								pt.r = pt.g = pt.b = subImage.at<unsigned char>(pointToPixel[i].second.y * subImage.rows, pointToPixel[i].second.x * subImage.cols);
							}
							++coloredPoints;
						}

						int exportedId = nodeID;
						pointToCamId[i] = exportedId;
						if(!pointToCamIntensity.empty())
						{
							pointToCamIntensity[i] = intensity;
						}
						assembledCloudValidPoints->at(i) = pt;
					}
				}
				if(!image.empty())
				{
					printf("Processed %d/%d images\n", imagesDone++, (int)robotPoses.size());
				}
				else
				{
					printf("Node %d doesn't have image! (%d/%d)\n", iter->first, imagesDone++, (int)robotPoses.size());
				}		
			}
			printf("Colored %d/%d points.\n", coloredPoints, (int)assembledCloudValidPoints->size());

			pcl::IndicesPtr validIndices(new std::vector<int>(pointToPixel.size()));
			size_t oi = 0;
			for(size_t i=0; i<pointToPixel.size(); ++i)
			{
				if(pointToPixel[i].first.first <=0)
				{
					if(camProjectionKeepAll)
					{
						pcl::PointXYZRGBNormal pt;
						float intensity = 0;
						if(!cloudToExport->empty())
						{
							pt = cloudToExport->at(i);
						}
						else if(!cloudIToExport->empty())
						{
							pt.x = cloudIToExport->at(i).x;
							pt.y = cloudIToExport->at(i).y;
							pt.z = cloudIToExport->at(i).z;
							pt.normal_x = cloudIToExport->at(i).normal_x;
							pt.normal_y = cloudIToExport->at(i).normal_y;
							pt.normal_z = cloudIToExport->at(i).normal_z;
							intensity = cloudIToExport->at(i).intensity;
						}

						pointToCamId[i] = 0; // invalid
						pt.b = 0;
						pt.g = 0;
						pt.r = 255;
						if(!pointToCamIntensity.empty())
						{
							pointToCamIntensity[i] = intensity;
						}
						assembledCloudValidPoints->at(i) = pt; // red
						validIndices->at(oi++) = i;
					}
				}
				else
				{
					validIndices->at(oi++) = i;
				}
			}
			if(oi != validIndices->size())
			{
				validIndices->resize(oi);
				assembledCloudValidPoints = util3d::extractIndices(assembledCloudValidPoints, validIndices, false, false);
				std::vector<int> pointToCamIdTmp(validIndices->size());
				std::vector<float> pointToCamIntensityTmp(pointToCamIntensity.empty()?0:validIndices->size());
				for(size_t i=0; i<validIndices->size(); ++i)
				{
					size_t index = validIndices->at(i);
					UASSERT(index < pointToCamId.size());
					pointToCamIdTmp[i] = pointToCamId[index];
					if(!pointToCamIntensity.empty())
					{
						UASSERT(index < pointToCamIntensity.size());
						pointToCamIntensityTmp[i] = pointToCamIntensity[index];
					}
				}
				pointToCamId = pointToCamIdTmp;
				pointToCamIntensity = pointToCamIntensityTmp;
				pointToCamIdTmp.clear();
				pointToCamIntensityTmp.clear();
			}
			cloudToExport = assembledCloudValidPoints;
			cloudIToExport->clear();

			printf("Camera projection... done! (%fs)\n", timer.ticks());
		}

		if(exportCloud)
		{
			if(saveInDb)
			{
				if(exportMesh)
				{
					printf("Option --save_in_db is set and both --cloud and --mesh are set, we will only save the mesh in the database.\n");
				}
				else
				{
					printf("Saving cloud in db... (%d points)\n", !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
					if(!cloudToExport->empty())
						driver->saveOptimizedMesh(util3d::laserScanFromPointCloud(*cloudToExport, Transform(), false).data());
					else if(!cloudIToExport->empty())
						driver->saveOptimizedMesh(util3d::laserScanFromPointCloud(*cloudIToExport, Transform(), false).data());
					printf("Saving cloud in db... done!\n");
				}
			}
			else
			{
				std::string ext = las?"las":"ply";
				std::string outputPath=outputDirectory+"/"+baseName+"_cloud."+ext;
				printf("Saving %s... (%d points)\n", outputPath.c_str(), !cloudToExport->empty()?(int)cloudToExport->size():(int)cloudIToExport->size());
#ifdef RTABMAP_PDAL
				if(las || !pointToCamId.empty() || !pointToCamIntensity.empty())
				{
					if(!cloudToExport->empty())
					{
						if(!pointToCamIntensity.empty())
						{
							savePDALFile(outputPath, *cloudToExport, pointToCamId, binary, pointToCamIntensity);
						}
						else
						{
							savePDALFile(outputPath, *cloudToExport, pointToCamId, binary);
						}
					}
					else if(!cloudIToExport->empty())
						savePDALFile(outputPath, *cloudIToExport, pointToCamId, binary);
				}
				else
#endif
				{
					if(!pointToCamId.empty())
					{
						if(!pointToCamIntensity.empty())
						{
							printf("Option --cam_projection is enabled but rtabmap is not built "
									"with PDAL support, so camera IDs and lidar intensities won't be exported in the output cloud.\n");
						}
						else
						{
							printf("Option --cam_projection is enabled but rtabmap is not built "
									"with PDAL support, so camera IDs won't be exported in the output cloud.\n");
						}
					}
					if(!cloudToExport->empty())
						pcl::io::savePLYFile(outputPath, *cloudToExport, binary);
					else if(!cloudIToExport->empty())
						pcl::io::savePLYFile(outputPath, *cloudIToExport, binary);
				}
				printf("Saving %s... done!\n", outputPath.c_str());
			}
		}

		// Meshing...
		if(exportMesh)
		{
			if(!cloudIToExport->empty())
			{
				pcl::copyPointCloud(*cloudIToExport, *cloudToExport);
				cloudIToExport->clear();
			}

			Eigen::Vector4f min,max;
			pcl::getMinMax3D(*cloudToExport, min, max);
			float mapLength = uMax3(max[0]-min[0], max[1]-min[1], max[2]-min[2]);
			int optimizedDepth = 12;
			for(int i=6; i<12; ++i)
			{
				if(mapLength/float(1<<i) < poissonSize)
				{
					optimizedDepth = i;
					break;
				}
			}
			if(poissonDepth>0)
			{
				optimizedDepth = poissonDepth;
			}

			// Mesh reconstruction
			printf("Mesh reconstruction... depth=%d\n", optimizedDepth);
			pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
			pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
			poisson.setDepth(optimizedDepth);
			poisson.setInputCloud(cloudToExport);
			poisson.reconstruct(*mesh);
			printf("Mesh reconstruction... done (%fs, %d polygons).\n", timer.ticks(), (int)mesh->polygons.size());

			if(mesh->polygons.size())
			{
				printf("Mesh color transfer (max polygons=%d, color radius=%f, clean=%s)...\n",
						maxPolygons,
						colorRadius,
						doClean?"true":"false");
				rtabmap::util3d::denseMeshPostProcessing<pcl::PointXYZRGBNormal>(
						mesh,
						0.0f,
						maxPolygons,
						cloudToExport,
						colorRadius,
						!texture,
						doClean,
						minCluster);
				printf("Mesh color transfer... done (%fs).\n", timer.ticks());

				if(!texture)
				{
					if(saveInDb)
					{
						printf("Saving mesh in db...\n");
						std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX> > > polygons;
						polygons.push_back(util3d::convertPolygonsFromPCL(mesh->polygons));
						driver->saveOptimizedMesh(
								util3d::laserScanFromPointCloud(mesh->cloud, false).data(),
								polygons);
						printf("Saving mesh in db... done!\n");
					}
					else
					{
						std::string outputPath=outputDirectory+"/"+baseName+"_mesh.ply";
						printf("Saving %s...\n", outputPath.c_str());
						if(binary)
							pcl::io::savePLYFileBinary(outputPath, *mesh);
						else
							pcl::io::savePLYFile(outputPath, *mesh);
						printf("Saving %s... done!\n", outputPath.c_str());
					}
				}
				else
				{
					// Camera filtering for texturing
					std::map<int, rtabmap::Transform> robotPosesFiltered;
					if(laplacianThr>0)
					{
						printf("Filtering %ld images from texturing...\n", robotPoses.size());
						for(std::map<int, rtabmap::Transform>::iterator iter=robotPoses.lower_bound(1); iter!=robotPoses.end(); ++iter)
						{
							SensorData data;
							dbDriver->getNodeData(iter->first, data, true, false, false, false);
							cv::Mat img;
							data.uncompressDataConst(&img, 0);
							if(!img.empty())
							{
								cv::Mat imgLaplacian;
								cv::Laplacian(img, imgLaplacian, CV_16S);
								cv::Mat m, s;
								cv::meanStdDev(imgLaplacian, m, s);
								double stddev_pxl = s.at<double>(0);
								double var = stddev_pxl*stddev_pxl;
								if(var < (double)laplacianThr)
								{
									printf("Camera's image %d is detected as blurry (var=%f < thr=%d), camera is ignored for texturing.\n", iter->first, var, laplacianThr);
								}
								else
								{
									robotPosesFiltered.insert(*iter);
								}
							}
						}
						printf("Filtered %ld/%ld images from texturing", robotPosesFiltered.size(), robotPoses.size());
					}
					else
					{
						robotPosesFiltered = robotPoses;
					}

					printf("Texturing %d polygons... robotPoses=%d, cameraModels=%d, cameraDepths=%d\n", (int)mesh->polygons.size(), (int)robotPosesFiltered.size(), (int)cameraModels.size(), (int)cameraDepths.size());
					std::vector<std::map<int, pcl::PointXY> > vertexToPixels;
					pcl::TextureMeshPtr textureMesh = rtabmap::util3d::createTextureMesh(
							mesh,
							robotPosesFiltered,
							cameraModels,
							cameraDepths,
							textureRange,
							textureDepthError,
							textureAngle,
							multiband?0:50, // Min polygons in camera view to be textured by this camera
							textureRoiRatios,
							&progressState,
							&vertexToPixels,
							distanceToCamPolicy);
					printf("Texturing... done (%fs).\n", timer.ticks());

					// Remove occluded polygons (polygons with no texture)
					if(doClean && textureMesh->tex_coordinates.size())
					{
						printf("Cleanup mesh...\n");
						rtabmap::util3d::cleanTextureMesh(*textureMesh, 100); // Min polygons in a cluster to keep them
						printf("Cleanup mesh... done (%fs).\n", timer.ticks());
					}

					if(textureMesh->tex_materials.size())
					{
						if(multiband)
						{
							printf("Merging %d texture(s) to single one (multiband enabled)...\n", (int)textureMesh->tex_materials.size());
						}
						else
						{
							printf("Merging %d texture(s)... (%d max textures)\n", (int)textureMesh->tex_materials.size(), textureCount);
						}
						std::map<int, std::map<int, cv::Vec4d> > gains;
						std::map<int, std::map<int, cv::Mat> > blendingGains;
						std::pair<float, float> contrastValues(0,0);
						cv::Mat textures = rtabmap::util3d::mergeTextures(
								*textureMesh,
								std::map<int, cv::Mat>(),
								std::map<int, std::vector<rtabmap::CameraModel> >(),
								0,
								dbDriver.get(),
								textureSize,
								multiband?1:textureCount, // to get contrast values based on all images in multiband mode
								vertexToPixels,
								gainValue>0.0f, gainValue, doGainCompensationRGB,
								doBlending, 0,
								lowBrightnessGain, highBrightnessGain, // low-high brightness/contrast balance
								false, // exposure fusion
								0,     // state
								0,     // blank value (0=black)
								&gains,
								&blendingGains,
								&contrastValues);
						printf("Merging to %d texture(s)... done (%fs).\n", (int)textureMesh->tex_materials.size(), timer.ticks());

						if(saveInDb)
						{
							printf("Saving texture mesh in db...\n");
							driver->saveOptimizedMesh(
									util3d::laserScanFromPointCloud(textureMesh->cloud, false).data(),
									util3d::convertPolygonsFromPCL(textureMesh->tex_polygons),
									textureMesh->tex_coordinates,
									textures);
							printf("Saving texture mesh in db... done!\n");
						}
						else
						{
							// Save multiband first
							if(multiband)
							{
								timer.restart();
								std::string outputPath=outputDirectory+"/"+baseName+"_mesh_multiband.obj";
								printf("MultiBand texturing (size=%d, downscale=%d, unwrap method=%s, fill holes=%s, padding=%d, best score thr=%f, angle thr=%f, force visible=%s)... \"%s\"\n",
										textureSize,
										multibandDownScale,
										multibandUnwrap==1?"ABF":multibandUnwrap==2?"LSCM":"Basic",
										multibandFillHoles?"true":"false",
										multibandPadding,
										multibandBestScoreThr,
										multibandAngleHardthr,
										multibandForceVisible?"false":"true",
										outputPath.c_str());
								if(util3d::multiBandTexturing(outputPath,
										textureMesh->cloud,
										textureMesh->tex_polygons[0],
										robotPosesFiltered,
										vertexToPixels,
										std::map<int, cv::Mat >(),
										std::map<int, std::vector<CameraModel> >(),
										0,
										dbDriver.get(),
										textureSize,
										multibandDownScale,
										multibandNbContrib,
										"jpg",
										gains,
										blendingGains,
										contrastValues,
										doGainCompensationRGB,
										multibandUnwrap,
										multibandFillHoles,
										multibandPadding,
										multibandBestScoreThr,
										multibandAngleHardthr,
										multibandForceVisible))
								{
									printf("MultiBand texturing...done (%fs).\n", timer.ticks());
								}
								else
								{
									printf("MultiBand texturing...failed! (%fs)\n", timer.ticks());
								}
							}
							else
							{
								// TextureMesh OBJ
								bool success = false;
								UASSERT(!textures.empty());
								for(size_t i=0; i<textureMesh->tex_materials.size(); ++i)
								{
									// Texture file name format is texture[#], replace "texture" by base filename
									std::list<std::string> values = uSplitNumChar(textureMesh->tex_materials[i].tex_file);
									textureMesh->tex_materials[i].tex_file.clear();
									for(const auto & v: values)
									{
										if(v.compare("texture") == 0){
											textureMesh->tex_materials[i].tex_file.append(baseName+"_mesh");
										}
										else {
											textureMesh->tex_materials[i].tex_file.append(v);
										}
									}
									textureMesh->tex_materials[i].tex_file += ".jpg";
									printf("Saving texture to %s.\n", textureMesh->tex_materials[i].tex_file.c_str());
									UASSERT(textures.cols % textures.rows == 0);
									success = cv::imwrite(outputDirectory+"/"+textureMesh->tex_materials[i].tex_file, cv::Mat(textures, cv::Range::all(), cv::Range(textures.rows*i, textures.rows*(i+1))));
									if(!success)
									{
										UERROR("Failed saving %s!", textureMesh->tex_materials[i].tex_file.c_str());
									}
									else
									{
										printf("Saved %s.\n", textureMesh->tex_materials[i].tex_file.c_str());
									}
								}
								if(success)
								{
									std::string outputPath=outputDirectory+"/"+baseName+"_mesh.obj";
									printf("Saving obj (%d vertices) to %s.\n", (int)textureMesh->cloud.data.size()/textureMesh->cloud.point_step, outputPath.c_str());
	#if PCL_VERSION_COMPARE(>=, 1, 13, 0)
									textureMesh->tex_coord_indices = std::vector<std::vector<pcl::Vertices>>();
									auto nr_meshes = static_cast<unsigned>(textureMesh->tex_polygons.size());
									unsigned f_idx = 0;
									for (unsigned m = 0; m < nr_meshes; m++) {
										std::vector<pcl::Vertices> ci = textureMesh->tex_polygons[m];
										for(std::size_t i = 0; i < ci.size(); i++) {
											for (std::size_t j = 0; j < ci[i].vertices.size(); j++) {
												ci[i].vertices[j] = ci[i].vertices.size() * (i + f_idx) + j;
											}
										}
										textureMesh->tex_coord_indices.push_back(ci);
										f_idx += static_cast<unsigned>(textureMesh->tex_polygons[m].size());
									}
	#endif								
									success = pcl::io::saveOBJFile(outputPath, *textureMesh) == 0;

									if(success)
									{
										printf("Saved obj to %s!\n", outputPath.c_str());
									}
									else
									{
										UERROR("Failed saving obj to %s!", outputPath.c_str());
									}
								}
							}
						}
					}
				}
			}
		}
	}
	else
	{
		printf("Export failed! The cloud is empty.\n");
	}

	if(driver)
	{
		driver->closeConnection();
		delete driver;
		driver = 0;
	}

	return 0;
}
