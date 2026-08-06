Command-line tools {#tools}
==================

Building RTAB-Map installs a set of executables next to the libraries. They are
built on the same API these pages document, so each one is also a worked example
of it -- and the quickest way to inspect, replay or convert a map without
writing code.

All of them take the `--Param Group/Name value` arguments described in the
@ref parameters "Parameter reference", and print their options with `--help`.

Working with a map
------------------

| Command | What it does |
| ------- | ------------ |
| @anchor tool_info `rtabmap-info` | Prints a database summary: version, sizes, parameters and per-node statistics. `--diff` shows only the parameters that differ from the defaults, or from another database. |
| @anchor tool_export `rtabmap-export` | Exports the map: assembled point cloud, mesh or textured mesh (`.ply`, `.pcd`, `.obj`), 2D occupancy grid, poses and camera images. Filtering, decimation, colour and texturing are all options. |
| @anchor tool_report `rtabmap-report` | Reports the statistics recorded in one or several databases: RMSE against ground truth, timings, loop closures. Plots them when built with Qt. |
| @anchor tool_recovery `rtabmap-recovery` | Repairs a database whose session was not closed properly (e.g. after a crash), by rebuilding it from the nodes and links that can still be read. The original is kept as `*.backup.db` unless `-d` is given. |
| @anchor tool_reprocess `rtabmap-reprocess` | Replays the data of a database through a fresh @ref rtabmap::Rtabmap "Rtabmap" instance, with different parameters, and writes a new database. Several databases can be merged in one run. The way to try a new configuration on recorded data. |
| @anchor tool_detectmoreloopclosures `rtabmap-detectMoreLoopClosures` | Looks for additional loop closures in an existing map, by clustering nodes that are close in the optimized graph, and re-optimizes it. |
| @anchor tool_globalba `rtabmap-globalBundleAdjustment` | Runs a global bundle adjustment over the whole graph and saves the refined poses. |
| @anchor tool_reducegraph `rtabmap-reduceGraph` | Merges nodes of the same location to shrink the graph, keeping the map usable for localization. |
| @anchor tool_cleanuplocalgrids `rtabmap-cleanupLocalGrids` | Clears from the local occupancy grids the space that the assembled global grid shows as empty, so that removed obstacles do not reappear when the map is regenerated. |

Recording and datasets
----------------------

| Command | What it does |
| ------- | ------------ |
| @anchor tool_console `rtabmap-console` | Runs loop closure detection without a GUI on a directory of images, a video or a database, and reports the loop closures found. Appearance-only: it forces @ref rtabmap::Parameters::kRGBDEnabled() "RGBD/Enabled" to false and feeds images without a pose, so there is no odometry, no graph optimization and no metric map -- unlike the dataset tools below, which run the whole SLAM pipeline. |
| @anchor tool_datarecorder `rtabmap-dataRecorder` | Records a live sensor into a database, using a configuration file exported from the GUI preferences. |
| @anchor tool_kitti `rtabmap-kitti_dataset` | Runs a [KITTI](https://www.cvlibs.net/datasets/kitti/) odometry sequence (stereo, optionally Velodyne) and reports the KITTI errors against ground truth. |
| @anchor tool_rgbd `rtabmap-rgbd_dataset` | Same for a [TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) sequence. |
| @anchor tool_euroc `rtabmap-euroc_dataset` | Same for a [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) sequence (stereo and IMU). |
| @anchor tool_cidsims `rtabmap-cidsims_dataset` | Same for a CID-SIMS sequence (RGB-D, IMU and wheel odometry). |
| @anchor tool_imagesjoiner `rtabmap-imagesJoiner` | Joins images of two directories side by side, to build a stereo sequence out of two monocular ones. |

Sensors and calibration
-----------------------

These need the GUI library (Qt), so they are only built when it is available.

| Command | What it does |
| ------- | ------------ |
| @anchor tool_camera `rtabmap-camera` | Streams a USB camera, a video file or a directory of images, and shows what the driver delivers. |
| @anchor tool_rgbdcamera `rtabmap-rgbd_camera` | Same for the RGB-D and stereo drivers (OpenNI, Freenect, RealSense, Kinect for Azure, ZED...), to check a device before mapping with it. |
| @anchor tool_lidarviewer `rtabmap-lidar_viewer` | Shows the scans of a network lidar (VLP-16). |
| @anchor tool_calibration `rtabmap-calibration` | Calibrates a camera or a stereo pair from a chessboard, and saves the model RTAB-Map reads. |
| @anchor tool_odometryviewer `rtabmap-odometryViewer` | Runs @ref rtabmap::Odometry "Odometry" alone on a live sensor and shows the features, the local map and the estimated trajectory. Useful to tune the `Odom/` and `Vis/` parameters. |
| @anchor tool_databaseviewer `rtabmap-databaseViewer` | Inspects a database: browse the nodes and their data, the graph and its links, add or remove constraints, re-optimize, regenerate the grids and export. |

Algorithm evaluation
--------------------

| Command | What it does |
| ------- | ------------ |
| @anchor tool_matcher `rtabmap-matcher` | Registers two images with the @ref rtabmap::RegistrationVis "visual" or @ref rtabmap::RegistrationIcp "ICP" pipeline and draws the correspondences. The direct way to compare feature detectors and matching parameters on a hard pair. |
| @anchor tool_stereoeval `rtabmap-stereoEval` | Evaluates the stereo correspondence parameters against a ground truth disparity map (Middlebury format). |
| @anchor tool_epipolar `rtabmap-epipolar_geometry` | Shows the epipolar geometry between two images. |
| @anchor tool_extractobject `rtabmap-extractObject` | Extracts an object lying on a plane from a point cloud file. |
| @anchor tool_vocabulary `rtabmap-vocabularyComparison` | Compares the nearest-neighbour strategies of the visual dictionary on a saved vocabulary. Built only with the OpenCV non-free module. |

The GUI application itself is `rtabmap`; everything the tools above do offline is
also reachable from its menus.
