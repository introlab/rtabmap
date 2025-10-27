
<a href="https://youtu.be/71eRxTc1DaU" target="_blank"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSessionTango/youtube.jpg"
 alt="video" title="video"/></a>

## Overview 
<img src="https://raw.githubusercontent.com/introlab/rtabmap/master/guilib/src/images/RTAB-Map.png" alt="RTAB-Map logo" title="RTAB-Map" align="left" width="120">

**RTAB-Map** (Real-Time Appearance-Based Mapping) is a RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. The loop closure detector uses a bag-of-words approach to determinate how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map's graph, then a graph optimizer minimizes the errors in the map. A memory management approach is used to limit the number of locations used for loop closure detection and graph optimization, so that real-time constraints on large-scale environnements are always respected. RTAB-Map can be used alone with a handheld Kinect, a stereo camera or a 3D lidar for 6DoF mapping, or on a robot equipped with a laser rangefinder for 3DoF mapping.

#### Illumination-Invariant Visual Re-Localization
* M. Labbé and F. Michaud, “[Multi-Session Visual SLAM for Illumination-Invariant Re-Localization in Indoor Environments](https://arxiv.org/abs/2103.03827),” in _Frontiers in Robotics and AI_, vol. 9, 2022. ([Frontiers](https://doi.org/10.3389/frobt.2022.801886)) ([Dataset link](https://github.com/introlab/rtabmap/tree/master/archive/2022-IlluminationInvariant)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:ufrVoPGSRksC))

#### Lidar and Visual SLAM
* M. Labbé and F. Michaud, “[RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation](https://arxiv.org/abs/2403.06341),” in _Journal of Field Robotics_, vol. 36, no. 2, pp. 416–446, 2019.  ([Wiley](https://doi.org/10.1002/rob.21831)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:Y0pCki6q_DkC))

#### Simultaneous Planning, Localization and Mapping (SPLAM)
* M. Labbé and F. Michaud, “[Long-term online multi-session graph-based SPLAM with memory management](https://arxiv.org/abs/2301.00050),” in _Autonomous Robots_, vol. 42, no. 6, pp. 1133-1150, 2018.  ([Springer](http://dx.doi.org/10.1007/s10514-017-9682-5)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:Tyk-4Ss8FVUC))

#### Multi-session SLAM
* M. Labbé and F. Michaud, “[Online Global Loop Closure Detection for Large-Scale Multi-Session Graph-Based SLAM](https://arxiv.org/abs/2407.15305),” in _Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems_, 2014. ([IEEE Xplore](http://ieeexplore.ieee.org/document/6942926/)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:9yKSN-GCB0IC))
    * Results shown in this paper can be reproduced by the [Multi-session mapping](https://github.com/introlab/rtabmap/wiki/Multi-session) tutorial.
   
#### Loop closure detection
* M. Labbé and F. Michaud, “[Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation](https://arxiv.org/abs/2407.15304),” in _IEEE Transactions on Robotics_, vol. 29, no. 3, pp. 734-745, 2013. ([IEEE Xplore](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6459608)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:u-x6o8ySG0sC))
* M. Labbé and F. Michaud, “[Memory management for real-time appearance-based loop closure detection](https://arxiv.org/abs/2407.15890),” in _Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems_, 2011, pp. 1271–1276. ([IEEE Xplore](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6094602)) ([Google Scholar](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=G3BrBkMAAAAJ&citation_for_view=G3BrBkMAAAAJ:u5HHmVD_uO8C))
    * Visit [RTAB-Map's page on IntRoLab](https://introlab.3it.usherbrooke.ca/index.php/RTAB-Map) for detailed information on the loop closure detection approach and related datasets.
    * Visit this [page](https://github.com/introlab/rtabmap/tree/master/archive/2010-LoopClosure) for usage example of the CLI tool that van be used to evaluate only RTAB-Map's loop closure detector on new datasets.

## Install 
<a href="https://github.com/introlab/rtabmap_ros#rtabmap_ros"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/ros.png" alt="ROS" width="125"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#ubuntu"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/ubuntu.png" alt="Ubuntu" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#macosx"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/apple.png" alt="Mac OS X" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#windows"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/windows.png" alt="Windows" width="50"></a>  <a href="https://apps.apple.com/ca/app/rtab-map-3d-lidar-scanner/id1564774365"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/ios.png" alt="iOS" width="50"></a> <a href="https://play.google.com/store/apps/details?id=com.introlab.rtabmap"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/tango.png" alt="Google Tango" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#raspberrypi"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/raspberrypi.png" alt="Raspberry Pi" width="40"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#docker"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/docker.png" alt="Docker" width="50"></a> <a href="https://www.youtube.com/@matlabbe/videos" target="_blank"><img src="https://www.gstatic.com/youtube/img/branding/youtubelogo/svg/youtubelogo.svg"
 alt="youtube" title="youtube" width="100"/></a>


* [Installation](https://github.com/introlab/rtabmap/wiki/Installation) instructions. 
* [Tutorials](https://github.com/introlab/rtabmap/wiki/Tutorials).
* [Tools](https://github.com/introlab/rtabmap/wiki/Tools).
* For **ROS** users, take a look to [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki for a package overview. See also [SetupOnYourRobot](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot) to know how to integrate RTAB-Map on your robot.


### Troubleshooting
**Standalone**

* Visit the [wiki](https://github.com/introlab/rtabmap/wiki).
* Ask a question on [RTAB-Map Forum](http://official-rtab-map-forum.206.s1.nabble.com/) (**New address! August 9, 2021**).
* Post an [issue on GitHub](https://github.com/introlab/rtabmap/issues) 
* For the loop closure detection approach, visit [RTAB-Map on IntRoLab website](https://introlab.3it.usherbrooke.ca/index.php/RTAB-Map)
* Enabled [Github Discussions](https://github.com/introlab/rtabmap/discussions) (**New! November 2022**)

**ROS**

* Visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) wiki page for nodes documentation, demos and tutorials on ROS. 
* Ask a question on ~~[answers.ros.org](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:rtabmap_ros/page:1/)~~ [robotics.stackexchange.com](https://robotics.stackexchange.com/questions/tagged/rtabmap) with **rtabmap** or **rtabmap-ros** tag.

## License
* If OpenCV is built **without the nonfree** module, RTAB-Map can be used under the permissive BSD License.
* If OpenCV is built **with the nonfree** module, RTAB-Map is free for research only because it depends on **SURF** features. **SURF** is not free for commercial use. Note that SIFT patent has expired, so it can be a good free equivalent of SURF.
    * SURF noncommercial notice: http://www.vision.ee.ethz.ch/~surf/download.html

## Privacy Policy
RTAB-Map App on [Google Play Store](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en) or [Apple Store](https://apps.apple.com/ca/app/rtab-map-3d-lidar-scanner/id1564774365) requires access to camera to record images that will be used for creating the map. When saving, a database containing these images is created. That database is saved locally on the device (on the sd-card under RTAB-Map folder). While location permission is required to install RTAB-Map Tango, the GPS coordinates are not saved by default, the option "Settings->Mapping...->Save GPS" should be enabled first. RTAB-Map requires read/write access to RTAB-Map folder only, to save, export and open maps. RTAB-Map doesn't access any other information outside the RTAB-Map folder. RTAB-Map doesn't share information over Internet unless the user explicitly exports a map to Sketchfab or anywhere else, for which RTAB-Map needs the network. If so, the user will be asked for authorization ([oauth2](https://oauth.net/)) by Sketchfab (see their Privacy Policy [here](https://sketchfab.com/privacy)).

This website uses Google Analytics. See their Privacy Policy [here](https://support.google.com/analytics/answer/6004245?hl=en).

## Author
* [Mathieu Labbé](https://introlab.3it.usherbrooke.ca/index.php?title=Mathieu_Labbe&setlang=en)
    * [RTAB-Map's page at IntRoLab](http://introlab.3it.usherbrooke.ca/index.php/RTAB-Map)
    * [Papers](https://introlab.3it.usherbrooke.ca/index.php/RTAB-Map#Publications)
    * Similar projects: [Find-Object](http://introlab.github.io/find-object/)

## What's new 

### December 2024
ROS2 packages overhaul (see [ros2 branch](https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#usage)): we added new ROS2 [demos](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos#rtabmap_demos) and [examples](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch). We also fixed `message_filters` related synchronization [issues](https://github.com/introlab/rtabmap_ros/pull/1206) causing significant lags when using ROS2 nodes (in comparison to ROS1). Binaries `ros-$ROS_DISTRO-rtabmap-ros` will be released under version `0.21.9`.
[![Peek 2024-11-30 20-35](https://github.com/user-attachments/assets/f64a44bf-148b-4658-9b25-b46bd0411a69)](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos#rtabmap_demos)

### November 2023
We had new papers published this year on a very fun project about underground mines scanning. Here is a video of the SLAM part of the project realized with RTAB-Map (it is an early field test we did before going in the mines):
[![Watch the video](https://img.youtube.com/vi/ytsfhMdv9W0/sddefault.jpg)](https://youtu.be/ytsfhMdv9W0)

Related papers:
* Leclerc, M.A., Bass, J., Labbé, M., Dozois, D., Delisle, J., Rancourt, D. and Lussier Desbiens, A., 2023. "[NetherDrone: A tethered and ducted propulsion multirotor drone for complex underground mining stopes inspection](https://cdnsciencepub.com/doi/pdf/10.1139/dsa-2023-0001)". Drone Systems and Applications. ([Canadian Science Publishing](https://cdnsciencepub.com/doi/full/10.1139/dsa-2023-0001)) (**Editor's Choice**)

* Petit, L. and Desbiens, A.L., 2022. "[Tape: Tether-aware path planning for autonomous exploration of unknown 3d cavities using a tangle-compatible tethered aerial robot](https://www.researchgate.net/profile/Louis-Petit/publication/362336640_TAPE_Tether-Aware_Path_Planning_for_Autonomous_Exploration_of_Unknown_3D_Cavities_using_a_Tangle-compatible_Tethered_Aerial_Robot/links/62e94ab93c0ea8788776c506/TAPE-Tether-Aware-Path-Planning-for-Autonomous-Exploration-of-Unknown-3D-Cavities-Using-a-Tangle-Compatible-Tethered-Aerial-Robot.pdf)". IEEE Robotics and Automation Letters, 7(4), pp.10550-10557. ([IEEE Xplore](https://ieeexplore.ieee.org/document/9844242))
   [![Watch the video](https://img.youtube.com/vi/nROO0BFK4lc/maxresdefault.jpg)](https://youtu.be/nROO0BFK4lc)
  
### March 2023
New release [v0.21.0](https://github.com/introlab/rtabmap/releases/tag/0.21.0)!

### June 2022
A new paper has been published: **Multi-Session Visual SLAM for Illumination-Invariant Re-Localization in Indoor Environments**. The general idea is to remap multiple times the same environment to capture multiple illumination variations caused by natural and artificial lighting, then the robot would be able to localize afterwards at any hour of the day. For more details, see this [page](https://github.com/introlab/rtabmap/tree/master/archive/2022-IlluminationInvariant) and the linked paper. Some great comparisons about robustness to illumination variations between binary descriptors (BRIEF/ORB, BRISK), float descriptors (SURF/SIFT/KAZE/DAISY) and learned descriptors (SuperPoint).
![Illumination-invariant](https://github.com/introlab/rtabmap/raw/master/archive/2022-IlluminationInvariant/images/fig_overview.jpg)

### January 2022
Added [demo](https://github.com/introlab/rtabmap_ros/blob/master/launch/demo/demo_catvehicle_mapping.launch) for car mapping and localization with [CitySim](https://github.com/osrf/citysim) simulator and [CAT Vehicle](https://github.com/jmscslgroup/catvehicle):
[![Watch the video](https://img.youtube.com/vi/vKCTg4plPkw/maxresdefault.jpg)](https://youtu.be/vKCTg4plPkw)

### December 2021
Added indoor drone visual navigation example using [move_base](http://wiki.ros.org/move_base), [PX4](https://github.com/PX4/PX4-Autopilot) and [mavros](http://wiki.ros.org/mavros):
[![Watch the video](https://img.youtube.com/vi/A487ybS7E4E/maxresdefault.jpg)](https://youtu.be/A487ybS7E4E)

More info on the [rtabmap-drone-example](https://github.com/matlabbe/rtabmap_drone_example) github repo.

### June 2021
* I'm pleased to announce that RTAB-Map is now on **iOS** (iPhone/iPad with LiDAR required). The app is [available](https://apps.apple.com/ca/app/rtab-map-3d-lidar-scanner/id1564774365) on App Store.

    <a href="https://youtu.be/rVpIcrgD5c0"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap-ios.jpg" alt="video" title="video"></a>

### December 2020
New release [v0.20.7](https://github.com/introlab/rtabmap/releases/tag/0.20.7)!

### August 2020
New release [v0.20.3](https://github.com/introlab/rtabmap/releases/tag/0.20.3)!

### July 2020
New release [v0.20.2](https://github.com/introlab/rtabmap/releases/tag/0.20.2)!

### November 2019
 * [AliceVision](https://alicevision.org/) library has been integrated to RTAB-Map to provide higher texture quality. Compare the [updated version of the Ski Cottage on Sketchfab](https://skfb.ly/6OyUy) with the [old version](https://skfb.ly/6KBFz). Look at how the edges between camera texures are smoother (thx to multi-band blending), decreasing significantly the sharp edge artifacts. Multi-band blending approach can be enabled in File->Export Clouds dialog under Texturing section. RTAB-Map should be built with AliceVision support (CUDA is not required as only texture pipeline is used). A [patch](https://gist.github.com/matlabbe/469bba5e7733ad6f2e3d7857b84f1f9e) is required to avoid problems with Eigen, refer to Docker file [here](https://github.com/introlab/rtabmap/blob/e7be12e0ff7ae95e492837f0414d553c977d6d4d/docker/bionic/Dockerfile#L69-L116).
 
<img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSessionTango/texture_old.jpg" alt="texture_old" width="300"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSessionTango/texture_new.jpg" alt="texture_new" width="300">

### September 2017
 * New version 0.14 of RTAB-Map Tango with GPS support. See it on [play store](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en).

### July 2017
* New version 0.13 of RTAB-Map Tango. See it on [play store](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en).
* I uploaded a [presentation](https://introlab.3it.usherbrooke.ca/images/3/31/Labbe2015ULaval.pdf) that I did in 2015 at Université Laval in Québec! A summary of RTAB-Map as a RGBD-SLAM approach:

    <img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Labbe2015ULavalOverview.jpg" alt="RTAB-Map overview" width="400">
 

### March 2017
* New tutorial: [Multi-Session Mapping with RTAB-Map Tango](https://github.com/introlab/rtabmap/wiki/Multi-Session-Mapping-with-RTAB-Map-Tango)

    <img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSessionTango/mesh.jpg" alt="Mesh" width="400"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSessionTango/cloudSessionColor.jpg" alt="Cloud" width="200">

### February 2017
* **Version 0.11.14** : Visit the [release page](https://github.com/introlab/rtabmap/releases/0.11.14) for more info!
    * Tango app also updated:
    
    <a href="https://youtu.be/FvhxdUhsNUk"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/tango0.11.14.jpg" alt="video" title="video"></a>

### October 2016
* Application example: See how RTAB-Map is helping nuclear dismantling with Orano's MANUELA project (Mobile Apparatus for Nuclear Expertise and Localisation Assistance):

    <a href="https://www.youtube.com/watch?v=V4dN2qnJXOU"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/manuela.jpg" alt="video" title="video"></a>

* **Version 0.11.11**: Visit the [release page](https://github.com/introlab/rtabmap/releases/0.11.11) for more info!

### July 2016

* **Version 0.11.8**: Visit the [release page](https://github.com/introlab/rtabmap/releases/0.11.8) for more info!

### June 2016

* **Version 0.11.7**: Visit the [release page](https://github.com/introlab/rtabmap/releases/0.11.7) for more info!


### February 2016

* I'm pleased to announce that RTAB-Map is now on **Project Tango**. The app is [available](https://play.google.com/store/apps/details?id=com.introlab.rtabmap) on Google Play Store.

    <a href="https://youtu.be/BE8kMkrCeuA"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap-tango.jpeg" alt="video" title="video"></a>

   * Screenshots:

   <img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap-tango.png" alt="rtabmap tango 1" width="600">

   <img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap-tango2.png" alt="rtabmap tango 2" width="600">

### October 2015

* **Version 0.10.10**: Visit the [release page](https://github.com/introlab/rtabmap/releases/0.10.10) for more info!

### September 2015

* **Version 0.10.6**: Integration of a robust graph optimization approach called <a href="https://openslam.org/vertigo.html">Vertigo</a> (which uses [g2o](https://openslam.org/g2o.html) or [GTSAM](https://collab.cc.gatech.edu/borg/gtsam)), see [this page](https://github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization):

    <a href="http://www.youtube.com/watch?v=A8v70DZxLF8"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/robust_graph_optimization.jpg" alt="video" title="video"></a>

### August 2015

* **Version 0.10.5**: New example to export data to [MeshLab](http://meshlab.sourceforge.net/) in order to add textures on a created mesh with low polygons, see [this page](https://github.com/introlab/rtabmap/wiki/Export-Raster-Layers-to-MeshLab):

    <a href="https://github.com/introlab/rtabmap/wiki/Export-Raster-Layers-to-MeshLab"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/Texture/texture_tutorial.jpeg" alt="Texture tutorial" width="400"></a>

### October 2014

* New example to speed up RTAB-Map's odometry, see [this page](https://github.com/introlab/rtabmap/wiki/Change-parameters):

    <a href="http://www.youtube.com/watch?v=Bh8WZsU4YC8"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/speedup.jpeg" alt="video" title="video"></a>

### September 2014

* At **IROS 2014** in Chicago, a team using RTAB-Map for SLAM **won** the Kinect navigation contest held during the conference. See their press release for more details: [Winning the IROS2014 Microsoft Kinect Challenge](http://www.meetup.com/SV-ROS-users/pages/Winning_the_IROS2014_Microsoft_Connect_Challenge/). I also added the Wiki page [IROS2014KinectChallenge](https://github.com/introlab/rtabmap/wiki/IROS-2014-Kinect-Challenge) showing in details the RTAB-Map part used in their solution.

    <a href="http://www.youtube.com/watch?v=_qiLAWp7AqQ"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/iros2014.jpeg" alt="video" title="video"></a>

### August 2014

* Here a comparison between reality and what can be shown in RVIZ (you can [reproduce this demo here](http://wiki.ros.org/rtabmap_ros#Robot_mapping_with_Find-Object)):

    <a href="http://www.youtube.com/watch?v=o1GSQanY-Do"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/findobject.jpeg" alt="video" title="video"></a>

### July 2014

* Added [Setup on your robot](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot) wiki page to know how to integrate RTAB-Map on your ROS robot. Multiple sensor configurations are shown but the optimal configuration is to have a 2D laser, a Kinect-like sensor and odometry.

    <img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSession/4.png" alt="AZIMUT3" align="left" width="300">

   Onboard mapping | Remote mapping
   ------------ | -------------
   <a href="http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Bring-up_your_robot"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/rgbd/setupA.png" alt="" width="300"></a> | <a href="http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#Remote_mapping"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/rgbd/remoteMapping.png" alt="" width="300"></a>

### June 2014

* I'm glad to announce that my paper submitted to [IROS 2014](http://www.iros2014.org/) was accepted! [This paper](https://introlab.3it.usherbrooke.ca/images/e/eb/Labbe14-IROS.pdf) explains in details how RGB-D mapping with RTAB-Map is done. Results shown in this paper can be reproduced by the [Multi-session mapping](https://github.com/introlab/rtabmap/wiki/Multi-session) tutorial:

    <a href="https://github.com/introlab/rtabmap/wiki/Multi-session"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSession/3.png" alt="Multi-session mapping" width="600"></a>


## Videos

* **RGBD-SLAM**

   <a href="http://www.youtube.com/watch?v=Nm2ggyAW4rw"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/desktop.jpeg" alt="video" title="video"></a>

   <a href="http://www.youtube.com/watch?v=joV7VCvGrKM"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/globalonly.jpeg" alt="video" title="video"></a>
   
   [![video](https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/onlinergbdslam.jpeg)](http://www.youtube.com/watch?v=AMLwjo80WzI)

* **Appearance-based loop closure detection**

   <a href="http://www.youtube.com/watch?v=CAk-QGMlQmI"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap.jpeg" alt="video" title="video"></a>

   <a href="http://www.youtube.com/watch?v=1dImRinTJSE"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/newcollegeomni.jpeg" alt="video" title="video"></a>

* More loop closure detection videos [here](http://introlab.3it.usherbrooke.ca/index.php/RTAB-Map).

