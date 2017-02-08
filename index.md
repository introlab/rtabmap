
[![video](https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/onlinergbdslam.jpeg)](http://www.youtube.com/watch?v=AMLwjo80WzI)

## Overview 
<img src="https://raw.githubusercontent.com/introlab/rtabmap/master/guilib/src/images/RTAB-Map.png" alt="RTAB-Map logo" title="RTAB-Map" align="left" width="120">
**RTAB-Map** (Real-Time Appearance-Based Mapping) is a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. The loop closure detector uses a bag-of-words approach to determinate how likely a new image comes from a previous location or a new location. When a loop closure hypothesis is accepted, a new constraint is added to the map's graph, then a graph optimizer minimizes the errors in the map. A memory management approach is used to limit the number of locations used for loop closure detection and graph optimization, so that real-time constraints on large-scale environnements are always respected. RTAB-Map can be used alone with a hand-held Kinect or stereo camera for 6DoF RGB-D mapping, or on a robot equipped with a laser rangefinder for 3DoF mapping.

#### RGB-D mapping
  * M. Labbé and F. Michaud, “[Online Global Loop Closure Detection for Large-Scale Multi-Session Graph-Based SLAM](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf),” in _Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems_, 2014. ([IEEE Xplore](http://ieeexplore.ieee.org/document/6942926/))
   * Results shown in this paper can be reproduced by the [Multi-session mapping](https://github.com/introlab/rtabmap/wiki/Multi-session) tutorial.
   
#### Loop closure detection
  * M. Labbé and F. Michaud, “[Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/b/bc/TRO2013.pdf),” in _IEEE Transactions on Robotics_, vol. 29, no. 3, pp. 734-745, 2013. ([IEEE Xplore](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6459608))
  * M. Labbé and F. Michaud, “[Memory management for real-time appearance-based loop closure detection](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/f/f0/Labbe11memory.pdf),” in _Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems_, 2011, pp. 1271–1276. ([IEEE Xplore](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6094602))
  * Visit [RTAB-Map's page on IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map) for detailed information on the loop closure detection approach and related datasets.

## Install 
<a href="https://github.com/introlab/rtabmap_ros#rtabmap_ros"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/ros.png" alt="ROS" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#ubuntu"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/ubuntu.png" alt="Ubuntu" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#macosx"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/apple.png" alt="Mac OS X" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#windows"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/windows.png" alt="Windows" width="50"></a>  <a href="https://play.google.com/store/apps/details?id=com.introlab.rtabmap"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/tango.png" alt="Google Tango" width="50"></a>  <a href="https://github.com/introlab/rtabmap/wiki/Installation#raspberrypi"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/logos/raspberrypi.png" alt="Raspberry Pi" width="40"></a>


 * [Installation](https://github.com/introlab/rtabmap/wiki/Installation) instructions. 
 * [Tutorials](https://github.com/introlab/rtabmap/wiki/Tutorials).
 * [Tools](https://github.com/introlab/rtabmap/wiki/Tools).
 * For **ROS** users, take a look to [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki for a package overview. See also [SetupOnYourRobot](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot) to know how to integrate RTAB-Map on your robot.

### Troubleshooting
**Standalone**

 * Visit the [wiki](https://github.com/introlab/rtabmap/wiki).
 * Ask a question on the [RTAB-Map Forum](http://official-rtab-map-forum.67519.x6.nabble.com/).
 * Post an [issue on GitHub](https://github.com/introlab/rtabmap/issues) 
 * For the loop closure detection approach, visit [RTAB-Map on IntRoLab website](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map)

**ROS**

 * Visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) wiki page for nodes documentation, demos and tutorials on ROS. 
 * Ask a question on [answers.ros.org](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:rtabmap_ros/page:1/) with **rtabmap** or **rtabmap_ros** tag.

## License
 * If OpenCV is built **without the nonfree** module, RTAB-Map can be used under the permissive BSD License.
 * If OpenCV is built **with the nonfree** module, RTAB-Map is free for research only because it depends on **SURF** and **SIFT** features. **SIFT** and **SURF** are not free for commercial use.
  * SURF noncommercial notice: http://www.vision.ee.ethz.ch/~surf/download.html
  * SIFT patent: http://www.cs.ubc.ca/~lowe/keypoints/

## Privacy Policy
[RTAB-Map](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en) Tango app on Google Play Store requires access to camera to record images that will be used for creating the map. When saving, a database containing these images is created. That database is saved locally on the device (on the sd-card under RTAB-Map folder). RTAB-Map requires read/write access to RTAB-Map folder only, to save, export and open maps. RTAB-Map doesn't access any other information outside the RTAB-Map folder. RTAB-Map doesn't share information over Internet unless the user explicitly exports a map to Sketchfab, for which RTAB-Map needs the network. If so, the user will be asked for authorization ([oauth2](https://oauth.net/)) by Sketchfab (see their Privacy Policy [here](https://sketchfab.com/privacy)).

This website uses Google Analytics. See their Privacy Policy [here](https://support.google.com/analytics/answer/6004245?hl=en).

## Author
 * [Mathieu Labbé](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php?title=Mathieu_Labbé&setlang=en)
  * [RTAB-Map's page at IntRoLab](http://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map)
  * [Papers](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map#Publications)
  * Similar projects: [Find-Object](http://introlab.github.io/find-object/)
  * If you find this project useful and to help me keeping this project updated, you can buy me a cup of coffee with the link below :P. It is also nice to receive new sensors to test with and even supporting them in RTAB-Map for quick SLAM demonstrations (e.g., stereo cameras, RGB-D cameras, 2D/3D LiDARs). Thanks [Stereolabs](https://www.stereolabs.com/) for the [ZED](https://www.stereolabs.com/zed/specs/), thanks Walt (with Tango coupon discount) and Google for [Google Tango Development Kits](https://store.google.com/product/tango_tablet_development_kit) and thanks to all contributors (for donations, reporting bugs, helping me fixing bugs or making pull requests).

    [![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=X8TCXXHHDL62Q)

## What's new 

### February 2017
 * **Version 0.11.14** : Visit the [release page](https://github.com/introlab/rtabmap/releases/0.11.14) for more info!
  * Tango app also updated:
    
    <a href="https://youtu.be/FvhxdUhsNUk"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/tango0.11.14.jpg" alt="video" title="video"></a>

### October 2016
 * Application example: See how RTAB-Map is helping nuclear dismantling with Areva's MANUELA project (Mobile Apparatus for Nuclear Expertise and Localisation Assistance):

  <a href="https://youtu.be/bHsNVoRkK2w"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/manuela.jpg" alt="video" title="video"></a>

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

 * I'm glad to announce that my paper submitted to [IROS 2014](http://www.iros2014.org/) was accepted! [This paper](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/e/eb/Labbe14-IROS.pdf) explains in details how RGB-D mapping with RTAB-Map is done. Results shown in this paper can be reproduced by the [Multi-session mapping](https://github.com/introlab/rtabmap/wiki/Multi-session) tutorial:

   <a href="https://github.com/introlab/rtabmap/wiki/Multi-session"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/Tutorials/MultiSession/3.png" alt="Multi-session mapping" width="600"></a>


## Videos

 * **RGBD-SLAM**

   <a href="http://www.youtube.com/watch?v=Nm2ggyAW4rw"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/desktop.jpeg" alt="video" title="video"></a>

   <a href="http://www.youtube.com/watch?v=joV7VCvGrKM"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/globalonly.jpeg" alt="video" title="video"></a>

 * **Appearance-based loop closure detection**

   <a href="http://www.youtube.com/watch?v=CAk-QGMlQmI"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/rtabmap.jpeg" alt="video" title="video"></a>

   <a href="http://www.youtube.com/watch?v=1dImRinTJSE"><img src="https://raw.githubusercontent.com/wiki/introlab/rtabmap/doc/video_screenshots/newcollegeomni.jpeg" alt="video" title="video"></a>

 * More loop closure detection videos [here](http://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map).
