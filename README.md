rtabmap [![Build Status](https://travis-ci.org/introlab/rtabmap.svg?branch=master)](https://travis-ci.org/introlab/rtabmap)
=======

RTAB-Map library and standalone application.

For more information, visit the [RTAB-Map's home page](http://introlab.github.io/rtabmap) or the [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

To use RTAB-Map under ROS, visit the [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki.

## Docker

* RTAB-Map Desktop (ubuntu 16.04)
    ```bash
  $ docker pull introlab3it/rtabmap:latest
  $ docker run --rm -it introlab3it/rtabmap
  # Inside image:
  $ rtabmap
     ```

    * TODO: Try it with redirecting GUI to host computer. Also test if we can actually use a usb device (Kinect) from docker.

* RTAB-Map Tango
    ```bash
  $ docker pull introlab3it/rtabmap:tango
  $ docker run --name=rtabmap-tango introlab3it/rtabmap:tango
  $ docker cp rtabmap-tango:/root/rtabmap-tango/build/app/android/bin/RTABMap-debug.apk .
  $ docker rm rtabmap-tango # cleanup container
  $ adb install -r RTABMap-debug.apk
     ```
    * Note that if we can redirect USB to docker, we could do `adb install` directly from the folder inside docker (which has already adb installed) instead of copying to host.
