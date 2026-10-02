rtabmap
=======

[![RTAB-Map Logo](https://raw.githubusercontent.com/introlab/rtabmap/master/guilib/src/images/RTAB-Map100.png)](http://introlab.github.io/rtabmap)

[![Release][release-image]][releases]
[![Downloads][downloads-image]][downloads]
[![codecov](https://codecov.io/gh/introlab/rtabmap/graph/badge.svg?token=mPwvfZMOia)](https://codecov.io/gh/introlab/rtabmap)
[![License][license-image]][license]

[release-image]: https://img.shields.io/github/v/release/introlab/rtabmap?color=green&style=flat
[releases]: https://github.com/introlab/rtabmap/releases

[downloads-image]: https://img.shields.io/github/downloads/introlab/rtabmap/total?label=downloads
[downloads]: https://github.com/introlab/rtabmap/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/introlab/rtabmap/blob/master/LICENSE

RTAB-Map library and standalone application.

 * For more information (e.g., papers, major updates), visit [RTAB-Map's home page](http://introlab.github.io/rtabmap).
 * For installation instructions and examples, visit [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).
 * For the C++ API of the library, see the [API documentation](https://introlab.github.io/rtabmap/api/latest/), which also lists all [parameters](https://introlab.github.io/rtabmap/api/latest/parameters.html) and [command-line tools](https://introlab.github.io/rtabmap/api/latest/tools.html).

To use RTAB-Map under ROS, visit the [rtabmap](http://wiki.ros.org/rtabmap) page on the ROS wiki.

### Acknowledgements
This project is supported by [IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab](https://introlab.3it.usherbrooke.ca/), Sherbrooke, Québec, Canada.

<a href="https://introlab.3it.usherbrooke.ca/">
<img src="https://github.com/introlab/16SoundsUSB/blob/master/images/IntRoLab.png" alt="IntRoLab" height="100">
</a>

#### CI Latest

| | Build |
|---|---|
| Desktop | [![Linux](https://github.com/introlab/rtabmap/actions/workflows/cmake-linux.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/cmake-linux.yml) [![Windows](https://github.com/introlab/rtabmap/actions/workflows/cmake-windows.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/cmake-windows.yml) [![macOS](https://github.com/introlab/rtabmap/actions/workflows/cmake-macos.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/cmake-macos.yml) |
| ROS | [![CMake ROS](https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/cmake-ros.yml) [![Docker ROS](https://github.com/introlab/rtabmap/actions/workflows/docker-ros.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/docker-ros.yml) |
| Mobile | [![Android](https://github.com/introlab/rtabmap/actions/workflows/android.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/android.yml) [![iOS](https://github.com/introlab/rtabmap/actions/workflows/ios.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/ios.yml) |
| Quality | [![Coverage](https://github.com/introlab/rtabmap/actions/workflows/coverage.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/coverage.yml) [![Documentation](https://github.com/introlab/rtabmap/actions/workflows/docs.yml/badge.svg)](https://github.com/introlab/rtabmap/actions/workflows/docs.yml) |
 
 #### ROS Binaries
 
 `ros-$ROS_DISTRO-rtabmap`
 
| | Distro | Ubuntu | Released | In apt | Build |
|---|---|---|---|---|---|
| ROS 1 | Noetic (EOL) | 20.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fnoetic%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/noetic/distribution.yaml) | [![apt](https://img.shields.io/ros/v/noetic/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#noetic) |  |
| ROS 2 | Humble | 22.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fhumble%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) | [![apt](https://img.shields.io/ros/v/humble/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#humble) | [![build](http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary)](http://build.ros2.org/job/Hbin_uJ64__rtabmap__ubuntu_jammy_amd64__binary/) |
| ROS 2 | Iron (EOL) | 22.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Firon%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/iron/distribution.yaml) | [![apt](https://img.shields.io/ros/v/iron/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#iron) |  |
| ROS 2 | Jazzy | 24.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fjazzy%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml) | [![apt](https://img.shields.io/ros/v/jazzy/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#jazzy) | [![build](http://build.ros2.org/buildStatus/icon?job=Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Jbin_uN64__rtabmap__ubuntu_noble_amd64__binary/) |
| ROS 2 | Kilted | 24.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Fkilted%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/kilted/distribution.yaml) | [![apt](https://img.shields.io/ros/v/kilted/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#kilted) | [![build](http://build.ros2.org/buildStatus/icon?job=Kbin_uN64__rtabmap__ubuntu_noble_amd64__binary)](http://build.ros2.org/job/Kbin_uN64__rtabmap__ubuntu_noble_amd64__binary/) |
| ROS 2 | Lyrical | 26.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Flyrical%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml) | [![apt](https://img.shields.io/ros/v/lyrical/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#lyrical) | [![build](http://build.ros2.org/buildStatus/icon?job=Lbin_uR64__rtabmap__ubuntu_resolute_amd64__binary)](http://build.ros2.org/job/Lbin_uR64__rtabmap__ubuntu_resolute_amd64__binary/) |
| ROS 2 | Rolling | 26.04 | [![released](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fraw.githubusercontent.com%2Fros%2Frosdistro%2Fmaster%2Frolling%2Fdistribution.yaml&query=%24.repositories.rtabmap.release.version&label=%20)](https://github.com/ros/rosdistro/blob/master/rolling/distribution.yaml) | [![apt](https://img.shields.io/ros/v/rolling/rtabmap?label=%20)](https://index.ros.org/p/rtabmap/#rolling) | [![build](http://build.ros2.org/buildStatus/icon?job=Rbin_uR64__rtabmap__ubuntu_resolute_amd64__binary)](http://build.ros2.org/job/Rbin_uR64__rtabmap__ubuntu_resolute_amd64__binary/) |
| Docker | [rtabmap](https://hub.docker.com/r/introlab3it/rtabmap) | | | ![Docker Pulls](https://img.shields.io/docker/pulls/introlab3it/rtabmap.svg?label=pulls) | |

*Released* is the version bloomed into [rosdistro](https://github.com/ros/rosdistro); *In apt* is what `apt install` actually gives you today. They differ while a release is waiting on a buildfarm sync.
