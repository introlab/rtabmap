RTAB-Map C++ API {#mainpage}
================

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D, stereo and lidar
graph-based SLAM library built around an incremental appearance-based loop
closure detector, with memory management that keeps the online constraints
satisfiable on large-scale, long-term maps.

These pages document the public C++ API of the `rtabmap_core` and
`rtabmap_utilite` libraries. For installation, tutorials and the ROS packages,
see the [project website](https://introlab.github.io/rtabmap/) and the
[wiki](https://github.com/introlab/rtabmap/wiki).

Start here
----------

rtabmap::Rtabmap is the entry point: it owns the map and runs one full SLAM
iteration per call to rtabmap::Rtabmap::process(). A minimal loop feeds it a
rtabmap::SensorData and the odometry pose that goes with it:

~~~{.cpp}
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>

rtabmap::Odometry * odometry = rtabmap::Odometry::create();
rtabmap::Rtabmap rtabmap;
rtabmap.init();                       // optionally: init(parameters, databasePath)

while(/* frames available */)
{
    rtabmap::SensorData data = camera.takeImage();
    rtabmap::Transform pose = odometry->process(data);

    if(rtabmap.process(data, pose))   // true when a new node was added
    {
        const rtabmap::Statistics & stats = rtabmap.getStatistics();
        if(rtabmap.getLoopClosureId() > 0)
        {
            // a loop closure was accepted on this iteration
        }
    }
}
~~~

Complete programs live under
[`examples/`](https://github.com/introlab/rtabmap/tree/master/examples) in the
source tree:

| Example | What it shows |
| ------- | ------------- |
| [BOWMapping](https://github.com/introlab/rtabmap/blob/master/examples/BOWMapping/main.cpp) | The smallest useful loop: images from disk into rtabmap::Rtabmap, appearance-only loop closure detection (no odometry, no GUI) |
| [NoEventsExample](https://github.com/introlab/rtabmap/blob/master/examples/NoEventsExample/main.cpp) | Driving the pipeline by direct calls -- camera, rtabmap::Odometry and rtabmap::Rtabmap in one explicit loop, without the event system |
| [RGBDMapping](https://github.com/introlab/rtabmap/blob/master/examples/RGBDMapping/main.cpp) | The threaded event-based pipeline (rtabmap::SensorCaptureThread &rarr; rtabmap::OdometryThread &rarr; rtabmap::RtabmapThread) with any supported RGB-D or stereo camera |
| [LidarMapping](https://github.com/introlab/rtabmap/blob/master/examples/LidarMapping/main.cpp) | The same threaded pipeline driven by a 3D lidar (rtabmap::LidarVLP16) instead of a camera |

What each iteration does, and which parameters influence it, is documented on
rtabmap::Rtabmap itself -- memory update, loop-closure hypothesis, hypothesis
selection, retrieval, proximity detection and transfer to long-term memory.

Configuration
-------------

Every tunable is a string key/value pair in a rtabmap::ParametersMap, declared
with its default and description in `Parameters.h`
(for example `Parameters::kMemSTMSize()`, `Parameters::kRGBDLinearUpdate()`).
The same keys are used by the applications, the ROS wrappers and the
`--Param value` command-line arguments of the tools, so a setting found here
applies everywhere.

~~~{.cpp}
rtabmap::ParametersMap parameters;
parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "20"));
rtabmap.init(parameters, "map.db");
~~~


The map
-------

| Class | Role |
| ----- | ---- |
| rtabmap::Memory | Three-tiered memory (STM / WM / LTM) holding the map and deciding what stays online |
| rtabmap::Signature | One node: sensor data, visual words, pose and links |
| rtabmap::Link | One edge: neighbour, loop closure, landmark or prior constraint |
| rtabmap::DBDriver | Persistence of the map to the database (see rtabmap::DBDriverSqlite3) |
| rtabmap::Statistics | Everything the pipeline reports about an iteration |

Inputs
------

| Class | Role |
| ----- | ---- |
| rtabmap::SensorData | An observation: images, depth, laser scan, IMU, GPS, landmarks |
| rtabmap::CameraModel, rtabmap::StereoCameraModel | Intrinsics, extrinsics and rectification |
| rtabmap::LaserScan | Point cloud / laser scan container and its formats |
| rtabmap::Transform | The 3D rigid transform used everywhere in the API |
| rtabmap::SensorCapture, rtabmap::SensorCaptureThread | Drivers and the thread that pumps them |

Building blocks
---------------

| Class | Role |
| ----- | ---- |
| rtabmap::Odometry | Visual / lidar odometry front-ends |
| rtabmap::Registration, rtabmap::RegistrationVis, rtabmap::RegistrationIcp | Relative transform between two nodes |
| rtabmap::Optimizer | Graph optimization back-ends (g2o, GTSAM, Ceres, TORO) |
| rtabmap::Feature2D, rtabmap::VWDictionary | Keypoint detectors/descriptors and the bag-of-words dictionary |
| rtabmap::BayesFilter | Loop-closure hypothesis estimation |
| rtabmap::LocalGridMaker, rtabmap::GlobalMap | Occupancy grid generation and assembly |

Free functions for point cloud, image and geometry processing are grouped in
`util2d.h`, `util3d.h`, `util3d_filtering.h`, `util3d_registration.h`,
`util3d_surface.h`, `util3d_transforms.h` and `util3d_mapping.h`.