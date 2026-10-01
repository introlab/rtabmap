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

const double mapUpdateRate = 1.0;     // Hz, i.e. Rtabmap/DetectionRate
double lastProcessStamp = -1.0;
rtabmap::Transform mapToOdom = rtabmap::Transform::getIdentity();

while(/* frames available */)
{
    rtabmap::SensorData data = camera.takeImage();

    // Odometry sees every frame: dropping any would break the motion tracking.
    rtabmap::Transform odomPose = odometry->process(data);

    // The map is updated at a lower rate. The frames skipped here are not lost
    // work: the motion they carry is already integrated in the pose above, so
    // the next accepted frame arrives with an up-to-date odometry pose.
    if(lastProcessStamp < 0.0 ||
       data.stamp() - lastProcessStamp >= 1.0/mapUpdateRate)
    {
        lastProcessStamp = data.stamp();

        if(rtabmap.process(data, odomPose))   // true when a new node was added
        {
            const rtabmap::Statistics & stats = rtabmap.getStatistics();

            // A loop closure or a proximity detection re-optimizes the graph,
            // which shifts the map frame under the odometry frame.
            if(!stats.mapCorrection().isNull())
            {
                mapToOdom = stats.mapCorrection();
            }

            if(rtabmap.getLoopClosureId() > 0)
            {
                // a loop closure was accepted on this iteration
            }
        }
    }

    // Robot pose in the map frame, on every frame and always with the matching
    // correction: composed after the block above, so an iteration that just
    // re-optimized the graph uses its new correction rather than the previous
    // one. In ROS terms: (/map -> /odom) * (/odom -> /base_link).
    rtabmap::Transform mapPose = mapToOdom * odomPose;
}
~~~

Throttling is the caller's job here: rtabmap::Rtabmap::process() maps every
frame it is given. rtabmap::RtabmapThread does this same stamp comparison
internally, from @ref rtabmap::Parameters::kRtabmapDetectionRate() "Rtabmap/DetectionRate",
so the threaded pipeline only needs the parameter to be set.

Odometry drifts and the graph gets re-optimized, so the odometry pose is not a
map pose. rtabmap::Statistics::mapCorrection() is what reconciles the two, and
since it only changes on a map update it can be applied to every incoming frame
-- which is how the pose stays available at full rate while the map is built at
1 Hz. This is the transform published as `/map` &rarr; `/odom` by the ROS
wrapper, and what the `MapBuilder` of each example composes with the live
odometry pose to place the clouds. It is also readable outside the statistics,
as rtabmap::Rtabmap::getMapCorrection().

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

Occupancy grid
--------------

With @ref rtabmap::Parameters::kRGBDCreateOccupancyGrid() "RGBD/CreateOccupancyGrid"
enabled, every node carries a local occupancy grid computed from its depth images
or laser scan (rtabmap::LocalGridMaker). Assembling those into a global grid is
left to the caller, so that the result always follows the optimized poses:

~~~{.cpp}
#include <rtabmap/core/global_map/OccupancyGrid.h>

rtabmap::LocalGridCache localGrids;
rtabmap::OccupancyGrid grid(&localGrids, parameters);   // reads the Grid/... parameters

// ... inside the "if(rtabmap.process(data, odomPose))" block of the loop above:
const rtabmap::Signature & node = stats.getLastSignatureData();
if(node.sensorData().gridCellSize() > 0.0f &&
   grid.addedNodes().find(node.id()) == grid.addedNodes().end())
{
    // Local grid of the new node, as stored in the database (compressed).
    cv::Mat ground, obstacles, empty;
    node.sensorData().uncompressDataConst(0, 0, 0, 0, &ground, &obstacles, &empty);
    localGrids.add(node.id(), ground, obstacles, empty,
                   node.sensorData().gridCellSize(),
                   node.sensorData().gridViewPoint());
}

// Draws the nodes that are not assembled yet. If the last optimization moved
// poses by more than GridGlobal/UpdateError, the grid is cleared first and
// redrawn entirely from the cache -- which is why the cache is kept around.
grid.update(stats.poses());

float xMin, yMin;                        // grid origin (m), in the map frame
cv::Mat map = grid.getMap(xMin, yMin);   // CV_8S: -1 unknown, 0 free, 100 occupied
~~~

For the node that was just added, the cells are already there uncompressed, and
rtabmap::SensorData::uncompressDataConst() returns them as they are -- it only
decompresses what comes back empty, which is what makes the same code work for
a node retrieved from the database. The occupancy grid is kept on the published
copy even with
@ref rtabmap::Parameters::kRtabmapPublishLastSignature() "Rtabmap/PublishLastSignature"
disabled (only images, scans and user data are dropped), precisely so that the
global grid can still be assembled; statistics themselves must be published
(@ref rtabmap::Parameters::kRtabmapPublishStats() "Rtabmap/PublishStats", on by
default). rtabmap::OccupancyGrid is one of the rtabmap::GlobalMap
back-ends: rtabmap::OctoMap, rtabmap::CloudMap and rtabmap::GridMap consume the
same cache the same way. Each example's `MapBuilder` does exactly this, then
hands the result to the viewer.

For a 3D map, rtabmap::CloudMap assembles the very same cells into PCL clouds
instead of a 2D grid. It shares the cache, so both can be kept up to date from
one set of local grids:

~~~{.cpp}
#include <rtabmap/core/global_map/CloudMap.h>
#include <pcl/io/pcd_io.h>

rtabmap::CloudMap cloudMap(&localGrids, parameters);   // same cache as above

// ... right after the localGrids.add() of the block above:
cloudMap.update(stats.poses());

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground = cloudMap.getMapGround();
pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles = cloudMap.getMapObstacles();
pcl::PointCloud<pcl::PointXYZ>::Ptr emptySpace = cloudMap.getMapEmptyCells();

pcl::io::savePCDFileBinary("obstacles.pcd", *obstacles);
~~~

The clouds are in the map frame and voxelized at
@ref rtabmap::Parameters::kGridCellSize() "Grid/CellSize". Points keep the colour
of the local grid when it has one, otherwise ground is green and obstacles red.
Note that this assembles the *cells*, not the raw sensor clouds: with
@ref rtabmap::Parameters::kGrid3D() "Grid/3D" disabled they are flattened onto
the xy plane, so it must stay enabled for a 3D result.

For a full-resolution cloud, assemble the nodes themselves rather than their
cells. Ask rtabmap::Rtabmap::getGraph() for the optimized poses along with the
node data, then rebuild a cloud per node and transform it to its pose:

~~~{.cpp}
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>

std::map<int, rtabmap::Transform> poses;
std::multimap<int, rtabmap::Link> links;
std::map<int, rtabmap::Signature> nodes;
rtabmap.getGraph(poses, links, true, true, &nodes, true); // optimized, global, with images

pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZRGB>);
for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
{
    rtabmap::SensorData data = nodes.at(iter->first).sensorData();
    data.uncompressData();

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::cloudRGBFromSensorData(
            data,
            4,        // image decimation
            4.0f,     // max depth (m), 0 = no limit
            0.0f,     // min depth (m)
            indices.get());
    cloud = rtabmap::util3d::voxelize(cloud, indices, 0.01f);              // 1 cm
    *assembled += *rtabmap::util3d::transformPointCloud(cloud, iter->second);
}
assembled = rtabmap::util3d::voxelize(assembled, 0.01f);   // one last pass over the overlaps
pcl::io::savePCDFileBinary("cloud.pcd", *assembled);
~~~

@note Do this once the session is over, not on every iteration. It decompresses
and re-projects every node, so the cost grows with the whole map, and the poses
are only worth exporting once the graph has been optimized -- the same cloud
assembled mid-session would carry the drift that later loop closures correct.
This is what @ref tool_export "rtabmap-export" does, with more filtering options.

Memory management
-----------------

RTAB-Map keeps the map in three tiers (rtabmap::Memory): a **short-term memory**
of the last @ref rtabmap::Parameters::kMemSTMSize() "Mem/STMSize" nodes, where
neighbours are too similar to be loop closure candidates; a **working memory**
holding everything loop closure detection compares against; and a **long-term
memory**, the part of the map that stays in the database and is not searched.

By default nothing leaves the working memory, so the iteration time grows with
the map. Memory management caps it, and is enabled by setting a budget -- either
one, or both:

~~~{.cpp}
rtabmap::ParametersMap parameters;
// Keep each update under 700 ms...
parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), "700"));
// ... and/or keep at most 500 nodes in the working memory.
parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapMemoryThr(), "500"));
rtabmap.init(parameters, "map.db");
~~~

When an iteration goes over budget, the nodes of lowest weight are moved to the
long-term memory at the end of it -- age only breaks ties between equal weights,
so it is not simply the oldest that go. Weight is how often a place has been
seen: while a node is still in the short-term memory, a new node similar enough
to it (@ref rtabmap::Parameters::kMemRehearsalSimilarity() "Mem/RehearsalSimilarity")
is merged into it and raises its weight -- the rehearsal mechanism. Places the
robot dwells on or revisits therefore stay in the working memory, while views
seen once leave first. They are not lost: when a loop
closure is found, their neighbours are brought back into the working memory for
the next iterations, up to
@ref rtabmap::Parameters::kRtabmapMaxRetrieved() "Rtabmap/MaxRetrieved" nodes
(plus @ref rtabmap::Parameters::kRGBDMaxLocalRetrieved() "RGBD/MaxLocalRetrieved"
around the current pose and along a planned path). This is what makes long-term
mapping practical: the robot keeps a bounded, relevant working set and pulls the
rest back as it recognizes where it is. Retrieval and node immunization only run
when memory management is on.

Which nodes go first is controlled by three parameters:
@ref rtabmap::Parameters::kMemRecentWmRatio() "Mem/RecentWmRatio" protects the
most recent part of the working memory,
@ref rtabmap::Parameters::kRGBDLocalImmunizationRatio() "RGBD/LocalImmunizationRatio"
protects the nodes around the current pose, and
@ref rtabmap::Parameters::kMemTransferSortingByWeightId() "Mem/TransferSortingByWeightId"
selects the ordering. The step-by-step behaviour is documented on
rtabmap::Rtabmap (steps 4 and 6), and the `Memory/Working_memory_size` and
`Memory/Signatures_retrieved` entries of rtabmap::Statistics report what
happens at runtime.

Configuration
-------------

Every parameter is a string key/value pair in a rtabmap::ParametersMap, declared
with its default and description in `Parameters.h`
(for example `Parameters::kMemSTMSize()`, `Parameters::kRGBDLinearUpdate()`).
The same keys are used by the applications, the ROS wrappers and the
`--Param value` command-line arguments of the tools, so a setting found here
applies everywhere.

The @ref parameters "Parameter reference" lists all of them, grouped, with
their type, default value and description.

~~~{.cpp}
rtabmap::ParametersMap parameters;
parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "20"));
rtabmap.init(parameters, "map.db");
~~~


The main classes
----------------

Doxygen lists the classes alphabetically; this is the same set arranged by the
role they play, as a starting point into the API.

### The map structure

| Class | Role |
| ----- | ---- |
| rtabmap::Rtabmap | The entry point: one SLAM iteration per call, owning everything below |
| rtabmap::Memory | Three-tiered memory (STM / WM / LTM) holding the map and deciding what stays online |
| rtabmap::Signature | One node: sensor data, visual words, pose and links |
| rtabmap::Link | One edge: neighbour, loop closure, landmark or prior constraint |
| rtabmap::DBDriver | Persistence of the map to the database (see rtabmap::DBDriverSqlite3) |
| rtabmap::Statistics | Everything the pipeline reports about an iteration |

### Inputs

| Class | Role |
| ----- | ---- |
| rtabmap::SensorData | An observation: images, depth, laser scan, IMU, GPS, landmarks |
| rtabmap::CameraModel, rtabmap::StereoCameraModel | Intrinsics, extrinsics and rectification |
| rtabmap::LaserScan | Point cloud / laser scan container and its formats |
| rtabmap::Transform | The 3D rigid transform used everywhere in the API |
| rtabmap::SensorCapture, rtabmap::SensorCaptureThread | Drivers and the thread that pumps them |

### Building blocks

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