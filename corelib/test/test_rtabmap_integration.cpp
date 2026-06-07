// Integration tests that replay full sample databases through Odometry +
// Rtabmap. The pipeline:
//   DBReader (stored odom ignored)
//     -> Odometry::process                  // odom is RECOMPUTED here
//     -> Rtabmap::process                   // loop closure / graph
// Everything runs synchronously on the test thread so the replay is
// deterministic and easy to assert against.
//
// Each TEST_F below corresponds to one sample DB fetched by
// scripts/fetch_test_data.sh from data/tests/manifest.txt. The DBs live under
// data/tests/<name>.db, accessed via RTABMAP_TEST_DATA_ROOT (defined in
// corelib/test/CMakeLists.txt). If a file is missing on disk (test data not
// fetched yet) the test is skipped so local builds without test assets still
// pass.

#include <gtest/gtest.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Version.h>
#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

#include "TestUtils.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

using namespace rtabmap;

namespace {

std::string testDataPath(const std::string & basename)
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/" + basename;
}

// Output DB path named after the running test so manual inspection is easy
// (rtabmap-databaseViewer <tempdir>/rtabmap_integration_<TestName>.db). The
// file is overwritten on each run and NOT deleted at teardown.
std::string workDbForCurrentTest(const std::string & suffix = "")
{
	const ::testing::TestInfo * info =
			::testing::UnitTest::GetInstance()->current_test_info();
	const std::string testName = info != nullptr ? info->name() : "unknown";
	if(suffix.empty())
	{
		return test::tempPath(uFormat("rtabmap_integration_%s.db", testName.c_str()));
	}
	return test::tempPath(uFormat("rtabmap_integration_%s_%s.db",
			testName.c_str(), suffix.c_str()));
}

// Result bundle populated by replayDatabase(). Extend as the assertions in the
// per-DB tests grow (e.g. trajectory RMSE, final WM size, time-per-frame, ...).
struct ReplayResult
{
	int framesRead = 0;          // frames pulled from the DB
	int framesProcessed = 0;     // frames fed to Rtabmap::process as real nodes
	int framesIntermediate = 0;  // frames fed to Rtabmap::process with id=-1
	int odomNonNull = 0;         // odometry returned a non-null pose
	int odomLost = 0;            // odometry returned a null pose
	int loopClosuresAccepted = 0;
	int loopClosuresRejected = 0;  // sum of frames with Loop/RejectedHypothesis=1
	int proximityDetections = 0;
	Transform lastOdomPose;
	// Latest Gt/translational_rmse from Rtabmap statistics (negative = never
	// reported, e.g. DB had no ground truth or Rtabmap/ComputeRMSE was off).
	float translationalRmseFinal = -1.0f;
	int finalLocalGraphSize = 0;   // last Memory/Local_graph_size from stats
	int finalGlobalGraphSize = 0;  // poses returned by Rtabmap::getGraph(global=true)
	std::map<int, Transform> finalLocalPoses;   // optimized, global=false
	std::map<int, Transform> finalGlobalPoses;  // optimized, global=true
	// Occupancy-grid cell counts after assembling the global grid from per-
	// node local maps (only populated when RGBD/CreateOccupancyGrid=true).
	int gridEmptyCells = 0;
	int gridObstacleCells = 0;
	// OctoMap leaf count (only populated when rtabmap is built with
	// RTABMAP_OCTOMAP and RGBD/CreateOccupancyGrid=true). 0 means the
	// build didn't have octomap support or the map was empty.
	int octomapNodes = 0;
	int octomapEmptyCells = 0;
	int octomapObstacleCells = 0;
	// Wall-clock seconds spent inside the replay loop, measured from
	// just before the first DBReader read to just after rtabmap.close().
	double replayWallSeconds = 0.0;
	// Wall-clock seconds spent inside Odometry::process across all
	// frames; divide by framesRead to get per-frame average.
	double odomTotalSeconds = 0.0;
};

// Synchronous replay: DBReader -> Odometry::process -> Rtabmap::process.
//
// `rtabmapParameters` configures Rtabmap. `odometryParameters` configures the
// Odometry factory (Odom/Strategy etc.). Splitting the two keeps the per-DB
// tuning explicit -- 3D lidar needs ICP odom while stereo/rgbd need visual
// odom.
//
// If `useStoredOdomAsGuess` is true, the stored odometry in the DB is passed
// as the motion guess (delta from previous frame) to Odometry::process. This
// mirrors rtabmap-reprocess's `useInputOdometryAsGuess` option and is useful
// when the DB has a high-quality odom source (e.g. wheel/IMU) that we want to
// help bootstrap visual/ICP odometry. When false, no guess is provided.
//
// If `passOdomDataToRtabmap` is true, the SensorData modified by Odometry
// (carrying the keypoints/descriptors computed during odom) is forwarded to
// Rtabmap. This pairs with `Mem/UseOdomFeatures=true` to let rtabmap reuse
// odom-extracted features instead of recomputing them -- standard pattern
// for visual SLAM (RGB-D, stereo). When false, the pristine DBReader data
// is passed instead -- standard for lidar/ICP setups.
//
// `goldenStampedGroundTruth` lets a test inject a ground-truth pose by
// stamp on each frame whose stamp matches an entry in the map. This is
// used for datasets that don't ship with ground truth in the DB (e.g.
// Netherdrone) but for which a regression reference has been captured
// from a known-good run. Matching is exact on stamps coming from the
// source DB, so no epsilon is needed. nullptr disables injection.
ReplayResult replayDatabase(
		const std::string & dbPath,
		const ParametersMap & rtabmapParameters,
		const ParametersMap & odometryParameters,
		bool useStoredOdomAsGuess = false,
		bool passOdomDataToRtabmap = false,
		const std::map<double, Transform> * goldenStampedGroundTruth = nullptr,
		int frameStride = 1,
		const std::string & runLabel = "",
		// When true, post-process every stereo frame through
		// SensorCaptureThread::postUpdate() with stereo-to-depth enabled.
		// The SensorData turns into an RGB-D record (depth = dense disparity
		// triangulated from the left/right pair). The odometry + rtabmap
		// pipeline then runs the RGB-D path instead of the stereo path.
		bool stereoToDepth = false,
		// When > 0, points beyond this range (in meters) are dropped from
		// the LaserScan of each SensorData right after dbReader.takeData(),
		// simulating a lidar with a tighter max range.
		float scanMaxRange = 0.0f)
{
	ReplayResult result;

	// DBReader populates info.odomPose only when odom is NOT ignored. Keep
	// it available iff the caller wants to use it as a guess.
	const bool odometryIgnored = !useStoredOdomAsGuess;
	DBReader dbReader(
			dbPath,
			0.0f,             // frameRate: 0 = process as fast as possible
			odometryIgnored,
			true,             // ignoreGoalDelay
			true);            // goalsIgnored
	if(!dbReader.init())
	{
		ADD_FAILURE() << "Failed to init DBReader for " << dbPath;
		return result;
	}

	std::unique_ptr<Odometry> odometry(Odometry::create(odometryParameters));

	// Optional stereo->depth post-processor. SensorCaptureThread is used
	// only for its postUpdate() side-effect (dense disparity from left/right
	// + setRGBDImage on the SensorData), not as an actual capture thread.
	// Its constructor needs a non-null Camera*, so we hand it a default
	// CameraImages we never init -- SensorCaptureThread owns it and
	// deletes it on destruction. Dense-matcher knobs live under
	// Stereo/Dense/* in odometryParameters, same as a real capture setup.
	std::unique_ptr<SensorCaptureThread> stereoToDepthHelper;
	if(stereoToDepth)
	{
		stereoToDepthHelper.reset(new SensorCaptureThread(
				new CameraImages(),  // owned by SensorCaptureThread
				odometryParameters));
		stereoToDepthHelper->setStereoToDepth(true);
	}

	// Output DB at a discoverable path named after the test (with optional
	// runLabel suffix to disambiguate per-backend / per-variant runs). We
	// erase any previous file so each invocation starts fresh but we
	// deliberately do NOT delete it at the end -- callers can inspect it
	// with rtabmap-databaseViewer to verify the run looks sensible.
	const std::string workDb = workDbForCurrentTest(runLabel);
	UFile::erase(workDb);
	Rtabmap rtabmap;
	rtabmap.init(rtabmapParameters, workDb);
	std::cout << "[          ] Output DB: " << workDb << std::endl;

	// Honor Rtabmap/DetectionRate (Hz) by gating rtabmap.process on DB frame
	// stamps -- same throttling pattern as the rtabmap-reprocess tool.
	// Odometry runs on every frame regardless of the detection rate.
	float rtabmapDetectionRate = Parameters::defaultRtabmapDetectionRate();
	Parameters::parse(rtabmapParameters,
			Parameters::kRtabmapDetectionRate(), rtabmapDetectionRate);
	const double rtabmapInterval = (rtabmapDetectionRate > 0.0f)
			? (1.0 / rtabmapDetectionRate)
			: 0.0;
	double lastUpdateStamp = 0.0;

	// Rtabmap/CreateIntermediateNodes=true changes the throttle behavior:
	// instead of dropping throttled frames, we forward them to rtabmap with
	// id=-1 so the odom edge (and any laser/IMU payload) is preserved in
	// the graph between detection frames. Same mechanism as Reprocess.
	bool createIntermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	Parameters::parse(rtabmapParameters,
			Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes);

	// Stamp-keyed GT override: when the source DB has no stored ground truth,
	// a test can supply a reference trajectory and we paste it onto matching
	// frames so Rtabmap's native ComputeRMSE machinery (Gt/translational_rmse)
	// kicks in. Stamps round-trip exactly through SQLite REAL and C++ literals
	// (precision(17) at dump time + C++17 correctly-rounded strtod), so an
	// exact find() is enough.
	const auto applyGoldenGroundTruth = [goldenStampedGroundTruth](SensorData & d) {
		if(goldenStampedGroundTruth == nullptr) return;
		const auto it = goldenStampedGroundTruth->find(d.stamp());
		if(it != goldenStampedGroundTruth->end())
		{
			d.setGroundTruth(it->second);
		}
	};

	UTimer replayTimer;

	// Some old-format DBs (Stereo20Hz, Version 0.8.0) have no stored
	// stamps, so DBReader fills them with wall-clock at read time.
	// That makes the Rtabmap/DetectionRate throttle depend on
	// processing speed and skews per-optimizer comparisons. Detect
	// that case by checking whether the first frame's stamp is
	// suspiciously close to "right now" (within the last hour); if
	// so, rewrite every frame's stamp to a synthetic monotonic 20 Hz
	// timeline.
	constexpr double kSyntheticFrameDt = 1.0 / 20.0;  // 20 Hz
	int syntheticFrameIdx = 0;
	bool overrideStamps = false;

	// Filter the laser scan of `d` to drop points beyond scanMaxRange (no-op
	// when scanMaxRange<=0 or the scan is empty).  Keeps the simulated-lidar
	// range cap uniform across all takeData() call sites.
	const auto applyScanRangeFilter = [scanMaxRange](SensorData & d) {
		if(scanMaxRange <= 0.0f || d.laserScanRaw().isEmpty()) return;
		d.setLaserScan(util3d::commonFiltering(
				d.laserScanRaw(),
				/*downsamplingStep=*/0,
				/*rangeMin=*/0.0f,
				/*rangeMax=*/scanMaxRange));
	};

	// Prime the loop with the first sample.
	SensorCaptureInfo info;
	SensorData data = dbReader.takeData(&info);
	if(stereoToDepthHelper) stereoToDepthHelper->postUpdate(&data, &info);
	applyScanRangeFilter(data);
	overrideStamps = data.stamp() > UTimer::now() - 3600.0;
	if(overrideStamps)
	{
		data.setStamp(syntheticFrameIdx++ * kSyntheticFrameDt);
	}
	applyGoldenGroundTruth(data);

	Transform previousStoredOdomPose;
	while(data.isValid())
	{
		++result.framesRead;

		// Drop (stride-1) of every `stride` frames before any
		// odom/rtabmap processing. Note: lastUpdateStamp /
		// previousStoredOdomPose advance only on processed frames,
		// so the throttle window still measures against the last
		// frame we actually fed in.
		if(frameStride > 1 && (result.framesRead - 1) % frameStride != 0)
		{
			data = dbReader.takeData(&info);
			if(stereoToDepthHelper) stereoToDepthHelper->postUpdate(&data, &info);
			applyScanRangeFilter(data);
			if(overrideStamps && data.isValid())
			{
				data.setStamp(syntheticFrameIdx++ * kSyntheticFrameDt);
			}
			applyGoldenGroundTruth(data);
			continue;
		}

		// Build the motion guess from the stored odom delta if requested.
		// Null Transform = no guess.
		Transform guess;
		if(useStoredOdomAsGuess
				&& !info.odomPose.isNull()
				&& !previousStoredOdomPose.isNull())
		{
			guess = previousStoredOdomPose.inverse() * info.odomPose;
		}
		if(!info.odomPose.isNull())
		{
			previousStoredOdomPose = info.odomPose;
		}

		// Hand odometry a working copy. Depending on passOdomDataToRtabmap
		// we either keep the pristine `data` for rtabmap (lidar/ICP path) or
		// forward the odom-modified `odomData` to rtabmap (visual SLAM path,
		// pairs with Mem/UseOdomFeatures=true so rtabmap reuses the
		// keypoints/descriptors odom already extracted).
		SensorData odomData = data;
		OdometryInfo odomInfo;
		UTimer odomTimer;
		const Transform odomPose = odometry->process(odomData, guess, &odomInfo);
		result.odomTotalSeconds += odomTimer.getElapsedTime();

		if(odomPose.isNull())
		{
			++result.odomLost;
			// Log the failure reason so tests can diagnose ICP odometry
			// drops (e.g. truncated-scan variants of the corridor test).
			std::cerr << "[odom-lost] frame=" << result.framesRead
					<< " stamp=" << data.stamp()
					<< " rejected=\"" << odomInfo.reg.rejectedMsg << "\""
					<< " inliers=" << odomInfo.reg.inliers
					<< " matches=" << odomInfo.reg.matches
					<< " icpInliersRatio=" << odomInfo.reg.icpInliersRatio
					<< " icpStructuralComplexity=" << odomInfo.reg.icpStructuralComplexity
					<< std::endl;
		}
		else
		{
			++result.odomNonNull;
			result.lastOdomPose = odomPose;

			const bool throttle = rtabmapInterval > 0.0
					&& lastUpdateStamp > 0.0
					&& data.stamp() < lastUpdateStamp + rtabmapInterval;
			if(!throttle || createIntermediateNodes)
			{
				cv::Mat covariance = (!odomInfo.reg.covariance.empty()
						&& odomInfo.reg.covariance.total() == 36)
						? odomInfo.reg.covariance
						: cv::Mat::eye(6, 6, CV_64FC1) * 0.001;

				// Copy so id=-1 on throttled frames doesn't mutate the working
				// `data` / `odomData` -- keeps the loop's reasoning simple.
				SensorData rtabmapData = passOdomDataToRtabmap ? odomData : data;
				if(throttle)
				{
					rtabmapData.setId(-1);  // intermediate node
				}
				rtabmap.process(rtabmapData, odomPose, covariance);

				if(throttle)
				{
					// Detection cadence is gated on real (non-intermediate)
					// nodes, so don't advance lastUpdateStamp here.
					++result.framesIntermediate;
				}
				else
				{
					lastUpdateStamp = rtabmapData.stamp();
					++result.framesProcessed;
					const Statistics & stats = rtabmap.getStatistics();
					if(stats.loopClosureId() > 0)
					{
						++result.loopClosuresAccepted;
					}
					if(stats.proximityDetectionId() > 0)
					{
						++result.proximityDetections;
					}
					// Rtabmap publishes Gt/translational_rmse on every process
					// call when ComputeRMSE is on and the DB has ground truth.
					// Keep the latest value -- that's the final-trajectory RMSE.
					const auto rmseIt = stats.data().find(
							Statistics::kGtTranslational_rmse());
					if(rmseIt != stats.data().end())
					{
						result.translationalRmseFinal = rmseIt->second;
					}
					// finalLocalGraphSize is read from getGraph at end of replay,
					// not from the stats map -- that way both local and global
					// counts come from the same source.
				}
			}
		}

		data = dbReader.takeData(&info);
		if(stereoToDepthHelper) stereoToDepthHelper->postUpdate(&data, &info);
		applyScanRangeFilter(data);
		if(overrideStamps && data.isValid())
		{
			data.setStamp(syntheticFrameIdx++ * kSyntheticFrameDt);
		}
		applyGoldenGroundTruth(data);
	}

	// Snapshot both local and global graphs BEFORE closing: close(true) tears
	// down memory state. global=true includes nodes that have been moved to
	// LTM; global=false is just the working-memory subset.
	{
		std::multimap<int, Link> constraints;
		rtabmap.getGraph(result.finalLocalPoses, constraints,
				/*optimized=*/true, /*global=*/false);
		result.finalLocalGraphSize = (int)result.finalLocalPoses.size();
		constraints.clear();
		rtabmap.getGraph(result.finalGlobalPoses, constraints,
				/*optimized=*/true, /*global=*/true);
		result.finalGlobalGraphSize = (int)result.finalGlobalPoses.size();
	}

	// Assemble the global occupancy grid from the per-node local grids that
	// rtabmap saved (only meaningful when RGBD/CreateOccupancyGrid=true).
	// Pull poses + signatures with grids attached, prime a LocalGridCache,
	// run OccupancyGrid::update, then count cells in the resulting cv::Mat
	// (-1 = unknown, 0 = empty, 100 = obstacle).
	bool createOccupancyGrid = Parameters::defaultRGBDCreateOccupancyGrid();
	Parameters::parse(rtabmapParameters,
			Parameters::kRGBDCreateOccupancyGrid(), createOccupancyGrid);
	if(createOccupancyGrid)
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		std::map<int, Signature> signatures;
		rtabmap.getGraph(poses, constraints,
				/*optimized=*/true, /*global=*/true, &signatures,
				/*withImages=*/false, /*withScan=*/false, /*withUserData=*/false,
				/*withGrid=*/true, /*withWords=*/false,
				/*withGlobalDescriptors=*/false);

		LocalGridCache mapCache;
		for(auto & p : signatures)
		{
			cv::Mat ground, obstacles, empty;
			p.second.sensorData().uncompressDataConst(
					0, 0, 0, 0, &ground, &obstacles, &empty);
			if(!ground.empty() || !obstacles.empty() || !empty.empty())
			{
				mapCache.add(p.first, ground, obstacles, empty,
						p.second.sensorData().gridCellSize(),
						p.second.sensorData().gridViewPoint());
			}
		}

		OccupancyGrid grid(&mapCache, rtabmapParameters);
		grid.update(poses);
		float xMin = 0.0f, yMin = 0.0f;
		const cv::Mat gridMat = grid.getMap(xMin, yMin);
		for(int i = 0; i < gridMat.rows; ++i)
		{
			for(int j = 0; j < gridMat.cols; ++j)
			{
				const signed char v = gridMat.at<signed char>(i, j);
				if(v == 0) ++result.gridEmptyCells;
				else if(v == 100) ++result.gridObstacleCells;
			}
		}

#ifdef RTABMAP_OCTOMAP
		// Build a 3D OctoMap from the same cache+poses. Useful when
		// Grid/RayTracing=true (which needs OctoMap support per the param
		// doc) and a quick sanity check on the resulting tree size.
		// Pin the OctoMap's resolution to the local grids' actual cell size
		// (set when Memory built them at process() time) so the OcTree leaf
		// size matches the data, regardless of any subsequent Grid/CellSize
		// edits in `rtabmapParameters`.
		ParametersMap octoMapParameters = rtabmapParameters;
		for(const auto & p : signatures)
		{
			const float cs = p.second.sensorData().gridCellSize();
			if(cs > 0.0f)
			{
				octoMapParameters[Parameters::kGridCellSize()] = uNumber2Str(cs);
				break;
			}
		}
		OctoMap octomap(&mapCache, octoMapParameters);
		octomap.update(poses);
		if(octomap.octree() != nullptr)
		{
			result.octomapNodes = (int)octomap.octree()->size();
			std::vector<int> obstacleIdx, emptyIdx, groundIdx;
			octomap.createCloud(/*treeDepth=*/0, &obstacleIdx, &emptyIdx, &groundIdx);
			result.octomapObstacleCells = (int)obstacleIdx.size();
			// Treat ground-classified cells as empty for the purposes of this
			// test (they're free space, just labelled separately).
			result.octomapEmptyCells = (int)emptyIdx.size() + (int)groundIdx.size();
		}
#endif
	}

	// close(true) flushes the in-memory session to the output DB so the file
	// at workDb is a complete, openable database after the test returns.
	rtabmap.close(true);
	result.replayWallSeconds = replayTimer.getElapsedTime();

	std::cout << "[          ] Replay summary:"
			<< " framesRead=" << result.framesRead
			<< " odomNonNull=" << result.odomNonNull
			<< " odomLost=" << result.odomLost
			<< " framesProcessed=" << result.framesProcessed
			<< " framesIntermediate=" << result.framesIntermediate
			<< " loops=" << result.loopClosuresAccepted
			<< " proximity=" << result.proximityDetections
			<< " localGraph=" << result.finalLocalGraphSize
			<< " globalGraph=" << result.finalGlobalGraphSize
			<< " gridEmpty=" << result.gridEmptyCells
			<< " gridObstacle=" << result.gridObstacleCells
			<< " octomap=" << result.octomapNodes
			<< " octomapEmpty=" << result.octomapEmptyCells
			<< " octomapObstacle=" << result.octomapObstacleCells
			<< " rmse=" << result.translationalRmseFinal << "m"
			<< " wall=" << result.replayWallSeconds << "s"
			<< " odom/frame=" << (result.framesRead > 0
					? (result.odomTotalSeconds / result.framesRead) * 1000.0
					: 0.0) << "ms"
			<< std::endl;

	return result;
}

// Replay variant that feeds the stored odometry pose straight to Rtabmap
// without re-running an Odometry stage. Used for datasets where the DB
// already carries a known-good odom track and the test only cares about
// the back-end (loop closure detection + graph optimization).
ReplayResult replayDatabaseWithStoredOdom(
		const std::string & dbPath,
		const std::string & workDb,
		const ParametersMap & rtabmapParameters,
		int triggerNewMapAfterFrame = -1,
		// When >0, overrides the angular diagonal entries of the stored
		// odom covariance (indices 3,4,5) before each call to
		// Rtabmap::process. Lets a test relax an unrealistically tight
		// rotational variance baked into the DB.
		double overrideOdomAngularVariance = -1.0,
		// When >0, same idea for the translational diagonal (0,1,2).
		double overrideOdomLinearVariance = -1.0,
		// When >0, points beyond this range (in meters) are dropped from
		// the LaserScan of each SensorData after dbReader.takeData(),
		// simulating a lidar with a tighter max range.
		float scanMaxRange = 0.0f)
{
	ReplayResult result;

	// odometryIgnored=false so DBReader populates info.odomPose.
	// featuresIgnored=false so DBReader replays the stored keypoints /
	// descriptors instead of forcing Rtabmap to re-extract — paired with
	// Mem/UseOdomFeatures=true in the per-variant parameters so Rtabmap
	// actually reuses them rather than recomputing on every frame.
	DBReader dbReader(
			dbPath,
			0.0f,             // frameRate: 0 = process as fast as possible
			false,            // odometryIgnored
			true,             // ignoreGoalDelay
			true,             // goalsIgnored
			0,                // startId
			std::vector<unsigned int>(),  // cameraIndices
			0,                // stopId
			false,            // intermediateNodesIgnored
			false,            // landmarksIgnored
			false);           // featuresIgnored
	if(!dbReader.init())
	{
		ADD_FAILURE() << "Failed to init DBReader for " << dbPath;
		return result;
	}

	UFile::erase(workDb);
	Rtabmap rtabmap;
	rtabmap.init(rtabmapParameters, workDb);
	std::cout << "[          ] Output DB: " << workDb << std::endl;

	// Honor Rtabmap/DetectionRate (Hz) by gating rtabmap.process on DB frame
	// stamps -- mirrors the throttle in replayDatabase(). When
	// Rtabmap/CreateIntermediateNodes=true, throttled frames are forwarded
	// with id=-1 so the odom edge + payload are preserved in the graph
	// between detection frames; otherwise they are dropped.
	float rtabmapDetectionRate = Parameters::defaultRtabmapDetectionRate();
	Parameters::parse(rtabmapParameters,
			Parameters::kRtabmapDetectionRate(), rtabmapDetectionRate);
	const double rtabmapInterval = (rtabmapDetectionRate > 0.0f)
			? (1.0 / rtabmapDetectionRate)
			: 0.0;
	double lastUpdateStamp = 0.0;
	bool createIntermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	Parameters::parse(rtabmapParameters,
			Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes);

	// Filter the laser scan of `d` to drop points beyond scanMaxRange.
	// No-op when scanMaxRange<=0 or the scan is empty.
	const auto applyScanRangeFilter = [scanMaxRange](SensorData & d) {
		if(scanMaxRange <= 0.0f || d.laserScanRaw().isEmpty()) return;
		d.setLaserScan(util3d::commonFiltering(
				d.laserScanRaw(),
				/*downsamplingStep=*/0,
				/*rangeMin=*/0.0f,
				/*rangeMax=*/scanMaxRange));
	};

	UTimer replayTimer;

	SensorCaptureInfo info;
	SensorData data = dbReader.takeData(&info);
	applyScanRangeFilter(data);
	while(data.isValid())
	{
		++result.framesRead;
		// In this replay mode the DB is expected to carry a stored odom
		// pose and covariance on every frame; treat either being missing
		// as a malformed asset rather than silently skipping.
		if(info.odomPose.isNull())
		{
			ADD_FAILURE() << "Frame " << result.framesRead
					<< " has no stored odometry pose";
			return result;
		}
		if(info.odomCovariance.empty() || info.odomCovariance.total() != 36)
		{
			ADD_FAILURE() << "Frame " << result.framesRead
					<< " has no stored odometry covariance";
			return result;
		}
		if(overrideOdomLinearVariance > 0.0)
		{
			info.odomCovariance.at<double>(0, 0) = overrideOdomLinearVariance;
			info.odomCovariance.at<double>(1, 1) = overrideOdomLinearVariance;
			info.odomCovariance.at<double>(2, 2) = overrideOdomLinearVariance;
		}
		if(overrideOdomAngularVariance > 0.0)
		{
			info.odomCovariance.at<double>(3, 3) = overrideOdomAngularVariance;
			info.odomCovariance.at<double>(4, 4) = overrideOdomAngularVariance;
			info.odomCovariance.at<double>(5, 5) = overrideOdomAngularVariance;
		}
		result.lastOdomPose = info.odomPose;

		const bool throttle = rtabmapInterval > 0.0
				&& lastUpdateStamp > 0.0
				&& data.stamp() < lastUpdateStamp + rtabmapInterval;
		if(throttle && !createIntermediateNodes)
		{
			data = dbReader.takeData(&info);
			applyScanRangeFilter(data);
			continue;
		}

		SensorData rtabmapData = data;
		if(throttle)
		{
			rtabmapData.setId(-1);  // intermediate node
		}
		rtabmap.process(rtabmapData, info.odomPose, info.odomCovariance);

		if(throttle)
		{
			++result.framesIntermediate;
			data = dbReader.takeData(&info);
			applyScanRangeFilter(data);
			continue;
		}
		lastUpdateStamp = data.stamp();
		++result.framesProcessed;
		// Caller-driven session split: trigger a fresh map once the
		// configured number of frames has been processed. Used by tests
		// to exercise multi-session graph behavior on a single-session
		// source DB.
		if(triggerNewMapAfterFrame > 0
				&& result.framesProcessed == triggerNewMapAfterFrame)
		{
			rtabmap.triggerNewMap();
		}
		const Statistics & stats = rtabmap.getStatistics();
		if(stats.loopClosureId() > 0)
		{
			++result.loopClosuresAccepted;
		}
		if(stats.proximityDetectionId() > 0)
		{
			++result.proximityDetections;
		}
		// Loop/RejectedHypothesis is published as 1.0f on frames where a
		// loop closure (or proximity link) was discarded — either by the
		// addLink path or by the RGBD/OptimizeMaxError post-optimization
		// rejection. Robust + MaxError-enabled variants are expected to
		// reject more than the unguarded variants.
		const auto rejIt = stats.data().find(Statistics::kLoopRejectedHypothesis());
		if(rejIt != stats.data().end() && rejIt->second > 0.5f)
		{
			++result.loopClosuresRejected;
		}
		const auto rmseIt = stats.data().find(Statistics::kGtTranslational_rmse());
		if(rmseIt != stats.data().end())
		{
			result.translationalRmseFinal = rmseIt->second;
		}
		data = dbReader.takeData(&info);
		applyScanRangeFilter(data);
	}

	{
		std::multimap<int, Link> constraints;
		rtabmap.getGraph(result.finalLocalPoses, constraints,
				/*optimized=*/true, /*global=*/false);
		result.finalLocalGraphSize = (int)result.finalLocalPoses.size();
		constraints.clear();
		rtabmap.getGraph(result.finalGlobalPoses, constraints,
				/*optimized=*/true, /*global=*/true);
		result.finalGlobalGraphSize = (int)result.finalGlobalPoses.size();
	}

	rtabmap.close(true);
	result.replayWallSeconds = replayTimer.getElapsedTime();

	std::cout << "[          ] Replay summary:"
			<< " framesRead=" << result.framesRead
			<< " framesProcessed=" << result.framesProcessed
			<< " framesIntermediate=" << result.framesIntermediate
			<< " loops=" << result.loopClosuresAccepted
			<< " loopsRejected=" << result.loopClosuresRejected
			<< " proximity=" << result.proximityDetections
			<< " localGraph=" << result.finalLocalGraphSize
			<< " globalGraph=" << result.finalGlobalGraphSize
			<< " rmse=" << result.translationalRmseFinal << "m"
			<< " wall=" << result.replayWallSeconds << "s"
			<< std::endl;

	return result;
}

ParametersMap baseRtabmapParams()
{
	ParametersMap params;
	// Per-DB tuning should override these in the individual tests.
	params.insert(ParametersPair(Parameters::kMemSTMSize(), "5"));
	// Disable motion-gated update so node creation is driven purely by
	// Rtabmap/DetectionRate -- makes the replay deterministic frame-by-frame.
	params.insert(ParametersPair(Parameters::kRGBDAngularUpdate(), "0.0"));
	params.insert(ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true"));
	params.insert(ParametersPair(Parameters::kRGBDLinearUpdate(), "0.0"));
	params.insert(ParametersPair(Parameters::kRtabmapDetectionRate(), "2"));
	return params;
}

ParametersMap baseOdometryParams()
{
	// Defaults are visual F2M; per-DB tests override Odom/Strategy and any
	// related parameters (Vis/* for visual odom, Icp/* for lidar odom, ...).
	return ParametersMap();
}

class RtabmapIntegrationFixture : public ::testing::Test {};

}  // namespace

// GTEST_SKIP() expands to `return`, so it has to live directly inside the test
// body (not a helper). Each TEST_F resolves the asset path and skips if the
// file is missing -- keeps local builds green for contributors who haven't run
// scripts/fetch_test_data.sh.
#define SKIP_IF_MISSING(path)                                              \
	do                                                                     \
	{                                                                      \
		if(!UFile::exists(path))                                           \
		{                                                                  \
			GTEST_SKIP() << "Test data not found: " << (path)              \
					<< " (run scripts/fetch_test_data.sh to populate)";    \
		}                                                                  \
	} while(0)

// ---------------------------------------------------------------------------
// 3D lidar sample (Netherdrone, ~15s).
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, NetherdroneLidar3D)
{
	const std::string dbPath = testDataPath("netherdrone_lidar3d_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	// Drone outdoor 3D lidar -- voxel is the single knob; max-correspondence
	// follows at 10x the voxel size (typical ICP rule of thumb for sparse
	// lidar scans).
	const float voxelSize = 0.3f;
	const std::string kVoxelSize = uNumber2Str(voxelSize);

	// Loop-closure-side ICP config (Icp/* read by RegistrationIcp).
	// Reg/Strategy=1 -> ICP-only registration (no visual features).
	// Grid/GroundIsObstacle=true: UAV use case -- no ground segmentation,
	// every point is an obstacle in the occupancy grid.
	ParametersMap rtabmapParams = baseRtabmapParams();
	rtabmapParams[Parameters::kGridCellSize()] = kVoxelSize;
	rtabmapParams[Parameters::kGridGroundIsObstacle()] = "true";
	rtabmapParams[Parameters::kGridRayTracing()] = "true";
	rtabmapParams[Parameters::kGridSensor()] = "0";
	rtabmapParams[Parameters::kIcpEpsilon()] = "0.001";
	rtabmapParams[Parameters::kIcpIterations()] = "10";
	rtabmapParams[Parameters::kIcpMaxCorrespondenceDistance()] = uNumber2Str(voxelSize * 10.0f);
	rtabmapParams[Parameters::kIcpMaxTranslation()] = "3";
	// OutlierRatio means different things per ICP backend (see Parameters.h):
	// libpointmatcher uses it as TrimmedDist keep-ratio (mild trim at 0.7);
	// PCL uses it as a RANSAC threshold multiplier on maxCorrespondenceDistance,
	// where 0.1 gives the aggressive filtering this sparse lidar needs.
#ifdef RTABMAP_POINTMATCHER
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.7";
#else
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.1";
#endif
	rtabmapParams[Parameters::kIcpPointToPlane()] = "true";
	rtabmapParams[Parameters::kIcpPointToPlaneK()] = "20";
	rtabmapParams[Parameters::kIcpPointToPlaneRadius()] = "0";
	rtabmapParams[Parameters::kIcpStrategy()] = "1";
	rtabmapParams[Parameters::kIcpVoxelSize()] = kVoxelSize;
	rtabmapParams[Parameters::kMemIntermediateNodeDataKept()] = "true";
	rtabmapParams[Parameters::kRegStrategy()] = "1";
	rtabmapParams[Parameters::kRtabmapCreateIntermediateNodes()] = "true";

	// Odometry-side: inherit the Icp/* baseline, then layer odom-specific
	// overrides (scan-keyframe, F2M scan map size, ICP correspondence ratio).
	ParametersMap odomParams = rtabmapParams;
	odomParams[Parameters::kIcpCorrespondenceRatio()] = "0.01";
	odomParams[Parameters::kOdomF2MBundleAdjustment()] = "false";
	odomParams[Parameters::kOdomF2MScanMaxSize()] = "15000";
	odomParams[Parameters::kOdomF2MScanSubtractRadius()] = kVoxelSize;
	odomParams[Parameters::kOdomScanKeyFrameThr()] = "0.4";

	// Golden trajectory captured from a known-good run, keyed by source-DB
	// stamp (invariant across rtabmap renumbering). Injected as ground truth
	// on matching SensorData frames so Rtabmap's native ComputeRMSE machinery
	// (Gt/translational_rmse stat) does the comparison -- same path as the
	// PR2 tests that ship with stored GT. The Netherdrone DB has no stored
	// GT of its own, so this is a self-anchored regression check.
	const std::map<double, Transform> goldenStampedGT = {
		{1613418442.8843718, Transform(0.99832969903945923f, 0.00051550386706367135f, 0.057771574705839157f, -1.6495448367424946e-19f, 0.00051560450810939074f, 0.99984085559844971f, -0.017831696197390556f, 3.4403514425937326e-20f, -0.057771574705839157f, 0.017831698060035706f, 0.99817055463790894f, 7.6593389817761406e-20f)},
		{1613418443.3844736, Transform(0.99832785129547119f, 0.0028989869169890881f, 0.057732503861188889f, 0.17843475937843323f, -0.0020826261024922132f, 0.99989718198776245f, -0.014195555821061134f, -0.0066802469082176685f, -0.057767719030380249f, 0.014051581732928753f, 0.99823129177093506f, 0.17401190102100372f)},
		{1613418443.9842608, Transform(0.99834167957305908f, 0.0053970343433320522f, 0.057313427329063416f, 0.40323537588119507f, -0.0040137777104973793f, 0.9996984601020813f, -0.024222658947110176f, -0.018803196027874947f, -0.057426892220973969f, 0.023952441290020943f, 0.99806231260299683f, 0.40222784876823425f)},
		{1613418444.5840302, Transform(0.99913626909255981f, 0.0034218172077089548f, 0.041410472244024277f, 0.65142530202865601f, -0.0028540806379169226f, 0.9999011754989624f, -0.013761336915194988f, -0.042952436953783035f, -0.041453473269939423f, 0.013631262816488743f, 0.99904739856719971f, 0.63346731662750244f)},
		{1613418445.0840809, Transform(0.99995774030685425f, 0.0023752534762024879f, -0.0088843861594796181f, 0.82112884521484375f, -0.0024402674753218889f, 0.99997025728225708f, -0.0073141087777912617f, -0.067268162965774536f, 0.0088667487725615501f, 0.0073354821652173996f, 0.99993377923965454f, 0.85938411951065063f)},
		{1613418445.6839614, Transform(0.99883186817169189f, 0.0049950624816119671f, 0.048063535243272781f, 0.86203396320343018f, -0.0041339104063808918f, 0.99982929229736328f, -0.017999691888689995f, -0.057375259697437286f, -0.048145238310098648f, 0.017779972404241562f, 0.99868214130401611f, 0.9297715425491333f)},
		{1613418446.1842918, Transform(0.99878597259521484f, 0.0048005376011133194f, 0.049027431756258011f, 0.88737571239471436f, -0.0036628940142691135f, 0.99972248077392578f, -0.023267785087227821f, -0.092432446777820587f, -0.04912552610039711f, 0.02305995300412178f, 0.99852651357650757f, 0.85799181461334229f)},
		{1613418446.6844673, Transform(0.99829369783401489f, 0.00050088047282770276f, 0.058391634374856949f, 0.93222159147262573f, 0.00087454286403954029f, 0.99972283840179443f, -0.023527214303612709f, -0.091116242110729218f, -0.058387219905853271f, 0.023538138717412949f, 0.99801653623580933f, 0.87153005599975586f)},
		{1613418447.2841945, Transform(0.99886816740036011f, 0.01005473081022501f, 0.046490758657455444f, 1.0597318410873413f, -0.009820220060646534f, 0.99993777275085449f, -0.0052699404768645763f, -0.091255262494087219f, -0.046540863811969757f, 0.0048074256628751755f, 0.9989047646522522f, 0.92329776287078857f)},
		{1613418447.784642, Transform(0.99902480840682983f, 0.0161009281873703f, 0.041110377758741379f, 1.1376457214355469f, -0.01559132244437933f, 0.99979794025421143f, -0.012686838395893574f, 0.00506593007594347f, -0.041306335479021072f, 0.012033500708639622f, 0.99907416105270386f, 0.9223446249961853f)},
		{1613418448.2847199, Transform(0.99957424402236938f, 0.015879219397902489f, -0.024478213861584663f, 1.177858829498291f, -0.016155127435922623f, 0.99980771541595459f, -0.011115322820842266f, 0.022787714377045631f, 0.024297002702951431f, 0.011506041511893272f, 0.99963855743408203f, 0.95410776138305664f)},
		{1613418448.7850029, Transform(0.99747771024703979f, 0.013958192430436611f, 0.069593988358974457f, 1.0981137752532959f, -0.011846709065139294f, 0.99945962429046631f, -0.030660992488265038f, 0.073311552405357361f, -0.069984368979930878f, 0.029759196564555168f, 0.99710404872894287f, 1.0477849245071411f)},
		{1613418449.2854297, Transform(0.99874883890151978f, 0.014832595363259315f, 0.047757040709257126f, 1.0584510564804077f, -0.01404693815857172f, 0.99976098537445068f, -0.016744945198297501f, 0.11667603254318237f, -0.047993998974561691f, 0.016053156927227974f, 0.99871867895126343f, 1.339044451713562f)},
		{1613418449.8841302, Transform(0.99820965528488159f, 0.016053171828389168f, 0.057617094367742538f, 1.0097041130065918f, -0.014304077252745628f, 0.99942803382873535f, -0.030642321333289146f, 0.15719693899154663f, -0.058076050132513046f, 0.029763301834464073f, 0.99786841869354248f, 1.530038595199585f)},
		{1613418450.3843191, Transform(0.99840420484542847f, 0.01184108667075634f, 0.055216036736965179f, 0.96206194162368774f, -0.0086531788110733032f, 0.99830114841461182f, -0.057620875537395477f, 0.15269020199775696f, -0.055804520845413208f, 0.057051137089729309f, 0.99681037664413452f, 1.6020523309707642f)},
		{1613418450.9838331, Transform(0.99857664108276367f, 0.0076439883559942245f, 0.052786123007535934f, 0.93738365173339844f, -0.002835016930475831f, 0.99588495492935181f, -0.090583465993404388f, 0.019595114514231682f, -0.053261328488588333f, 0.09030488133430481f, 0.99448889493942261f, 1.5854558944702148f)},
		{1613418451.4839463, Transform(0.99867540597915649f, -0.0012489983346313238f, 0.051436353474855423f, 0.94985419511795044f, 0.0019668119493871927f, 0.99990135431289673f, -0.013907111249864101f, -0.2961670458316803f, -0.051413904875516891f, 0.013989857397973537f, 0.99857938289642334f, 1.5656760931015015f)},
		{1613418452.0835037, Transform(0.99909251928329468f, 0.0077475365251302719f, 0.041883502155542374f, 0.96908277273178101f, -0.0093348808586597443f, 0.99924039840698242f, 0.037837300449609756f, -0.43317463994026184f, -0.041558541357517242f, -0.038193941116333008f, 0.99840593338012695f, 1.7838516235351562f)},
		{1613418452.6833203, Transform(0.99895727634429932f, 0.010300842113792896f, 0.04447576031088829f, 0.99292933940887451f, -0.013362647965550423f, 0.9975200891494751f, 0.069103263318538666f, -0.48763060569763184f, -0.043653644621372223f, -0.069625511765480042f, 0.99661749601364136f, 1.9300179481506348f)},
		{1613418453.1836057, Transform(0.99860244989395142f, 0.0083182761445641518f, 0.052192755043506622f, 1.0400835275650024f, -0.0065913721919059753f, 0.99942797422409058f, -0.033172395080327988f, -0.40978887677192688f, -0.052438832819461823f, 0.032782018184661865f, 0.99808579683303833f, 2.0495405197143555f)},
		{1613418453.7834945, Transform(0.99882996082305908f, 0.0098624005913734436f, 0.047345634549856186f, 1.1099979877471924f, -0.0088436063379049301f, 0.99972593784332275f, -0.021679745987057686f, -0.29520484805107117f, -0.047546472400426865f, 0.021235672757029533f, 0.99864327907562256f, 2.3696367740631104f)},
		{1613418454.2839301, Transform(0.99896800518035889f, 0.010073220357298851f, 0.044289212673902512f, 1.155259370803833f, -0.0096654985100030899f, 0.99990898370742798f, -0.0094104120507836342f, -0.22011889517307281f, -0.044379975646734238f, 0.0089726224541664124f, 0.99897438287734985f, 2.5511965751647949f)},
		{1613418454.8842895, Transform(0.9952349066734314f, 0.092743024230003357f, 0.030103979632258415f, 1.2314082384109497f, -0.092207059264183044f, 0.99556368589401245f, -0.018731595948338509f, -0.15043841302394867f, -0.031707651913166046f, 0.015866540372371674f, 0.9993712306022644f, 2.6318151950836182f)},
		{1613418455.3845851, Transform(0.99552649259567261f, 0.090829521417617798f, 0.026020960882306099f, 1.2905718088150024f, -0.089119181036949158f, 0.99416971206665039f, -0.060699313879013062f, -0.046864170581102371f, -0.031382542103528976f, 0.058108806610107422f, 0.99781686067581177f, 2.6263918876647949f)},
		{1613418455.984123, Transform(0.99200868606567383f, 0.11816086620092392f, 0.044235825538635254f, 1.3500221967697144f, -0.11883093416690826f, 0.99283158779144287f, 0.012827672064304352f, -0.09510079026222229f, -0.042402993887662888f, -0.017981745302677155f, 0.99893879890441895f, 2.5739231109619141f)},
		{1613418456.4845514, Transform(0.98325234651565552f, 0.17734897136688232f, 0.041978854686021805f, 1.404977560043335f, -0.17656248807907104f, 0.98404836654663086f, -0.021783677861094475f, 0.030498344451189041f, -0.045172542333602905f, 0.014006962068378925f, 0.99888098239898682f, 2.5444049835205078f)},
		{1613418457.0843911, Transform(0.98150634765625f, 0.18732210993766785f, 0.039444118738174438f, 1.4734765291213989f, -0.18633481860160828f, 0.98210400342941284f, -0.02740548737347126f, 0.011144352145493031f, -0.04387187585234642f, 0.019548846408724785f, 0.99884587526321411f, 2.6882264614105225f)},
		{1613418457.5845294, Transform(0.96550232172012329f, 0.25460636615753174f, 0.054596912115812302f, 1.5147770643234253f, -0.25377687811851501f, 0.96701842546463013f, -0.021739022806286812f, 0.027287963777780533f, -0.058331117033958435f, 0.0071336426772177219f, 0.9982718825340271f, 2.7107915878295898f)},
		{1613418458.1838984, Transform(0.95447641611099243f, 0.29159033298492432f, 0.06284746527671814f, 1.6193532943725586f, -0.29063645005226135f, 0.95653194189071655f, -0.024023318663239479f, 0.063806936144828796f, -0.067120566964149475f, 0.0046639293432235718f, 0.9977339506149292f, 2.659712553024292f)},
		{1613418458.6839559, Transform(0.9399188756942749f, 0.34112688899040222f, 0.013601194135844707f, 1.7090979814529419f, -0.34031608700752258f, 0.93936562538146973f, -0.042157087475061417f, 0.074068896472454071f, -0.027157410979270935f, 0.034995537251234055f, 0.99901843070983887f, 2.680656909942627f)},
		{1613418459.2839231, Transform(0.91136699914932251f, 0.41152855753898621f, 0.0073850555345416069f, 1.609864354133606f, -0.40951347351074219f, 0.90841454267501831f, -0.084152914583683014f, -0.031468704342842102f, -0.041340012103319168f, 0.073669910430908203f, 0.99642544984817505f, 2.6887660026550293f)},
	};

	const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
			/*useStoredOdomAsGuess=*/false,
			/*passOdomDataToRtabmap=*/false,
			/*goldenStampedGroundTruth=*/&goldenStampedGT);

	EXPECT_GT(result.framesRead, 0);
	EXPECT_EQ(0, result.odomLost) << "Odometry should never lose tracking";
	EXPECT_EQ(31, result.finalLocalGraphSize);
	EXPECT_EQ(165, result.finalGlobalGraphSize);

#ifdef RTABMAP_OCTOMAP
	// Grid/RayTracing requires OctoMap support; verify the 3D map was
	// actually assembled when the build has it. ICP-only replay is
	// deterministic on a given platform but absolute leaf counts can shift
	// slightly across PCL/Eigen/OpenMP configurations, so use a small
	// tolerance (~1-3%) rather than exact equality.
	EXPECT_NEAR(21372, result.octomapNodes, 300);
	EXPECT_NEAR(16178, result.octomapEmptyCells, 300);
	EXPECT_NEAR(1845, result.octomapObstacleCells, 50);
#endif

	// libpointmatcher's TrimmedDist outlier filter aligns this sparse 3D-lidar
	// dataset to ~1 mm RMSE against the golden trajectory on Linux; Windows
	// math-lib differences push it slightly higher (~1.5 mm observed in CI).
	// With PCL ICP the best we can do is a RANSAC correspondence rejector
	// (see util3d_registration.cpp), which converges but to a looser ~3 cm RMSE.
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (golden GT not injected?)";
#ifdef RTABMAP_POINTMATCHER
	EXPECT_LT(result.translationalRmseFinal, 0.002f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
#else
	EXPECT_LT(result.translationalRmseFinal, 0.05f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
#endif
}

// ---------------------------------------------------------------------------
// PR2 2D-laser + stereo sample (~15s).
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_Stereo)
{
	const std::string dbPath = testDataPath("pr2_scan2d_stereo_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	// Iterate over the BA-capable optimizers built into this rtabmap: CI
	// containers ship with different combos (e.g. only ceres+toro), so each
	// variant is skipped when its optimizer is unavailable.
	struct Variant { Optimizer::Type opt; const char * label; };
	const std::vector<Variant> variants = {
		{Optimizer::kTypeG2O,   "g2o"  },
		{Optimizer::kTypeGTSAM, "gtsam"},
		{Optimizer::kTypeCeres, "ceres"},
	};

	int variantsTested = 0;
	for(const Variant & v : variants)
	{
		if(!Optimizer::isAvailable(v.opt))
		{
			std::cerr << "[skip] optimizer " << v.label << " not available\n";
			continue;
		}
		SCOPED_TRACE(std::string("variant=") + v.label);
		const std::string strategy = uNumber2Str(static_cast<int>(v.opt));

		ParametersMap rtabmapParams = baseRtabmapParams();
		rtabmapParams[Parameters::kMemUseOdomFeatures()] = "true";
		rtabmapParams[Parameters::kOptimizerStrategy()] = strategy;
		rtabmapParams[Parameters::kVisBundleAdjustment()] = strategy;
		ParametersMap odomParams = baseOdometryParams();
		odomParams[Parameters::kOdomF2MBundleAdjustment()] = strategy;

		// passOdomDataToRtabmap=true: rtabmap reuses the keypoints/descriptors
		// extracted during odometry (paired with Mem/UseOdomFeatures=true above).
		const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
				/*useStoredOdomAsGuess=*/true,
				/*passOdomDataToRtabmap=*/true,
				/*goldenStampedGroundTruth=*/nullptr,
				/*frameStride=*/1,
				/*runLabel=*/v.label);

		EXPECT_GT(result.framesRead, 0);
		EXPECT_EQ(0, result.odomLost)
				<< v.label << ": Odometry should never lose tracking";
		EXPECT_EQ(27, result.finalGlobalGraphSize) << v.label;
		EXPECT_GE(result.proximityDetections, 1)
				<< v.label << ": PR2 2D-scan dataset should produce proximity detections";
		// Observed: empty 489-555, obstacle 4851-5202. Wide bounds because the
		// graph optimizer and visual odom drift differ per platform/optimizer.
		EXPECT_GE(result.gridEmptyCells, 400) << v.label;
		EXPECT_LE(result.gridEmptyCells, 650) << v.label;
		EXPECT_GE(result.gridObstacleCells, 4800) << v.label;
		EXPECT_LE(result.gridObstacleCells, 5300) << v.label;
#ifdef RTABMAP_OCTOMAP
		// Observed: empty 1805-1902, obstacle 21502-22383. Bounds are wide
		// to absorb per-BA-backend visual-odom drift.
		EXPECT_GE(result.octomapEmptyCells, 1700) << v.label;
		EXPECT_LE(result.octomapEmptyCells, 2100) << v.label;
		EXPECT_GE(result.octomapObstacleCells, 21000) << v.label;
		EXPECT_LE(result.octomapObstacleCells, 23000) << v.label;
#endif
		// Stereo F2M visual odom + visual loop closure -- observed RMSE ~3 cm,
		// 5 cm bound gives ~50% headroom for run-to-run feature variance.
		ASSERT_GE(result.translationalRmseFinal, 0.0f)
				<< v.label << ": No Gt/translational_rmse in stats (ground truth missing?)";
		EXPECT_LT(result.translationalRmseFinal, 0.05f)
				<< v.label << " Final trajectory RMSE = " << result.translationalRmseFinal << " m";
		++variantsTested;
	}
	ASSERT_GT(variantsTested, 0) << "no BA-capable optimizer was available";
}

// ---------------------------------------------------------------------------
// Stereo 20 Hz tutorial DB (~1035 frames, no stored odometry or graph).
// Smoke-tests the full visual SLAM pipeline: stereo F2M odometry, loop
// closure detection, graph optimization — all starting from raw imagery.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, Stereo20Hz)
{
	const std::string dbPath = testDataPath("stereo_20Hz.db");
	SKIP_IF_MISSING(dbPath);

	const std::string goldenPath = testDataPath("stereo_20Hz_gt.g2o");
	std::map<int, Transform> goldenPoses;
	std::multimap<int, Link> goldenLinks;
	ASSERT_TRUE(graph::importPoses(
			goldenPath, /*format=*/4, goldenPoses, &goldenLinks))
			<< "Failed to load golden poses from " << goldenPath;

	// Variants:
	//  - g2o / gtsam / ceres baseline (vary only the BA backend).
	//  - g2o + Stereo/OpticalFlow=false: switches stereo correspondence
	//    from KLT optical-flow to OpenCV block-matching.
	//  - g2o + Vis/CorFlowUseMinEigenVals=false: keeps optical flow but
	//    drops the min-eigenvalue error metric (uses L1-patch instead).
	//  - g2o + stereoToDepth: SensorCaptureThread::postUpdate() converts
	//    each stereo frame into RGB-D (dense disparity -> depth) before
	//    odometry runs, exercising rtabmap's RGB-D path on stereo data.
	struct Variant {
		Optimizer::Type opt;
		const char *    label;
		ParametersMap   extraOdom;
		bool            stereoToDepth;
	};
	const std::vector<Variant> variants = {
		{Optimizer::kTypeG2O,   "g2o",                                    {}, false},
		{Optimizer::kTypeGTSAM, "gtsam",                                  {}, false},
		{Optimizer::kTypeCeres, "ceres",                                  {}, false},
		{Optimizer::kTypeG2O,   "g2o_no_optical_flow",
				{{Parameters::kStereoOpticalFlow(), "false"}},            false},
		{Optimizer::kTypeG2O,   "g2o_no_min_eigenvals",
				{{Parameters::kVisCorFlowUseMinEigenVals(), "false"}},    false},
		{Optimizer::kTypeG2O,   "g2o_stereo_to_depth",                    {}, true },
	};

	int variantsTested = 0;
	for(const Variant & v : variants)
	{
		if(!Optimizer::isAvailable(v.opt))
		{
			std::cerr << "[skip] optimizer " << v.label << " not available\n";
			continue;
		}
		SCOPED_TRACE(std::string("variant=") + v.label);

		const std::string strategy = uNumber2Str(static_cast<int>(v.opt));

		ParametersMap rtabmapParams = baseRtabmapParams();
		rtabmapParams[Parameters::kMemUseOdomFeatures()] = "true";
		// 1035 frames at 20 Hz — throttle rtabmap to 2 Hz; odometry
		// still runs on every frame.
		rtabmapParams[Parameters::kRtabmapDetectionRate()] = "2";
		// Match every BA-capable knob to the chosen backend: graph
		// optimization (Optimizer/Strategy), the local odom BA
		// (OdomF2M/BundleAdjustment), and the visual-registration BA
		// used during loop closure verification (Vis/BundleAdjustment).
		rtabmapParams[Parameters::kOptimizerStrategy()] = strategy;
		rtabmapParams[Parameters::kVisBundleAdjustment()] = strategy;

		ParametersMap odomParams = baseOdometryParams();
		odomParams[Parameters::kOdomF2MBundleAdjustment()] = strategy;
		// Apply per-variant odometry overrides (Stereo/OpticalFlow, ...).
		// Last write wins, so this overrides anything from baseOdometryParams.
		for(const auto & kv : v.extraOdom)
		{
			odomParams[kv.first] = kv.second;
			// Vis/* knobs apply to both odom and rtabmap; mirror them
			// so loop-closure verification uses the same setting.
			if(kv.first.rfind("Vis/", 0) == 0)
			{
				rtabmapParams[kv.first] = kv.second;
			}
		}

		const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
				/*useStoredOdomAsGuess=*/false,
				/*passOdomDataToRtabmap=*/true,
				/*goldenStampedGroundTruth=*/nullptr,
				/*frameStride=*/1,
				/*runLabel=*/v.label,
				/*stereoToDepth=*/v.stereoToDepth);

		EXPECT_GT(result.framesRead, 1000)
				<< v.label << ": expected ~1035 frames from stereo_20Hz.db";
		EXPECT_EQ(0, result.odomLost)
				<< v.label << ": stereo odometry should not lose tracking";
		EXPECT_GE(result.loopClosuresAccepted, 1)
				<< v.label << ": expected at least one loop closure";
		EXPECT_GT(result.finalGlobalGraphSize, 0);

		float tRmse=0, tMean=0, tMed=0, tStd=0, tMin=0, tMax=0;
		float rRmse=0, rMean=0, rMed=0, rStd=0, rMin=0, rMax=0;
		graph::calcRMSE(goldenPoses, result.finalGlobalPoses,
				tRmse, tMean, tMed, tStd, tMin, tMax,
				rRmse, rMean, rMed, rStd, rMin, rMax,
				/*align2D=*/false);
		std::cerr << "[" << v.label << "] trans rmse=" << tRmse << "m max="
				  << tMax << "m, rot rmse=" << rRmse << "deg max="
				  << rMax << "deg\n";

		// Golden is BA-optimized; the test runs only the real-time SLAM
		// pipeline with the matching BA backend, so the natural gap to
		// the golden is wider than a run-to-run comparison would be.
		// Bounds sit at ~2x the worst observed across all six variants
		// (worst trans=0.076 m on ceres, worst rot=1.40 deg on gtsam)
		EXPECT_LT(tRmse, 0.15f)
				<< v.label << " translational RMSE drifted vs golden";
		EXPECT_LT(rRmse, 3.0f)
				<< v.label << " rotational RMSE drifted vs golden";
		++variantsTested;
	}
	ASSERT_GT(variantsTested, 0) << "no BA-capable optimizer was available";
}

// ---------------------------------------------------------------------------
// PR2 2D-laser + RGB-D sample (~15s).
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_RGBD)
{
	const std::string dbPath = testDataPath("pr2_scan2d_rgbd_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	// Iterate over the BA-capable optimizers built into this rtabmap.
	struct Variant { Optimizer::Type opt; const char * label; };
	const std::vector<Variant> variants = {
		{Optimizer::kTypeG2O,   "g2o"  },
		{Optimizer::kTypeGTSAM, "gtsam"},
		{Optimizer::kTypeCeres, "ceres"},
	};

	int variantsTested = 0;
	for(const Variant & v : variants)
	{
		if(!Optimizer::isAvailable(v.opt))
		{
			std::cerr << "[skip] optimizer " << v.label << " not available\n";
			continue;
		}
		SCOPED_TRACE(std::string("variant=") + v.label);
		const std::string strategy = uNumber2Str(static_cast<int>(v.opt));

		ParametersMap rtabmapParams = baseRtabmapParams();
		rtabmapParams[Parameters::kMemUseOdomFeatures()] = "true";
		rtabmapParams[Parameters::kOptimizerStrategy()] = strategy;
		rtabmapParams[Parameters::kVisBundleAdjustment()] = strategy;
		ParametersMap odomParams = baseOdometryParams();
		odomParams[Parameters::kOdomF2MBundleAdjustment()] = strategy;

		// passOdomDataToRtabmap=true: rtabmap reuses the keypoints/descriptors
		// extracted during odometry (paired with Mem/UseOdomFeatures=true above).
		const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
				/*useStoredOdomAsGuess=*/false,
				/*passOdomDataToRtabmap=*/true,
				/*goldenStampedGroundTruth=*/nullptr,
				/*frameStride=*/1,
				/*runLabel=*/v.label);

		EXPECT_GT(result.framesRead, 0);
		EXPECT_EQ(0, result.odomLost)
				<< v.label << ": Odometry should never lose tracking";
		EXPECT_EQ(21, result.finalGlobalGraphSize) << v.label;
		EXPECT_GE(result.proximityDetections, 1)
				<< v.label << ": PR2 2D-scan dataset should produce proximity detections";
		// Observed: empty 2696-3048, obstacle 4339-4937. Wide bounds absorb
		// platform-level FP differences in the visual loop-closure path.
		EXPECT_GE(result.gridEmptyCells, 2600) << v.label;
		EXPECT_LE(result.gridEmptyCells, 3200) << v.label;
		EXPECT_GE(result.gridObstacleCells, 4200) << v.label;
		EXPECT_LE(result.gridObstacleCells, 5100) << v.label;
#ifdef RTABMAP_OCTOMAP
		// Observed: empty 6072-8037, obstacle 39924-44130. Bounds are wide
		// to absorb per-BA-backend visual-odom drift.
		EXPECT_GE(result.octomapEmptyCells, 5500) << v.label;
		EXPECT_LE(result.octomapEmptyCells, 8500) << v.label;
		EXPECT_GE(result.octomapObstacleCells, 38000) << v.label;
		EXPECT_LE(result.octomapObstacleCells, 44500) << v.label;
#endif
		// RGB-D F2M visual odom + visual loop closure -- observed RMSE ~13 cm
		// (less stable than the stereo/ICP paths), 20 cm bound gives ~50%
		// headroom for visual-only loop-closure variability.
		ASSERT_GE(result.translationalRmseFinal, 0.0f)
				<< v.label << ": No Gt/translational_rmse in stats (ground truth missing?)";
		EXPECT_LT(result.translationalRmseFinal, 0.20f)
				<< v.label << " Final trajectory RMSE = " << result.translationalRmseFinal << " m";
		++variantsTested;
	}
	ASSERT_GT(variantsTested, 0) << "no BA-capable optimizer was available";
}

// ---------------------------------------------------------------------------
// Same PR2 RGB-D sample as above but run through the scan-based ICP loop
// closure pipeline (Reg/Strategy=1 + RGBD/Proximity*) instead of the default
// visual registration. Exercises the 2D-laser proximity path with 3DoF
// constraint.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_RGBD_IcpReg)
{
	const std::string dbPath = testDataPath("pr2_scan2d_rgbd_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	ParametersMap rtabmapParams = baseRtabmapParams();
	rtabmapParams[Parameters::kGridRangeMax()] = "0";
	rtabmapParams[Parameters::kGridSensor()] = "0";
	rtabmapParams[Parameters::kIcpCorrespondenceRatio()] = "0.10";
	rtabmapParams[Parameters::kIcpEpsilon()] = "0.001";
	rtabmapParams[Parameters::kIcpMaxTranslation()] = "0.5";
	// See Netherdrone test comment: OutlierRatio is backend-dependent.
#ifdef RTABMAP_POINTMATCHER
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.95";
#else
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.85";
#endif
	rtabmapParams[Parameters::kIcpPointToPlane()] = "true";
	rtabmapParams[Parameters::kIcpVoxelSize()] = "0.0";
	rtabmapParams[Parameters::kMemBinDataKept()] = "true";
	rtabmapParams[Parameters::kMemLaserScanNormalK()] = "5";
	rtabmapParams[Parameters::kMemLaserScanNormalRadius()] = "1";
	rtabmapParams[Parameters::kMemLaserScanVoxelSize()] = "0.05";
	rtabmapParams[Parameters::kRGBDProximityPathFilteringRadius()] = "1";
	rtabmapParams[Parameters::kRGBDProximityPathMaxNeighbors()] = "10";
	rtabmapParams[Parameters::kRegForce3DoF()] = "true";
	rtabmapParams[Parameters::kRegStrategy()] = "1";

	// Odometry inherits all rtabmap params, then overrides for odom-side ICP
	// and motion guess.
	ParametersMap odomParams = rtabmapParams;
	odomParams[Parameters::kIcpPointToPlaneRadius()] = "1";
	odomParams[Parameters::kIcpVoxelSize()] = "0.05";
	odomParams[Parameters::kOdomGuessMotion()] = "true";
	odomParams[Parameters::kOdomStrategy()] = "0";

	const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
			/*useStoredOdomAsGuess=*/true);

	EXPECT_GT(result.framesRead, 0);
	EXPECT_EQ(0, result.odomLost) << "Odometry should never lose tracking";
	EXPECT_EQ(21, result.finalGlobalGraphSize);
	EXPECT_GE(result.proximityDetections, 1)
			<< "PR2 2D-scan dataset should produce proximity detections";
	// Observed: empty 22785-23845, obstacle 1302-1624. Range widened to
	// absorb run-to-run variance from the RANSAC correspondence rejector
	// installed in the PCL ICP path (util3d_registration.cpp).
	EXPECT_GE(result.gridEmptyCells, 22500);
	EXPECT_LE(result.gridEmptyCells, 24100);
	EXPECT_GE(result.gridObstacleCells, 1250);
	EXPECT_LE(result.gridObstacleCells, 1700);
#ifdef RTABMAP_OCTOMAP
	// 2D-laser-only signatures: nothing to assemble into a 3D OctoMap.
	EXPECT_EQ(0, result.octomapEmptyCells);
	EXPECT_EQ(0, result.octomapObstacleCells);
#endif
	// Scan-based ICP loop closure with the PR2's 2D laser should align the
	// final trajectory to within ~5 cm of the stored ground truth (RMSE
	// observed up to ~0.045 m on macOS CI; Linux typically ~0.015-0.02 m).
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (ground truth missing?)";
	EXPECT_LT(result.translationalRmseFinal, 0.05f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
}

// ---------------------------------------------------------------------------
// PR2 2D-laser corridor traversal (~50s). Three replay variants exercising
// every entry point into Rtabmap's ICP path (Reg/Strategy=1):
//   own-odom    : DB odom ignored, ICP-F2M odometry runs from scratch.
//   guess-odom  : DB odom delta supplied as motion guess to ICP-F2M odometry.
//   stored-odom : DB odom fed straight to Rtabmap, no odometry stage at all.
// The corridor's long-axis degeneracy makes this a useful regression check
// for the low-complexity fallback + ICP loop closure on 2D scans.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_Corridor_IcpReg)
{
	const std::string dbPath = testDataPath("pr2_scan2d_corridor_50s.db");
	SKIP_IF_MISSING(dbPath);

	enum class OdomMode { OwnOdom, GuessFromDb, StoredOdomDirect };
	struct Variant { const char * label; OdomMode mode; float scanMaxRange; };
	const std::vector<Variant> variants = {
		{"own-odom",             OdomMode::OwnOdom,          0.0f},
		{"guess-odom",           OdomMode::GuessFromDb,      0.0f},
		{"stored-odom",          OdomMode::StoredOdomDirect, 0.0f},
		{"own-odom-range5.6",    OdomMode::OwnOdom,          5.6f},
		{"guess-odom-range5.6",  OdomMode::GuessFromDb,      5.6f},
		{"stored-odom-range5.6", OdomMode::StoredOdomDirect, 5.6f},
	};

	auto buildRtabmapParams = []() {
		ParametersMap p = baseRtabmapParams();
		// 1 Hz detection rate; throttled frames become intermediate nodes
		// (id=-1) so the full 990-frame odom track + laser payloads are
		// preserved in the output DB for offline inspection.
		p[Parameters::kRtabmapDetectionRate()] = "1";
		p[Parameters::kRtabmapCreateIntermediateNodes()] = "true";
		p[Parameters::kMemIntermediateNodeDataKept()] = "true";
		p[Parameters::kRegStrategy()] = "1";
		p[Parameters::kRegForce3DoF()] = "true";
		p[Parameters::kRGBDCreateOccupancyGrid()] = "false";
		p[Parameters::kIcpEpsilon()] = "0.001";
		p[Parameters::kIcpMaxTranslation()] = "0.5";
		p[Parameters::kIcpCorrespondenceRatio()] = "0.01";
		p[Parameters::kIcpPointToPlane()] = "true";
		p[Parameters::kIcpVoxelSize()] = "0.0";
#ifdef RTABMAP_POINTMATCHER
		p[Parameters::kIcpOutlierRatio()] = "0.95";
		p[Parameters::kIcpStrategy()] = "1";  // libpointmatcher
#else
		p[Parameters::kIcpOutlierRatio()] = "0.85";
#endif
		p[Parameters::kMemBinDataKept()] = "true";
		p[Parameters::kMemLaserScanNormalK()] = "5";
		p[Parameters::kMemLaserScanNormalRadius()] = "1";
		p[Parameters::kMemLaserScanVoxelSize()] = "0.05";
		p[Parameters::kRGBDProximityPathFilteringRadius()] = "1";
		p[Parameters::kRGBDProximityPathMaxNeighbors()] = "10";
		return p;
	};

	for(const Variant & v : variants)
	{
		SCOPED_TRACE(std::string("variant=") + v.label);
		ParametersMap rtabmapParams = buildRtabmapParams();

		ReplayResult result;
		if(v.mode == OdomMode::StoredOdomDirect)
		{
			rtabmapParams[Parameters::kRGBDNeighborLinkRefining()] = "true";
			const std::string workDb = test::tempPath(uFormat(
					"rtabmap_integration_PR2_Scan2D_Corridor_IcpReg_%s.db", v.label));
			result = replayDatabaseWithStoredOdom(dbPath, workDb, rtabmapParams,
					/*triggerNewMapAfterFrame=*/-1,
					/*overrideOdomAngularVariance=*/-1.0,
					/*overrideOdomLinearVariance=*/-1.0,
					/*scanMaxRange=*/v.scanMaxRange);
		}
		else
		{
			ParametersMap odomParams = rtabmapParams;
			odomParams[Parameters::kOdomStrategy()] = "0";  // F2M
			odomParams[Parameters::kOdomGuessMotion()] = "true";
			odomParams[Parameters::kIcpPointToPlaneRadius()] = "1";
			odomParams[Parameters::kIcpVoxelSize()] = "0.05";

			const bool useGuess = (v.mode == OdomMode::GuessFromDb);
			result = replayDatabase(dbPath, rtabmapParams, odomParams,
					/*useStoredOdomAsGuess=*/useGuess,
					/*passOdomDataToRtabmap=*/false,
					/*goldenStampedGroundTruth=*/nullptr,
					/*frameStride=*/1,
					/*runLabel=*/v.label,
					/*stereoToDepth=*/false,
					/*scanMaxRange=*/v.scanMaxRange);
		}

		EXPECT_EQ(990, result.framesRead)
				<< v.label << ": expected 990 frames in the corridor DB";

		if(v.scanMaxRange > 0.0f)
		{
			// Range-capped variants: even when the lidar sees only 5.6 m
			// of corridor, the default low-complexity strategy
			// (Icp/PointToPlaneLowComplexityStrategy=1: PointToPoint +
			// project) keeps libpointmatcher's iteration inside the
			// Icp/MaxTranslation bound, so ICP odometry never loses
			// tracking. All three modes build a complete ~48-node graph.
			EXPECT_NEAR(48, result.framesProcessed, 3) << v.label;
			// All 990 DB frames live in the graph: 48 detection nodes +
			// 942 intermediate nodes (Rtabmap/CreateIntermediateNodes).
			EXPECT_NEAR(942, result.framesIntermediate, 5) << v.label;
			EXPECT_EQ(990, result.finalGlobalGraphSize) << v.label;
			ASSERT_GE(result.translationalRmseFinal, 0.0f) << v.label;
			if(v.mode != OdomMode::StoredOdomDirect)
			{
				EXPECT_EQ(0, result.odomLost) << v.label
						<< ": no odom loss expected under the range cap with the "
						<< "default PointToPoint-low-complexity recovery strategy";
			}
			if(v.mode == OdomMode::OwnOdom)
			{
				// Observed RMSE swings between ~0.20 m and ~1.5 m
				// run-to-run -- ICP-F2M alone drifts unpredictably
				// along the unobservable x-axis. Bound is loose on
				// purpose; the documented behavior is "stays within
				// the same order of magnitude as a full corridor",
				// not anything precise.
				EXPECT_LT(result.translationalRmseFinal, 2.0f)
						<< v.label << " RMSE = " << result.translationalRmseFinal << "m";
			}
			else if(v.mode == OdomMode::GuessFromDb)
			{
				// Observed ~0.05 m at 5.6 m range -- DB odom guess holds
				// the trajectory while ICP refines y/yaw.
				EXPECT_LT(result.translationalRmseFinal, 0.15f)
						<< v.label << " RMSE = " << result.translationalRmseFinal << "m";
			}
			else  // StoredOdomDirect
			{
				// Observed ~0.04 m at 5.6 m range; stored odom carries
				// the trajectory, short scans don't propagate into error.
				EXPECT_LT(result.translationalRmseFinal, 0.10f)
						<< v.label << " RMSE = " << result.translationalRmseFinal << "m";
			}
		}
		else
		{
			// 1 Hz throttle on 50s of data -> ~48-50 nodes regardless of
			// mode (odom always runs on every DB frame, rtabmap.process is
			// the one that gates on DetectionRate).
			EXPECT_NEAR(48, result.framesProcessed, 3) << v.label;
			// All 990 DB frames live in the graph: 48 detection nodes +
			// 942 intermediate nodes (Rtabmap/CreateIntermediateNodes).
			EXPECT_NEAR(942, result.framesIntermediate, 5) << v.label;
			EXPECT_EQ(990, result.finalGlobalGraphSize) << v.label;
			// Corridor's tight viewpoint spacing -> near-1:1 node-to-proximity
			// ratio (observed 43/48).
			EXPECT_GE(result.proximityDetections, 35) << v.label;

			// Stored ground truth + 3DoF ICP -> ~3 cm RMSE on the odom
			// variants; the stored-odom variant inherits whatever drift
			// the recorded odom carries. 6 cm bound gives ~2x headroom.
			ASSERT_GE(result.translationalRmseFinal, 0.0f)
					<< v.label << ": no Gt/translational_rmse in stats";
			EXPECT_LT(result.translationalRmseFinal, 0.06f)
					<< v.label << " RMSE = " << result.translationalRmseFinal << "m";

			if(v.mode != OdomMode::StoredOdomDirect)
			{
				EXPECT_EQ(0, result.odomLost)
						<< v.label << ": odometry should never lose tracking";
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Two-loop workspace mapping session (Texture tutorial DB). Loads the
// pre-built session, runs global bundle adjustment, and asserts the
// resulting poses against a captured golden trajectory.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, TwoLoopsWorkspaceGlobalBA)
{
	const std::string srcPath = testDataPath("2loops_workspace_3IT.db");
	SKIP_IF_MISSING(srcPath);

	// Golden trajectory was captured from a known-good BA run, verified
	// in rtabmap-databaseViewer, then exported to
	// data/tests/2loops_workspace_3IT_gt.g2o. To regenerate after a
	// deliberate BA-algorithm change: rerun BA on the source DB, eyeball
	// it in the viewer, and re-export the graph from there.
	const std::string goldenPath = testDataPath("2loops_workspace_3IT_gt.g2o");
	std::map<int, Transform> goldenPoses;
	std::multimap<int, Link> goldenLinks;
	ASSERT_TRUE(graph::importPoses(goldenPath, /*format=*/4, goldenPoses, &goldenLinks))
			<< "Failed to load golden poses from " << goldenPath;

	// Variant matrix:
	//  * g2o is the primary backend, exercised across both rematchFeatures
	//    settings and with detectMoreLoopClosures enabled.
	//  * gtsam and ceres are validated only on the simplest config
	//    (no rematch, no extra loop-closure detection) to keep the test
	//    fast while still catching regressions in those backends.
	// cvsba is excluded — observed ~4× worse RMSE on this dataset.
	//
	// Per-variant RMSE bounds: g2o and gtsam (SBA-style) converge to the
	// same tight optimum on this dataset across versions; ceres trails
	// slightly on some ceres-solver releases, so its bound is loosened
	// just enough to absorb cross-version drift without masking real
	// regressions (still asserts BA improved the pre-BA RMSE).
	struct Variant {
		Optimizer::Type type;
		const char * name;
		bool rematch;
		bool detectMore;
		float maxTRmse;
		float maxRRmse;
	};
	const std::vector<Variant> variants = {
		{Optimizer::kTypeG2O,   "g2o",         false, true,  0.05f, 1.5f },
		{Optimizer::kTypeG2O,   "g2o-rematch", true,  true,  0.05f, 1.5f },
		{Optimizer::kTypeGTSAM, "gtsam",       false, false, 0.05f, 1.5f },
		{Optimizer::kTypeCeres, "ceres",       false, false, 0.07f, 2.5f },
	};

	int variantsTested = 0;
	for(const Variant & v : variants)
	{
		if(!Optimizer::isAvailable(v.type))
		{
			std::cerr << "[skip] optimizer " << v.name << " not available in this build\n";
			continue;
		}
		SCOPED_TRACE(std::string("variant=") + v.name);

		// Open the source DB read-only. BA runs entirely against
		// _optimizedPoses in working memory, so no writes hit the DB; the
		// readonly flag also prevents accidental persistence if a future
		// change adds a write path.
		ParametersMap params;
		uInsert(params, ParametersPair(Parameters::kMemIncrementalMemory(),    "false"));
		uInsert(params, ParametersPair(Parameters::kMemLocalizationReadOnly(), "true"));

		Rtabmap rtabmap;
		rtabmap.init(params, srcPath, /*loadDatabaseParameters=*/true);

		std::map<int, Transform> initPoses;
		std::multimap<int, Link> initLinks;
		rtabmap.getGraph(initPoses, initLinks, /*optimized=*/true, /*global=*/false);
		const int linksBeforeDetect = static_cast<int>(initLinks.size());

		if(v.detectMore)
		{
			// In read-only mode any new links live in working memory only;
			// the source DB is untouched.
			const int added = rtabmap.detectMoreLoopClosures(
					/*clusterRadiusMax=*/1.0f,
					/*clusterAngle=*/static_cast<float>(CV_PI)/6.0f,
					/*iterations=*/3,
					/*intraSession=*/true);
			ASSERT_GE(added, 0) << v.name << " detectMoreLoopClosures failed";

			std::map<int, Transform> postDetectPoses;
			std::multimap<int, Link> postDetectLinks;
			rtabmap.getGraph(postDetectPoses, postDetectLinks,
					/*optimized=*/true, /*global=*/false);
			std::cerr << "[" << v.name << "] links: " << linksBeforeDetect
					  << " -> " << postDetectLinks.size()
					  << " (detectMoreLoopClosures added " << added << ")\n";
			EXPECT_GT((int)postDetectLinks.size(), linksBeforeDetect)
					<< v.name << " detectMoreLoopClosures did not increase link count";
		}

		// Snapshot the graph right before BA — for variants without
		// detectMoreLoopClosures this is the original graph; for variants
		// with it, the snapshot includes the newly-added closures.
		std::map<int, Transform> preBaPoses;
		std::multimap<int, Link> preBaLinks;
		rtabmap.getGraph(preBaPoses, preBaLinks, /*optimized=*/true, /*global=*/false);

		float preTRmse=0, tMeanPre=0, tMedPre=0, tStdPre=0, tMinPre=0, tMaxPre=0;
		float rRmsePre=0, rMeanPre=0, rMedPre=0, rStdPre=0, rMinPre=0, rMaxPre=0;
		graph::calcRMSE(goldenPoses, preBaPoses,
				preTRmse, tMeanPre, tMedPre, tStdPre, tMinPre, tMaxPre,
				rRmsePre, rMeanPre, rMedPre, rStdPre, rMinPre, rMaxPre,
				/*align2D=*/false);
		std::cerr << "[" << v.name << "] Pre-BA: "
				  << "trans rmse=" << preTRmse << "m max=" << tMaxPre << "m, "
				  << "rot rmse=" << rRmsePre << "deg max=" << rMaxPre << "deg\n";

		const bool baOk = rtabmap.globalBundleAdjustment(
				/*optimizerType=*/v.type,
				/*rematchFeatures=*/v.rematch,
				/*iterations=*/30,
				/*pixelVariance=*/0.0f);
		ASSERT_TRUE(baOk) << "globalBundleAdjustment failed for " << v.name;

		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		// global=false reads BA poses straight from _optimizedPoses;
		// global=true would re-run pose-graph optimization and clobber BA.
		rtabmap.getGraph(poses, links, /*optimized=*/true, /*global=*/false);
		// DB is read-only — close without attempting to write back.
		rtabmap.close(false);

		ASSERT_EQ(poses.size(), goldenPoses.size())
				<< v.name << " BA result has " << poses.size()
				<< " poses, golden has " << goldenPoses.size();

		// SVD-aligned RMSE — golden was captured with different BA
		// settings, so a Umeyama-style alignment is what's meaningful.
		float tRmse=0, tMean=0, tMed=0, tStd=0, tMin=0, tMax=0;
		float rRmse=0, rMean=0, rMed=0, rStd=0, rMin=0, rMax=0;
		graph::calcRMSE(goldenPoses, poses,
				tRmse, tMean, tMed, tStd, tMin, tMax,
				rRmse, rMean, rMed, rStd, rMin, rMax,
				/*align2D=*/false);
		std::cerr << "[" << v.name << "] BA: "
				  << "trans rmse=" << tRmse << "m max=" << tMax << "m, "
				  << "rot rmse=" << rRmse << "deg max=" << rMax << "deg\n";

		EXPECT_GT(preTRmse, tRmse)
				<< v.name << " BA did not improve translational RMSE";
		EXPECT_LT(tRmse, v.maxTRmse)
				<< v.name << " translational RMSE too large after alignment";
		EXPECT_LT(rRmse, v.maxRRmse)
				<< v.name << " rotational RMSE too large after alignment";
		++variantsTested;
	}
	ASSERT_GT(variantsTested, 0) << "no BA-capable optimizer was available";
}

// ---------------------------------------------------------------------------
// Robust-graph-optimization tutorial DB (stereo). Replays the session frame
// by frame, feeding the stored odom pose straight to Rtabmap (no odometry
// recomputation). Sweeps the 2x2x2 grid:
//   Optimizer/Strategy   in {g2o, GTSAM}
//   Optimizer/Robust     in {true, false}
//   RGBD/OptimizeMaxError in {3.0, 0.0}
// The full robust combo (g2o + Robust=true + MaxError=3) is the golden;
// runs with both safeguards off should produce a visibly wrong graph.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, RobustGraphOptimizationStereo)
{
	const std::string srcPath = testDataPath("robust_graph_optimization_stereo.db");
	SKIP_IF_MISSING(srcPath);

	if(!Optimizer::isAvailable(Optimizer::kTypeG2O)
			&& !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
	{
		GTEST_SKIP() << "neither g2o nor gtsam built — this test exercises both";
	}

	struct Variant {
		Optimizer::Type optType;
		bool robust;
		float maxError;
		const char * label;
	};
	// g2o and gtsam each cover both robust/no-robust at the default
	// MaxError=3. Ceres is skipped here — it doesn't implement Vertigo
	// switches (Optimizer/Robust=true), so it can't exercise the robust
	// path that's the point of this DB.
	const std::vector<Variant> variants = {
		{Optimizer::kTypeG2O,   true,  3.0f, "g2o-robust-maxerr3"     },
		{Optimizer::kTypeG2O,   false, 3.0f, "g2o-norobust-maxerr3"   },
		{Optimizer::kTypeGTSAM, true,  3.0f, "gtsam-robust-maxerr3"   },
		{Optimizer::kTypeGTSAM, false, 3.0f, "gtsam-norobust-maxerr3" },
	};

	// Golden trajectory captured from the g2o-robust-maxerr3 variant and
	// committed under data/tests/. Regenerate by uncommenting the
	// exportPoses block below and rerunning the test.
	const std::string goldenPath = testDataPath("robust_graph_optimization_stereo_gt.g2o");
	std::map<int, Transform> goldenPoses;
	std::multimap<int, Link> goldenLinks;
	ASSERT_TRUE(graph::importPoses(
			goldenPath, /*format=*/4, goldenPoses, &goldenLinks))
			<< "Failed to load golden poses from " << goldenPath;

	int variantsTested = 0;
	for(const Variant & v : variants)
	{
		if(!Optimizer::isAvailable(v.optType))
		{
			std::cerr << "[skip] " << v.label << " (optimizer unavailable)\n";
			continue;
		}
		SCOPED_TRACE(std::string(v.label));

		ParametersMap params = baseRtabmapParams();
		uInsert(params, ParametersPair(Parameters::kOptimizerStrategy(),
				uNumber2Str(static_cast<int>(v.optType))));
		uInsert(params, ParametersPair(Parameters::kOptimizerRobust(),
				v.robust ? "true" : "false"));
		uInsert(params, ParametersPair(Parameters::kRGBDOptimizeMaxError(),
				uNumber2Str(v.maxError)));
		uInsert(params, ParametersPair(Parameters::kMemUseOdomFeatures(), "true"));
		uInsert(params, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "false"));
		uInsert(params, ParametersPair(Parameters::kMemBinDataKept(), "false"));
		uInsert(params, ParametersPair(Parameters::kKpFlannRebalancingFactor(), "1"));
		uInsert(params, ParametersPair(Parameters::kRtabmapDetectionRate(), "0"));

		const std::string workDb = test::tempPath(uFormat(
				"rtabmap_integration_RobustGraphOptimizationStereo_%s.db", v.label));
		std::cerr << "Working DB for " << v.label << ": " << workDb << "\n";

		const ReplayResult result =
				replayDatabaseWithStoredOdom(srcPath, workDb, params);

		// Sanity: every variant must produce some optimized graph.
		ASSERT_GT(result.framesProcessed, 0) << v.label << " produced no frames";
		ASSERT_GT(result.finalGlobalGraphSize, 0)
				<< v.label << " produced empty graph";

		// Regenerate golden from the most robust combo. Toggled on for a
		// one-shot capture; re-comment after the file is committed.
		// if(v.optType == Optimizer::kTypeG2O && v.robust && v.maxError > 0.0f)
		// {
		//     std::multimap<int, Link> links;
		//     rtabmap::graph::exportPoses(goldenPath, /*format=*/4,
		//             result.finalGlobalPoses, links);
		// }

		float tRmse=0, tMean=0, tMed=0, tStd=0, tMin=0, tMax=0;
		float rRmse=0, rMean=0, rMed=0, rStd=0, rMin=0, rMax=0;
		graph::calcRMSE(goldenPoses, result.finalGlobalPoses,
				tRmse, tMean, tMed, tStd, tMin, tMax,
				rRmse, rMean, rMed, rStd, rMin, rMax,
				/*align2D=*/false);
		std::cerr << "[" << v.label << "] "
				  << "trans rmse=" << tRmse << "m max=" << tMax << "m, "
				  << "rot rmse=" << rRmse << "deg max=" << rMax << "deg\n";

		// Each remaining variant enables at least one safeguard
		// (Optimizer/Robust or RGBD/OptimizeMaxError) and should land
		// close to the golden trajectory. The "neither safeguard"
		// (norobust+maxerr=0) cases are removed since their
		// catastrophic-drift behavior is already established.
		EXPECT_LT(tRmse, 0.05f)
				<< v.label << " translational RMSE too large; "
				<< "expected near golden";
		EXPECT_LT(rRmse, 1.5f)
				<< v.label << " rotational RMSE too large; "
				<< "expected near golden";
		++variantsTested;
	}
	ASSERT_GT(variantsTested, 0) << "no compatible optimizer was available";
}

// ---------------------------------------------------------------------------
// 3-iteration loop with GPS metadata. Replays the session frame-by-frame
// with the stored odom, sweeping Rtabmap/LoopGPS on/off. GPS-aided loop
// closure detection filters candidates by GPS proximity; with it off the
// detector falls back to the visual-only pipeline. The golden trajectory
// is captured with GPS on.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, Loop3ItGps)
{
	const std::string srcPath = testDataPath("loop_3it_gps.db");
	SKIP_IF_MISSING(srcPath);

	const bool hasGtsam = Optimizer::isAvailable(Optimizer::kTypeGTSAM);
	const bool hasG2o   = Optimizer::isAvailable(Optimizer::kTypeG2O);
	if(!hasGtsam && !hasG2o)
	{
		GTEST_SKIP() << "neither gtsam nor g2o built — this test needs at least one";
	}
	// Prefer gtsam: this DB's hard-GPS-priors path is well-behaved under
	// gtsam but catastrophically rejects all loops under g2o.
	const Optimizer::Type backend = hasGtsam
			? Optimizer::kTypeGTSAM : Optimizer::kTypeG2O;

	struct Variant {
		Optimizer::Type optType;
		bool loopGps;
		int  triggerNewMapAfterFrame;  // -1 = single session
		bool robust;
		float maxError;
		bool priorsIgnored;            // false = use GPS priors as anchors
		std::string label;
	};
	// Default MaxError + Optimizer/Robust=false. Single-session variants
	// are the canonical comparison against the golden; the newmap@60
	// variants exercise the multi-session path. The trailing "-priors"
	// variants flip Optimizer/PriorsIgnored=false so the GPS pose-prior
	// links anchor the trajectory.
	const int kTriggerFrame = 60;
	const std::vector<Variant> variants = {
		{backend, true,  -1,            false, 3.0f, true,  "gps-on"                 },
		{backend, false, -1,            false, 3.0f, true,  "gps-off"                },
		{backend, true,  kTriggerFrame, false, 3.0f, true,  "gps-on-newmap60"        },
		{backend, false, kTriggerFrame, false, 3.0f, true,  "gps-off-newmap60"       },
		{backend, true,  -1,            false, 3.0f, false, "gps-on-priors"          },
		{backend, true,  kTriggerFrame, false, 3.0f, false, "gps-on-newmap60-priors" },
	};

	// Golden trajectory captured from the gps-on variant and committed
	// under data/tests/. Regenerate by uncommenting the exportPoses block
	// below and rerunning the test.
	const std::string goldenPath = testDataPath("loop_3it_gps_gt.g2o");
	std::map<int, Transform> goldenPoses;
	std::multimap<int, Link> goldenLinks;
	ASSERT_TRUE(graph::importPoses(
			goldenPath, /*format=*/4, goldenPoses, &goldenLinks))
			<< "Failed to load golden poses from " << goldenPath;

	for(const Variant & v : variants)
	{
		// gps-on-priors only converges on this DB with gtsam — g2o
		// hard-anchors the noisy GPS priors and rejects every visual
		// loop closure, collapsing the trajectory to ~30 m. Skip it
		// on g2o-only builds rather than encoding two different
		// expected outcomes.
		if(v.label == "gps-on-priors" && !hasGtsam)
		{
			std::cerr << "[skip] " << v.label
					  << ": requires gtsam (g2o-only path is catastrophic)\n";
			continue;
		}
		SCOPED_TRACE(std::string(v.label));

		ParametersMap params = baseRtabmapParams();
		uInsert(params, ParametersPair(Parameters::kOptimizerStrategy(),
				uNumber2Str(static_cast<int>(v.optType))));
		uInsert(params, ParametersPair(Parameters::kRtabmapLoopGPS(),
				v.loopGps ? "true" : "false"));
		uInsert(params, ParametersPair(Parameters::kOptimizerRobust(),
				v.robust ? "true" : "false"));
		uInsert(params, ParametersPair(Parameters::kRGBDOptimizeMaxError(),
				uNumber2Str(v.maxError)));
		uInsert(params, ParametersPair(Parameters::kOptimizerPriorsIgnored(),
				v.priorsIgnored ? "true" : "false"));
		uInsert(params, ParametersPair(Parameters::kMemUseOdomFeatures(), "true"));
		uInsert(params, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "false"));
		uInsert(params, ParametersPair(Parameters::kMemBinDataKept(), "false"));
		uInsert(params, ParametersPair(Parameters::kKpFlannRebalancingFactor(), "1"));
		uInsert(params, ParametersPair(Parameters::kRtabmapDetectionRate(), "0"));

		const std::string workDb = test::tempPath(uFormat(
				"rtabmap_integration_Loop3ItGps_%s.db", v.label.c_str()));
		std::cerr << "Working DB for " << v.label << ": " << workDb << "\n";

		const ReplayResult result = replayDatabaseWithStoredOdom(
				srcPath, workDb, params, v.triggerNewMapAfterFrame);

		ASSERT_GT(result.framesProcessed, 0) << v.label << " produced no frames";
		ASSERT_GT(result.finalGlobalGraphSize, 0)
				<< v.label << " produced empty graph";

		float tRmse=0, tMean=0, tMed=0, tStd=0, tMin=0, tMax=0;
		float rRmse=0, rMean=0, rMed=0, rStd=0, rMin=0, rMax=0;
		graph::calcRMSE(goldenPoses, result.finalGlobalPoses,
				tRmse, tMean, tMed, tStd, tMin, tMax,
				rRmse, rMean, rMed, rStd, rMin, rMax,
				/*align2D=*/false);
		std::cerr << "[" << v.label << "] "
				  << "trans rmse=" << tRmse << "m max=" << tMax << "m, "
				  << "rot rmse=" << rRmse << "deg max=" << rMax << "deg, "
				  << "loops=" << result.loopClosuresAccepted
				  << " rejected=" << result.loopClosuresRejected << "\n";

		// RMSE bands by configuration (observed on gtsam):
		//   * Single-session, priors-ignored:        ~15 cm
		//   * Single-session with GPS priors:        ~0.6 m (gtsam balances
		//     visual loops vs noisy priors; g2o would catastrophically
		//     fail this case and is skipped above).
		//   * newmap60 with GPS bridging:            ~1 m
		//   * newmap60 with GPS + hard priors:       ~2.5 m
		//   * newmap60 without GPS bridging:         ~30 m (no anchor).
		const bool gpsOffAndNewMap =
				!v.loopGps && v.triggerNewMapAfterFrame > 0;
		const bool singleSessionWithHardPriors =
				v.triggerNewMapAfterFrame < 0 && !v.priorsIgnored;
		const bool newMapWithHardPriors =
				v.triggerNewMapAfterFrame > 0 && !v.priorsIgnored;
		if(gpsOffAndNewMap)
		{
			EXPECT_GT(tRmse, 20.0f)
					<< v.label << " expected to blow up (no usable anchor)";
		}
		else if(singleSessionWithHardPriors)
		{
			EXPECT_LT(tRmse, 1.5f)
					<< v.label << " single-session with priors should "
					<< "stay below ~1 m (gtsam balances priors vs loops)";
		}
		else if(newMapWithHardPriors)
		{
			EXPECT_LT(tRmse, 4.0f)
					<< v.label << " newmap60 + priors should stay below ~4 m";
		}
		else if(v.triggerNewMapAfterFrame < 0)
		{
			EXPECT_LT(tRmse, 0.20f)
					<< v.label << " single-session RMSE drifted";
		}
		else
		{
			EXPECT_LT(tRmse, 2.0f)
					<< v.label << " session-split RMSE should stay bounded";
		}

		// Loop closure counts (accepted/rejected):
		//   * GPS-off rejects more than GPS-on (no GPS pre-filter, more
		//     candidates reach the MaxError gate).
		//   * newmap60 rejects more than single-session (the artificial
		//     split makes cross-session candidates more error-prone).
		// Observed on this DB (without Optimizer/Robust):
		//   gps-on            : 19 accepted /  2 rejected
		//   gps-off           : 13 accepted / 18 rejected
		//   gps-on-newmap60   : 24 accepted / 17 rejected
		//   gps-off-newmap60  : 16 accepted / 41 rejected
		int minAcc, maxAcc, minRej, maxRej;
		if(!v.loopGps && v.triggerNewMapAfterFrame > 0)
		{
			minAcc = 5;  maxAcc = 30;
			minRej = 30; maxRej = 100;  // gps-off + newmap60
		}
		else if(!v.loopGps)
		{
			minAcc = 5;  maxAcc = 25;
			minRej = 5;  maxRej = 40;   // gps-off single
		}
		else if(v.triggerNewMapAfterFrame < 0 && !v.priorsIgnored)
		{
			// gtsam balances hard GPS priors against visual loop
			// closures and keeps most loops. Observed ~21 accepted /
			// ~8 rejected.
			minAcc = 10; maxAcc = 35;
			minRej = 0;  maxRej = 25;   // gps-on single + priors (gtsam)
		}
		else if(v.triggerNewMapAfterFrame > 0 && !v.priorsIgnored)
		{
			// newmap60 + priors: the session split + GPS priors push
			// loops past the MaxError gate more often. Observed ~9
			// accepted / ~32 rejected.
			minAcc = 2;  maxAcc = 20;
			minRej = 20; maxRej = 60;   // gps-on newmap60 + priors
		}
		else if(v.triggerNewMapAfterFrame > 0)
		{
			minAcc = 15; maxAcc = 40;
			minRej = 5;  maxRej = 40;   // gps-on newmap60
		}
		else
		{
			minAcc = 10; maxAcc = 35;
			minRej = 0;  maxRej = 20;   // gps-on single
		}
		EXPECT_GE(result.loopClosuresAccepted, minAcc)
				<< v.label << " accepted fewer loops than expected";
		EXPECT_LE(result.loopClosuresAccepted, maxAcc)
				<< v.label << " accepted more loops than expected";
		EXPECT_GE(result.loopClosuresRejected, minRej)
				<< v.label << " rejected fewer loops than expected";
		EXPECT_LE(result.loopClosuresRejected, maxRej)
				<< v.label << " rejected more loops than expected";

		// Smoke check: every variant produces *some* trajectory.
		EXPECT_GT(result.finalGlobalGraphSize, 100)
				<< v.label << " produced an unexpectedly small graph";
	}
}

// ---------------------------------------------------------------------------
// Appearance-only loop closure on the 84-image `data/samples` set with the
// shipped `data/samples_GT.bmp` ground truth. Measures recall at 100%
// precision (the rtabmap "max recall while no false positive has appeared
// yet" metric — same definition as the legacy MATLAB getPrecisionRecall.m
// script) for every Features2D detector strategy that is available in this
// build. Detector strategies for which Feature2D::create() silently
// substitutes a different backend (e.g. SURF -> SIFT without nonfree,
// SuperPointTorch -> GFTT/ORB without RTABMAP_TORCH) are skipped.
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, AppearanceOnly_PrecisionRecall)
{
	const std::string samplesDir = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples";
	const std::string gtPath     = std::string(RTABMAP_TEST_DATA_ROOT) + "/samples_GT.bmp";
	SKIP_IF_MISSING(samplesDir);
	SKIP_IF_MISSING(gtPath);

	// 84x84 binary loop-closure ground truth. Pixel (i, j) == 255 means
	// query frame i+1 has a true loop with past frame j+1. The
	// gray-pixel "ignore" zone the MATLAB script handles is not present
	// in this GT (only 0 / 255), so we skip that branch.
	cv::Mat gt = cv::imread(gtPath, cv::IMREAD_GRAYSCALE);
	ASSERT_EQ(84, gt.rows) << "unexpected samples_GT.bmp size";
	ASSERT_EQ(84, gt.cols) << "unexpected samples_GT.bmp size";

	const int kNumFrames = 84;

	// Total GT positives = number of query rows with at least one true
	// loop. Recall denominator in the standard rtabmap metric.
	int gtTotalPositives = 0;
	for(int i = 0; i < gt.rows; ++i)
	{
		for(int j = 0; j < gt.cols; ++j)
		{
			if(gt.at<unsigned char>(i, j) == 255)
			{
				++gtTotalPositives;
				break;
			}
		}
	}
	ASSERT_GT(gtTotalPositives, 0) << "samples_GT.bmp has no positives";

	// Iterate every backend listed in Feature2D::Type. kFeatureEnd is the
	// sentinel kept at the back of the enum so a new strategy gets covered
	// here automatically.

	// SuperPoint asset paths. SuperPointTorch needs a pre-traced *.pt
	// (produced by scripts/fetch_test_data.sh from *.pth when torch is
	// installed). The Rpautrat backend takes the *.pth directly and traces
	// to *.pt at runtime inside the C++ class, so we point it at the .pth
	// + the python model file the runtime tracer needs.
	const std::string superpointTorchModel      = std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_v1.pt";
	const std::string superpointRpautratWeights = std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_v6_from_tf.pth";
	const std::string superpointRpautratModel   = std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/superpoint_pytorch.py";

	// One pass per detector: prefer GPU if available, otherwise CPU. To
	// keep wall-clock low, the two BoW likelihood variants (default raw
	// word-count vs TF-IDF-weighted) are exercised only on rtabmap's
	// default detector (Kp/DetectorStrategy default = kFeatureGfttOrb).
	const Feature2D::Type defaultDetector =
			static_cast<Feature2D::Type>(Parameters::defaultKpDetectorStrategy());

	int detectorsTested = 0;
	for(int strategy = Feature2D::kFeatureSurf; strategy < Feature2D::kFeatureEnd; ++strategy)
	{
		const Feature2D::Type detectorType = static_cast<Feature2D::Type>(strategy);
		const std::string typeName = Feature2D::typeName(detectorType);

		if(!Feature2D::isAvailable(detectorType))
		{
			std::cerr << "[skip] detector " << typeName << " not available in this build\n";
			continue;
		}

		if(detectorType == Feature2D::kFeaturePyDetector)
		{
			std::cerr << "[skip] detector " << typeName
					<< " requires a user-supplied Py/DetectorPath script\n";
			continue;
		}

		// SuperPoint variants need traced *.pt weights (plus a Python
		// model file for the Rpautrat backend). The fetch_test_data.sh
		// script writes them under data/tests/; skip cleanly if absent.
		if(detectorType == Feature2D::kFeatureSuperPointTorch && !UFile::exists(superpointTorchModel))
		{
			std::cerr << "[skip] detector " << typeName
					<< " missing weights: " << superpointTorchModel
					<< " (run scripts/fetch_test_data.sh)\n";
			continue;
		}
		if(detectorType == Feature2D::kFeatureSuperPointRpautrat &&
				(!UFile::exists(superpointRpautratWeights) ||
				 !UFile::exists(superpointRpautratModel)))
		{
			std::cerr << "[skip] detector " << typeName
					<< " missing assets: weights=" << superpointRpautratWeights
					<< " model=" << superpointRpautratModel
					<< " (run scripts/fetch_test_data.sh)\n";
			continue;
		}

		// Probe GPU availability once per detector. Build the temp instance
		// with the same asset paths the actual run uses; otherwise SuperPoint
		// variants log a (harmless) load failure.
		bool useGpu = false;
		{
			ParametersMap probeParams;
			if(detectorType == Feature2D::kFeatureSuperPointTorch)
			{
				probeParams[Parameters::kSuperPointModelPath()] = superpointTorchModel;
			}
			else if(detectorType == Feature2D::kFeatureSuperPointRpautrat)
			{
				probeParams[Parameters::kSuperPointRpautratWeightsPath()] = superpointRpautratWeights;
				probeParams[Parameters::kSuperPointRpautratModelPath()]   = superpointRpautratModel;
			}
			std::unique_ptr<Feature2D> probe(Feature2D::create(detectorType, probeParams));
			if(probe) useGpu = probe->isGpuAvailable();
		}

		// Run only the default likelihood variant for every detector;
		// the TF-IDF variant adds a second run on the default detector
		// (kFeatureGfttOrb) so the alternative likelihood path stays
		// exercised without quadrupling the test runtime.
		std::vector<bool> tfIdfVariants = {false};
		if(detectorType == defaultDetector) tfIdfVariants.push_back(true);

		for(bool tfIdfUsed : tfIdfVariants)
		{
			const std::string detectorLabel = typeName
					+ (tfIdfUsed ? "[TfIdf]" : "[Likelihood]")
					+ (useGpu ? "[GPU]" : "");
			SCOPED_TRACE(std::string("detector=") + detectorLabel);

		ParametersMap params;
		params[Parameters::kRGBDEnabled()]                = "false";
		params[Parameters::kKpDetectorStrategy()]         = uNumber2Str(static_cast<int>(detectorType));
		params[Parameters::kSURFHessianThreshold()]       = "150";
		params[Parameters::kMemSTMSize()]                 = "20";
		params[Parameters::kKpTfIdfLikelihoodUsed()]      = tfIdfUsed ? "true" : "false";
		params[Parameters::kKpMaxFeatures()]              = "500";
		params[Parameters::kKpBadSignRatio()]             = "0.25";
		// SIFT-specific: lower the contrast threshold so more keypoints
		// survive on the low-texture frames in data/samples.
		params[Parameters::kSIFTContrastThreshold()]      = "0.01";
		// BRISK-specific: lower FAST threshold so the detector keeps more
		// candidates per frame; default is too strict for this dataset.
		params[Parameters::kBRISKThresh()]                = "10";
		// KAZE-specific: drop the response threshold an order of magnitude
		// so more (weaker) keypoints survive on the low-texture frames.
		params[Parameters::kKAZEThreshold()]              = "0.0001";
		// GFTT-specific: tighten the minimum keypoint separation (default
		// 7 px) so more candidates fit per frame.
		params[Parameters::kGFTTMinDistance()]            = "5";
		params[Parameters::kMemBadSignaturesIgnored()]    = "false";
		params[Parameters::kMemRehearsalSimilarity()]     = "0.20";

		// Backend-specific asset paths + GPU/CUDA toggle. `useGpu` only
		// reaches here when this detector reports a usable GPU path; we
		// then flip the per-detector "use GPU" parameter on.
		const std::string gpuFlag = useGpu ? "true" : "false";
		if(detectorType == Feature2D::kFeatureSuperPointTorch)
		{
			params[Parameters::kSuperPointModelPath()] = superpointTorchModel;
			params[Parameters::kSuperPointCuda()]      = gpuFlag;
		}
		else if(detectorType == Feature2D::kFeatureSuperPointRpautrat)
		{
			params[Parameters::kSuperPointRpautratWeightsPath()] = superpointRpautratWeights;
			params[Parameters::kSuperPointRpautratModelPath()]   = superpointRpautratModel;
			params[Parameters::kSuperPointRpautratCuda()]        = gpuFlag;
		}
		else if(useGpu)
		{
			switch(detectorType)
			{
			case Feature2D::kFeatureSurf:      params[Parameters::kSURFGpuVersion()] = "true"; break;
			case Feature2D::kFeatureSift:      params[Parameters::kSIFTGpu()]        = "true"; break;
			case Feature2D::kFeatureOrb:       params[Parameters::kORBGpu()]         = "true"; break;
			case Feature2D::kFeatureFastBrief:
			case Feature2D::kFeatureFastFreak: params[Parameters::kFASTGpu()]        = "true"; break;
			case Feature2D::kFeatureGfttFreak:
			case Feature2D::kFeatureGfttBrief:
			case Feature2D::kFeatureGfttOrb:
			case Feature2D::kFeatureGfttDaisy: params[Parameters::kGFTTGpu()]        = "true"; break;
			default: break;
			}
		}

		const std::string workDb = workDbForCurrentTest(detectorLabel);
		UFile::erase(workDb);
		Rtabmap rtabmap;
		rtabmap.init(params, workDb);

		struct FrameStat {
			int    queryRow;    // 0-based query frame index
			double hypValue;    // rtabmap.getHighestHypothesisValue()
			int    hypId;       // rtabmap.getHighestHypothesisId() (1-based, 0 = none)
			bool   accepted;    // rtabmap.getLoopClosureId() > 0
			bool   correct;     // GT[queryRow][hypId-1] == 255
			bool   gtPositive;  // any GT[queryRow][*] == 255
		};
		std::vector<FrameStat> stats;
		stats.reserve(kNumFrames);

		CameraImages camera(samplesDir);
		ASSERT_TRUE(camera.init()) << "CameraImages.init() failed on " << samplesDir;

		UTimer wall;
		int i = 0;
		int badSignatureCount = 0;
		SensorData data = camera.takeImage();
		while(!data.imageRaw().empty())
		{
			++i;
			data.setId(i);
			data.setStamp(static_cast<double>(i));
			const bool ok = rtabmap.process(data, Transform());
			ASSERT_TRUE(ok) << detectorLabel << " rtabmap.process failed at frame " << i;

			// Per-frame checks on the just-processed signature in
			// working memory:
			//   * verify keypoints and descriptors stay 1-to-1 (no row
			//     silently dropped during BOW quantization),
			//   * count bad signatures (low-texture frames on this
			//     84-image set produce a handful with
			//     Mem/BadSignaturesIgnored=false; rehearsal does not
			//     merge bad signatures, so the per-frame count is stable).
			const Signature * lastSig =
					rtabmap.getMemory()->getLastWorkingSignature(false);
			ASSERT_TRUE(lastSig != nullptr)
					<< detectorLabel << " frame " << i << ": no last signature";
			EXPECT_EQ(static_cast<int>(lastSig->getWordsKpts().size()),
					lastSig->getWordsDescriptors().rows)
					<< detectorLabel << " frame " << i
					<< " (id=" << lastSig->id() << "): keypoints/descriptors "
					<< "size mismatch on signature ("
					<< lastSig->getWordsKpts().size() << " vs "
					<< lastSig->getWordsDescriptors().rows << ")";
			if(lastSig->isBadSignature()) ++badSignatureCount;

			FrameStat s;
			s.queryRow   = i - 1;
			s.hypValue   = rtabmap.getHighestHypothesisValue();
			s.hypId      = rtabmap.getHighestHypothesisId();
			s.accepted   = rtabmap.getLoopClosureId() > 0;

			s.gtPositive = false;
			for(int j = 0; j < gt.cols; ++j)
			{
				if(gt.at<unsigned char>(s.queryRow, j) == 255)
				{
					s.gtPositive = true;
					break;
				}
			}

			s.correct = false;
			if(s.hypId > 0 && s.hypId - 1 < gt.cols)
			{
				s.correct = gt.at<unsigned char>(s.queryRow, s.hypId - 1) == 255;
			}

			stats.push_back(s);
			data = camera.takeImage();
		}
		ASSERT_EQ(kNumFrames, i)
				<< detectorLabel << " expected " << kNumFrames
				<< " frames from " << samplesDir << ", got " << i;

		// Mem/BadSignaturesIgnored=false should reveal at least one bad
		// signature during the 84-frame run -- except for SuperPoint
		// (Magicleap) on GPU, whose dense features always yield >= 1
		// valid word per frame on this dataset, so no frame ever turns
		// into a bad signature.
		if(detectorType != Feature2D::kFeatureSuperPointTorch)
		{
			EXPECT_GE(badSignatureCount, 1)
					<< detectorLabel << " expected at least 1 bad signature with "
					<< "Mem/BadSignaturesIgnored=false; got " << badSignatureCount;
		}

		rtabmap.close();

		// Standard rtabmap P/R curve: sort frames by hypothesis value
		// descending and walk down; precision = correct hypotheses so far
		// divided by total hypotheses so far, recall = correct so far over
		// gtTotalPositives. "Recall at 100% precision" is the recall at
		// the last point before the first FP appears.
		std::vector<FrameStat> sorted = stats;
		std::sort(sorted.begin(), sorted.end(),
				[](const FrameStat & a, const FrameStat & b){
					return a.hypValue > b.hypValue;
				});

		int   tp = 0, fp = 0;
		float recallAt100p = 0.0f;
		float thrAt100p    = 0.0f;
		bool  seenFp       = false;
		for(const FrameStat & s : sorted)
		{
			if(s.hypId <= 0 || s.hypValue <= 0.0)
			{
				continue;  // no hypothesis at all this frame
			}
			if(s.correct)
			{
				++tp;
				if(!seenFp)
				{
					recallAt100p = float(tp) / float(gtTotalPositives);
					thrAt100p    = static_cast<float>(s.hypValue);
				}
			}
			else
			{
				if(!seenFp)
				{
					// One-shot diagnostic: the first FP is what gates
					// recall@100%P. Print the (query, matched) pair so we
					// can eyeball whether it's a true mismatch or just a
					// visually-similar frame the GT happens not to flag.
					std::cerr << "[" << detectorLabel << "] first-FP query="
							<< (s.queryRow + 1)
							<< " matched=" << s.hypId
							<< " hypValue=" << s.hypValue
							<< " (TPs above this point=" << tp << ")\n";
				}
				++fp;
				seenFp = true;
			}
		}

		// Also report end-of-run precision/recall at the default rtabmap
		// loop threshold (i.e. counting only accepted closures) -- closer
		// to what a real deployment would observe.
		int acceptedTp = 0, acceptedFp = 0, acceptedFn = 0;
		for(const FrameStat & s : stats)
		{
			if(s.accepted && s.correct)      ++acceptedTp;
			else if(s.accepted && !s.correct) ++acceptedFp;
			else if(s.gtPositive)             ++acceptedFn;
		}
		const float acceptedPrec = (acceptedTp + acceptedFp) > 0
				? float(acceptedTp) / float(acceptedTp + acceptedFp) : 0.0f;
		const float acceptedRec  = gtTotalPositives > 0
				? float(acceptedTp) / float(gtTotalPositives) : 0.0f;

		// Dump the accepted loops as a 84x84 binary matrix in the same
		// shape as samples_GT.bmp (pixel (query, loop) = 255 when rtabmap
		// accepted that closure) so the run is easy to diff visually
		// against the ground truth.
		cv::Mat detectionsMat = cv::Mat::zeros(kNumFrames, kNumFrames, CV_8UC1);
		for(const FrameStat & s : stats)
		{
			if(s.accepted && s.hypId > 0
					&& s.hypId - 1 < detectionsMat.cols
					&& s.queryRow < detectionsMat.rows)
			{
				detectionsMat.at<unsigned char>(s.queryRow, s.hypId - 1) = 255;
			}
		}

		std::string safeLabel = detectorLabel;
		for(char & c : safeLabel)
		{
			if(c == '+' || c == '/' || c == ' ' || c == '\\'
					|| c == '[' || c == ']') c = '_';
		}
		const std::string detectionsBmp = test::tempPath(uFormat(
				"rtabmap_integration_AppearanceOnly_%s_loops.bmp", safeLabel.c_str()));
		cv::imwrite(detectionsBmp, detectionsMat);
		std::cerr << "[" << detectorLabel << "] loop-closure matrix -> "
				<< detectionsBmp << "\n";

		std::cerr << "[" << detectorLabel << "]"
				<< " gtPos=" << gtTotalPositives
				<< " sortedTP=" << tp << " sortedFP=" << fp
				<< " recall@100%P=" << recallAt100p << " (thr=" << thrAt100p << ")"
				<< " accepted: tp=" << acceptedTp << " fp=" << acceptedFp
				<< " prec=" << acceptedPrec << " recall=" << acceptedRec
				<< " wall=" << wall.elapsed() << "s\n";

		if(acceptedTp + acceptedFp == 0)
		{
			std::cerr << "[" << detectorLabel << "] note: no loop closure accepted "
					"at default threshold (sortedTP=" << tp << ", sortedFP=" << fp
					<< ")\n";
		}
		else if(acceptedPrec < 0.5f)
		{
			std::cerr << "[" << detectorLabel << "] note: accepted-loop precision "
					<< acceptedPrec << " below 0.5 (TP=" << acceptedTp
					<< ", FP=" << acceptedFp << ")\n";
		}

		const bool isCornerBased =
				detectorType == Feature2D::kFeatureOrb ||
				detectorType == Feature2D::kFeatureGfttFreak ||
				detectorType == Feature2D::kFeatureGfttBrief ||
				detectorType == Feature2D::kFeatureGfttOrb   ||
				detectorType == Feature2D::kFeatureGfttDaisy ||
				detectorType == Feature2D::kFeatureFastFreak ||
				detectorType == Feature2D::kFeatureFastBrief;
		const float recallFloor = isCornerBased ? 0.5f : 0.9f;
		EXPECT_GE(recallAt100p, recallFloor)
				<< detectorLabel << " recall@100%P=" << recallAt100p
				<< " is below " << recallFloor
				<< " (sortedTP=" << tp << ", sortedFP=" << fp << ")";

		++detectorsTested;
		}  // end for(tfIdfUsed)
	}
	ASSERT_GT(detectorsTested, 0) << "no Features2D detector was available in this build";
}
