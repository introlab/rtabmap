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
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Version.h>
#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorCaptureInfo.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>

#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

using namespace rtabmap;

namespace {

std::string testDataPath(const std::string & basename)
{
	return std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/" + basename;
}

// Output DB path named after the running test so manual inspection is easy
// (rtabmap-databaseViewer /tmp/rtabmap_integration_<TestName>.db). The file
// is overwritten on each run and NOT deleted at teardown.
std::string workDbForCurrentTest()
{
	const ::testing::TestInfo * info =
			::testing::UnitTest::GetInstance()->current_test_info();
	const std::string testName = info != nullptr ? info->name() : "unknown";
	return uFormat("/tmp/rtabmap_integration_%s.db", testName.c_str());
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
		const std::map<double, Transform> * goldenStampedGroundTruth = nullptr)
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

	// Output DB at a discoverable path named after the test. We erase any
	// previous run's file so each invocation starts fresh, but we deliberately
	// do NOT delete it at the end -- callers can inspect it with
	// rtabmap-databaseViewer to verify the run looks sensible.
	const std::string workDb = workDbForCurrentTest();
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

	// Prime the loop with the first sample.
	SensorCaptureInfo info;
	SensorData data = dbReader.takeData(&info);
	applyGoldenGroundTruth(data);

	Transform previousStoredOdomPose;
	while(data.isValid())
	{
		++result.framesRead;

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
		const Transform odomPose = odometry->process(odomData, guess, &odomInfo);

		if(odomPose.isNull())
		{
			++result.odomLost;
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
			<< std::endl;

	// close(true) flushes the in-memory session to the output DB so the file
	// at workDb is a complete, openable database after the test returns.
	rtabmap.close(true);
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
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.7";
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
	// actually assembled when the build has it. ICP-only replay is fully
	// deterministic so the leaf counts are exact across runs.
	EXPECT_EQ(21372, result.octomapNodes);
	EXPECT_EQ(16178, result.octomapEmptyCells);
	EXPECT_EQ(1845, result.octomapObstacleCells);
#endif

	// Replay is deterministic; matching golden GT was captured from a clean
	// run of this exact configuration, so RMSE should be ~0.
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (golden GT not injected?)";
	EXPECT_LT(result.translationalRmseFinal, 0.001f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
}

// ---------------------------------------------------------------------------
// PR2 2D-laser + stereo sample (~15s).
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_Stereo)
{
	const std::string dbPath = testDataPath("pr2_scan2d_stereo_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	ParametersMap rtabmapParams = baseRtabmapParams();
	rtabmapParams[Parameters::kMemUseOdomFeatures()] = "true";
	ParametersMap odomParams = baseOdometryParams();

	// passOdomDataToRtabmap=true: rtabmap reuses the keypoints/descriptors
	// extracted during odometry (paired with Mem/UseOdomFeatures=true above).
	const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
			/*useStoredOdomAsGuess=*/true,
			/*passOdomDataToRtabmap=*/true);

	EXPECT_GT(result.framesRead, 0);
	EXPECT_EQ(0, result.odomLost) << "Odometry should never lose tracking";
	EXPECT_EQ(27, result.finalGlobalGraphSize);
	EXPECT_GE(result.proximityDetections, 1)
			<< "PR2 2D-scan dataset should produce proximity detections";
	// Observed across 5 runs: empty 531-553, obstacle 5074-5102.
	EXPECT_GE(result.gridEmptyCells, 450);
	EXPECT_LE(result.gridEmptyCells, 650);
	EXPECT_GE(result.gridObstacleCells, 4900);
	EXPECT_LE(result.gridObstacleCells, 5300);
#ifdef RTABMAP_OCTOMAP
	// Observed across 5 runs: empty 1834-1857, obstacle 21502-21671.
	EXPECT_GE(result.octomapEmptyCells, 1700);
	EXPECT_LE(result.octomapEmptyCells, 2000);
	EXPECT_GE(result.octomapObstacleCells, 21000);
	EXPECT_LE(result.octomapObstacleCells, 22000);
#endif
	// Stereo F2M visual odom + visual loop closure -- observed RMSE ~3 cm,
	// 5 cm bound gives ~50% headroom for run-to-run feature variance.
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (ground truth missing?)";
	EXPECT_LT(result.translationalRmseFinal, 0.05f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
}

// ---------------------------------------------------------------------------
// PR2 2D-laser + RGB-D sample (~15s).
// ---------------------------------------------------------------------------
TEST_F(RtabmapIntegrationFixture, PR2_Scan2D_RGBD)
{
	const std::string dbPath = testDataPath("pr2_scan2d_rgbd_sample_15s.db");
	SKIP_IF_MISSING(dbPath);

	ParametersMap rtabmapParams = baseRtabmapParams();
	rtabmapParams[Parameters::kMemUseOdomFeatures()] = "true";
	ParametersMap odomParams = baseOdometryParams();

	// passOdomDataToRtabmap=true: rtabmap reuses the keypoints/descriptors
	// extracted during odometry (paired with Mem/UseOdomFeatures=true above).
	const ReplayResult result = replayDatabase(dbPath, rtabmapParams, odomParams,
			/*useStoredOdomAsGuess=*/false,
			/*passOdomDataToRtabmap=*/true);

	EXPECT_GT(result.framesRead, 0);
	EXPECT_EQ(0, result.odomLost) << "Odometry should never lose tracking";
	EXPECT_EQ(21, result.finalGlobalGraphSize);
	EXPECT_GE(result.proximityDetections, 1)
			<< "PR2 2D-scan dataset should produce proximity detections";
	// Observed across 5 runs: empty 2900-3001, obstacle 4574-4698.
	EXPECT_GE(result.gridEmptyCells, 2700);
	EXPECT_LE(result.gridEmptyCells, 3200);
	EXPECT_GE(result.gridObstacleCells, 4400);
	EXPECT_LE(result.gridObstacleCells, 4900);
#ifdef RTABMAP_OCTOMAP
	// Observed across 5 runs: empty 6452-7474, obstacle 41125-42883.
	EXPECT_GE(result.octomapEmptyCells, 6000);
	EXPECT_LE(result.octomapEmptyCells, 8000);
	EXPECT_GE(result.octomapObstacleCells, 40000);
	EXPECT_LE(result.octomapObstacleCells, 44000);
#endif
	// RGB-D F2M visual odom + visual loop closure -- observed RMSE ~13 cm
	// (less stable than the stereo/ICP paths), 20 cm bound gives ~50%
	// headroom for visual-only loop-closure variability.
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (ground truth missing?)";
	EXPECT_LT(result.translationalRmseFinal, 0.20f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
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
	rtabmapParams[Parameters::kIcpOutlierRatio()] = "0.95";
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
	// Observed across 5 runs: empty 22785-22850, obstacle 1308-1316.
	EXPECT_GE(result.gridEmptyCells, 22500);
	EXPECT_LE(result.gridEmptyCells, 23200);
	EXPECT_GE(result.gridObstacleCells, 1250);
	EXPECT_LE(result.gridObstacleCells, 1400);
#ifdef RTABMAP_OCTOMAP
	// 2D-laser-only signatures: nothing to assemble into a 3D OctoMap.
	EXPECT_EQ(0, result.octomapEmptyCells);
	EXPECT_EQ(0, result.octomapObstacleCells);
#endif
	// Scan-based ICP loop closure with the PR2's 2D laser should align the
	// final trajectory to within 2 cm of the stored ground truth.
	ASSERT_GE(result.translationalRmseFinal, 0.0f)
			<< "No Gt/translational_rmse in stats (ground truth missing?)";
	EXPECT_LT(result.translationalRmseFinal, 0.02f)
			<< "Final trajectory RMSE = " << result.translationalRmseFinal << " m";
}
