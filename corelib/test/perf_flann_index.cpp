// Comparison of the FlannIndex backends: times, memory and recall of every
// algorithm on synthetic point clouds and descriptors.
//
// Their own executable, run by ctest under the "performance" label, so that
// they don't slow down the unit tests and can be scaled up freely:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//
// Nothing is asserted on the times, which depend on the machine: they are
// printed so that a change of backend, of parameters or of a vendored library
// version can be compared to what it replaces.
#include "FlannIndexBackends.h"

// The times are reported rather than asserted on: which backend is the fastest
// depends on the machine. They are here so that a change of backend, of
// parameters or of nanoflann version can be compared to what it replaces.
TEST(FlannIndexPerfTest, AllBackendsOnPointClouds)
{
	const int cloudSize = 100000;
	const int querySize = 10000;
	const int knn = 2;
	const float radius = 1.0f;

	for(int dim = 2; dim <= 3; ++dim)
	{
		const cv::Mat cloud = makeCloud(cloudSize, dim, 5);
		const cv::Mat queries = makeCloud(querySize, dim, 6);

		for(float factor: REBALANCING_FACTORS)
		{
			std::cout << "[          ] " << dim << "D cloud of " << cloudSize
					  << " points, " << querySize << " queries, knn=" << knn
					  << ", rebalancing factor=" << factor << std::endl;
			compare(FLOAT_BACKENDS, sizeof(FLOAT_BACKENDS)/sizeof(Backend), cloud, queries, knn, radius, factor);
		}
	}
}

// The same backends on descriptor dimensions, where the exact kd-trees lose
// their advantage over the exhaustive search: the higher the dimension, the
// more of the tree a search has to visit. A single rebalancing factor here, it
// doesn't affect the searches being compared.
TEST(FlannIndexPerfTest, AllBackendsOnFloatDescriptors)
{
	const int descriptorCount = 50000;
	const int querySize = 300;
	const int knn = 2;

	for(int dim: {32, 64, 128, 256})
	{
		const cv::Mat descriptors = makeDescriptors(descriptorCount, dim, 1000, 11);
		const cv::Mat queries = perturbedQueries(descriptors, querySize, 12);

		std::cout << "[          ] " << dim << "D float descriptors, " << descriptorCount
				  << " indexed, " << querySize << " queries, knn=" << knn << std::endl;
		compare(FLOAT_BACKENDS, sizeof(FLOAT_BACKENDS)/sizeof(Backend), descriptors, queries, knn);
	}
}

TEST(FlannIndexPerfTest, AllBackendsOnBinaryDescriptors)
{
	const int descriptorCount = 50000;
	const int querySize = 300;
	const int knn = 2;

	for(int bytes: {32, 64}) // 256 and 512 bits
	{
		const cv::Mat descriptors = makeBinaryDescriptors(descriptorCount, bytes, 1000, 13);
		const cv::Mat queries = perturbedQueries(descriptors, querySize, 14);

		std::cout << "[          ] " << bytes*8 << " bits binary descriptors, " << descriptorCount
				  << " indexed, " << querySize << " queries, knn=" << knn << std::endl;
		compare(BINARY_BACKENDS, sizeof(BINARY_BACKENDS)/sizeof(Backend), descriptors, queries, knn);
	}
}

// rtflann's single kd-tree rebuilds itself entirely on every addPoints() call
// (see KDTreeSingleIndex::addPoints()), which is what the incremental nanoflann
// tree is for. The cloud is kept small here because of it.
TEST(FlannIndexPerfTest, IncrementalInsertionSpeed)
{
	const int dim = 3;
	const int cloudSize = 10000;
	const int addedPoints = 500;

	const cv::Mat cloud = makeCloud(cloudSize, dim, 7);
	const cv::Mat addedCloud = makeCloud(addedPoints, dim, 8);

	for(float factor: REBALANCING_FACTORS)
	{
		std::cout << "[          ] " << dim << "D cloud of " << cloudSize << " points, "
				  << addedPoints << " points added one by one, rebalancing factor=" << factor << std::endl;

		for(const Backend & backend: INCREMENTAL_BACKENDS)
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, cloud, false, factor);

			UTimer timer;
			for(int i=0; i<addedPoints; ++i)
			{
				ASSERT_EQ(index.addPoints(addedCloud.row(i)).size(), 1u) << backend.name;
			}
			const double addTime = timer.ticks();

			EXPECT_EQ(index.indexedFeatures(), (size_t)(cloudSize + addedPoints)) << backend.name;

			// The added points have to be findable, whichever way they got in.
			cv::Mat indices;
			cv::Mat dists;
			index.knnSearch(addedCloud.row(addedPoints-1), indices, dists, 1);
			EXPECT_NEAR(dists.at<float>(0, 0), 0.0f, 1e-3f) << backend.name;

			std::cout << "[          ]   " << backend.name
					  << " insertions=" << uFormat("%7.1f", addTime*1000.0) << " ms"
					  << " memory=" << uFormat("%6d", (int)(index.memoryUsed()/1024)) << " KB"
					  << std::endl;
			RecordProperty(uFormat("alg%d_factor%d_insert_us", (int)backend.algorithm, (int)factor), (int)(addTime*1e6));
		}
	}
}

// What the rebalancing factor buys. rtflann inserts new points into the tree
// the split planes of which were chosen for the points it was built with, so
// the tree slowly degrades as it grows; over 1, the factor tells by how much it
// is allowed to grow before being rebuilt. That only shows on an index that
// grew a lot since it was built, which is what this does: a quarter of the
// points are indexed, the rest is added one by one, as the dictionary does.
//
// The single kd-trees are not part of it: rtflann's rebuilds itself on every
// addPoints() whatever the factor, which takes minutes at this size.
TEST(FlannIndexPerfTest, RebalancingFactorOnAGrowingIndex)
{
	const int dim = 128;
	const int initialCount = 5000;
	const int addedCount = 10000;
	const int querySize = 500;
	const int knn = 2;

	const cv::Mat descriptors = makeDescriptors(initialCount+addedCount, dim, 200, 15);
	const cv::Mat queries = perturbedQueries(descriptors, querySize, 16);

	// Ground truth over all the points, indexed or added.
	cv::Mat reference;
	{
		FlannIndex linear;
		cv::Mat dists;
		linear.buildIndex(FlannIndex::FLANN_INDEX_LINEAR, descriptors, false, 1.0f);
		linear.knnSearch(queries, reference, dists, knn);
	}

	const Backend growingBackends[] = {
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	std::cout << "[          ] " << dim << "D float descriptors, " << initialCount
			  << " indexed then " << addedCount << " added one by one, "
			  << querySize << " queries, knn=" << knn << std::endl;

	for(const Backend & backend: growingBackends)
	{
		for(float factor: REBALANCING_FACTORS)
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, descriptors.rowRange(0, initialCount), false, factor);

			UTimer timer;
			for(int i=0; i<addedCount; ++i)
			{
				index.addPoints(descriptors.row(initialCount+i));
			}
			const double insertTime = timer.ticks();

			cv::Mat indices;
			cv::Mat dists;
			index.knnSearch(queries, indices, dists, knn);
			const double knnTime = timer.ticks();

			ASSERT_EQ(index.indexedFeatures(), (size_t)descriptors.rows) << backend.name;

			std::cout << "[          ]   " << backend.name
					  << " factor=" << factor
					  << " insertions=" << uFormat("%7.1f", insertTime*1000.0) << " ms"
					  << " knn=" << uFormat("%7.1f", knnTime*1000.0) << " ms"
					  << " memory=" << uFormat("%6d", (int)(index.memoryUsed()/1024)) << " KB"
					  << " recall=" << uFormat("%5.1f", recall(indices, reference)*100.0f) << " %"
					  << std::endl;
		}
	}
}

// The index of a dictionary that forgets: as many points removed as added, so
// the number of indexed points stays the same while the dead ones accumulate.
// Only a rebuild drops them, which is what a factor over 1 allows.
TEST(FlannIndexPerfTest, RebalancingFactorOnAChurningIndex)
{
	const int dim = 128;
	const int initialCount = 5000;
	const int churnCount = 10000;
	const int querySize = 500;
	const int knn = 2;

	const cv::Mat descriptors = makeDescriptors(initialCount+churnCount, dim, 200, 17);
	const cv::Mat queries = perturbedQueries(descriptors, querySize, 18);

	const Backend churningBackends[] = {
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	std::cout << "[          ] " << dim << "D float descriptors, " << initialCount
			  << " indexed, then " << churnCount << " added and as many removed" << std::endl;

	for(const Backend & backend: churningBackends)
	{
		for(float factor: REBALANCING_FACTORS)
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, descriptors.rowRange(0, initialCount), false, factor);

			UTimer timer;
			for(int i=0; i<churnCount; ++i)
			{
				index.addPoints(descriptors.row(initialCount+i));
				index.removePoint(i); // the oldest one still indexed
			}
			const double churnTime = timer.ticks();

			cv::Mat indices;
			cv::Mat dists;
			index.knnSearch(queries, indices, dists, knn);
			const double knnTime = timer.ticks();

			// Whatever the factor, the same points are searched: the removed
			// ones are gone from the results either way.
			EXPECT_EQ(index.indexedFeatures(), (size_t)initialCount) << backend.name;

			std::cout << "[          ]   " << backend.name
					  << " factor=" << factor
					  << " add+remove=" << uFormat("%7.1f", churnTime*1000.0) << " ms"
					  << " knn=" << uFormat("%7.1f", knnTime*1000.0) << " ms"
					  << " memory=" << uFormat("%6d", (int)(index.memoryUsed()/1024)) << " KB"
					  << std::endl;
		}
	}
}

namespace {

// Both comparisons below are about the randomized kd-tree: it is the only
// strategy the dictionary uses whose accuracy can move, the exact ones always
// returning the true neighbors. Its split dimensions are picked at random, so
// every configuration is averaged over several seeds, without which a couple of
// points of recall cannot be told from run to run variation.
const int DIM = 128;
const int KNN = 2;

struct Average
{
	void add(float recallRatio, double buildTime, double knnTime, size_t memory)
	{
		recall_ += recallRatio;
		buildTime_ += buildTime;
		knnTime_ += knnTime;
		memory_ += memory;
		++count_;
	}
	float recall() const {return recall_/float(count_);}
	double buildTime() const {return buildTime_/double(count_);}
	double knnTime() const {return knnTime_/double(count_);}
	size_t memory() const {return memory_/count_;}

	float recall_ = 0.0f;
	double buildTime_ = 0.0;
	double knnTime_ = 0.0;
	size_t memory_ = 0;
	int count_ = 0;
};

void reportAverage(const std::string & name, const Average & average)
{
	std::cout << "[          ]   " << name
			  << " build=" << uFormat("%8.1f", average.buildTime()*1000.0) << " ms"
			  << " knn=" << uFormat("%7.1f", average.knnTime()*1000.0) << " ms"
			  << " memory=" << uFormat("%7d", (int)(average.memory()/1024)) << " KB"
			  << " recall=" << uFormat("%5.1f", average.recall()*100.0f) << " %"
			  << std::endl;
}

// Descriptors are drawn around cluster centers, and the density of a cluster is
// what makes a nearest neighbor unambiguous: with too few centers for the
// number of descriptors, every point of a cluster is about as close to a query
// as its neighbors are, and the recall of every approximate search collapses
// whatever the index. Ten descriptors per center keeps that out of the way.
int clusterCount(int descriptors)
{
	return std::max(100, descriptors/10);
}

// The neighbors an exact search finds, the reference recall is measured
// against. The single kd-tree is used rather than the exhaustive search: it
// returns the same neighbors an order of magnitude faster, which is what makes
// the million descriptors comparisons affordable.
cv::Mat groundTruth(const cv::Mat & data, const cv::Mat & queries)
{
	FlannIndex exact;
	cv::Mat indices;
	cv::Mat dists;
	exact.buildIndex(FlannIndex::FLANN_INDEX_KDTREE_SINGLE, data, false, 1.0f);
	exact.knnSearch(queries, indices, dists, KNN);
	return indices;
}

// Is an index that grew by insertion worse than the same points indexed in one
// go? The freshly built index is what a rebuild would give, so the difference
// between the two is exactly what rebuilding on growth would recover.
void compareGrownAndFreshlyBuilt(int finalCount, int seeds, int queryCount, const std::vector<int> & growths)
{
	std::cout << "[          ] " << DIM << "D float descriptors, " << finalCount
			  << " indexed, " << queryCount << " queries, knn=" << KNN
			  << ", averaged over " << seeds << " seed" << (seeds>1?"s":"") << std::endl;

	std::map<int, Average> grown; // growth factor -> average
	Average fresh;
	for(int seed=0; seed<seeds; ++seed)
	{
		const cv::Mat descriptors = makeDescriptors(finalCount, DIM, clusterCount(finalCount), 40+seed);
		const cv::Mat queries = perturbedQueries(descriptors, queryCount, 60+seed);
		const cv::Mat reference = groundTruth(descriptors, queries);

		cv::Mat indices;
		cv::Mat dists;

		// The whole set indexed at once: the tree a rebuild would give.
		{
			FlannIndex index;
			UTimer timer;
			index.buildIndex(FlannIndex::FLANN_INDEX_KDTREE, descriptors, false, 1.0f);
			const double buildTime = timer.ticks();
			index.knnSearch(queries, indices, dists, KNN);
			fresh.add(recall(indices, reference), buildTime, timer.ticks(), index.memoryUsed());
		}

		// The same set reached by building the index with finalCount/growth of
		// the descriptors, then adding all the others one by one, never
		// rebuilding: the state the index would be in after growing that much.
		for(int growth: growths)
		{
			const int initialCount = finalCount/growth;
			FlannIndex index;
			UTimer timer;
			index.buildIndex(FlannIndex::FLANN_INDEX_KDTREE, descriptors.rowRange(0, initialCount), false, 1.0f);
			for(int i=initialCount; i<finalCount; ++i)
			{
				index.addPoints(descriptors.row(i));
			}
			const double buildTime = timer.ticks();
			ASSERT_EQ(index.indexedFeatures(), (size_t)finalCount);

			index.knnSearch(queries, indices, dists, KNN);
			grown[growth].add(recall(indices, reference), buildTime, timer.ticks(), index.memoryUsed());
		}
	}

	reportAverage("all indexed in one go               ", fresh);
	for(const auto & iter: grown)
	{
		// e.g. 100x: built with 10000 of them, the 990000 others added one by one
		reportAverage(uFormat("grown %4dx (%8d indexed first)", iter.first, finalCount/iter.first), iter.second);
	}
}

// What accumulating removed points costs, and what a rebuild recovers. The
// index that is never rebuilt keeps them: they still take memory and are still
// visited by the searches.
void compareRemovedFractions(int count, int seeds, int queryCount, const std::vector<int> & removedPercents)
{
	std::cout << "[          ] " << DIM << "D float descriptors, " << count
			  << " indexed then partly removed, " << queryCount << " queries, knn="
			  << KNN << ", averaged over " << seeds << " seed" << (seeds>1?"s":"") << std::endl;

	std::map<int, Average> kept;    // removed % -> index that kept the removed points
	std::map<int, Average> rebuilt; // removed % -> index built on the live ones only
	for(int seed=0; seed<seeds; ++seed)
	{
		const cv::Mat descriptors = makeDescriptors(count, DIM, clusterCount(count), 80+seed);

		for(int removed: removedPercents)
		{
			// Spread the removed points over the whole set.
			std::vector<int> live;
			std::vector<int> indexOfLive(count, -1);
			for(int i=0; i<count; ++i)
			{
				if(i%100 >= removed)
				{
					indexOfLive[i] = (int)live.size();
					live.push_back(i);
				}
			}
			cv::Mat liveDescriptors((int)live.size(), DIM, CV_32FC1);
			for(size_t i=0; i<live.size(); ++i)
			{
				descriptors.row(live[i]).copyTo(liveDescriptors.row((int)i));
			}
			// Queried with points that are still indexed: queries taken from the
			// removed ones would lower the recall of both configurations, their
			// nearest live neighbor being another point altogether.
			const cv::Mat queries = perturbedQueries(liveDescriptors, queryCount, 100+seed);
			const cv::Mat reference = groundTruth(liveDescriptors, queries);

			cv::Mat indices;
			cv::Mat dists;

			// Everything indexed, the removed points only marked as such.
			{
				FlannIndex index;
				UTimer timer;
				index.buildIndex(FlannIndex::FLANN_INDEX_KDTREE, descriptors, false, 1.0f);
				for(int i=0; i<count; ++i)
				{
					if(indexOfLive[i] < 0)
					{
						index.removePoint(i);
					}
				}
				const double buildTime = timer.ticks();
				ASSERT_EQ(index.indexedFeatures(), live.size());

				index.knnSearch(queries, indices, dists, KNN);
				const double knnTime = timer.ticks();

				// Its indexes are those of the whole set, the reference's are
				// those of the live points only.
				cv::Mat translated(indices.size(), CV_32SC1, cv::Scalar(-1));
				for(int i=0; i<indices.rows; ++i)
				{
					for(int j=0; j<indices.cols; ++j)
					{
						const int found = indices.at<int>(i, j);
						translated.at<int>(i, j) = found>=0?indexOfLive[found]:-1;
					}
				}
				kept[removed].add(recall(translated, reference), buildTime, knnTime, index.memoryUsed());
			}

			// Only the live points indexed: what rebuilding gives.
			{
				FlannIndex index;
				UTimer timer;
				index.buildIndex(FlannIndex::FLANN_INDEX_KDTREE, liveDescriptors, false, 1.0f);
				const double buildTime = timer.ticks();
				index.knnSearch(queries, indices, dists, KNN);
				rebuilt[removed].add(recall(indices, reference), buildTime, timer.ticks(), index.memoryUsed());
			}
		}
	}

	for(const auto & iter: kept)
	{
		reportAverage(uFormat("%3d%% removed, kept in the index     ", iter.first), iter.second);
		reportAverage(uFormat("%3d%% removed, rebuilt without them  ", iter.first), rebuilt[iter.first]);
	}
}

} // namespace

TEST(FlannIndexPerfTest, GrownIndexAgainstFreshlyBuiltOne)
{
	compareGrownAndFreshlyBuilt(20000, 3, 200, {2, 10, 100, 1000});
}

TEST(FlannIndexPerfTest, RecallAgainstTheFractionOfRemovedPoints)
{
	compareRemovedFractions(10000, 3, 200, {0, 25, 50, 75, 90});
}

// The same two comparisons on a dictionary of a million words, where a single
// descriptor matrix is already 512 MB and each of them takes minutes. Disabled
// so that a plain run of this executable stays in the seconds, run them with:
//   bin/test_flann_index_perf --gtest_also_run_disabled_tests
// They stay compiled, so they cannot rot as FlannIndex changes.
TEST(FlannIndexPerfTest, DISABLED_GrownIndexAgainstFreshlyBuiltOneOnAMillionWords)
{
	compareGrownAndFreshlyBuilt(1000000, 3, 100, {2, 100, 1000});
}

TEST(FlannIndexPerfTest, DISABLED_RecallAgainstTheFractionOfRemovedPointsOnAMillionWords)
{
	compareRemovedFractions(1000000, 3, 100, {50, 90});
}

// The search RegistrationVis does per frame when a guess transform is given
// (Vis/CorGuessWinSize): the keypoints of the frame are indexed, and the points
// projected from the previous frame are looked up around their projection. The
// index is built and thrown away every frame, so its build time weighs as much
// as its search time. Before the nanoflann backend, this was a rtflann
// randomized kd-tree forest.
TEST(FlannIndexPerfTest, RegistrationGuessMatching)
{
	const int keypoints = 1000;   // Vis/MaxFeatures
	const float radius = 40.0f;   // Vis/CorGuessWinSize
	const int frames = 1000;      // ~ a 50 s sequence at 20 Hz

	// Image points rather than a cube of them.
	cv::RNG rng(140);
	cv::Mat points(keypoints, 2, CV_32FC1);
	cv::Mat projected(keypoints, 2, CV_32FC1);
	for(int i=0; i<keypoints; ++i)
	{
		points.at<float>(i, 0) = rng.uniform(0.0f, 640.0f);
		points.at<float>(i, 1) = rng.uniform(0.0f, 480.0f);
		projected.at<float>(i, 0) = rng.uniform(0.0f, 640.0f);
		projected.at<float>(i, 1) = rng.uniform(0.0f, 480.0f);
	}

	// A factor of 1 for the rtflann rows keeps their per-point bookkeeping out
	// of the measurement, and picks the nanoflann tree that is built once.
	const Backend backends[] = {
		{"cv        BFMatcher                 ", FlannIndex::FLANN_INDEX_LINEAR, 1.0f, true},
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE, 1.0f},
		{"rtflann   kd-tree single            ", FlannIndex::FLANN_INDEX_KDTREE_SINGLE, 1.0f},
		{"nanoflann kd-tree single            ", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 1.0f},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	std::cout << "[          ] " << keypoints << " keypoints indexed and as many looked up in a "
			  << radius << " px radius, per frame" << std::endl;

	for(const Backend & backend: backends)
	{
		std::vector<std::vector<size_t> > indices;
		std::vector<std::vector<float> > dists;

		UTimer timer;
		for(int frame=0; frame<frames; ++frame)
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, points, false, backend.rebalancingFactor);
			index.radiusSearch(projected, indices, dists, radius, 0, 32, 0.0f, false);
		}
		const double perFrame = timer.ticks()/double(frames);

		size_t found = 0;
		for(const auto & neighbors: indices)
		{
			found += neighbors.size();
		}
		ASSERT_GT(found, 0u) << backend.name;

		std::cout << "[          ]   " << backend.name
				  << " build+search=" << uFormat("%6.3f", perFrame*1000.0) << " ms/frame"
				  << " (" << uFormat("%5.2f", perFrame*1000.0*20.0) << " ms/s at 20 Hz)"
				  << std::endl;
	}
}

// The dictionary that is built to match two sets of descriptors and thrown
// away: the "from" ones are indexed, the "to" ones are searched with knn=2 for
// the ratio test. Both are a frame's worth of features, or a feature map's
// worth for the odometry, which makes the build weigh as much as the searches,
// unlike the vocabulary sized comparison above.
namespace {

void compareDictionaryMatching(int indexedCount, int queriedCount)
{
	const int frames = 10;

	// A factor of 1 as RegistrationVis sets it for that dictionary: the index is
	// built once, so it is neither kept ready to be added to nor rebuilt. The
	// incremental nanoflann tree is kept in the comparison to show what asking
	// for one costs here.
	const Backend backends[] = {
		{"linear    exhaustive                ", FlannIndex::FLANN_INDEX_LINEAR, 1.0f},
		{"cv        BFMatcher                 ", FlannIndex::FLANN_INDEX_LINEAR, 1.0f, true},
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE, 1.0f},
		{"rtflann   kd-tree single            ", FlannIndex::FLANN_INDEX_KDTREE_SINGLE, 1.0f},
		{"nanoflann kd-tree single            ", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 1.0f},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	for(int dim: {32, 64, 128, 256})
	{
		const cv::Mat from = makeDescriptors(indexedCount, dim, clusterCount(indexedCount), 150);
		const cv::Mat to = perturbedQueries(from, queriedCount, 151);
		const cv::Mat reference = groundTruth(from, to);

		std::cout << "[          ] " << dim << "D float descriptors, " << indexedCount
				  << " indexed and " << queriedCount << " matched against them, per frame" << std::endl;

		for(const Backend & backend: backends)
		{
			cv::Mat indices;
			cv::Mat dists;

			UTimer timer;
			for(int frame=0; frame<frames; ++frame)
			{
				FlannIndex index;
				index.buildIndex(backend.algorithm, from, false, backend.rebalancingFactor);
				index.knnSearch(to, indices, dists, KNN);
			}
			const double perFrame = timer.ticks()/double(frames);

			std::cout << "[          ]   " << backend.name
					  << " build+search=" << uFormat("%7.2f", perFrame*1000.0) << " ms/frame"
					  << " recall=" << uFormat("%5.1f", recall(indices, reference)*100.0f) << " %"
					  << std::endl;
		}
	}
}

} // namespace

// What RegistrationVis does to match two frames: as many descriptors indexed as
// searched (Vis/MaxFeatures on both sides).
TEST(FlannIndexPerfTest, RegistrationDictionaryMatching)
{
	compareDictionaryMatching(1000, 1000);
}

// What OdometryF2M does: the frame is matched against the feature map, which
// holds more of them (Odom/F2M/MaxSize), so the index is bigger than the set of
// queries and its build weighs more.
TEST(FlannIndexPerfTest, OdometryFrameToMapMatching)
{
	compareDictionaryMatching(2000, 1000);
}
