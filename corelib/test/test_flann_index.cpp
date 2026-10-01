#include "FlannIndexBackends.h"

TEST(FlannIndexTest, ExactBackendsFindTheSameNeighbors)
{
	for(int dim = 2; dim <= 3; ++dim)
	{
		const cv::Mat cloud = makeCloud(5000, dim, 1);
		// An odd number of queries: with an odd knn, the rtflann backend used
		// to write one index past the end of the output matrix, which needs
		// query.rows*knn to be odd to show.
		const cv::Mat queries = makeCloud(501, dim, 2);
		for(int knn = 1; knn <= 2; ++knn)
		{
			cv::Mat referenceIndices;
			cv::Mat referenceDists;
			for(const Backend & backend: EXACT_BACKENDS)
			{
				FlannIndex index;
				index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);
				ASSERT_TRUE(index.isBuilt()) << backend.name;
				EXPECT_EQ(index.indexedFeatures(), (size_t)cloud.rows) << backend.name;

				cv::Mat indices;
				cv::Mat dists;
				index.knnSearch(queries, indices, dists, knn);
				ASSERT_EQ(indices.rows, queries.rows) << backend.name;
				ASSERT_EQ(indices.cols, knn) << backend.name;

				if(referenceIndices.empty())
				{
					referenceIndices = indices;
					referenceDists = dists;
					// Sanity check the reference itself: every query is inside
					// the cloud's box, so all neighbors have been found.
					for(int i=0; i<indices.rows; ++i)
					{
						for(int j=0; j<knn; ++j)
						{
							ASSERT_GE(indices.at<int>(i, j), 0) << "dim=" << dim << " knn=" << knn;
						}
					}
					continue;
				}

				for(int i=0; i<indices.rows; ++i)
				{
					for(int j=0; j<knn; ++j)
					{
						EXPECT_EQ(indices.at<int>(i, j), referenceIndices.at<int>(i, j))
							<< backend.name << " dim=" << dim << " knn=" << knn << " query=" << i << " n=" << j;
						EXPECT_NEAR(dists.at<float>(i, j), referenceDists.at<float>(i, j), 1e-3f)
							<< backend.name << " dim=" << dim << " knn=" << knn << " query=" << i << " n=" << j;
					}
				}
			}
		}
	}
}

TEST(FlannIndexTest, ExactBackendsFindTheSamePointsInRadius)
{
	for(int dim = 2; dim <= 3; ++dim)
	{
		const cv::Mat cloud = makeCloud(5000, dim, 3);
		const cv::Mat queries = makeCloud(200, dim, 4);
		const float radius = 5.0f;

		std::vector<std::vector<size_t> > referenceIndices;
		for(const Backend & backend: EXACT_BACKENDS)
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);

			std::vector<std::vector<size_t> > indices;
			std::vector<std::vector<float> > dists;
			index.radiusSearch(queries, indices, dists, radius);
			ASSERT_EQ(indices.size(), (size_t)queries.rows) << backend.name;

			// The backends don't return the points in the same order when they
			// are not sorted by distance, compare them as sets.
			for(size_t i=0; i<indices.size(); ++i)
			{
				std::sort(indices[i].begin(), indices[i].end());
			}

			if(referenceIndices.empty())
			{
				referenceIndices = indices;
				size_t found = 0;
				for(const auto & neighbors: indices)
				{
					found += neighbors.size();
				}
				ASSERT_GT(found, 0u) << "dim=" << dim << ", the radius is too small to compare anything";
				continue;
			}

			for(size_t i=0; i<indices.size(); ++i)
			{
				EXPECT_EQ(indices[i], referenceIndices[i]) << backend.name << " dim=" << dim << " query=" << i;
			}
		}
	}
}

TEST(FlannIndexTest, SerializedIndexIsLoadedBack)
{
	const cv::Mat cloud = makeCloud(2000, 3, 20);
	const cv::Mat queries = makeCloud(101, 3, 21);
	const int knn = 2;

	for(const Backend & backend: EXACT_BACKENDS)
	{
		for(bool checksum: {true, false})
		{
			FlannIndex index;
			index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);
			cv::Mat indices;
			cv::Mat dists;
			index.knnSearch(queries, indices, dists, knn);

			const std::vector<unsigned char> data = index.serializeIndex(checksum);
#ifdef _WIN32
			// rtflann serialization needs fmemopen, only the nanoflann backends
			// give back something on Windows.
			if(backend.algorithm != FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE)
			{
				EXPECT_TRUE(data.empty()) << backend.name;
				continue;
			}
#endif
			ASSERT_FALSE(data.empty()) << backend.name << " checksum=" << checksum;

			FlannIndex loaded;
			std::string error;
			ASSERT_TRUE(loaded.loadIndex(data, backend.algorithm, cloud, false, backend.rebalancingFactor, &error))
				<< backend.name << " checksum=" << checksum << ": " << error;
			EXPECT_TRUE(loaded.isBuilt()) << backend.name;
			EXPECT_EQ(loaded.indexedFeatures(), (size_t)cloud.rows) << backend.name;
			EXPECT_EQ(loaded.featuresType(), cloud.type()) << backend.name;
			EXPECT_EQ(loaded.featuresDim(), cloud.cols) << backend.name;

			// The loaded index has to give the very same neighbors.
			cv::Mat loadedIndices;
			cv::Mat loadedDists;
			loaded.knnSearch(queries, loadedIndices, loadedDists, knn);
			ASSERT_EQ(loadedIndices.size(), indices.size()) << backend.name;
			for(int i=0; i<indices.rows; ++i)
			{
				for(int j=0; j<knn; ++j)
				{
					EXPECT_EQ(loadedIndices.at<int>(i, j), indices.at<int>(i, j))
						<< backend.name << " checksum=" << checksum << " query=" << i;
				}
			}

			// The raw pointer overload takes the same data.
			FlannIndex loadedRaw;
			EXPECT_TRUE(loadedRaw.loadIndex(data.data(), data.size(), backend.algorithm, cloud, false, backend.rebalancingFactor, &error))
				<< backend.name << ": " << error;
		}
	}
}

TEST(FlannIndexTest, LoadIndexRefusesDataItCannotUse)
{
	const cv::Mat cloud = makeCloud(500, 3, 22);
	FlannIndex index;
	index.buildIndex(FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, cloud);
	const std::vector<unsigned char> data = index.serializeIndex(true);
	ASSERT_FALSE(data.empty());

	const FlannIndex::flann_algorithm_t algorithm = FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE;
	FlannIndex loaded;
	std::string error;

	// Another algorithm than the one it was built with
	EXPECT_FALSE(loaded.loadIndex(data, FlannIndex::FLANN_INDEX_KDTREE_SINGLE, cloud, false, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Another number of features
	error.clear();
	EXPECT_FALSE(loaded.loadIndex(data, algorithm, cloud.rowRange(0, cloud.rows-1), false, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Another dimension
	error.clear();
	EXPECT_FALSE(loaded.loadIndex(data, algorithm, makeCloud(cloud.rows, 2, 22), false, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Another distance
	error.clear();
	EXPECT_FALSE(loaded.loadIndex(data, algorithm, cloud, true, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Same shape, other content: caught by the checksum
	error.clear();
	EXPECT_FALSE(loaded.loadIndex(data, algorithm, makeCloud(cloud.rows, cloud.cols, 23), false, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Truncated
	error.clear();
	EXPECT_FALSE(loaded.loadIndex(data.data(), data.size()/2, algorithm, cloud, false, 2.0f, &error));
	EXPECT_FALSE(error.empty());

	// Nothing at all
	error.clear();
	std::vector<unsigned char> empty;
	EXPECT_FALSE(loaded.loadIndex(empty, algorithm, cloud, false, 2.0f, &error));

	// None of it left a half loaded index behind
	EXPECT_FALSE(loaded.isBuilt());
}

// A descriptor header can cover a whole batch of points when the index is never
// rebuilt (rebalancing factor of 1), which used to make serializeIndex() look
// them up one by one and throw.
TEST(FlannIndexTest, SerializesAnIndexWithBatchedHeadersAndRemovedPoints)
{
	const cv::Mat cloud = makeCloud(500, 3, 24);
	const cv::Mat added = makeCloud(100, 3, 25);

	for(float factor: {1.0f, 2.0f})
	{
		FlannIndex index;
		index.buildIndex(FlannIndex::FLANN_INDEX_KDTREE, cloud, false, factor);
		const std::vector<unsigned int> indexes = index.addPoints(added);
		ASSERT_EQ(indexes.size(), (size_t)added.rows);

		for(size_t i=0; i<10; ++i)
		{
			index.removePoint(indexes[i]);
		}
		EXPECT_EQ(index.indexedFeatures(), (size_t)(cloud.rows + added.rows - 10)) << "factor=" << factor;

		EXPECT_NO_THROW(index.serializeIndex(true)) << "factor=" << factor;
		EXPECT_NO_THROW(index.serializeIndex(false)) << "factor=" << factor;
	}
}

TEST(FlannIndexTest, AddedPointsAreFoundAndRemovedOnesAreNot)
{
	const cv::Mat cloud = makeCloud(500, 3, 26);
	const cv::Mat added = makeCloud(50, 3, 27);

	const Backend backends[] = {
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	for(const Backend & backend: backends)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);
		ASSERT_EQ(index.indexedFeatures(), (size_t)cloud.rows) << backend.name;

		const std::vector<unsigned int> indexes = index.addPoints(added);
		ASSERT_EQ(indexes.size(), (size_t)added.rows) << backend.name;
		for(size_t i=0; i<indexes.size(); ++i)
		{
			EXPECT_EQ(indexes[i], (unsigned int)(cloud.rows + i)) << backend.name;
		}
		EXPECT_EQ(index.indexedFeatures(), (size_t)(cloud.rows + added.rows)) << backend.name;

		// An added point is its own nearest neighbor.
		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(added.row(0), indices, dists, 1);
		EXPECT_EQ(indices.at<int>(0, 0), (int)indexes[0]) << backend.name;
		EXPECT_NEAR(dists.at<float>(0, 0), 0.0f, 1e-3f) << backend.name;

		// Once removed, it is not returned anymore, by either search.
		index.removePoint(indexes[0]);
		EXPECT_EQ(index.indexedFeatures(), (size_t)(cloud.rows + added.rows - 1)) << backend.name;

		index.knnSearch(added.row(0), indices, dists, 1);
		EXPECT_NE(indices.at<int>(0, 0), (int)indexes[0]) << backend.name;

		std::vector<std::vector<size_t> > radiusIndices;
		std::vector<std::vector<float> > radiusDists;
		index.radiusSearch(added.row(0), radiusIndices, radiusDists, 1.0f);
		ASSERT_EQ(radiusIndices.size(), 1u) << backend.name;
		for(size_t neighbor: radiusIndices[0])
		{
			EXPECT_NE(neighbor, (size_t)indexes[0]) << backend.name;
		}
	}
}

// The rebuild triggered by the removals renumbers nothing: the indexes handed
// out before it still designate the same points, which VWDictionary relies on.
TEST(FlannIndexTest, IndexesSurviveARebuild)
{
	const cv::Mat cloud = makeCloud(400, 3, 28);
	const cv::Mat added = makeCloud(400, 3, 29);

	const Backend backends[] = {
		{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
		{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, 2.0f},
	};

	for(const Backend & backend: backends)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, cloud, false, 2.0f);

		std::vector<unsigned int> indexes;
		for(int i=0; i<added.rows; ++i)
		{
			const std::vector<unsigned int> added1 = index.addPoints(added.row(i));
			ASSERT_EQ(added1.size(), 1u) << backend.name;
			indexes.push_back(added1[0]);
		}

		// Enough removals to get over the ratio a factor of 2 tolerates.
		for(int i=0; i<cloud.rows; ++i)
		{
			index.removePoint(i);
		}
		EXPECT_EQ(index.indexedFeatures(), (size_t)added.rows) << backend.name;

		// The points added before the rebuild are still where they were.
		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(added, indices, dists, 1);
		for(int i=0; i<added.rows; ++i)
		{
			EXPECT_EQ(indices.at<int>(i, 0), (int)indexes[i]) << backend.name << " point=" << i;
		}
	}
}

TEST(FlannIndexTest, UnsupportedOperationsAreRefused)
{
	const cv::Mat cloud = makeCloud(200, 3, 30);
	const cv::Mat added = makeCloud(10, 3, 31);

	// A nanoflann index built to never be rebuilt (factor 1) still takes points,
	// rebuilding itself as the tree that accepts them.
	FlannIndex staticIndex;
	staticIndex.buildIndex(FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, cloud, false, 1.0f);
	const std::vector<unsigned int> addedIndexes = staticIndex.addPoints(added);
	ASSERT_EQ(addedIndexes.size(), (size_t)added.rows);
	EXPECT_EQ(addedIndexes[0], (unsigned int)cloud.rows);
	EXPECT_EQ(staticIndex.indexedFeatures(), (size_t)(cloud.rows + added.rows));
	staticIndex.removePoint(0);
	EXPECT_EQ(staticIndex.indexedFeatures(), (size_t)(cloud.rows + added.rows - 1));

	// The points it held are still there, under the same indexes.
	cv::Mat indices;
	cv::Mat dists;
	staticIndex.knnSearch(cloud.row(1), indices, dists, 1);
	EXPECT_EQ(indices.at<int>(0, 0), 1);
	staticIndex.knnSearch(added.row(0), indices, dists, 1);
	EXPECT_EQ(indices.at<int>(0, 0), (int)addedIndexes[0]);

	// An index with removed points refers to holes in the features it was built
	// with, which the ones given back to loadIndex() cannot reproduce.
	FlannIndex incremental;
	incremental.buildIndex(FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE, cloud, false, 2.0f);
	EXPECT_FALSE(incremental.serializeIndex(true).empty());
	incremental.removePoint(0);
	EXPECT_TRUE(incremental.serializeIndex(true).empty());
}

TEST(FlannIndexTest, DistanceL1BackendsAgree)
{
	const cv::Mat cloud = makeCloud(1000, 8, 32);
	const cv::Mat queries = makeCloud(51, 8, 33);
	const int knn = 2;

	cv::Mat reference;
	cv::Mat referenceDists;
	for(const Backend & backend: EXACT_BACKENDS)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, cloud, true /* useDistanceL1 */, backend.rebalancingFactor);

		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(queries, indices, dists, knn);
		ASSERT_EQ(indices.rows, queries.rows) << backend.name;

		if(reference.empty())
		{
			reference = indices;
			referenceDists = dists;
			// The exhaustive reference is L1 indeed, not L2.
			const float * query = queries.ptr<float>(0);
			const float * neighbor = cloud.ptr<float>(indices.at<int>(0, 0));
			float l1 = 0.0f;
			for(int i=0; i<cloud.cols; ++i)
			{
				l1 += std::abs(query[i] - neighbor[i]);
			}
			EXPECT_NEAR(dists.at<float>(0, 0), l1, 1e-2f);
			continue;
		}

		for(int i=0; i<indices.rows; ++i)
		{
			for(int j=0; j<knn; ++j)
			{
				EXPECT_EQ(indices.at<int>(i, j), reference.at<int>(i, j)) << backend.name << " query=" << i;
				EXPECT_NEAR(dists.at<float>(i, j), referenceDists.at<float>(i, j), 1e-2f) << backend.name;
			}
		}
	}
}

TEST(FlannIndexTest, BinaryDescriptorsUseHammingDistances)
{
	const cv::Mat descriptors = makeBinaryDescriptors(1000, 32, 100, 34);
	const cv::Mat queries = descriptors.rowRange(0, 20); // the indexed ones

	const Backend backends[] = {
		{"linear    exhaustive (hamming)      ", FlannIndex::FLANN_INDEX_LINEAR},
		{"rtflann   LSH                       ", FlannIndex::FLANN_INDEX_LSH},
	};

	for(const Backend & backend: backends)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, descriptors, false, backend.rebalancingFactor);
		EXPECT_EQ(index.featuresType(), CV_8UC1) << backend.name;
		EXPECT_EQ(index.featuresDim(), descriptors.cols) << backend.name;

		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(queries, indices, dists, 1);
		// Hamming distances are integers
		ASSERT_EQ(dists.type(), CV_32S) << backend.name;
		for(int i=0; i<queries.rows; ++i)
		{
			EXPECT_EQ(indices.at<int>(i, 0), i) << backend.name << " query=" << i;
			EXPECT_EQ(dists.at<int>(i, 0), 0) << backend.name << " query=" << i;
		}
	}
}

TEST(FlannIndexTest, RadiusSearchKeepsTheNearestMaxNeighbors)
{
	const cv::Mat cloud = makeCloud(2000, 2, 35);
	const cv::Mat queries = makeCloud(50, 2, 36);
	const float radius = 10.0f;
	const int maxNeighbors = 3;

	std::vector<std::vector<size_t> > reference;
	for(const Backend & backend: EXACT_BACKENDS)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);

		std::vector<std::vector<size_t> > indices;
		std::vector<std::vector<float> > dists;
		index.radiusSearch(queries, indices, dists, radius, maxNeighbors, 32, 0.0f, true);
		ASSERT_EQ(indices.size(), (size_t)queries.rows) << backend.name;

		for(size_t i=0; i<indices.size(); ++i)
		{
			EXPECT_LE(indices[i].size(), (size_t)maxNeighbors) << backend.name << " query=" << i;
			ASSERT_EQ(indices[i].size(), dists[i].size()) << backend.name;
			// sorted=true, so they come back closest first
			for(size_t j=1; j<dists[i].size(); ++j)
			{
				EXPECT_LE(dists[i][j-1], dists[i][j]) << backend.name << " query=" << i;
			}
		}

		if(reference.empty())
		{
			reference = indices;
			size_t truncated = 0;
			for(const auto & neighbors: reference)
			{
				truncated += neighbors.size() == (size_t)maxNeighbors ? 1 : 0;
			}
			ASSERT_GT(truncated, 0u) << "the radius is too small to truncate anything";
			continue;
		}
		for(size_t i=0; i<indices.size(); ++i)
		{
			EXPECT_EQ(indices[i], reference[i]) << backend.name << " query=" << i;
		}
	}
}

TEST(FlannIndexTest, ReleasedIndexIsEmptyAndSearchable)
{
	const cv::Mat cloud = makeCloud(100, 3, 37);

	for(const Backend & backend: EXACT_BACKENDS)
	{
		FlannIndex index;
		EXPECT_FALSE(index.isBuilt()) << backend.name;
		EXPECT_EQ(index.indexedFeatures(), 0u) << backend.name;
		EXPECT_EQ(index.memoryUsed(), 0u) << backend.name;

		index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);
		EXPECT_TRUE(index.isBuilt()) << backend.name;
		EXPECT_GT(index.memoryUsed(), 0u) << backend.name;

		index.release();
		EXPECT_FALSE(index.isBuilt()) << backend.name;
		EXPECT_EQ(index.indexedFeatures(), 0u) << backend.name;

		// Searching an index that is not built is an error, not a crash.
		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(cloud.row(0), indices, dists, 1);
		EXPECT_TRUE(indices.empty()) << backend.name;

		std::vector<std::vector<size_t> > radiusIndices;
		std::vector<std::vector<float> > radiusDists;
		index.radiusSearch(cloud.row(0), radiusIndices, radiusDists, 1.0f);
		EXPECT_TRUE(radiusIndices.empty()) << backend.name;
	}
}

TEST(FlannIndexTest, AsksForMoreNeighborsThanIndexed)
{
	const cv::Mat cloud = makeCloud(3, 3, 38);
	const int knn = 5;

	for(const Backend & backend: EXACT_BACKENDS)
	{
		FlannIndex index;
		index.buildIndex(backend.algorithm, cloud, false, backend.rebalancingFactor);

		cv::Mat indices;
		cv::Mat dists;
		index.knnSearch(cloud.row(0), indices, dists, knn);
		ASSERT_EQ(indices.cols, knn) << backend.name;

		// The neighbors that couldn't be found are marked
		for(int j=0; j<knn; ++j)
		{
			if(j < cloud.rows)
			{
				EXPECT_GE(indices.at<int>(0, j), 0) << backend.name << " n=" << j;
			}
			else
			{
				EXPECT_EQ(indices.at<int>(0, j), -1) << backend.name << " n=" << j;
			}
		}
	}
}
