#ifndef RTABMAP_CORELIB_TEST_FLANNINDEXBACKENDS_H_
#define RTABMAP_CORELIB_TEST_FLANNINDEXBACKENDS_H_

#include <gtest/gtest.h>
#include <rtabmap/core/FlannIndex.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <iostream>

using namespace rtabmap;

// Everything here is inline rather than static: the two test files including
// this header each use a subset of it, and unused functions with internal
// linkage warn.
namespace {

struct Backend
{
	const char * name;
	FlannIndex::flann_algorithm_t algorithm;
};

// Every algorithm that indexes float features. The exhaustive search comes
// first: it is the reference the others are compared to, both for the neighbors
// found and for the time taken.
const Backend FLOAT_BACKENDS[] = {
	{"linear    exhaustive                ", FlannIndex::FLANN_INDEX_LINEAR},
	{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
	{"rtflann   kd-tree single            ", FlannIndex::FLANN_INDEX_KDTREE_SINGLE},
	{"nanoflann kd-tree single            ", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE},
	{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE_INCREMENTAL},
};

// Those of them that search exactly, so are expected to return the very same
// neighbors. The randomized kd-tree is left out: it trades recall for speed.
const Backend EXACT_BACKENDS[] = {
	{"linear    exhaustive                ", FlannIndex::FLANN_INDEX_LINEAR},
	{"rtflann   kd-tree single            ", FlannIndex::FLANN_INDEX_KDTREE_SINGLE},
	{"nanoflann kd-tree single            ", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE},
	{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE_INCREMENTAL},
};

// Binary features only have the Hamming based algorithms: nanoflann has no
// Hamming metric, and a kd-tree splitting on bits doesn't work, which is what
// LSH is for.
const Backend BINARY_BACKENDS[] = {
	{"linear    exhaustive (hamming)      ", FlannIndex::FLANN_INDEX_LINEAR},
	{"rtflann   LSH                       ", FlannIndex::FLANN_INDEX_LSH},
};

// Those accepting points after the index is built. The static nanoflann tree
// rejects them by design, and so is not part of that comparison.
const Backend INCREMENTAL_BACKENDS[] = {
	{"linear    exhaustive                ", FlannIndex::FLANN_INDEX_LINEAR},
	{"rtflann   kd-tree (4 randomized)    ", FlannIndex::FLANN_INDEX_KDTREE},
	{"rtflann   kd-tree single            ", FlannIndex::FLANN_INDEX_KDTREE_SINGLE},
	{"nanoflann kd-tree single incremental", FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE_INCREMENTAL},
};

// Note when reading the times of the first configuration of a comparison: it
// pays the warm up of the allocator, which the runs after it reuse. It shows on
// the nanoflann trees, which allocate a node per point, and the smaller the
// index the more it weighs.
//
// The rebalancing factor doesn't change how a search is done, only how the
// index is built and grown: with a factor over 1 the rtflann backend keeps one
// cv::Mat header per indexed point so that it can rebuild itself later, and the
// nanoflann incremental tree turns it into the fraction of removed points it
// tolerates. Both are compared where that matters, building and inserting.
const float REBALANCING_FACTORS[] = {1.0f, 2.0f};

// Synthetic point cloud, uniformly distributed in a 100 m box. The seeds are
// fixed so that a comparison that passes keeps passing.
inline cv::Mat makeCloud(int points, int dim, uint64_t seed)
{
	cv::RNG rng(seed);
	cv::Mat cloud(points, dim, CV_32FC1);
	rng.fill(cloud, cv::RNG::UNIFORM, 0.0f, 100.0f);
	return cloud;
}

// Float descriptors drawn around a limited number of centers. Uniformly
// distributed descriptors would be the worst case there is for any of the
// approximate algorithms, and nothing like the clustered distribution real
// descriptors have.
inline cv::Mat makeDescriptors(int count, int dim, int clusters, uint64_t seed)
{
	cv::RNG rng(seed);
	cv::Mat centers(clusters, dim, CV_32FC1);
	rng.fill(centers, cv::RNG::UNIFORM, 0.0f, 255.0f);

	// Filled in one go: a fill() per row costs seconds on a million descriptors.
	cv::Mat descriptors(count, dim, CV_32FC1);
	rng.fill(descriptors, cv::RNG::NORMAL, 0.0f, 12.0f);
	for(int i=0; i<count; ++i)
	{
		descriptors.row(i) += centers.row(rng.uniform(0, clusters));
	}
	return descriptors;
}

// Binary descriptors around a limited number of centers, a few bits flipped.
inline cv::Mat makeBinaryDescriptors(int count, int bytes, int clusters, uint64_t seed)
{
	cv::RNG rng(seed);
	cv::Mat centers(clusters, bytes, CV_8UC1);
	rng.fill(centers, cv::RNG::UNIFORM, 0, 256);

	cv::Mat descriptors(count, bytes, CV_8UC1);
	for(int i=0; i<count; ++i)
	{
		centers.row(rng.uniform(0, clusters)).copyTo(descriptors.row(i));
		for(int bit=0; bit<bytes; ++bit) // ~1 bit flipped per byte
		{
			descriptors.at<unsigned char>(i, rng.uniform(0, bytes)) ^= (1 << rng.uniform(0, 8));
		}
	}
	return descriptors;
}

// Queries taken from the indexed descriptors and perturbed, the way the same
// feature observed twice would be. Their nearest neighbor is unambiguous, which
// is what makes the recall of the approximate algorithms meaningful.
inline cv::Mat perturbedQueries(const cv::Mat & descriptors, int count, uint64_t seed)
{
	cv::RNG rng(seed);
	cv::Mat queries(count, descriptors.cols, descriptors.type());
	for(int i=0; i<count; ++i)
	{
		descriptors.row(rng.uniform(0, descriptors.rows)).copyTo(queries.row(i));
		if(descriptors.type() == CV_32FC1)
		{
			cv::Mat noise(1, descriptors.cols, CV_32FC1);
			rng.fill(noise, cv::RNG::NORMAL, 0.0f, 2.0f);
			queries.row(i) += noise;
		}
		else
		{
			for(int bit=0; bit<descriptors.cols/8; ++bit)
			{
				queries.at<unsigned char>(i, rng.uniform(0, descriptors.cols)) ^= (1 << rng.uniform(0, 8));
			}
		}
	}
	return queries;
}

struct Result
{
	double buildTime;
	double knnTime;
	double radiusTime; // negative when no radius search was done
	size_t memory;
	cv::Mat indices;
};

inline Result run(
		FlannIndex::flann_algorithm_t algorithm,
		const cv::Mat & data,
		const cv::Mat & queries,
		int knn,
		float radius,
		float rebalancingFactor)
{
	FlannIndex index;
	Result result;
	result.radiusTime = -1.0;
	cv::Mat dists;

	UTimer timer;
	index.buildIndex(algorithm, data, false, rebalancingFactor);
	result.buildTime = timer.ticks();

	index.knnSearch(queries, result.indices, dists, knn);
	result.knnTime = timer.ticks();

	if(radius > 0.0f)
	{
		std::vector<std::vector<size_t> > radiusIndices;
		std::vector<std::vector<float> > radiusDists;
		index.radiusSearch(queries, radiusIndices, radiusDists, radius);
		result.radiusTime = timer.ticks();
	}

	result.memory = index.memoryUsed();
	return result;
}

// Fraction of the neighbors that are the ones the exhaustive search finds.
inline float recall(const cv::Mat & indices, const cv::Mat & reference)
{
	UASSERT(indices.size() == reference.size());
	int found = 0;
	for(int i=0; i<indices.rows; ++i)
	{
		for(int j=0; j<indices.cols; ++j)
		{
			if(indices.at<int>(i, j) == reference.at<int>(i, j))
			{
				++found;
			}
		}
	}
	return float(found)/float(indices.total());
}

// Note on the memory column: the rtflann indexes point into the features they
// were built with, while the nanoflann ones copy them into their own storage,
// which their memoryUsed() includes.
inline void report(const char * name, const Result & result, float recallRatio)
{
	std::cout << "[          ]   " << name
			  << " build=" << uFormat("%7.1f", result.buildTime*1000.0) << " ms"
			  << " knn=" << uFormat("%7.1f", result.knnTime*1000.0) << " ms";
	if(result.radiusTime >= 0.0)
	{
		std::cout << " radius=" << uFormat("%7.1f", result.radiusTime*1000.0) << " ms";
	}
	std::cout << " memory=" << uFormat("%6d", (int)(result.memory/1024)) << " KB"
			  << " recall=" << uFormat("%5.1f", recallRatio*100.0f) << " %"
			  << std::endl;
}

// Run every backend of a list on the same data, the first one being the
// reference the others are compared to.
inline void compare(
		const Backend * backends,
		size_t count,
		const cv::Mat & data,
		const cv::Mat & queries,
		int knn,
		float radius = 0.0f,
		float rebalancingFactor = 1.0f)
{
	cv::Mat reference;
	for(size_t i=0; i<count; ++i)
	{
		const Result result = run(backends[i].algorithm, data, queries, knn, radius, rebalancingFactor);
		ASSERT_EQ(result.indices.rows, queries.rows) << backends[i].name;
		if(reference.empty())
		{
			reference = result.indices;
		}
		report(backends[i].name, result, recall(result.indices, reference));
	}
}

} // namespace

#endif /* RTABMAP_CORELIB_TEST_FLANNINDEXBACKENDS_H_ */
