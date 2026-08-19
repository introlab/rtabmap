// Comparison of the dense and the sparse (Parameters::kBayesSparsePrediction())
// multiplication of the prediction matrix with the last posterior, which is where
// BayesFilter::computePosterior() spends nearly all of its time on a large map.
//
// Its own executable, run by ctest under the "performance" label, so that its
// seconds of benchmarking stay out of the unit test shards:
//   ctest -L performance    to run them
//   ctest -LE performance   to skip them
//   bin/test_bayesfilter_perf --gtest_filter=*Growing*
//
// The times are reported rather than asserted on, as they depend on the machine.
// What is asserted is that both multiplications give the same posterior, so that
// the numbers below compare two ways of computing the same thing.
#include <gtest/gtest.h>
#include <rtabmap/core/BayesFilter.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UTimer.h>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <vector>

using namespace rtabmap;

namespace {

// Sizes of the maps compared. The dense prediction matrix is n x n floats, so the
// largest one below already allocates 244 MB.
static const int MAP_SIZES[] = {1000, 4000, 8000};

// How many values of the prediction model decides how deep in the graph a column of
// the prediction matrix reaches, and thus how many values it holds. The default model
// has 18, so it stores neighbors up to 17 links away, with probabilities down to
// 6.9e-23. Truncating it to 8 keeps every value above 1e-4 and stops there. The
// difference is given to the loop closure probability so that the values still sum to
// slightly more than 1: below 1, normalize() spreads what is missing over every zero
// of a column and the matrix is no longer sparse at all.
static const char PREDICTION_DEFAULT[] =
		"0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 "
		"1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23";
static const char PREDICTION_TRUNCATED[] =
		"0.1 0.36003 0.30 0.16 0.062 0.0151 0.00255 0.000324";

// A chain of signatures linked by odometry, with a global loop closure every
// loopEvery nodes back to the node loopSpan earlier. The loop closures matter here:
// Memory::getNeighborsId() follows them, so each one is a shortcut that widens the
// neighborhood a column of the prediction matrix holds. They are what decides how
// sparse the matrix is, so they are a knob of these benchmarks rather than a detail.
class SyntheticMap
{
public:
	// Built by a mapping session, then turned to localization mode unless asked
	// otherwise: the graph is then fixed, which is what the sparse prediction is built
	// for, and what the dense one is compared against here.
	SyntheticMap(int nodes, int loopEvery, int loopSpan, bool localization = true)
	{
		ParametersMap params;
		// No features extracted: the graph is what the prediction matrix is built from.
		params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
		// Only the latest signature stays in STM, the rest are WM nodes the filter uses.
		params.insert(ParametersPair(Parameters::kMemSTMSize(), "1"));
		params.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
		params.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
		memory_ = new Memory(params);

		const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		const cv::Mat information = cv::Mat::eye(6, 6, CV_64FC1);

		UTimer timer;
		std::vector<int> ids;
		ids.reserve(nodes);
		for(int i=0; i<nodes; ++i)
		{
			SensorData data(image);
			UASSERT(memory_->update(data, Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
			ids.push_back(memory_->getLastSignatureId());
			if(loopEvery > 0 && i >= loopSpan && i % loopEvery == 0)
			{
				UASSERT(memory_->addLink(Link(
						ids.back(),
						ids[ids.size()-1-loopSpan],
						Link::kGlobalClosure,
						Transform::getIdentity(),
						information)));
				++loopClosures_;
			}
		}
		buildTime_ = timer.ticks();

		if(localization)
		{
			ParametersMap localizationParams;
			localizationParams.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
			memory_->parseParameters(localizationParams);
			UASSERT(!memory_->isIncremental());
		}
	}

	~SyntheticMap()
	{
		delete memory_;
	}

	const Memory * memory() const {return memory_;}
	int loopClosures() const {return loopClosures_;}
	double buildTime() const {return buildTime_;}

	// What Rtabmap passes to the filter: the virtual place (new location hypothesis)
	// followed by the WM locations that are not in STM.
	std::vector<int> bayesIds() const
	{
		std::vector<int> ids;
		ids.push_back(Memory::kIdVirtual);
		const std::set<int> & stm = memory_->getStMem();
		for(std::map<int, double>::const_iterator iter=memory_->getWorkingMem().begin();
			iter!=memory_->getWorkingMem().end();
			++iter)
		{
			if(iter->first > 0 && stm.find(iter->first) == stm.end())
			{
				ids.push_back(iter->first);
			}
		}
		return ids;
	}

	// Uniform, which is the worst case for the sparse multiplication: no location is
	// ruled out, so no column of the prediction can be skipped whole.
	std::map<int, float> uniformLikelihood(const std::vector<int> & ids) const
	{
		std::map<int, float> likelihood;
		for(size_t i=0; i<ids.size(); ++i)
		{
			likelihood.insert(std::make_pair(ids[i], 1.0f));
		}
		return likelihood;
	}

private:
	Memory * memory_ = nullptr;
	int loopClosures_ = 0;
	double buildTime_ = 0.0;
};

struct Result
{
	double firstIteration = 0.0;   // includes generating the prediction matrix, and its sparse view
	double steadyState = 0.0;      // mean of the following iterations, the matrix being unchanged
	unsigned long memoryUsed = 0;
	std::map<int, float> posterior;
};

Result run(const SyntheticMap & map, const char * predictionLC, bool sparse, int iterations)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), predictionLC));
	params.insert(ParametersPair(Parameters::kBayesSparsePrediction(), sparse?"true":"false"));
	BayesFilter filter(params);

	const std::vector<int> ids = map.bayesIds();
	const std::map<int, float> likelihood = map.uniformLikelihood(ids);

	Result result;
	UTimer timer;
	filter.computePosterior(map.memory(), likelihood);
	result.firstIteration = timer.ticks();

	double total = 0.0;
	for(int i=1; i<iterations; ++i)
	{
		timer.restart();
		filter.computePosterior(map.memory(), likelihood);
		total += timer.ticks();
	}
	result.steadyState = iterations > 1 ? total/double(iterations-1) : result.firstIteration;
	result.memoryUsed = filter.getMemoryUsed();
	result.posterior = filter.getPosterior();
	return result;
}

// The two multiplications sum the products of a row in a different order, so the
// posteriors differ by the rounding of a few thousand float additions rather than
// being bit identical. Reported relative to the largest probability, which is the
// scale Rtabmap compares hypotheses at.
double maxPosteriorDifference(const std::map<int, float> & a, const std::map<int, float> & b)
{
	if(a.size() != b.size())
	{
		return 1.0;
	}
	double maxDiff = 0.0;
	double maxValue = 0.0;
	for(std::map<int, float>::const_iterator iter=a.begin(); iter!=a.end(); ++iter)
	{
		std::map<int, float>::const_iterator jter = b.find(iter->first);
		if(jter == b.end())
		{
			return 1.0;
		}
		maxDiff = std::max(maxDiff, std::fabs(double(iter->second) - double(jter->second)));
		maxValue = std::max(maxValue, std::fabs(double(iter->second)));
	}
	return maxValue > 0.0 ? maxDiff/maxValue : maxDiff;
}

void report(const char * name, const Result & result)
{
	printf("[          ]   %-7s first iteration %9.1f ms, steady state %8.2f ms, filter memory %7.1f MB\n",
			name, result.firstIteration*1000.0, result.steadyState*1000.0, result.memoryUsed/1048576.0);
}

void compare(const SyntheticMap & map, const char * predictionLC, int iterations)
{
	const size_t size = map.bayesIds().size();
	const Result dense = run(map, predictionLC, false, iterations);
	const Result sparse = run(map, predictionLC, true, iterations);

	report("dense", dense);
	report("sparse", sparse);
	printf("[          ]   steady state speedup x%.1f, dense/sparse memory x%.2f, "
		   "relative posterior difference %.1e\n",
			sparse.steadyState > 0.0 ? dense.steadyState/sparse.steadyState : 0.0,
			sparse.memoryUsed > 0 ? double(dense.memoryUsed)/double(sparse.memoryUsed) : 0.0,
			maxPosteriorDifference(dense.posterior, sparse.posterior));

	EXPECT_EQ(dense.posterior.size(), size);
	// Same probabilities up to the rounding of the sums, which the iterations compound:
	// the sums are of a few thousand floats spanning the whole range of the model, down
	// to 6.9e-23 for the default one, and each iteration starts from the previous
	// posterior. Which location comes out highest is not compared: the likelihood is
	// uniform here, so the visited locations are all within rounding of each other and
	// the highest is whichever the rounding favors.
	EXPECT_LT(maxPosteriorDifference(dense.posterior, sparse.posterior), 1e-3);
}

} // namespace

// Both modes on the same graph, over maps of growing size. The dense multiplication
// reads the whole n x n matrix on every iteration, so its cost grows with the square
// of the number of nodes, while the sparse one grows with the number of values the
// graph actually puts in the matrix.
TEST(BayesFilterPerfTest, DenseVsSparsePredictionOnGrowingMaps)
{
	// The steady state of the smaller maps is a fraction of a millisecond, so it is
	// averaged over enough iterations that one hiccup doesn't carry the mean.
	const int iterations = 30;

	for(size_t s=0; s<sizeof(MAP_SIZES)/sizeof(int); ++s)
	{
		const int nodes = MAP_SIZES[s];
		// A moderately connected graph, so that this measures the effect of the size.
		// How much the connectivity itself matters is measured by the test below.
		SyntheticMap map(nodes, 100, 500);
		const size_t size = map.bayesIds().size();

		std::cout << "[          ] " << nodes << " nodes (" << size << " ids with the virtual place), "
				  << map.loopClosures() << " loop closures, graph built in " << map.buildTime()
				  << "s, dense matrix = " << (size*size*sizeof(float))/1048576 << " MB" << std::endl;

		compare(map, PREDICTION_DEFAULT, iterations);
	}
}

// How sparse the prediction matrix is, and so how much the sparse multiplication can
// win, is decided by two things: how connected the graph is, every loop closure being
// a shortcut that getNeighborsId() follows, and how deep the prediction model reaches.
// The default 18 values model stores neighbors up to 17 links away with probabilities
// down to 6.9e-23, which are numerically irrelevant next to the 0.36 of the first
// level but fill most of the matrix.
TEST(BayesFilterPerfTest, SparsityAgainstGraphConnectivityAndModelDepth)
{
	const int nodes = 4000;
	const int iterations = 30;

	struct Connectivity { const char * name; int loopEvery; int loopSpan; };
	const Connectivity connectivities[] = {
		{"chain only, no loop closure", 0, 0},
		{"a loop closure every 100 nodes, spanning 500", 100, 500},
		{"a loop closure every 20 nodes, spanning 200", 20, 200},
	};

	for(size_t c=0; c<sizeof(connectivities)/sizeof(Connectivity); ++c)
	{
		SyntheticMap map(nodes, connectivities[c].loopEvery, connectivities[c].loopSpan);
		std::cout << "[          ] " << nodes << " nodes, " << connectivities[c].name
				  << " (" << map.loopClosures() << " loop closures)" << std::endl;

		std::cout << "[          ]  18 values model (default, depth 17):" << std::endl;
		compare(map, PREDICTION_DEFAULT, iterations);
		std::cout << "[          ]   8 values model (depth 7, every value above 1e-4):" << std::endl;
		compare(map, PREDICTION_TRUNCATED, iterations);
	}
}

// The size of a real large map, over which a localization session iterates without
// ever changing the graph: the prediction matrix is generated once and every
// following iteration reuses it, so the sparse view is built once too. This is the
// case the sparse multiplication is for.
//
// The dense matrix alone is a gigabyte at that size, and takes seconds to generate;
// exclude this one with
//   bin/test_bayesfilter_perf --gtest_filter=-*LargeMap*
TEST(BayesFilterPerfTest, DenseVsSparsePredictionOnALargeMap)
{
	const int nodes = 16384;
	const int iterations = 10;

	SyntheticMap map(nodes, 100, 500);
	const size_t size = map.bayesIds().size();
	std::cout << "[          ] " << nodes << " nodes (" << size << " ids with the virtual place), "
			  << map.loopClosures() << " loop closures, graph built in " << map.buildTime()
			  << "s, dense matrix = " << (size*size*sizeof(float))/1048576 << " MB" << std::endl;

	std::cout << "[          ]  18 values model (default, depth 17):" << std::endl;
	compare(map, PREDICTION_DEFAULT, iterations);
	std::cout << "[          ]   8 values model (depth 7, every value above 1e-4):" << std::endl;
	compare(map, PREDICTION_TRUNCATED, iterations);
}

// While mapping, the graph changes on every node, so the matrix is kept for
// updatePrediction() to carry its unchanged columns over and the sparse prediction is
// taken from it rather than built instead of it. Same multiplication, no memory saved.
TEST(BayesFilterPerfTest, DenseVsSparsePredictionWhileMapping)
{
	const int nodes = 4000;
	const int iterations = 30;

	SyntheticMap map(nodes, 100, 500, false /*stay in mapping mode*/);
	const size_t size = map.bayesIds().size();
	std::cout << "[          ] " << nodes << " nodes, mapping mode (the matrix is kept), "
			  << map.loopClosures() << " loop closures, dense matrix = "
			  << (size*size*sizeof(float))/1048576 << " MB" << std::endl;

	compare(map, PREDICTION_DEFAULT, iterations);
}
