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
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
#include <algorithm>
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
		"0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1e-06 4.8e-08 "
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

// A real map's graph, read from the g2o file it was exported to. What makes it worth
// measuring against the synthetic graphs above is its link types: the file holds mostly
// merged neighbor links, which cost a margin like an ordinary neighbor, and only a few
// hundred closures that Memory::getNeighborsId() follows without spending one. How many
// of those there are is what decides how much of the map a column of the prediction
// holds, so a real graph's answer is not a synthetic one's.
//
// The graph is rebuilt in a Memory rather than optimized: Memory::update() creates a
// signature per pose and links each to the previous one, so the links the file does not
// have are removed and the ones it has are added with their own type.
// A real map's graph, read from the g2o file it was exported to. What makes it worth
// measuring against the synthetic graphs above is its link types: how many links
// Memory::getNeighborsId() follows without spending a margin is what decides how much of
// the map a column of the prediction holds, and a real graph's answer is not a synthetic
// one's. A graph that went through the reduction holds mostly merged neighbor links,
// which cost a margin like an ordinary neighbor; one that did not holds none.
struct RealGraph
{
	std::vector<int> ids;                              // the locations, in the order they were created
	std::map<int, Transform> poses;
	// The links between two locations, as indices into ids, each on the later of the two:
	// that is the one a mapping session adds them on.
	std::vector<std::vector<std::pair<size_t, Link::Type> > > linksTo;
	std::map<int, int> byType;
	int skippedLinks = 0;
	double loadTime = 0.0;
};

bool loadRealGraph(const std::string & path, RealGraph & graph)
{
	UTimer timer;
	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	if(!graph::importPoses(path, 4 /*g2o*/, poses, &links))
	{
		return false;
	}
	graph.loadTime = timer.ticks();
	graph.poses = poses;

	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first > 0)
		{
			graph.ids.push_back(iter->first);
		}
	}
	std::sort(graph.ids.begin(), graph.ids.end());
	std::map<int, size_t> indexOf;
	for(size_t i=0; i<graph.ids.size(); ++i)
	{
		indexOf.insert(std::make_pair(graph.ids[i], i));
	}

	// One entry per pair of locations. Links on a single location (a prior, gravity) and
	// landmark observations are left out: getNeighborsId() doesn't walk the first, and the
	// second would need the landmark index of a memory that mapped them.
	graph.linksTo.resize(graph.ids.size());
	std::set<std::pair<size_t, size_t> > seen;
	for(std::multimap<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		const Link & link = iter->second;
		if(link.from() == link.to() || link.from() < 0 || link.to() < 0)
		{
			++graph.skippedLinks;
			continue;
		}
		const size_t a = indexOf.at(link.from()), b = indexOf.at(link.to());
		if(!seen.insert(std::make_pair(std::min(a,b), std::max(a,b))).second)
		{
			continue;
		}
		graph.linksTo[std::max(a,b)].push_back(std::make_pair(std::min(a,b), link.type()));
		++graph.byType[link.type()];
	}
	return true;
}

// Adds one location and the links the graph has between it and the ones already there.
// Memory::update() links each new signature to the previous one as a kNeighbor, so that
// one is dropped when the graph does not have it, or has it with another type.
void addRealNode(
		Memory * memory,
		const RealGraph & graph,
		size_t index,
		std::vector<int> & newIds,
		int * removedLinks = 0)
{
	static const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	static const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0001;
	static const cv::Mat information = cv::Mat::eye(6, 6, CV_64FC1);

	SensorData data(image);
	UASSERT(memory->update(data, graph.poses.at(graph.ids[index]), covariance));
	newIds.push_back(memory->getLastSignatureId());

	bool previousLinked = false;
	for(size_t i=0; i<graph.linksTo[index].size(); ++i)
	{
		const size_t other = graph.linksTo[index][i].first;
		const Link::Type type = graph.linksTo[index][i].second;
		if(other == index-1 && type == Link::kNeighbor)
		{
			previousLinked = true;   // update() already made this one
			continue;
		}
		memory->addLink(Link(newIds[index], newIds[other], type,
				Transform::getIdentity(), information));
	}
	if(index > 0 && !previousLinked)
	{
		// Either the graph has no link between these two, or it has one of another type
		// which the loop above has just added.
		memory->removeLink(newIds[index-1], newIds[index]);
		if(removedLinks)
		{
			++(*removedLinks);
		}
	}
}

Memory * newRealMemory()
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
	params.insert(ParametersPair(Parameters::kMemSTMSize(), "1"));
	params.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
	params.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	return new Memory(params);
}

// What Rtabmap passes to the filter: the virtual place followed by the locations of the
// working memory that are not in the short term memory.
std::vector<int> bayesIdsOf(const Memory * memory)
{
	std::vector<int> ids;
	ids.push_back(Memory::kIdVirtual);
	const std::set<int> & stm = memory->getStMem();
	for(std::map<int, double>::const_iterator iter=memory->getWorkingMem().begin();
		iter!=memory->getWorkingMem().end();
		++iter)
	{
		if(iter->first > 0 && stm.find(iter->first) == stm.end())
		{
			ids.push_back(iter->first);
		}
	}
	return ids;
}

std::map<int, float> uniformLikelihoodOf(const std::vector<int> & ids)
{
	std::map<int, float> likelihood;
	for(size_t i=0; i<ids.size(); ++i)
	{
		likelihood.insert(std::make_pair(ids[i], 1.0f));
	}
	return likelihood;
}

const char * linkTypeName(int type)
{
	switch(type)
	{
	case Link::kNeighbor:           return "kNeighbor";
	case Link::kGlobalClosure:      return "kGlobalClosure";
	case Link::kLocalSpaceClosure:  return "kLocalSpaceClosure";
	case Link::kLocalTimeClosure:   return "kLocalTimeClosure";
	case Link::kUserClosure:        return "kUserClosure";
	case Link::kVirtualClosure:     return "kVirtualClosure";
	case Link::kNeighborMerged:     return "kNeighborMerged";
	case Link::kPosePrior:          return "kPosePrior";
	case Link::kLandmark:           return "kLandmark";
	case Link::kGravity:            return "kGravity";
	default:                        return "other";
	}
}

void printGraph(const std::string & path, const RealGraph & graph, int removedLinks)
{
	std::cout << "[          ] " << path << ": read in " << graph.loadTime << "s, "
			  << graph.ids.size() << " locations, links:";
	for(std::map<int,int>::const_iterator iter=graph.byType.begin(); iter!=graph.byType.end(); ++iter)
	{
		std::cout << " " << linkTypeName(iter->first) << "=" << iter->second;
	}
	std::cout << " (" << graph.skippedLinks << " on a single location or on a landmark not rebuilt, "
			  << removedLinks << " gaps in the chain)" << std::endl;
}
// The posterior as a map, which the filter no longer builds: it holds the locations and
// their probabilities as two vectors, and a test reads them more easily as a map.
static std::map<int, float> posteriorOf(const BayesFilter & filter)
{
	const std::vector<int> & ids = filter.getPosteriorIds();
	const std::vector<float> & values = filter.getPosteriorValues();
	std::map<int, float> posterior;
	for(size_t i = 0; i < ids.size(); ++i)
	{
		posterior.insert(posterior.end(), std::make_pair(ids[i], values[i]));
	}
	return posterior;
}

struct Result
{
	double firstIteration = 0.0;   // includes generating the prediction, sparse or dense
	double steadyState = 0.0;      // fastest of the following iterations, the prediction being unchanged
	unsigned long memoryUsed = 0;
	std::map<int, float> posterior;
};

Result run(const Memory * memory,
		const std::vector<int> & ids,
		const std::map<int, float> & likelihood,
		const char * predictionLC,
		bool sparse,
		int iterations)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), predictionLC));
	params.insert(ParametersPair(Parameters::kBayesSparsePrediction(), sparse?"true":"false"));
	BayesFilter filter(params);

	Result result;
	UTimer timer;
	filter.computePosterior(memory, likelihood);
	result.firstIteration = timer.ticks();

	// A few untimed iterations to let the caches and the processor clock settle before
	// measuring. A fixed count, the same whatever the mode: the filter is recursive, so
	// how many iterations it has run decides where its posterior is, and warming up for a
	// fixed duration instead would run hundreds of them in the fast mode against one in
	// the slow one and leave the two posteriors nowhere near each other.
	for(int i=0; i<3; ++i)
	{
		filter.computePosterior(memory, likelihood);
	}

	// The fastest iteration rather than the mean or the median of them. Everything that
	// makes an iteration slower than its own best is the machine rather than the code
	// being measured, and the dense multiplication reads the whole prediction matrix from
	// memory, which makes it sensitive to whatever else is using that memory. The fastest
	// is the one measurement of the run that is the least of it, so it is the one that
	// compares between runs and between machines.
	double best = 0.0;
	for(int i=1; i<iterations; ++i)
	{
		timer.restart();
		filter.computePosterior(memory, likelihood);
		const double elapsed = timer.ticks();
		if(best == 0.0 || elapsed < best)
		{
			best = elapsed;
		}
	}
	result.steadyState = best > 0.0 ? best : result.firstIteration;
	result.memoryUsed = filter.getMemoryUsed();
	result.posterior = posteriorOf(filter);
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
	printf("[          ]   %-7s first iteration %9.1f ms, fastest iteration %8.2f ms, filter memory %7.1f MB\n",
			name, result.firstIteration*1000.0, result.steadyState*1000.0, result.memoryUsed/1048576.0);
}

// sparseFirst measures the sparse mode before the dense one. It matters on a large map:
// the dense mode allocates the prediction matrix, and running it first leaves the
// allocator holding hundreds of megabytes, which the sparse measurement that follows then
// pays for. Measuring the two in separate processes is the only way to have both clean;
// within one, the cheaper mode is the one to protect.
void compare(const Memory * memory,
		const std::vector<int> & ids,
		const std::map<int, float> & likelihood,
		const char * predictionLC,
		int iterations,
		bool sparseFirst = false)
{
	const size_t size = ids.size();
	Result dense, sparse;
	if(sparseFirst)
	{
		sparse = run(memory, ids, likelihood, predictionLC, true, iterations);
		dense = run(memory, ids, likelihood, predictionLC, false, iterations);
	}
	else
	{
		dense = run(memory, ids, likelihood, predictionLC, false, iterations);
		sparse = run(memory, ids, likelihood, predictionLC, true, iterations);
	}

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
	// The steady state of the smaller maps is a fraction of a millisecond, so enough
	// iterations for the fastest of them to be a stable number.
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

		const std::vector<int> ids = map.bayesIds();
		compare(map.memory(), ids, map.uniformLikelihood(ids), PREDICTION_DEFAULT, iterations);
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

		const std::vector<int> ids = map.bayesIds();
		const std::map<int, float> likelihood = map.uniformLikelihood(ids);
		std::cout << "[          ]  18 values model (default, depth 17):" << std::endl;
		compare(map.memory(), ids, likelihood, PREDICTION_DEFAULT, iterations);
		std::cout << "[          ]   8 values model (depth 7, every value above 1e-4):" << std::endl;
		compare(map.memory(), ids, likelihood, PREDICTION_TRUNCATED, iterations);
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

	const std::vector<int> ids = map.bayesIds();
	const std::map<int, float> likelihood = map.uniformLikelihood(ids);
	std::cout << "[          ]  18 values model (default, depth 17):" << std::endl;
	compare(map.memory(), ids, likelihood, PREDICTION_DEFAULT, iterations);
	std::cout << "[          ]   8 values model (depth 7, every value above 1e-4):" << std::endl;
	compare(map.memory(), ids, likelihood, PREDICTION_TRUNCATED, iterations);
}

// Mapping mode, over a graph that has stopped growing: the matrix is kept, because
// updatePrediction() needs it to carry its unchanged columns over whenever the graph does
// grow, and the sparse form is taken from it rather than built instead of it. It is worth
// taking here because the prediction outlasts an iteration, which is what
// DenseVsSparsePredictionWhileMappingARealSession does not have: there a location is added
// on every iteration and the sparse form is never built at all.
TEST(BayesFilterPerfTest, DenseVsSparsePredictionWhileMapping)
{
	const int nodes = 4000;
	const int iterations = 30;

	SyntheticMap map(nodes, 100, 500, false /*stay in mapping mode*/);
	const size_t size = map.bayesIds().size();
	std::cout << "[          ] " << nodes << " nodes, mapping mode (the matrix is kept), "
			  << map.loopClosures() << " loop closures, dense matrix = "
			  << (size*size*sizeof(float))/1048576 << " MB" << std::endl;

	const std::vector<int> ids = map.bayesIds();
	compare(map.memory(), ids, map.uniformLikelihood(ids), PREDICTION_DEFAULT, iterations);
}

// The graphs of real maps, against the synthetic ones above, in localization mode where
// the graph is fixed. Two of them: one that went through the graph reduction, whose merged
// neighbor links cost a margin like ordinary neighbors, and one that did not.
//
// Needs the same ~1 GB as the largest synthetic map for the dense prediction, and reads
// the graphs from data/tests. Exclude with
//   bin/test_bayesfilter_perf --gtest_filter=-*RealMap*
TEST(BayesFilterPerfTest, DenseVsSparsePredictionOnRealMaps)
{
	const int iterations = 10;
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		GTEST_SKIP() << "g2o optimizer not built in, needed to read the graphs";
	}

	const char * files[] = {"large_reduced_graph.g2o", "large_mapping_session.g2o"};
	const char * labels[] = {"graph reduction applied", "no graph reduction"};
	for(size_t f=0; f<sizeof(files)/sizeof(const char *); ++f)
	{
		const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/" + files[f];
		if(!UFile::exists(path))
		{
			std::cout << "[          ] " << path << " not found, skipped" << std::endl;
			continue;
		}

		RealGraph graph;
		ASSERT_TRUE(loadRealGraph(path, graph)) << "could not read " << path;

		Memory * memory = newRealMemory();
		std::vector<int> newIds;
		int removedLinks = 0;
		UTimer timer;
		for(size_t i=0; i<graph.ids.size(); ++i)
		{
			addRealNode(memory, graph, i, newIds, &removedLinks);
		}
		const double buildTime = timer.ticks();

		ParametersMap localization;
		localization.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
		memory->parseParameters(localization);
		ASSERT_FALSE(memory->isIncremental());

		const std::vector<int> ids = bayesIdsOf(memory);
		std::cout << "[          ] " << files[f] << " (" << labels[f] << ")" << std::endl;
		printGraph(path, graph, removedLinks);
		std::cout << "[          ] rebuilt in " << buildTime << "s, " << ids.size()
				  << " locations (with the virtual place), dense matrix = "
				  << (ids.size()*ids.size()*sizeof(float))/1048576 << " MB" << std::endl;

		const std::map<int, float> likelihood = uniformLikelihoodOf(ids);
		std::cout << "[          ]  18 values model (default, depth 17):" << std::endl;
		compare(memory, ids, likelihood, PREDICTION_DEFAULT, iterations, /*sparseFirst=*/true);
		std::cout << "[          ]   8 values model (depth 7, every value above 1e-4):" << std::endl;
		compare(memory, ids, likelihood, PREDICTION_TRUNCATED, iterations, /*sparseFirst=*/true);
		delete memory;
	}
}

// A mapping session as it runs: a location added, then an iteration of the filter, over and
// over. Every added location changes the prediction, so this is the case the sparse form
// cannot amortize -- unlike localization, where it is built once and reused for the rest of
// the session. What it costs to keep it up to date against what its multiplication saves is
// what this measures.
//
// The session is replayed from its end: the locations before the window are added without
// running the filter, so the per-location cost is measured at the size the map really
// reaches rather than at the sizes it passes through.
TEST(BayesFilterPerfTest, DenseVsSparsePredictionWhileMappingARealSession)
{
	const size_t window = 15;   // locations added one at a time, with an iteration each
	if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		GTEST_SKIP() << "g2o optimizer not built in, needed to read the graph";
	}
	const std::string path = std::string(RTABMAP_TEST_DATA_ROOT) + "/tests/large_mapping_session.g2o";
	if(!UFile::exists(path))
	{
		GTEST_SKIP() << path << " not found";
	}

	RealGraph graph;
	ASSERT_TRUE(loadRealGraph(path, graph)) << "could not read " << path;
	ASSERT_GT(graph.ids.size(), window);
	const size_t prepared = graph.ids.size() - window;

	std::cout << "[          ] large_mapping_session.g2o, mapping mode: " << prepared
			  << " locations already mapped, " << window
			  << " more added one at a time with an iteration of the filter each" << std::endl;

	std::map<int, float> lastPosterior[2];
	for(int sparse=1; sparse>=0; --sparse)   // the sparse mode first, see compare()
	{
		Memory * memory = newRealMemory();
		std::vector<int> newIds;
		int removedLinks = 0;
		for(size_t i=0; i<prepared; ++i)
		{
			addRealNode(memory, graph, i, newIds, &removedLinks);
		}

		ParametersMap params;
		params.insert(ParametersPair(Parameters::kBayesSparsePrediction(), sparse?"true":"false"));
		BayesFilter filter(params);

		// The first iteration generates the whole prediction, as it does at the start of a
		// session; the ones after it are what a mapping session pays per location.
		std::vector<int> ids = bayesIdsOf(memory);
		UTimer timer;
		filter.computePosterior(memory, uniformLikelihoodOf(ids));
		const double first = timer.ticks();

		double total = 0.0, best = 0.0, worst = 0.0;
		for(size_t i=prepared; i<graph.ids.size(); ++i)
		{
			addRealNode(memory, graph, i, newIds, &removedLinks);
			ids = bayesIdsOf(memory);
			timer.restart();
			filter.computePosterior(memory, uniformLikelihoodOf(ids));
			const double elapsed = timer.ticks();
			total += elapsed;
			if(best == 0.0 || elapsed < best) best = elapsed;
			if(elapsed > worst) worst = elapsed;
		}
		lastPosterior[sparse] = posteriorOf(filter);

		printf("[          ]   %-6s first iteration %8.1f ms, then per added location: "
			   "fastest %8.1f ms, mean %8.1f ms, slowest %8.1f ms, filter memory %7.1f MB\n",
				sparse?"sparse":"dense", first*1000.0, best*1000.0, total*1000.0/double(window),
				worst*1000.0, filter.getMemoryUsed()/1048576.0);
		delete memory;
	}

	EXPECT_LT(maxPosteriorDifference(lastPosterior[0], lastPosterior[1]), 1e-3);
}
