#include <gtest/gtest.h>
#include <rtabmap/core/BayesFilter.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

using namespace rtabmap;

// PredictionLC format (when at a previously visited location), per Parameters::kBayesPredictionLC():
//   {newPlace, staySame, neighborLvl1, neighborLvl2, ...}
//   [0] probability to move to a new (unvisited) location
//   [1] probability to stay at the same location
//   [2+] probability to move to a neighbor at increasing graph depth
// The sum of values should normally be 1. If not, the difference is spread over remaining visited locations.
//
// VirtualPlacePriorThr (when at a new / unvisited location):
//   prior to move again to a new location; (1 - prior) is split equally over visited locations.

static const char kPredictionNewPlace10Stay90[] = "0.1 0.9";                    // sum = 1
static const char kPredictionNewPlace10Stay70Neighbor20[] = "0.1 0.7 0.2";     // sum = 1
static const char kPredictionNewPlace10Stay50Neighbor25_15[] = "0.1 0.5 0.25 0.15"; // sum = 1
static const char kPredictionSumBelowOne[] = "0.1 0.5";                         // sum = 0.6 (non-default, tests normalization)

static bool approxEqual(float a, float b, float epsilon = 1e-5f)
{
	return std::fabs(a - b) < epsilon;
}

// BayesFilter stores each prior as a column vector: index = col + row*cols.
static float predictionAt(const cv::Mat & prediction, int row, int col)
{
	return ((const float*)prediction.data)[col + row*prediction.cols];
}

static void expectPredictionColumn(
		const cv::Mat & prediction,
		int col,
		const std::vector<float> & expected)
{
	ASSERT_EQ(prediction.rows, (int)expected.size());
	for(int row = 0; row < prediction.rows; ++row)
	{
		EXPECT_NEAR(predictionAt(prediction, row, col), expected[row], 1e-4f)
			<< "row=" << row << " col=" << col;
	}
}

static void expectPosterior(
		const std::map<int, float> & posterior,
		const std::vector<int> & ids,
		const std::vector<float> & expected,
		int iter)
{
	ASSERT_EQ(ids.size(), expected.size());
	for(size_t i = 0; i < ids.size(); ++i)
	{
		EXPECT_NEAR(posterior.at(ids[i]), expected[i], 1e-4f)
			<< "iter=" << iter << " id=" << ids[i];
	}
}

// Expected posteriors for ComputePosteriorSequentialIterations (STM=5; ids: virtual, WM not in STM).
static const std::vector<float> & sequentialIterationExpectedPosterior(int iter)
{
	static const std::vector<std::vector<float>> kExpected = {
		{}, // iter 0-4: only virtual place in likelihood set
		{},
		{},
		{},
		{},
		{0.75f, 0.25f},
		{0.875f, 0.0833333f, 0.0416667f},
		{0.923077f, 0.0308494f, 0.0264423f, 0.0196314f},
		{0.939655f, 0.0171763f, 0.0168018f, 0.0151516f, 0.0112151f},
		{0.945153f, 0.0119263f, 0.0124289f, 0.0118722f, 0.0102744f, 0.00834519f},
		{0.814764f, 0.0724626f, 0.0254286f, 0.0255441f, 0.0232743f, 0.020965f, 0.0175609f},
		{0.687787f, 0.0507724f, 0.12774f, 0.0389096f, 0.0299996f, 0.0265426f, 0.0224654f, 0.0157832f},
		{0.577856f, 0.0560026f, 0.0720956f, 0.166958f, 0.0423361f, 0.0291128f, 0.0241468f, 0.019034f, 0.012458f},
		{0.496734f, 0.058187f, 0.0723748f, 0.0857542f, 0.174869f, 0.0418029f, 0.0255937f, 0.0200088f, 0.0151042f, 0.00957157f},
		{0.440075f, 0.0505439f, 0.0715218f, 0.081529f, 0.0873538f, 0.172839f, 0.0391538f, 0.0215903f, 0.0160934f, 0.0118602f, 0.00743967f},
		{0.402226f, 0.0460296f, 0.0603308f, 0.0772891f, 0.0807244f, 0.0858031f, 0.165356f, 0.0357832f, 0.0180625f, 0.0129809f, 0.00946335f, 0.00595065f},
		{0.378114f, 0.0414141f, 0.0539917f, 0.0642734f, 0.0751031f, 0.078339f, 0.0823795f, 0.155293f, 0.032451f, 0.0152614f, 0.0106809f, 0.00776909f, 0.00492958f},
		{0.363703f, 0.0369166f, 0.0481749f, 0.0569888f, 0.061882f, 0.0723659f, 0.0750282f, 0.0779455f, 0.144557f, 0.0294491f, 0.0131392f, 0.0090325f, 0.00658868f, 0.00422884f},
		{0.355991f, 0.0331439f, 0.0427665f, 0.0505475f, 0.0544998f, 0.05951f, 0.0692224f, 0.0711662f, 0.0731903f, 0.134173f, 0.0268633f, 0.0115581f, 0.00786145f, 0.00576469f, 0.00374188f},
		{0.652344f, 0.0184008f, 0.0236226f, 0.0275895f, 0.0296656f, 0.0322255f, 0.0351563f, 0.0405571f, 0.0413802f, 0.0422275f, 0.0256058f, 0.0152104f, 0.0063979f, 0.00432969f, 0.00319375f, 0.00209316f},
		{0.831476f, 0.00925432f, 0.0114882f, 0.0131433f, 0.0138708f, 0.0149429f, 0.0161433f, 0.0174237f, 0.017399f, 0.0160799f, 0.0124093f, 0.00878715f, 0.00564433f, 0.00403844f, 0.00312244f, 0.0026289f, 0.00214745f},
		{0.907199f, 0.00518929f, 0.00609211f, 0.00672985f, 0.00694705f, 0.00728456f, 0.00763853f, 0.00788398f, 0.00777658f, 0.00724726f, 0.00627715f, 0.00519555f, 0.00421978f, 0.00355356f, 0.00314741f, 0.00283793f, 0.00257299f, 0.00220688f},
		{0.934287f, 0.00359245f, 0.0040005f, 0.00426123f, 0.00429139f, 0.00437621f, 0.00446477f, 0.0045054f, 0.00444871f, 0.00427653f, 0.00400231f, 0.00368727f, 0.00339189f, 0.00316023f, 0.00299493f, 0.00288156f, 0.00270805f, 0.00250558f, 0.00216441f},
		{0.943385f, 0.00294588f, 0.0031779f, 0.00330671f, 0.00327698f, 0.003287f, 0.00330541f, 0.00330931f, 0.00328643f, 0.00323249f, 0.0031524f, 0.00305979f, 0.0029698f, 0.00289395f, 0.00283249f, 0.00277944f, 0.00272823f, 0.00258864f, 0.00240425f, 0.00207839f},
		{0.946375f, 0.0026408f, 0.00280855f, 0.00289129f, 0.00284519f, 0.00283494f, 0.00283617f, 0.0028347f, 0.00282621f, 0.00280935f, 0.00278543f, 0.00275775f, 0.00273007f, 0.00270512f, 0.00268362f, 0.00266139f, 0.00263312f, 0.0025961f, 0.00246812f, 0.00229391f, 0.00198317f}};
	return kExpected[iter];
}

// Expected posteriors for ComputePosteriorSequentialIterationsWithLoopClosures
// (STM=5, global loop closures added; ids order: virtual, WM nodes not in STM).
static const std::vector<float> & sequentialIterationsWithLoopClosuresExpectedPosterior(int iter)
{
	static const std::vector<std::vector<float>> kExpected = {
		{}, // iter 0-4: only virtual place in likelihood set
		{},
		{},
		{},
		{},
		{0.75f, 0.25f},
		{0.875f, 0.0833333f, 0.0416667f},
		{0.923077f, 0.0308494f, 0.0264423f, 0.0196314f},
		{0.939655f, 0.0171763f, 0.0168018f, 0.0151516f, 0.0112151f},
		{0.945153f, 0.0119263f, 0.0124289f, 0.0118722f, 0.0102744f, 0.00834519f},
		{0.814764f, 0.0724626f, 0.0254286f, 0.0255441f, 0.0232743f, 0.020965f, 0.0175609f},
		{0.687787f, 0.0507724f, 0.12774f, 0.0389096f, 0.0299996f, 0.0265426f, 0.0224654f, 0.0157832f},
		{0.577856f, 0.0560026f, 0.0720956f, 0.166958f, 0.0423361f, 0.0291128f, 0.0241468f, 0.019034f, 0.012458f},
		{0.496734f, 0.0546531f, 0.0706079f, 0.084694f, 0.174869f, 0.0418029f, 0.0255937f, 0.0200088f, 0.0151042f, 0.0159325f},
		{0.440161f, 0.0411996f, 0.0634381f, 0.0768023f, 0.0861785f, 0.172581f, 0.0391614f, 0.0220643f, 0.0168796f, 0.0184511f, 0.0230838f},
		{0.420178f, 0.0249628f, 0.045013f, 0.0649444f, 0.0740554f, 0.0815264f, 0.172711f, 0.036486f, 0.0191379f, 0.0169987f, 0.0190227f, 0.0249628f},
		{0.412364f, 0.0207239f, 0.0259606f, 0.0431051f, 0.0634659f, 0.0727025f, 0.0810265f, 0.163952f, 0.0349961f, 0.017393f, 0.0176253f, 0.0207239f, 0.0259606f},
		{0.420189f, 0.0136672f, 0.021482f, 0.0199287f, 0.0430812f, 0.0637132f, 0.07283f, 0.0781999f, 0.160618f, 0.0332184f, 0.0179942f, 0.0136672f, 0.021482f, 0.0199287f},
		{0.437437f, 0.0110064f, 0.0137286f, 0.0147716f, 0.0203057f, 0.0438235f, 0.06444f, 0.069375f, 0.077517f, 0.155293f, 0.0324897f, 0.0110064f, 0.0137286f, 0.0147716f, 0.0203057f},
		{0.771644f, 0.00669918f, 0.00583698f, 0.00459007f, 0.00849329f, 0.0116668f, 0.0247358f, 0.0265199f, 0.0369352f, 0.0393452f, 0.0262473f, 0.00669918f, 0.00583698f, 0.00459007f, 0.00849329f, 0.0116668f},
		{0.911612f, 0.00467563f, 0.00353185f, 0.00271612f, 0.00328848f, 0.00493026f, 0.0056733f, 0.00851644f, 0.0109721f, 0.0111152f, 0.0081531f, 0.00467563f, 0.00353185f, 0.00271612f, 0.00328848f, 0.00493026f, 0.0056733f},
		{0.947737f, 0.0031825f, 0.00290135f, 0.00240094f, 0.00247641f, 0.00269291f, 0.00316849f, 0.00325958f, 0.00407643f, 0.00419794f, 0.00382469f, 0.0031825f, 0.00290135f, 0.00240094f, 0.00247641f, 0.00269291f, 0.00316849f, 0.00325958f},
		{0.955789f, 0.00261069f, 0.002592f, 0.00226321f, 0.0022729f, 0.00230073f, 0.00236184f, 0.00255397f, 0.00245376f, 0.00266121f, 0.00273125f, 0.00261069f, 0.002592f, 0.00226321f, 0.0022729f, 0.00230073f, 0.00236184f, 0.00255397f, 0.00245376f},
		{0.958539f, 0.00233656f, 0.0022108f, 0.00214845f, 0.00214969f, 0.00215325f, 0.00216106f, 0.00218564f, 0.00219204f, 0.0021554f, 0.00207522f, 0.00233656f, 0.0022108f, 0.00214845f, 0.00214969f, 0.00215325f, 0.00216106f, 0.00218564f, 0.00219204f, 0.0021554f},
		{0.959256f, 0.00207819f, 0.00206748f, 0.00204223f, 0.00204239f, 0.00204285f, 0.00204384f, 0.00203176f, 0.00203743f, 0.00201898f, 0.00191449f, 0.00207819f, 0.00206748f, 0.00204223f, 0.00204239f, 0.00204285f, 0.00204384f, 0.00203176f, 0.00203743f, 0.00201898f, 0.00201898f}};
	return kExpected[iter];
}

static int highestVisitedPosteriorId(const std::map<int, float> & posterior)
{
	int bestId = 0;
	float bestProb = -1.0f;
	for(std::map<int, float>::const_iterator it = posterior.begin(); it != posterior.end(); ++it)
	{
		if(it->first > 0 && it->second > bestProb)
		{
			bestProb = it->second;
			bestId = it->first;
		}
	}
	return bestId;
}

/**
 * Builds a minimal linear Memory graph for BayesFilter tests.
 * Signatures are linked as neighbors along the odometry chain.
 * With Mem/STMSize=1, only the latest signature stays in STM so that
 * BayesFilter can find margin-0 neighbors outside STM.
 */
class BayesFilterMemoryFixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		image_ = cv::Mat(8, 8, CV_8UC1, cv::Scalar(128));
		covariance_ = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		initMemory(1);
	}

	void initMemory(int stmSize)
	{
		delete memory_;
		memory_ = nullptr;

		ParametersMap params;
		params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
		const std::string stmSizeStr = std::to_string(stmSize);
		params.insert(ParametersPair(Parameters::kMemSTMSize(), stmSizeStr.c_str()));
		params.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0"));
		params.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));

		memory_ = new Memory(params);
	}

	void TearDown() override
	{
		delete memory_;
		memory_ = 0;
	}

	void addChain(int count)
	{
		ASSERT_GT(count, 0);
		for(int i = 0; i < count; ++i)
		{
			SensorData data(image_);
			Transform pose(float(i), 0.0f, 0.0f, 0, 0, 0);
			ASSERT_TRUE(memory_->update(data, pose, covariance_));
		}
	}

	// Virtual place (new location hypothesis) + WM locations not in STM.
	std::vector<int> getBayesIds(bool includeVirtual = true) const
	{
		std::vector<int> ids;
		if(includeVirtual)
		{
			ids.push_back(Memory::kIdVirtual);
		}
		const std::set<int> & stm = memory_->getStMem();
		for(std::map<int, double>::const_iterator iter = memory_->getWorkingMem().begin();
			iter != memory_->getWorkingMem().end();
			++iter)
		{
			if(iter->first > 0 && stm.find(iter->first) == stm.end())
			{
				ids.push_back(iter->first);
			}
		}
		return ids;
	}

	std::map<int, float> uniformLikelihood(const std::vector<int> & ids, float value = 1.0f) const
	{
		std::map<int, float> likelihood;
		for(size_t i = 0; i < ids.size(); ++i)
		{
			likelihood.insert(std::make_pair(ids[i], value));
		}
		return likelihood;
	}

	// Grow chain by one signature after an initial chain; returns posterior and prediction.
	std::pair<std::map<int, float>, cv::Mat> computePosteriorAfterGrowingChain(
			int initialChainCount,
			bool fullPredictionUpdate)
	{
		initMemory(1);
		addChain(initialChainCount);

		ParametersMap params;
		params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
		params.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), fullPredictionUpdate ? "true" : "false"));
		BayesFilter filter(params);

		std::vector<int> ids = getBayesIds();
		std::map<int, float> likelihood = uniformLikelihood(ids);
		filter.computePosterior(memory_, likelihood);

		SensorData data(image_);
		Transform pose(float(initialChainCount), 0.0f, 0.0f, 0, 0, 0);
		EXPECT_TRUE(memory_->update(data, pose, covariance_));

		ids = getBayesIds();
		EXPECT_EQ(ids.size(), (size_t)initialChainCount + 1); // virtual + WM nodes (latest in STM)
		likelihood = uniformLikelihood(ids);
		filter.computePosterior(memory_, likelihood);

		return std::make_pair(filter.getPosterior(), filter.generatePrediction(memory_, ids));
	}

	Memory * memory_ = nullptr;
	cv::Mat image_;
	cv::Mat covariance_;
};

// Constructor Tests

TEST(BayesFilterTest, DefaultConstructor)
{
	BayesFilter filter;

	EXPECT_TRUE(filter.getPosterior().empty());
	EXPECT_FLOAT_EQ(filter.getVirtualPlacePrior(), Parameters::defaultBayesVirtualPlacePriorThr());
	EXPECT_GE(filter.getPredictionLC().size(), 2u);
	EXPECT_FALSE(filter.getPredictionLCStr().empty());
}

TEST(BayesFilterTest, ParameterizedConstructor)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesVirtualPlacePriorThr(), "0.5"));
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), "0.2 0.5 0.3"));
	params.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), "true"));

	BayesFilter filter(params);

	EXPECT_FLOAT_EQ(filter.getVirtualPlacePrior(), 0.5f);
	ASSERT_EQ(filter.getPredictionLC().size(), 3u);
	EXPECT_FLOAT_EQ(filter.getPredictionLC()[0], 0.2f); // new place
	EXPECT_FLOAT_EQ(filter.getPredictionLC()[1], 0.5f); // stay at same location
	EXPECT_FLOAT_EQ(filter.getPredictionLC()[2], 0.3f); // neighbor level 1
}

// parseParameters Tests

TEST(BayesFilterTest, ParseParameters)
{
	BayesFilter filter;
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesVirtualPlacePriorThr(), "0.25"));
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), "0.3 0.7"));
	params.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), "true"));

	filter.parseParameters(params);

	EXPECT_FLOAT_EQ(filter.getVirtualPlacePrior(), 0.25f);
	ASSERT_EQ(filter.getPredictionLC().size(), 2u);
	EXPECT_FLOAT_EQ(filter.getPredictionLC()[0], 0.3f);
	EXPECT_FLOAT_EQ(filter.getPredictionLC()[1], 0.7f);
}

// setPredictionLC / getPredictionLC Tests

TEST(BayesFilterTest, SetPredictionLCValid)
{
	BayesFilter filter;
	const std::string prediction = kPredictionNewPlace10Stay50Neighbor25_15;
	filter.setPredictionLC(prediction);

	const std::vector<double> & values = filter.getPredictionLC();
	ASSERT_EQ(values.size(), 4u);
	EXPECT_NEAR(values[0], 0.1, 1e-6); // new place
	EXPECT_NEAR(values[1], 0.5, 1e-6); // stay at same location
	EXPECT_NEAR(values[2], 0.25, 1e-6); // neighbor level 1
	EXPECT_NEAR(values[3], 0.15, 1e-6); // neighbor level 2
	double sum = 0.0;
	for(size_t i = 0; i < values.size(); ++i) { sum += values[i]; }
	EXPECT_NEAR(sum, 1.0, 1e-6);
	EXPECT_EQ(filter.getPredictionLCStr(), prediction);
}

TEST(BayesFilterTest, SetPredictionLCInvalidKeepsPrevious)
{
	BayesFilter filter;
	filter.setPredictionLC(kPredictionNewPlace10Stay90);

	const std::vector<double> before = filter.getPredictionLC();
	filter.setPredictionLC("only_one_value");
	EXPECT_EQ(filter.getPredictionLC(), before);

	filter.setPredictionLC("0.2 1.5");
	EXPECT_EQ(filter.getPredictionLC(), before);

	filter.setPredictionLC("-0.1 0.5");
	EXPECT_EQ(filter.getPredictionLC(), before);
}

TEST(BayesFilterTest, GetPredictionLCStrRoundTrip)
{
	BayesFilter filter;
	const std::string prediction = "0.05 0.45 0.5"; // sum = 1
	filter.setPredictionLC(prediction);
	EXPECT_EQ(filter.getPredictionLCStr(), prediction);
}

// reset Tests

TEST(BayesFilterTest, Reset)
{
	Memory memory;
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay90));
	BayesFilter filter(params);

	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 1.0f;
	filter.computePosterior(&memory, likelihood);
	ASSERT_FALSE(filter.getPosterior().empty());

	filter.reset();
	EXPECT_TRUE(filter.getPosterior().empty());
}

// getMemoryUsed Tests

TEST(BayesFilterTest, GetMemoryUsed)
{
	BayesFilter filter;
	EXPECT_GE(filter.getMemoryUsed(), sizeof(BayesFilter));

	Memory memory;
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 1.0f;
	filter.computePosterior(&memory, likelihood);
	EXPECT_GT(filter.getMemoryUsed(), sizeof(BayesFilter));
}

// computePosterior error paths

TEST(BayesFilterTest, ComputePosteriorNullMemory)
{
	BayesFilter filter;
	std::map<int, float> likelihood;
	likelihood[1] = 1.0f;

	const std::map<int, float> & result = filter.computePosterior(nullptr, likelihood);
	EXPECT_TRUE(result.empty());
}

TEST(BayesFilterTest, ComputePosteriorEmptyLikelihood)
{
	Memory memory;
	BayesFilter filter;
	std::map<int, float> likelihood;

	const std::map<int, float> & result = filter.computePosterior(&memory, likelihood);
	EXPECT_TRUE(result.empty());
}

// generatePrediction Tests (virtual place only, no graph)

TEST(BayesFilterTest, GeneratePredictionVirtualPlaceOnly)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesVirtualPlacePriorThr(), "0.9"));
	BayesFilter filter(params);

	Memory memory;
	std::vector<int> ids = {Memory::kIdVirtual};
	cv::Mat prediction = filter.generatePrediction(&memory, ids);

	ASSERT_EQ(prediction.rows, 1);
	ASSERT_EQ(prediction.cols, 1);
	EXPECT_FLOAT_EQ(prediction.at<float>(0, 0), 1.0f);
}

TEST(BayesFilterTest, GeneratePredictionCachedWhenIdsUnchanged)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay90));
	BayesFilter filter(params);

	Memory memory;
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 1.0f;
	filter.computePosterior(&memory, likelihood);

	std::vector<int> ids = {Memory::kIdVirtual};
	cv::Mat prediction1 = filter.generatePrediction(&memory, ids);
	cv::Mat prediction2 = filter.generatePrediction(&memory, ids);

	ASSERT_FALSE(prediction1.empty());
	EXPECT_EQ(prediction1.data, prediction2.data);
}

TEST(BayesFilterTest, ComputePosteriorVirtualPlaceOnly)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay90));
	BayesFilter filter(params);

	Memory memory;
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 1.0f;

	filter.computePosterior(&memory, likelihood);

	const std::map<int, float> & posterior = filter.getPosterior();
	ASSERT_EQ(posterior.size(), 1u);
	EXPECT_EQ(posterior.begin()->first, Memory::kIdVirtual);
	EXPECT_TRUE(approxEqual(posterior.begin()->second, 1.0f));
}

TEST(BayesFilterTest, ComputePosteriorNormalizesPosterior)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay90));
	BayesFilter filter(params);

	Memory memory;
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 0.6f;

	filter.computePosterior(&memory, likelihood);

	const std::map<int, float> & posterior = filter.getPosterior();
	ASSERT_EQ(posterior.size(), 1u);
	EXPECT_TRUE(approxEqual(posterior.begin()->second, 1.0f));
}

TEST(BayesFilterTest, ComputePosteriorUpdatesWithNewLikelihood)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay90));
	BayesFilter filter(params);

	Memory memory;
	std::map<int, float> likelihood;
	likelihood[Memory::kIdVirtual] = 1.0f;
	filter.computePosterior(&memory, likelihood);

	likelihood[Memory::kIdVirtual] = 0.5f;
	filter.computePosterior(&memory, likelihood);

	const std::map<int, float> & posterior = filter.getPosterior();
	ASSERT_EQ(posterior.size(), 1u);
	EXPECT_TRUE(approxEqual(posterior.begin()->second, 1.0f));
}

// Integration tests with real Memory graph

TEST_F(BayesFilterMemoryFixture, GeneratePredictionLinearChain)
{
	addChain(5);

	const float virtualPlacePrior = 0.9f;
	const float otherVisitedPrior = (1.0f - virtualPlacePrior) / 4.0f; // split over 4 visited locations

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
	params.insert(ParametersPair(Parameters::kBayesVirtualPlacePriorThr(), "0.9"));
	BayesFilter filter(params);

	const std::vector<int> ids = getBayesIds();
	ASSERT_EQ(ids.size(), 5u); // virtual + 4 visited WM nodes (latest in STM)
	ASSERT_EQ(ids[0], Memory::kIdVirtual);

	const std::vector<double> & predictionLC = filter.getPredictionLC();
	ASSERT_EQ(predictionLC.size(), 4u);

	// Middle node (id 2): stay + two depth-1 neighbors + one depth-2 neighbor -> sum 1.15, scaled to 0.9.
	const float maxNorm = 1.0f - (float)predictionLC[0];
	const float middleNodeNeighborSum =
			(float)predictionLC[1] + 2.0f * (float)predictionLC[2] + (float)predictionLC[3];
	const float scaleRatio = maxNorm / middleNodeNeighborSum;

	cv::Mat prediction = filter.generatePrediction(memory_, ids);
	ASSERT_EQ(prediction.rows, 5);
	ASSERT_EQ(prediction.cols, 5);

	// Linear chain ids: [virtual, 1, 2, 3, 4] (signature 5 in STM). Each column is the prior
	// "if we are at this location". PredictionLC = {0.1, 0.5, 0.25, 0.15}.

	// Column 0 (virtual / new place): VirtualPlacePriorThr, remainder split on visited locations.
	expectPredictionColumn(prediction, 0, {virtualPlacePrior, otherVisitedPrior, otherVisitedPrior, otherVisitedPrior, otherVisitedPrior});

	// Column 1 (chain start): one neighbor per depth -> raw PredictionLC.
	expectPredictionColumn(prediction, 1, {
		(float)predictionLC[0],
		(float)predictionLC[1],
		(float)predictionLC[2],
		(float)predictionLC[3],
		0.0f});

	// Column 2 (middle, id 2): two depth-1 neighbors -> LC values scaled by scaleRatio.
	expectPredictionColumn(prediction, 2, {
		(float)predictionLC[0],
		(float)predictionLC[2] * scaleRatio,
		(float)predictionLC[1] * scaleRatio,
		(float)predictionLC[2] * scaleRatio,
		(float)predictionLC[3] * scaleRatio});

	// Column 3 (middle, id 3): symmetric to column 2.
	expectPredictionColumn(prediction, 3, {
		(float)predictionLC[0],
		(float)predictionLC[3] * scaleRatio,
		(float)predictionLC[2] * scaleRatio,
		(float)predictionLC[1] * scaleRatio,
		(float)predictionLC[2] * scaleRatio});

	// Column 4 (chain end, id 4): one neighbor per depth, no scaling needed.
	expectPredictionColumn(prediction, 4, {
		(float)predictionLC[0],
		0.0f,
		(float)predictionLC[3],
		(float)predictionLC[2],
		(float)predictionLC[1]});
}

TEST_F(BayesFilterMemoryFixture, GeneratePredictionNormalizesWhenSumBelowOne)
{
	addChain(4);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionSumBelowOne));
	BayesFilter filter(params);

	const std::vector<int> ids = getBayesIds();
	ASSERT_EQ(ids.size(), 4u); // virtual + 3 visited WM nodes (latest in STM)
	ASSERT_EQ(ids[0], Memory::kIdVirtual);

	const std::vector<double> & predictionLC = filter.getPredictionLC();
	ASSERT_EQ(predictionLC.size(), 2u);

	const float virtualPlacePrior = filter.getVirtualPlacePrior();
	const float otherVisitedPrior = (1.0f - virtualPlacePrior) / 3.0f;
	const float totalLC = (float)predictionLC[0] + (float)predictionLC[1]; // 0.6
	const float remainderPerVisited = (1.0f - totalLC) / 3.0f; // (1 - sum(LC)) / (cols - 1)
	const float maxNorm = 1.0f - (float)predictionLC[0];
	// stay + two other visited rows filled with remainder before scaling to maxNorm
	const float addedBeforeScale = (float)predictionLC[1] + 2.0f * remainderPerVisited;
	const float scaleRatio = maxNorm / addedBeforeScale;

	cv::Mat prediction = filter.generatePrediction(memory_, ids);
	ASSERT_EQ(prediction.rows, 4);
	ASSERT_EQ(prediction.cols, 4);

	// ids: [virtual, 1, 2, 3]. PredictionLC = {0.1, 0.5} (sum < 1).

	// Column 0 (virtual / new place).
	expectPredictionColumn(prediction, 0, {virtualPlacePrior, otherVisitedPrior, otherVisitedPrior, otherVisitedPrior});

	// Column 1 (chain start, id 1).
	expectPredictionColumn(prediction, 1, {
		(float)predictionLC[0],
		(float)predictionLC[1] * scaleRatio,
		remainderPerVisited * scaleRatio,
		remainderPerVisited * scaleRatio});

	// Column 2 (middle, id 2).
	expectPredictionColumn(prediction, 2, {
		(float)predictionLC[0],
		remainderPerVisited * scaleRatio,
		(float)predictionLC[1] * scaleRatio,
		remainderPerVisited * scaleRatio});

	// Column 3 (chain end, id 3).
	expectPredictionColumn(prediction, 3, {
		(float)predictionLC[0],
		remainderPerVisited * scaleRatio,
		remainderPerVisited * scaleRatio,
		(float)predictionLC[1] * scaleRatio});
}

TEST_F(BayesFilterMemoryFixture, GeneratePredictionCachedWithGraph)
{
	addChain(4);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay70Neighbor20));
	BayesFilter filter(params);

	const std::vector<int> ids = getBayesIds();
	std::map<int, float> likelihood = uniformLikelihood(ids);
	filter.computePosterior(memory_, likelihood);

	cv::Mat prediction1 = filter.generatePrediction(memory_, ids);
	cv::Mat prediction2 = filter.generatePrediction(memory_, ids);
	ASSERT_FALSE(prediction1.empty());
	EXPECT_EQ(prediction1.data, prediction2.data);
}

TEST_F(BayesFilterMemoryFixture, ComputePosteriorNormalizedWithGraph)
{
	addChain(16);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
	BayesFilter filter(params);

	const std::vector<int> ids = getBayesIds();
	ASSERT_EQ(ids.size(), 16u); // virtual + 15 visited WM nodes (latest in STM)
	ASSERT_EQ(ids[0], Memory::kIdVirtual);

	std::map<int, float> likelihood = uniformLikelihood(ids);
	filter.computePosterior(memory_, likelihood);

	const std::map<int, float> & posterior = filter.getPosterior();
	ASSERT_EQ(posterior.size(), ids.size());

	// Linear chain ids: [virtual, 1..15]. Uniform likelihood, init posterior on virtual place.
	EXPECT_NEAR(posterior.at(Memory::kIdVirtual), 0.15f, 1e-4f);
	EXPECT_NEAR(posterior.at(1), 0.0503853f, 1e-4f);
	EXPECT_NEAR(posterior.at(2), 0.0578059f, 1e-4f);
	EXPECT_NEAR(posterior.at(3), 0.0609622f, 1e-4f);
	EXPECT_NEAR(posterior.at(4), 0.0575132f, 1e-4f);
	EXPECT_NEAR(posterior.at(5), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(6), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(7), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(8), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(9), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(10), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(11), 0.0566667f, 1e-4f);
	EXPECT_NEAR(posterior.at(12), 0.0575132f, 1e-4f);
	EXPECT_NEAR(posterior.at(13), 0.0609622f, 1e-4f);
	EXPECT_NEAR(posterior.at(14), 0.0578059f, 1e-4f);
	EXPECT_NEAR(posterior.at(15), 0.0503853f, 1e-4f);

	float sum = 0.0f;
	for(std::map<int, float>::const_iterator iter = posterior.begin(); iter != posterior.end(); ++iter)
	{
		sum += iter->second;
	}
	EXPECT_NEAR(sum, 1.0f, 1e-4f);
}

TEST_F(BayesFilterMemoryFixture, ComputePosteriorFavorsHighLikelihood)
{
	addChain(16);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay70Neighbor20));
	BayesFilter filter(params);

	const std::vector<int> ids = getBayesIds();
	ASSERT_EQ(ids.size(), 16u); // virtual + 15 visited WM nodes (latest in STM)
	ASSERT_EQ(ids[0], Memory::kIdVirtual);

	// Pick the oldest visited WM location (smallest id > 0).
	int favoredId = 0;
	for(size_t i = 0; i < ids.size(); ++i)
	{
		if(ids[i] > 0 && (favoredId == 0 || ids[i] < favoredId))
		{
			favoredId = ids[i];
		}
	}
	ASSERT_EQ(favoredId, 1);

	std::map<int, float> likelihood = uniformLikelihood(ids, 1.0f);
	likelihood[favoredId] = 3.0f;

	// ids: [virtual, 1..15], favoredId = 1. Same likelihood each iteration.
	const float kExpectedPosterior[5][16] = {
		{0.135283f, 0.147171f, 0.0531566f, 0.0511069f, 0.0511069f, 0.0511069f, 0.0511069f, 0.0511069f,
		 0.0511069f, 0.0511069f, 0.0511069f, 0.0511069f, 0.0511069f, 0.0511069f, 0.0531566f, 0.0490571f},
		{0.169947f, 0.27575f, 0.0564325f, 0.0385504f, 0.0382766f, 0.0382766f, 0.0382766f, 0.0382766f,
		 0.0382766f, 0.0382766f, 0.0382766f, 0.0382766f, 0.0382766f, 0.0385504f, 0.0404169f, 0.0358625f},
		{0.167728f, 0.433739f, 0.0674671f, 0.0275164f, 0.0253249f, 0.0252931f, 0.0252931f, 0.0252931f,
		 0.0252931f, 0.0252931f, 0.0252931f, 0.0252931f, 0.0253249f, 0.0256535f, 0.0268425f, 0.0233514f},
		{0.143534f, 0.58063f, 0.0802972f, 0.0196511f, 0.0148717f, 0.0146408f, 0.0146376f, 0.0146376f,
		 0.0146376f, 0.0146376f, 0.0146376f, 0.0146408f, 0.0146849f, 0.0149227f, 0.0155433f, 0.0133962f},
		{0.116686f, 0.685258f, 0.0903205f, 0.0150916f, 0.00819399f, 0.00769706f, 0.00767554f, 0.00767525f,
		 0.00767525f, 0.00767525f, 0.00767554f, 0.00768045f, 0.0077156f, 0.00784865f, 0.00813662f, 0.00699466f}};

	for(int iter = 0; iter < 5; ++iter)
	{
		filter.computePosterior(memory_, likelihood);

		const std::map<int, float> & posterior = filter.getPosterior();
		ASSERT_EQ(posterior.size(), ids.size());

		for(size_t i = 0; i < ids.size(); ++i)
		{
			EXPECT_NEAR(posterior.at(ids[i]), kExpectedPosterior[iter][i], 1e-4f)
				<< "iter=" << iter << " id=" << ids[i];
		}

		for(size_t i = 0; i < ids.size(); ++i)
		{
			if(ids[i] > 0 && ids[i] != favoredId)
			{
				EXPECT_GT(posterior.at(favoredId), posterior.at(ids[i]))
					<< "iter=" << iter << " favoredId=" << favoredId << " otherId=" << ids[i];
			}
		}
	}
}

TEST_F(BayesFilterMemoryFixture, ComputePosteriorSequentialIterations)
{
	initMemory(5);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
	params.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), "true"));
	BayesFilter filter(params);

	const float favoredLikelihood = 3.0f;

	// STM=5 (same as loop-closure test). 25 iterations: grow chain, revisit nodes 1..9, then new place.
	// Iterations 0-4: only virtual place in likelihood set.
	// Iterations 5-9: favor virtual place; iter 10-18: favor id 1..9; iter 19-24: favor virtual again.
	for(int iter = 0; iter < 25; ++iter)
	{
		SensorData data(image_);
		Transform pose(float(iter), 0.0f, 0.0f, 0, 0, 0);
		ASSERT_TRUE(memory_->update(data, pose, covariance_));

		const std::vector<int> ids = getBayesIds();
		ASSERT_GE(ids.size(), 1u);
		// Until a signature leaves STM, only the virtual place is in the likelihood set.
		if(ids.size() < 2u)
		{
			continue;
		}

		std::map<int, float> likelihood = uniformLikelihood(ids);
		int favoredId = 0;
		if(iter < 10 || iter >= 19)
		{
			favoredId = Memory::kIdVirtual;
			likelihood[favoredId] = favoredLikelihood;
		}
		else if(iter < 19)
		{
			favoredId = iter - 9; // iter 10 -> id 1, iter 18 -> id 9
			ASSERT_GT(favoredId, 0);
			ASSERT_TRUE(likelihood.count(favoredId) > 0);
			likelihood[favoredId] = favoredLikelihood;
		}

		filter.computePosterior(memory_, likelihood);

		const std::map<int, float> & posterior = filter.getPosterior();
		ASSERT_EQ(posterior.size(), ids.size());

		ASSERT_GE(iter, 5);
		ASSERT_LT(iter, 25);
		expectPosterior(posterior, ids, sequentialIterationExpectedPosterior(iter), iter);
	}
}

TEST_F(BayesFilterMemoryFixture, ComputePosteriorSequentialIterationsWithLoopClosures)
{
	// STM size 5: current signature stays in STM, so loop closures target older WM nodes
	// (not the odometry neighbor link to the node added in the previous iteration).
	initMemory(5);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
	params.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), "true"));
	BayesFilter filter(params);

	const float favoredLikelihood = 3.0f;
	const float visitedMassThreshold = 0.15f; // 1 - virtualPlacePosterior
	const cv::Mat loopClosureInfMatrix = cv::Mat::eye(6, 6, CV_64FC1);

	int loopClosuresAdded = 0;

	// Same schedule as ComputePosteriorSequentialIterations. When visited mass
	// (1 - virtual place posterior) > 0.15, add a global loop closure from the current
	// signature to the visited location with highest posterior.
	for(int iter = 0; iter < 25; ++iter)
	{
		SensorData data(image_);
		Transform pose(float(iter), 0.0f, 0.0f, 0, 0, 0);
		ASSERT_TRUE(memory_->update(data, pose, covariance_));

		const std::vector<int> ids = getBayesIds();
		if(ids.size() < 2u)
		{
			continue;
		}

		std::map<int, float> likelihood = uniformLikelihood(ids);
		int favoredId = 0;
		if(iter < 10 || iter >= 19)
		{
			favoredId = Memory::kIdVirtual;
			likelihood[favoredId] = favoredLikelihood;
		}
		else if(iter < 19)
		{
			favoredId = iter - 9;
			ASSERT_TRUE(likelihood.count(favoredId) > 0);
			likelihood[favoredId] = favoredLikelihood;
		}

		filter.computePosterior(memory_, likelihood);

		const std::map<int, float> & posterior = filter.getPosterior();
		ASSERT_EQ(posterior.size(), ids.size());

		ASSERT_GE(iter, 5);
		ASSERT_LT(iter, 25);
		expectPosterior(posterior, ids, sequentialIterationsWithLoopClosuresExpectedPosterior(iter), iter);

		const float virtualPosterior = posterior.at(Memory::kIdVirtual);
		// Same as Rtabmap: ignore loop closure when there is only one hypothesis
		// (virtual place + one visited location).
		if(posterior.size() > 2 && 1.0f - virtualPosterior > visitedMassThreshold)
		{
			const int currentSignatureId = memory_->getLastSignatureId();
			const int bestVisitedId = highestVisitedPosteriorId(posterior);
			ASSERT_GT(currentSignatureId, 0);
			ASSERT_GT(bestVisitedId, 0);
			ASSERT_NE(currentSignatureId, bestVisitedId);

			const Signature * currentSignature = memory_->getSignature(currentSignatureId);
			ASSERT_NE(currentSignature, nullptr);
			ASSERT_TRUE(memory_->addLink(Link(
					currentSignatureId,
					bestVisitedId,
					Link::kGlobalClosure,
					Transform::getIdentity(),
					loopClosureInfMatrix)));
			EXPECT_TRUE(currentSignature->hasLink(bestVisitedId, Link::kGlobalClosure));
			++loopClosuresAdded;
		}
	}

	EXPECT_GT(loopClosuresAdded, 0);
	// iter 10-19 (visited mass > 0.15, multiple WM hypotheses)
	EXPECT_EQ(loopClosuresAdded, 10);
}

TEST_F(BayesFilterMemoryFixture, CompareFullPredictionUpdateModes)
{
	ParametersMap paramsIncremental;
	paramsIncremental.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay50Neighbor25_15));
	paramsIncremental.insert(ParametersPair(Parameters::kBayesFullPredictionUpdate(), "false"));

	ParametersMap paramsFull = paramsIncremental;
	paramsFull[Parameters::kBayesFullPredictionUpdate()] = "true";

	// --- First step (empty prediction): both modes fully build the matrix.
	addChain(5);

	const std::vector<int> ids = getBayesIds();
	ASSERT_EQ(ids.size(), 5u);
	std::map<int, float> likelihood = uniformLikelihood(ids);

	BayesFilter filterIncremental(paramsIncremental);
	BayesFilter filterFull(paramsFull);

	filterIncremental.computePosterior(memory_, likelihood);
	filterFull.computePosterior(memory_, likelihood);

	const std::map<int, float> & posteriorIncremental = filterIncremental.getPosterior();
	const std::map<int, float> & posteriorFull = filterFull.getPosterior();
	ASSERT_EQ(posteriorIncremental.size(), posteriorFull.size());
	for(size_t i = 0; i < ids.size(); ++i)
	{
		EXPECT_NEAR(posteriorIncremental.at(ids[i]), posteriorFull.at(ids[i]), 1e-4f) << "id=" << ids[i];
	}

	cv::Mat predictionIncremental = filterIncremental.generatePrediction(memory_, ids);
	cv::Mat predictionFull = filterFull.generatePrediction(memory_, ids);
	ASSERT_EQ(predictionIncremental.rows, predictionFull.rows);
	ASSERT_EQ(predictionIncremental.cols, predictionFull.cols);
	for(int col = 0; col < predictionIncremental.cols; ++col)
	{
		for(int row = 0; row < predictionIncremental.rows; ++row)
		{
			EXPECT_NEAR(predictionAt(predictionIncremental, row, col),
			            predictionAt(predictionFull, row, col), 1e-4f)
				<< "row=" << row << " col=" << col;
		}
	}

	// --- After graph growth: incremental update vs full rebuild, same result.
	const std::pair<std::map<int, float>, cv::Mat> incremental =
			computePosteriorAfterGrowingChain(4, false);
	const std::pair<std::map<int, float>, cv::Mat> full =
			computePosteriorAfterGrowingChain(4, true);

	ASSERT_EQ(incremental.first.size(), full.first.size());
	ASSERT_EQ(incremental.second.rows, full.second.rows);
	ASSERT_EQ(incremental.second.cols, full.second.cols);

	for(std::map<int, float>::const_iterator it = incremental.first.begin();
		it != incremental.first.end();
		++it)
	{
		ASSERT_TRUE(full.first.count(it->first) > 0);
		EXPECT_NEAR(it->second, full.first.at(it->first), 1e-4f) << "id=" << it->first;
	}

	for(int col = 0; col < incremental.second.cols; ++col)
	{
		for(int row = 0; row < incremental.second.rows; ++row)
		{
			EXPECT_NEAR(predictionAt(incremental.second, row, col),
			            predictionAt(full.second, row, col), 1e-4f)
				<< "row=" << row << " col=" << col;
		}
	}
}

TEST_F(BayesFilterMemoryFixture, FullPredictionUpdateRegeneratesMatrix)
{
	addChain(4);

	ParametersMap params;
	params.insert(ParametersPair(Parameters::kBayesPredictionLC(), kPredictionNewPlace10Stay70Neighbor20));

	auto expectMatrixGrowsOnNewNode = [&](bool fullPredictionUpdate) {
		params[Parameters::kBayesFullPredictionUpdate()] = fullPredictionUpdate ? "true" : "false";
		BayesFilter filter(params);

		const std::vector<int> ids = getBayesIds();
		std::map<int, float> likelihood = uniformLikelihood(ids);
		filter.computePosterior(memory_, likelihood);

		const cv::Mat predictionBefore = filter.generatePrediction(memory_, ids);

		SensorData data(image_);
		Transform pose(10.0f, 0.0f, 0.0f, 0, 0, 0);
		ASSERT_TRUE(memory_->update(data, pose, covariance_));

		const std::vector<int> idsAfter = getBayesIds();
		ASSERT_GT(idsAfter.size(), ids.size());

		likelihood = uniformLikelihood(idsAfter);
		filter.computePosterior(memory_, likelihood);

		const cv::Mat predictionAfter = filter.generatePrediction(memory_, idsAfter);
		EXPECT_EQ(predictionAfter.rows, (int)idsAfter.size());
		EXPECT_EQ(predictionAfter.cols, (int)idsAfter.size());
		EXPECT_NE(predictionBefore.rows, predictionAfter.rows);
	};

	expectMatrixGrowsOnNewNode(true);
	expectMatrixGrowsOnNewNode(false);
}
