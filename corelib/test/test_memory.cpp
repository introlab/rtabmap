#include <gtest/gtest.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include "TestUtils.h"
#include <cstdio>
#include <string>
#include <vector>

using namespace rtabmap;

namespace {

// Build a Memory that does not extract visual features (kKpMaxFeatures=-1).
// Tests focus on Memory's graph/STM/WM bookkeeping, not on Feature2D, VWDictionary
// or Registration, which have their own dedicated tests.
ParametersMap defaultMemoryParams(int stmSize = 5)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
	params.insert(ParametersPair(Parameters::kMemSTMSize(), uNumber2Str(stmSize).c_str()));
	params.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // disable rehearsal merge
	params.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	params.insert(ParametersPair(Parameters::kMemBadSignaturesIgnored(), "false"));
	return params;
}

std::string uniqueDbPath()
{
	static int counter = 0;
	return test::tempPath(uFormat("rtabmap_memory_test_%d_%d.db", test::getPid(), ++counter));
}

class MemoryFixture : public ::testing::Test
{
protected:
	void SetUp() override
	{
		image_ = cv::Mat(8, 8, CV_8UC1, cv::Scalar(128));
		covariance_ = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		memory_ = new Memory(defaultMemoryParams());
	}

	void TearDown() override
	{
		delete memory_;
		memory_ = nullptr;
	}

	void reinit(const ParametersMap & params)
	{
		delete memory_;
		memory_ = new Memory(params);
	}

	bool update(int /*idHint*/ = 0)
	{
		SensorData data(image_);
		Transform pose(float(updateCount_), 0.0f, 0.0f, 0, 0, 0);
		bool ok = memory_->update(data, pose, covariance_);
		++updateCount_;
		return ok;
	}

	bool updateWith(const Transform & pose)
	{
		SensorData data(image_);
		bool ok = memory_->update(data, pose, covariance_);
		++updateCount_;
		return ok;
	}

	cv::Mat image_;
	cv::Mat covariance_;
	Memory * memory_ = nullptr;
	int updateCount_ = 0;
};

} // namespace

// ---------------------------------------------------------------------------
// Constructor / parseParameters
// ---------------------------------------------------------------------------

TEST(MemoryTest, DefaultConstructor)
{
	Memory memory;
	EXPECT_EQ(memory.getStMem().size(), 0u);
	EXPECT_EQ(memory.getWorkingMem().size(), 0u);
	EXPECT_FALSE(memory.memoryChanged());
	EXPECT_EQ(memory.getLastSignatureId(), 0);
	EXPECT_EQ(memory.getMaxStMemSize(), (int)Parameters::defaultMemSTMSize());
	EXPECT_TRUE(memory.isIncremental());
}

TEST(MemoryTest, ParameterizedConstructor)
{
	ParametersMap params;
	params.insert(ParametersPair(Parameters::kMemSTMSize(), "3"));
	params.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
	params.insert(ParametersPair(Parameters::kMemLocalizationDataSaved(), "true"));
	Memory memory(params);

	EXPECT_EQ(memory.getMaxStMemSize(), 3);
	EXPECT_FALSE(memory.isIncremental());
	EXPECT_TRUE(memory.isLocalizationDataSaved());
}

TEST(MemoryTest, ConstantIds)
{
	// kIdStart is the seed used by Memory::initCountId(); first generated id is kIdStart + 1.
	// kIdVirtual is the negative id used by the Bayes filter for the "new place" hypothesis.
	// kIdInvalid is the sentinel returned when no id is available.
	EXPECT_EQ(Memory::kIdStart, 0);
	EXPECT_EQ(Memory::kIdVirtual, -1);
	EXPECT_EQ(Memory::kIdInvalid, 0);
}

TEST(MemoryTest, ParseParametersUpdatesSettings)
{
	Memory memory(defaultMemoryParams(3));
	EXPECT_EQ(memory.getMaxStMemSize(), 3);

	ParametersMap newParams;
	newParams.insert(ParametersPair(Parameters::kMemSTMSize(), "7"));
	memory.parseParameters(newParams);
	EXPECT_EQ(memory.getMaxStMemSize(), 7);
}

TEST(MemoryTest, GetParametersReflectsLastApplied)
{
	ParametersMap params = defaultMemoryParams(4);
	Memory memory(params);
	const ParametersMap & got = memory.getParameters();
	ASSERT_TRUE(got.count(Parameters::kMemSTMSize()) > 0);
	EXPECT_EQ(got.at(Parameters::kMemSTMSize()), std::string("4"));
}

// ---------------------------------------------------------------------------
// init / close
// ---------------------------------------------------------------------------

TEST(MemoryTest, InitInMemoryDatabase)
{
	Memory memory(defaultMemoryParams());
	EXPECT_TRUE(memory.init("")); // empty url == in-memory
	EXPECT_NE(memory.getDatabaseVersion(), std::string(""));
	memory.close(false);
}

TEST(MemoryTest, InitFileDatabaseCreatesFile)
{
	const std::string dbPath = uniqueDbPath();
	{
		Memory memory(defaultMemoryParams());
		ASSERT_TRUE(memory.init(dbPath, true));
		EXPECT_EQ(memory.getDatabaseUrl(), dbPath);
		memory.close(true);
	}
	EXPECT_TRUE(UFile::exists(dbPath.c_str()));
	UFile::erase(dbPath.c_str());
}

TEST(MemoryTest, CloseWithoutInitIsSafe)
{
	Memory memory(defaultMemoryParams());
	// Should not crash without init().
	memory.close(false);
}

// ---------------------------------------------------------------------------
// update() / STM / WM
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, UpdateAddsSignatureToStm)
{
	ASSERT_TRUE(update());
	EXPECT_EQ(memory_->getStMem().size(), 1u);
	EXPECT_EQ(memory_->getWorkingMem().size(), 0u);
	const int id = memory_->getLastSignatureId();
	// First generated id is kIdStart + 1 = 1.
	EXPECT_EQ(id, Memory::kIdStart + 1);
	EXPECT_TRUE(memory_->isInSTM(id));
	EXPECT_FALSE(memory_->isInWM(id));
	EXPECT_FALSE(memory_->isInLTM(id));
}

TEST_F(MemoryFixture, UpdateAssignsSequentialIds)
{
	ASSERT_TRUE(update());
	const int id1 = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int id2 = memory_->getLastSignatureId();
	EXPECT_EQ(id2, id1 + 1);
}

TEST_F(MemoryFixture, StmCapacityPromotesOldestToWm)
{
	const int stmSize = memory_->getMaxStMemSize(); // 5
	std::vector<int> ids;
	for(int i = 0; i < stmSize + 2; ++i)
	{
		ASSERT_TRUE(update());
		ids.push_back(memory_->getLastSignatureId());
	}

	// Only the last stmSize ids remain in STM.
	EXPECT_EQ((int)memory_->getStMem().size(), stmSize);
	// The 2 oldest are in WM.
	for(int i = 0; i < 2; ++i)
	{
		EXPECT_TRUE(memory_->isInWM(ids[i])) << "id=" << ids[i];
		EXPECT_FALSE(memory_->isInSTM(ids[i]));
	}
	// The newest stmSize are in STM.
	for(int i = 2; i < stmSize + 2; ++i)
	{
		EXPECT_TRUE(memory_->isInSTM(ids[i])) << "id=" << ids[i];
		EXPECT_FALSE(memory_->isInWM(ids[i]));
	}
}

TEST_F(MemoryFixture, NeighborLinkConnectsConsecutiveSignatures)
{
	ASSERT_TRUE(update());
	const int id1 = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int id2 = memory_->getLastSignatureId();

	std::multimap<int, Link> links = memory_->getNeighborLinks(id2);
	ASSERT_EQ(links.size(), 1u);
	EXPECT_EQ(links.begin()->first, id1);
	EXPECT_EQ(links.begin()->second.type(), Link::kNeighbor);
}

TEST_F(MemoryFixture, GetSignatureReturnsLoadedSignature)
{
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	const Signature * s = memory_->getSignature(id);
	ASSERT_NE(s, nullptr);
	EXPECT_EQ(s->id(), id);
}

TEST_F(MemoryFixture, GetSignatureReturnsNullForUnknownId)
{
	EXPECT_EQ(memory_->getSignature(999), nullptr);
}

TEST_F(MemoryFixture, GetLastWorkingSignatureSkipsIntermediate)
{
	ASSERT_TRUE(update());
	const int id1 = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int id2 = memory_->getLastSignatureId();
	memory_->convertToIntermediate(id2);

	const Signature * last = memory_->getLastWorkingSignature(true);
	ASSERT_NE(last, nullptr);
	EXPECT_EQ(last->id(), id1);

	const Signature * lastIncludingIntermediate = memory_->getLastWorkingSignature(false);
	ASSERT_NE(lastIncludingIntermediate, nullptr);
	EXPECT_EQ(lastIncludingIntermediate->id(), id2);
}

TEST_F(MemoryFixture, AllSignatureIdsReturnsLoadedIds)
{
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	EXPECT_EQ(ids.size(), 3u);
}

// ---------------------------------------------------------------------------
// memoryChanged / mode flags
// ---------------------------------------------------------------------------

TEST(MemoryTest, MemoryChangedInitiallyFalse)
{
	Memory memory(defaultMemoryParams());
	EXPECT_FALSE(memory.memoryChanged());
}

TEST_F(MemoryFixture, MemoryChangedTrueAfterUpdateInMappingMode)
{
	ASSERT_TRUE(update());
	EXPECT_TRUE(memory_->memoryChanged());
}

TEST(MemoryTest, MemoryChangedRemainsFalseInPureLocalizationMode)
{
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemIncrementalMemory()] = "false";
	params[Parameters::kMemLocalizationDataSaved()] = "false";
	Memory memory(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance);
	EXPECT_FALSE(memory.memoryChanged());
}

TEST(MemoryTest, IsIncrementalFromParameter)
{
	ParametersMap mapping = defaultMemoryParams();
	mapping[Parameters::kMemIncrementalMemory()] = "true";
	EXPECT_TRUE(Memory(mapping).isIncremental());

	ParametersMap loc = defaultMemoryParams();
	loc[Parameters::kMemIncrementalMemory()] = "false";
	EXPECT_FALSE(Memory(loc).isIncremental());
}

TEST(MemoryTest, IsReadOnlyOnlyInLocalizationMode)
{
	ParametersMap mapping = defaultMemoryParams();
	mapping[Parameters::kMemIncrementalMemory()] = "true";
	mapping[Parameters::kMemLocalizationReadOnly()] = "true"; // ignored in mapping mode
	EXPECT_FALSE(Memory(mapping).isReadOnly());

	ParametersMap localizationRW = defaultMemoryParams();
	localizationRW[Parameters::kMemIncrementalMemory()] = "false";
	localizationRW[Parameters::kMemLocalizationReadOnly()] = "false";
	EXPECT_FALSE(Memory(localizationRW).isReadOnly());

	ParametersMap localizationRO = defaultMemoryParams();
	localizationRO[Parameters::kMemIncrementalMemory()] = "false";
	localizationRO[Parameters::kMemLocalizationReadOnly()] = "true";
	EXPECT_TRUE(Memory(localizationRO).isReadOnly());
}

// ---------------------------------------------------------------------------
// Links
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, AddLinkBetweenWmNodes)
{
	for(int i = 0; i < 4; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	ASSERT_EQ(ids.size(), 4u);
	int idA = *ids.begin();
	int idB = *ids.rbegin();

	Link link(idA, idB, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1));
	EXPECT_TRUE(memory_->addLink(link));

	std::multimap<int, Link> loops = memory_->getLoopClosureLinks(idA);
	ASSERT_EQ(loops.size(), 1u);
	EXPECT_EQ(loops.begin()->first, idB);
	EXPECT_EQ(loops.begin()->second.type(), Link::kGlobalClosure);
}

TEST_F(MemoryFixture, GetLinksReturnsBothNeighborAndLoop)
{
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	int idA = *ids.begin();
	int idC = *ids.rbegin();
	int idB = *(++ids.begin());

	// idA <-> idB (neighbor, auto-added), idB <-> idC (neighbor, auto-added).
	// Add a loop closure idA <-> idC.
	memory_->addLink(Link(idA, idC, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1)));

	std::multimap<int, Link> allLinks = memory_->getLinks(idA);
	// idA has neighbor link to idB and loop link to idC.
	bool foundNeighbor = false, foundLoop = false;
	for(std::multimap<int, Link>::iterator it = allLinks.begin(); it != allLinks.end(); ++it)
	{
		if(it->second.type() == Link::kNeighbor && it->first == idB) foundNeighbor = true;
		if(it->second.type() == Link::kGlobalClosure && it->first == idC) foundLoop = true;
	}
	EXPECT_TRUE(foundNeighbor);
	EXPECT_TRUE(foundLoop);
}

TEST_F(MemoryFixture, UpdateLinkChangesTransform)
{
	for(int i = 0; i < 2; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	int idA = *ids.begin();
	int idB = *ids.rbegin();

	memory_->addLink(Link(idA, idB, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1)));

	Transform updated(2.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	memory_->updateLink(Link(idA, idB, Link::kGlobalClosure, updated, cv::Mat::eye(6, 6, CV_64FC1)));

	std::multimap<int, Link> loops = memory_->getLoopClosureLinks(idA);
	ASSERT_EQ(loops.size(), 1u);
	EXPECT_NEAR(loops.begin()->second.transform().x(), 2.5f, 1e-5f);
}

TEST_F(MemoryFixture, RemoveLinkBreaksBothDirections)
{
	for(int i = 0; i < 2; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	int idA = *ids.begin();
	int idB = *ids.rbegin();

	// Neighbor link was added implicitly; remove it.
	memory_->removeLink(idA, idB);
	EXPECT_EQ(memory_->getNeighborLinks(idA).size(), 0u);
	EXPECT_EQ(memory_->getNeighborLinks(idB).size(), 0u);
}

TEST_F(MemoryFixture, RemoveAllVirtualLinks)
{
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	int idA = *ids.begin();
	int idC = *ids.rbegin();

	memory_->addLink(Link(idA, idC, Link::kVirtualClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1)));
	std::multimap<int, Link> linksBefore = memory_->getLinks(idA);
	bool hadVirtual = false;
	for(auto & p: linksBefore) { if(p.second.type() == Link::kVirtualClosure) hadVirtual = true; }
	ASSERT_TRUE(hadVirtual);

	memory_->removeAllVirtualLinks();

	std::multimap<int, Link> linksAfter = memory_->getLinks(idA);
	for(auto & p: linksAfter)
	{
		EXPECT_NE(p.second.type(), Link::kVirtualClosure);
	}
}

// ---------------------------------------------------------------------------
// Graph queries (getNeighborsId)
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetNeighborsIdLinearChain)
{
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	const int start = *ids.begin();

	// getNeighborsId loops while m < maxGraphDepth, so depth=N walks to depth N-1.
	// depth=3 from a chain start visits {start (m=0), neighbor (m=1), 2nd neighbor (m=2)}.
	std::map<int, int> neighbors = memory_->getNeighborsId(start, 3);
	EXPECT_EQ(neighbors.size(), 3u);
	EXPECT_EQ(neighbors.at(start), 0);
}

TEST_F(MemoryFixture, GetNeighborsIdIntermediateNodesAreTransparentDepthWise)
{
	// Build a 5-node chain {A, B, C, D, E} and mark C (the middle node) as intermediate.
	// Intermediate nodes do not consume a depth level when traversed: their neighbors are
	// pushed to the *current* margin, and intermediates themselves are reported with
	// effectiveMargin = m - 1. As a result, starting at A with depth=3 reaches D (which
	// would normally sit at margin 3, i.e. outside the bound) because C "collapses".
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	std::vector<int> chain(ids.begin(), ids.end()); // sorted ascending = chain order
	ASSERT_EQ(chain.size(), 5u);
	const int A = chain[0];
	const int B = chain[1];
	const int C = chain[2];
	const int D = chain[3];
	const int E = chain[4];

	memory_->convertToIntermediate(C);
	ASSERT_EQ(memory_->getSignature(C)->getWeight(), -1);

	// Default (ignoreIntermediateNodes=false): C is included, with effective margin = m-1
	// (so margin 1 instead of 2). D is reached at margin 2. E is still outside.
	std::map<int, int> withIntermediate = memory_->getNeighborsId(
			A,
			/*maxGraphDepth=*/3,
			/*maxCheckedInDatabase=*/0,
			/*incrementMarginOnLoop=*/false,
			/*ignoreLoopIds=*/false,
			/*ignoreIntermediateNodes=*/false);
	EXPECT_EQ(withIntermediate.size(), 4u);
	EXPECT_EQ(withIntermediate.at(A), 0);
	EXPECT_EQ(withIntermediate.at(B), 1);
	EXPECT_EQ(withIntermediate.at(C), 1); // effective margin (would be 2 without the -1 rule)
	EXPECT_EQ(withIntermediate.at(D), 2);
	EXPECT_TRUE(withIntermediate.find(E) == withIntermediate.end());

	// ignoreIntermediateNodes=true: C is dropped from the result, but traversal still
	// "passes through" it, so D is reached and E remains outside.
	std::map<int, int> ignoringIntermediate = memory_->getNeighborsId(
			A,
			/*maxGraphDepth=*/3,
			/*maxCheckedInDatabase=*/0,
			/*incrementMarginOnLoop=*/false,
			/*ignoreLoopIds=*/false,
			/*ignoreIntermediateNodes=*/true);
	EXPECT_EQ(ignoringIntermediate.size(), 3u);
	EXPECT_EQ(ignoringIntermediate.at(A), 0);
	EXPECT_EQ(ignoringIntermediate.at(B), 1);
	EXPECT_TRUE(ignoringIntermediate.find(C) == ignoringIntermediate.end());
	EXPECT_EQ(ignoringIntermediate.at(D), 2);
	EXPECT_TRUE(ignoringIntermediate.find(E) == ignoringIntermediate.end());

	// Sanity check: without any intermediate node, the same depth=3 walk would stop at C
	// (margin 2) and never see D. We verify this by restoring a fresh chain and replaying.
	reinit(defaultMemoryParams(/*stmSize=*/5));
	updateCount_ = 0;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> plainIds = memory_->getAllSignatureIds();
	std::vector<int> plainChain(plainIds.begin(), plainIds.end());
	const int plainA = plainChain[0];
	const int plainC = plainChain[2];
	const int plainD = plainChain[3];
	std::map<int, int> plain = memory_->getNeighborsId(plainA, 3);
	EXPECT_EQ(plain.size(), 3u);
	EXPECT_EQ(plain.at(plainC), 2);
	EXPECT_TRUE(plain.find(plainD) == plain.end());
}

TEST_F(MemoryFixture, GetNeighborsIdDepthZeroMeansUnbounded)
{
	for(int i = 0; i < 4; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	const int start = *ids.begin();

	std::map<int, int> all = memory_->getNeighborsId(start, 0);
	EXPECT_EQ(all.size(), ids.size());
}

TEST_F(MemoryFixture, GetNeighborsIdIgnoreLoopIds)
{
	for(int i = 0; i < 4; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::set<int> ids = memory_->getAllSignatureIds();
	const int idA = *ids.begin();
	const int idLast = *ids.rbegin();
	// Add a loop closure shortcut idA -> idLast.
	memory_->addLink(Link(idA, idLast, Link::kGlobalClosure, Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1)));

	std::map<int, int> withLoops = memory_->getNeighborsId(idA, 1);
	std::map<int, int> withoutLoops = memory_->getNeighborsId(idA, 1, -1, false, true /* ignoreLoopIds */);

	// idLast is reachable via loop link at depth 1 -> present without ignoring loops.
	EXPECT_TRUE(withLoops.count(idLast) > 0);
	// When ignoring loop ids, idLast at depth 1 is no longer reachable.
	EXPECT_TRUE(withoutLoops.count(idLast) == 0);
}

// ---------------------------------------------------------------------------
// Labels
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, LabelSignatureAndLookup)
{
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();

	EXPECT_TRUE(memory_->labelSignature(id, "kitchen"));
	EXPECT_EQ(memory_->getSignatureIdByLabel("kitchen", false), id);

	const std::map<int, std::string> & labels = memory_->getAllLabels();
	ASSERT_EQ(labels.size(), 1u);
	EXPECT_EQ(labels.at(id), std::string("kitchen"));
}

TEST_F(MemoryFixture, LabelSignatureRejectsDuplicate)
{
	ASSERT_TRUE(update());
	const int id1 = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int id2 = memory_->getLastSignatureId();

	ASSERT_TRUE(memory_->labelSignature(id1, "office"));
	// Same label on a different id is rejected.
	EXPECT_FALSE(memory_->labelSignature(id2, "office"));
}

TEST_F(MemoryFixture, LabelSignatureRemoveByEmptyString)
{
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	memory_->labelSignature(id, "lobby");
	ASSERT_EQ(memory_->getAllLabels().size(), 1u);

	EXPECT_TRUE(memory_->labelSignature(id, ""));
	EXPECT_EQ(memory_->getAllLabels().size(), 0u);
}

// ---------------------------------------------------------------------------
// Intermediate nodes / deletion
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, ConvertToIntermediateSetsWeightMinusOne)
{
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	memory_->convertToIntermediate(id);
	const Signature * s = memory_->getSignature(id);
	ASSERT_NE(s, nullptr);
	EXPECT_EQ(s->getWeight(), -1);
}

TEST(MemoryTest, ConvertToIntermediateClearsAllPayloadsByDefault)
{
	// kMemBinDataKept=true so update() leaves compressed binary data in the signature
	// (with default kMemBinDataKept=false the fixture has already cleared it).
	// kMemIntermediateNodeDataKept=false (default) so convertToIntermediate() must also
	// wipe words, global descriptors and compressed binary data.
	ParametersMap params = defaultMemoryParams(5);
	params[Parameters::kMemBinDataKept()] = "true";

	Memory memory(params);
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	const Signature * sConst = memory.getSignature(id);
	ASSERT_NE(sConst, nullptr);

	// Inject synthetic features, words and a global descriptor via a writable handle on
	// the signature, so we can verify the cleanup branches that fire when these payloads
	// are present. Memory keeps internal Signature* pointers; const_cast is safe here
	// because the storage is the same object exposed by getSignature().
	Signature * s = const_cast<Signature *>(sConst);

	std::vector<cv::KeyPoint> kpts(2, cv::KeyPoint(1.f, 1.f, 1.f));
	std::vector<cv::Point3f> pts3(2, cv::Point3f(0.f, 0.f, 1.f));
	cv::Mat descriptors = cv::Mat::ones(2, 8, CV_32F);
	s->sensorData().setFeatures(kpts, pts3, descriptors);

	std::multimap<int, int> wordIds;
	wordIds.insert(std::make_pair(11, 0));
	wordIds.insert(std::make_pair(22, 1));
	s->setWords(wordIds, kpts, pts3, descriptors);

	s->sensorData().addGlobalDescriptor(GlobalDescriptor(0, cv::Mat::ones(1, 4, CV_32F)));

	// Sanity-check the pre-state: 2 keypoints / 3D points / descriptors / words, 1
	// global descriptor, and a single compressed image row from update().
	ASSERT_EQ(s->sensorData().imageCompressed().rows, 1)
			<< "update() should have produced compressed binary data";
	ASSERT_EQ(s->sensorData().keypoints().size(), 2u);
	ASSERT_EQ(s->sensorData().descriptors().rows, 2);
	ASSERT_EQ(s->getWords().size(), 2u);
	ASSERT_EQ(s->getWordsKpts().size(), 2u);
	ASSERT_EQ(s->getWords3().size(), 2u);
	ASSERT_EQ(s->getWordsDescriptors().rows, 2);
	ASSERT_EQ(s->sensorData().globalDescriptors().size(), 1u);

	memory.convertToIntermediate(id);

	// Re-fetch after conversion (same Signature* in practice, but re-look up for clarity).
	const Signature * after = memory.getSignature(id);
	ASSERT_NE(after, nullptr);

	// 1) Weight is the intermediate sentinel.
	EXPECT_EQ(after->getWeight(), -1);

	// 2) sensorData().setFeatures(empty, empty, cv::Mat()) -> features wiped.
	EXPECT_EQ(after->sensorData().keypoints().size(), 0u);
	EXPECT_EQ(after->sensorData().keypoints3D().size(), 0u);
	EXPECT_EQ(after->sensorData().descriptors().rows, 0);

	// 3) Visual words / word kpts / 3D word points / word descriptors wiped
	//    (removeAllWords() because kMemIntermediateNodeDataKept=false).
	EXPECT_EQ(after->getWords().size(), 0u);
	EXPECT_EQ(after->getWordsKpts().size(), 0u);
	EXPECT_EQ(after->getWords3().size(), 0u);
	EXPECT_EQ(after->getWordsDescriptors().rows, 0);

	// 4) Global descriptors wiped (clearGlobalDescriptors() because intermediate data not kept).
	EXPECT_EQ(after->sensorData().globalDescriptors().size(), 0u);

	// 5) Raw RGB/depth/scan wiped (clearRawData(), always).
	EXPECT_EQ(after->sensorData().imageRaw().rows, 0);
	EXPECT_EQ(after->sensorData().depthOrRightRaw().rows, 0);
	EXPECT_TRUE(after->sensorData().laserScanRaw().size() == 0);

	// 6) Compressed RGB/depth/scan wiped (clearCompressedData() because !binDataKept).
	EXPECT_EQ(after->sensorData().imageCompressed().rows, 0);
	EXPECT_EQ(after->sensorData().depthOrRightCompressed().rows, 0);
	EXPECT_TRUE(after->sensorData().laserScanCompressed().size() == 0);
}

TEST(MemoryTest, ConvertToIntermediateClearsAllPayloadsWhenBothFlagsFalse)
{
	// kMemBinDataKept=false AND kMemIntermediateNodeDataKept=false (the fixture default).
	// In this config update() already discards compressed binary data, so we re-inject
	// compressed bytes into the signature's SensorData to exercise the clearCompressedData()
	// branch of convertToIntermediate() rather than have it be a trivial no-op.
	ParametersMap params = defaultMemoryParams(5);
	params[Parameters::kMemBinDataKept()] = "false";
	// kMemIntermediateNodeDataKept already defaults to false.

	Memory memory(params);
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	Signature * s = const_cast<Signature *>(memory.getSignature(id));
	ASSERT_NE(s, nullptr);

	// After update() with binDataKept=false, the compressed image is already empty.
	ASSERT_EQ(s->sensorData().imageCompressed().rows, 0);

	// Re-inject compressed image bytes by feeding a 1xN CV_8UC1 row matrix to setRGBDImage,
	// which is the documented "already-compressed" path for SensorData.
	cv::Mat compressedRgb = compressData2(image);
	ASSERT_EQ(compressedRgb.rows, 1);
	ASSERT_EQ(compressedRgb.type(), CV_8UC1);
	s->sensorData().setRGBDImage(compressedRgb, cv::Mat(), CameraModel(), /*clearPreviousData=*/false);
	ASSERT_EQ(s->sensorData().imageCompressed().rows, 1);

	// Also seed features, words and a global descriptor so every cleanup branch fires.
	std::vector<cv::KeyPoint> kpts(2, cv::KeyPoint(1.f, 1.f, 1.f));
	std::vector<cv::Point3f> pts3(2, cv::Point3f(0.f, 0.f, 1.f));
	cv::Mat descriptors = cv::Mat::ones(2, 8, CV_32F);
	s->sensorData().setFeatures(kpts, pts3, descriptors);

	std::multimap<int, int> wordIds;
	wordIds.insert(std::make_pair(11, 0));
	wordIds.insert(std::make_pair(22, 1));
	s->setWords(wordIds, kpts, pts3, descriptors);

	s->sensorData().addGlobalDescriptor(GlobalDescriptor(0, cv::Mat::ones(1, 4, CV_32F)));

	ASSERT_EQ(s->sensorData().keypoints().size(), 2u);
	ASSERT_EQ(s->getWords().size(), 2u);
	ASSERT_EQ(s->sensorData().globalDescriptors().size(), 1u);

	memory.convertToIntermediate(id);

	const Signature * after = memory.getSignature(id);
	ASSERT_NE(after, nullptr);

	// Weight + features always cleared.
	EXPECT_EQ(after->getWeight(), -1);
	EXPECT_EQ(after->sensorData().keypoints().size(), 0u);
	EXPECT_EQ(after->sensorData().keypoints3D().size(), 0u);
	EXPECT_EQ(after->sensorData().descriptors().rows, 0);

	// !_saveIntermediateNodeData -> words + global descriptors cleared.
	EXPECT_EQ(after->getWords().size(), 0u);
	EXPECT_EQ(after->getWordsKpts().size(), 0u);
	EXPECT_EQ(after->getWords3().size(), 0u);
	EXPECT_EQ(after->getWordsDescriptors().rows, 0);
	EXPECT_EQ(after->sensorData().globalDescriptors().size(), 0u);

	// clearRawData() unconditional.
	EXPECT_EQ(after->sensorData().imageRaw().rows, 0);
	EXPECT_EQ(after->sensorData().depthOrRightRaw().rows, 0);
	EXPECT_TRUE(after->sensorData().laserScanRaw().size() == 0);

	// (!_saveIntermediateNodeData || !_binDataKept) -> compressed binary data cleared.
	// Both branches are true here, so the OR is true and the injected compressed image
	// is wiped.
	EXPECT_EQ(after->sensorData().imageCompressed().rows, 0);
	EXPECT_EQ(after->sensorData().depthOrRightCompressed().rows, 0);
	EXPECT_TRUE(after->sensorData().laserScanCompressed().size() == 0);
}

TEST(MemoryTest, ConvertToIntermediateKeepsDataWhenIntermediateNodeDataKept)
{
	// kMemIntermediateNodeDataKept=true: words, global descriptors, and (when binDataKept=true)
	// compressed binary data must survive convertToIntermediate(). Features and raw data are
	// still always wiped because their clearing is unconditional.
	ParametersMap params = defaultMemoryParams(5);
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kMemIntermediateNodeDataKept()] = "true";

	Memory memory(params);
	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	Signature * s = const_cast<Signature *>(memory.getSignature(id));
	ASSERT_NE(s, nullptr);

	std::vector<cv::KeyPoint> kpts(2, cv::KeyPoint(1.f, 1.f, 1.f));
	std::vector<cv::Point3f> pts3(2, cv::Point3f(0.f, 0.f, 1.f));
	cv::Mat descriptors = cv::Mat::ones(2, 8, CV_32F);
	std::multimap<int, int> wordIds;
	wordIds.insert(std::make_pair(11, 0));
	wordIds.insert(std::make_pair(22, 1));
	s->sensorData().setFeatures(kpts, pts3, descriptors);
	s->setWords(wordIds, kpts, pts3, descriptors);
	s->sensorData().addGlobalDescriptor(GlobalDescriptor(0, cv::Mat::ones(1, 4, CV_32F)));

	ASSERT_EQ(s->sensorData().imageCompressed().rows, 1);

	memory.convertToIntermediate(id);

	const Signature * after = memory.getSignature(id);
	ASSERT_NE(after, nullptr);

	// Always-wiped: weight, features, raw data.
	EXPECT_EQ(after->getWeight(), -1);
	EXPECT_EQ(after->sensorData().keypoints().size(), 0u);
	EXPECT_EQ(after->sensorData().keypoints3D().size(), 0u);
	EXPECT_EQ(after->sensorData().descriptors().rows, 0);
	EXPECT_EQ(after->sensorData().imageRaw().rows, 0);

	// Conditionally kept: words, global descriptors, compressed binary data.
	EXPECT_EQ(after->getWords().size(), 2u);
	EXPECT_EQ(after->getWordsDescriptors().rows, 2);
	EXPECT_EQ(after->sensorData().globalDescriptors().size(), 1u);
	EXPECT_EQ(after->sensorData().imageCompressed().rows, 1);
}

TEST_F(MemoryFixture, DeleteLocationRemovesFromWorkingMem)
{
	for(int i = 0; i < 2; ++i)
	{
		ASSERT_TRUE(update());
	}
	const int id = memory_->getLastSignatureId();
	memory_->deleteLocation(id);
	EXPECT_EQ(memory_->getSignature(id), nullptr);
	EXPECT_FALSE(memory_->isInSTM(id));
	EXPECT_FALSE(memory_->isInWM(id));
}

// ---------------------------------------------------------------------------
// moveToTrash (exercised through deleteLocation, which calls
// moveToTrash(s, /*keepLinkedToGraph=*/false, deletedWords))
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, MoveToTrashBreaksNeighborLinksOnPeers)
{
	// Build a chain A-B-C. deleteLocation(B) calls moveToTrash with keepLinkedToGraph=false,
	// which iterates B's links and calls sTo->removeLink(B) on every peer, so A and C
	// must lose their kNeighbor links to B.
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	const std::set<int> idsSet = memory_->getAllSignatureIds();
	std::vector<int> ids(idsSet.begin(), idsSet.end());
	const int A = ids[0];
	const int B = ids[1];
	const int C = ids[2];

	const Signature * aS = memory_->getSignature(A);
	const Signature * cS = memory_->getSignature(C);
	ASSERT_TRUE(aS->hasLink(B));
	ASSERT_TRUE(cS->hasLink(B));

	memory_->deleteLocation(B);

	EXPECT_EQ(memory_->getSignature(B), nullptr);
	aS = memory_->getSignature(A);
	cS = memory_->getSignature(C);
	ASSERT_NE(aS, nullptr);
	ASSERT_NE(cS, nullptr);
	EXPECT_FALSE(aS->hasLink(B));
	EXPECT_FALSE(cS->hasLink(B));
}

TEST_F(MemoryFixture, MoveToTrashTransfersGlobalLoopClosureWeight)
{
	// When a removed signature has a kGlobalClosure link to a peer and a positive
	// weight, moveToTrash copies that weight to the peer (kGlobalClosure branch in
	// the link-cleanup loop of Memory::moveToTrash).
	// Use a 3-node chain so the loop closure is added between non-adjacent nodes A
	// and C (otherwise addLink early-returns because the endpoints are already linked
	// by the kNeighbor edge from update()).
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	const std::set<int> idsSet = memory_->getAllSignatureIds();
	std::vector<int> ids(idsSet.begin(), idsSet.end());
	const int A = ids[0];
	const int C = ids[2];

	{
		Signature * cS = const_cast<Signature *>(memory_->getSignature(C));
		ASSERT_NE(cS, nullptr);
		cS->setWeight(5);
	}

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory_->addLink(Link(C, A, Link::kGlobalClosure, Transform::getIdentity(), infMatrix)));

	// After addLink with default kRGBDReduceGraph=false: max(A=1, C=3)=C keeps its
	// weight; the other endpoint is zeroed (it was already 0 here).
	EXPECT_EQ(memory_->getSignature(C)->getWeight(), 5);
	EXPECT_EQ(memory_->getSignature(A)->getWeight(), 0);

	memory_->deleteLocation(C);

	// A absorbs C's weight via the kGlobalClosure branch in moveToTrash.
	const Signature * aS = memory_->getSignature(A);
	ASSERT_NE(aS, nullptr);
	EXPECT_EQ(aS->getWeight(), 5);
}

TEST_F(MemoryFixture, MoveToTrashCleansLandmarkIndexWhenLastObserver)
{
	// B is the only observer of a landmark. Removing B must drop the landmark id from
	// _landmarksIndex entirely (the entry is erased once the observer set becomes empty).
	// _landmarksIndex is populated by Memory::createSignature when SensorData carries
	// a Landmarks observation, so we attach the landmark through update().
	const int landmarkPositiveId = 7;
	const int landmarkInternalId = -landmarkPositiveId; // Memory stores landmark ids as negative
	const cv::Mat lmCov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	SensorData data(image_);
	Landmarks landmarks;
	landmarks.insert(std::make_pair(
			landmarkPositiveId,
			Landmark(landmarkPositiveId, 0.1f, Transform(1.0f, 0, 0, 0, 0, 0), lmCov)));
	data.setLandmarks(landmarks);
	Transform pose(0.0f, 0.0f, 0.0f, 0, 0, 0);
	ASSERT_TRUE(memory_->update(data, pose, covariance_));
	const int B = memory_->getLastSignatureId();

	// Pre-state: index has the landmark with B as sole observer.
	const std::map<int, std::set<int> > & idxBefore = memory_->getLandmarksIndex();
	ASSERT_EQ(idxBefore.count(landmarkInternalId), 1u);
	ASSERT_EQ(idxBefore.at(landmarkInternalId).size(), 1u);
	ASSERT_EQ(idxBefore.at(landmarkInternalId).count(B), 1u);

	memory_->deleteLocation(B);

	// Post-state: the landmark id is gone from the index entirely (set became empty).
	const std::map<int, std::set<int> > & idxAfter = memory_->getLandmarksIndex();
	EXPECT_EQ(idxAfter.count(landmarkInternalId), 0u);
}

TEST_F(MemoryFixture, MoveToTrashPreservesLandmarkIndexWhenOtherObserverRemains)
{
	// Two signatures observe the same landmark; only B is removed. The landmark id
	// must stay in _landmarksIndex with C still listed in its observer set.
	const int landmarkPositiveId = 8;
	const int landmarkInternalId = -landmarkPositiveId;
	const cv::Mat lmCov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	auto makeDataWithLandmark = [&](float poseX) {
		SensorData data(image_);
		Landmarks landmarks;
		landmarks.insert(std::make_pair(
				landmarkPositiveId,
				Landmark(landmarkPositiveId, 0.1f, Transform(1.0f, 0, 0, 0, 0, 0), lmCov)));
		data.setLandmarks(landmarks);
		Transform pose(poseX, 0.0f, 0.0f, 0, 0, 0);
		ASSERT_TRUE(memory_->update(data, pose, covariance_));
	};

	makeDataWithLandmark(0.0f);
	const int B = memory_->getLastSignatureId();
	makeDataWithLandmark(1.0f);
	const int C = memory_->getLastSignatureId();

	// Pre-state: index has the landmark with both B and C as observers.
	const std::map<int, std::set<int> > & idxBefore = memory_->getLandmarksIndex();
	ASSERT_EQ(idxBefore.count(landmarkInternalId), 1u);
	ASSERT_EQ(idxBefore.at(landmarkInternalId).size(), 2u);
	ASSERT_EQ(idxBefore.at(landmarkInternalId).count(B), 1u);
	ASSERT_EQ(idxBefore.at(landmarkInternalId).count(C), 1u);

	memory_->deleteLocation(B);

	// Post-state: the landmark id is still in the index, but only C remains as observer.
	const std::map<int, std::set<int> > & idxAfter = memory_->getLandmarksIndex();
	ASSERT_EQ(idxAfter.count(landmarkInternalId), 1u);
	EXPECT_EQ(idxAfter.at(landmarkInternalId).size(), 1u);
	EXPECT_EQ(idxAfter.at(landmarkInternalId).count(C), 1u);
	EXPECT_EQ(idxAfter.at(landmarkInternalId).count(B), 0u);

	// C still carries the landmark observation on its own signature.
	const Signature * cS = memory_->getSignature(C);
	ASSERT_NE(cS, nullptr);
	EXPECT_EQ(cS->getLandmarks().count(landmarkInternalId), 1u);
}

TEST_F(MemoryFixture, MoveToTrashReassignsLastWorkingSignature)
{
	// After update(), _lastSignature points to the just-added node. moveToTrash on the
	// last signature must reassign _lastSignature to the next-most-recent member of
	// STM (or WM if STM is empty).
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	const int lastId = memory_->getLastSignatureId();
	const std::set<int> idsSet = memory_->getAllSignatureIds();
	std::vector<int> ids(idsSet.begin(), idsSet.end());
	const int secondLastId = ids[ids.size() - 2];

	ASSERT_EQ(memory_->getLastWorkingSignature(false)->id(), lastId);

	memory_->deleteLocation(lastId);

	const Signature * newLast = memory_->getLastWorkingSignature(false);
	ASSERT_NE(newLast, nullptr);
	EXPECT_EQ(newLast->id(), secondLastId);
}

TEST_F(MemoryFixture, MoveToTrashResetsLastGlobalLoopClosureId)
{
	// addLink(kGlobalClosure) sets _lastGlobalLoopClosureId to max(from, to). Removing
	// that node must reset _lastGlobalLoopClosureId back to 0. Loop closure is added
	// between non-adjacent nodes A and C to avoid addLink's early-return on already-linked
	// endpoints.
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	const std::set<int> idsSet = memory_->getAllSignatureIds();
	std::vector<int> ids(idsSet.begin(), idsSet.end());
	const int A = ids[0];
	const int C = ids[2];

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory_->addLink(Link(A, C, Link::kGlobalClosure, Transform::getIdentity(), infMatrix)));
	ASSERT_EQ(memory_->getLastGlobalLoopClosureId(), C);

	memory_->deleteLocation(C);

	EXPECT_EQ(memory_->getLastGlobalLoopClosureId(), 0);
}

TEST_F(MemoryFixture, MoveToTrashIsNoOpOnRepeatedDelete)
{
	// deleteLocation on an id that was already deleted is silently a no-op. The
	// underlying moveToTrash early-returns when its Signature* is null.
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	memory_->deleteLocation(id);
	EXPECT_EQ(memory_->getSignature(id), nullptr);
	// Second call should not crash.
	memory_->deleteLocation(id);
	EXPECT_EQ(memory_->getSignature(id), nullptr);
}

// ---------------------------------------------------------------------------
// reduceNode
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, ReduceNodeReturnsZeroForUnknownId)
{
	// No matching signature in WM/STM.
	EXPECT_EQ(memory_->reduceNode(99999), 0);
}

TEST_F(MemoryFixture, ReduceNodeReturnsZeroForLabeledNode)
{
	// Labeled signatures are explicitly excluded from reduction.
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	ASSERT_TRUE(memory_->labelSignature(id, "anchor"));
	EXPECT_EQ(memory_->reduceNode(id), 0);
	// Signature must still exist.
	EXPECT_NE(memory_->getSignature(id), nullptr);
}

TEST_F(MemoryFixture, ReduceNodeReturnsZeroWhenNoReducibleLink)
{
	// canBeReduced rejects kNeighbor / kNeighborMerged links and self-loops, so a node
	// with only neighbor links to its chain peers has nothing to reduce against.
	for(int i = 0; i < 3; ++i)
	{
		ASSERT_TRUE(update());
	}
	const std::set<int> idsSet = memory_->getAllSignatureIds();
	std::vector<int> ids(idsSet.begin(), idsSet.end());
	const int idMiddle = ids[1];
	EXPECT_EQ(memory_->reduceNode(idMiddle), 0);
	EXPECT_NE(memory_->getSignature(idMiddle), nullptr);
}

TEST(MemoryTest, ReduceNodeMergesAndRemovesNodeViaLoopClosure)
{
	// Build a chain A-B-C-D-E with all five nodes in WM (STM=1 + 6 updates so the
	// first five ids are promoted to WM and only the last one stays in STM). Add a
	// loop closure D->B and call reduceNode(D): it should pick the loop closure as
	// the reduction target, redirect D's neighbor links to its non-already-linked
	// peers as kNeighborMerged links on B, remove D from memory, and return B.
	//
	// Chain neighbor structure:
	//   A=1 -- B=2 -- C=3 -- D=4 -- E=5  (each pair connected by kNeighbor)
	// Loop closure: D=4 -> B=2 (kGlobalClosure).
	// After reduceNode(D):
	//   - D is removed from Memory.
	//   - B's existing neighbor link to C is untouched (no duplicate merge link added).
	//   - B gains a kNeighborMerged link to E (E was previously only linked to D).
	ParametersMap params = defaultMemoryParams(/*stmSize=*/1);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	ASSERT_EQ(ids.size(), 6u);
	const int A = ids[0];
	const int B = ids[1];
	const int C = ids[2];
	const int D = ids[3];
	const int E = ids[4];
	// A, B, C, D, E must all be in WM (only the most recent id stays in STM=1).
	ASSERT_TRUE(memory.isInWM(A));
	ASSERT_TRUE(memory.isInWM(B));
	ASSERT_TRUE(memory.isInWM(C));
	ASSERT_TRUE(memory.isInWM(D));
	ASSERT_TRUE(memory.isInWM(E));

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory.addLink(Link(D, B, Link::kGlobalClosure, Transform(2.0f, 0, 0, 0, 0, 0), infMatrix)));

	const int reducedTo = memory.reduceNode(D);
	EXPECT_EQ(reducedTo, B);

	// D is removed from memory.
	EXPECT_EQ(memory.getSignature(D), nullptr);
	EXPECT_FALSE(memory.isInSTM(D));
	EXPECT_FALSE(memory.isInWM(D));

	// C, B, E are still around. Their previous links to D are gone.
	const Signature * bS = memory.getSignature(B);
	const Signature * cS = memory.getSignature(C);
	const Signature * eS = memory.getSignature(E);
	ASSERT_NE(bS, nullptr);
	ASSERT_NE(cS, nullptr);
	ASSERT_NE(eS, nullptr);
	EXPECT_FALSE(bS->hasLink(D));
	EXPECT_FALSE(cS->hasLink(D));
	EXPECT_FALSE(eS->hasLink(D));

	// B was already connected to C by a kNeighbor link in the chain, so no merge link
	// between B and C is added (the merge step skips pairs already sharing a link).
	EXPECT_TRUE(bS->hasLink(C, Link::kNeighbor));

	// B and E were NOT directly linked before -- E was only connected via D. Reducing
	// D merges D's neighbor link to E onto B as a kNeighborMerged link in both
	// directions.
	EXPECT_TRUE(bS->hasLink(E, Link::kNeighborMerged));
	EXPECT_TRUE(eS->hasLink(B, Link::kNeighborMerged));

	// reduceNode marks the memory + links as changed.
	EXPECT_TRUE(memory.memoryChanged());

	memory.close(false);
}

TEST(MemoryTest, ReduceNodeCreatesTwoMergedLinksWhenNeighborsAreUnlinkedToTarget)
{
	// Extend the chain so there are two nodes between B and D:
	//   A=1 -- B=2 -- M1=3 -- M2=4 -- D=5 -- E=6
	// All six are in WM (STM=1 + 7 updates puts the last id in STM, the first six in WM).
	// Loop closure D=5 -> B=2. D's neighbors are M2 (key=4) and E (key=6). B is *not*
	// directly linked to either, so reduceNode(D) must add TWO kNeighborMerged links:
	//   B <-> M2  and  B <-> E.
	// B's existing kNeighbor links to A and M1 are untouched.
	ParametersMap params = defaultMemoryParams(/*stmSize=*/1);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 7; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	ASSERT_EQ(ids.size(), 7u);
	const int A  = ids[0];
	const int B  = ids[1];
	const int M1 = ids[2];
	const int M2 = ids[3];
	const int D  = ids[4];
	const int E  = ids[5];
	ASSERT_TRUE(memory.isInWM(A));
	ASSERT_TRUE(memory.isInWM(B));
	ASSERT_TRUE(memory.isInWM(M1));
	ASSERT_TRUE(memory.isInWM(M2));
	ASSERT_TRUE(memory.isInWM(D));
	ASSERT_TRUE(memory.isInWM(E));

	// Sanity-check the pre-state: B is linked to A and M1 only; M2 and E are not yet
	// linked to B (only via the chain through D).
	{
		const Signature * bS = memory.getSignature(B);
		ASSERT_NE(bS, nullptr);
		EXPECT_TRUE(bS->hasLink(A, Link::kNeighbor));
		EXPECT_TRUE(bS->hasLink(M1, Link::kNeighbor));
		EXPECT_FALSE(bS->hasLink(M2));
		EXPECT_FALSE(bS->hasLink(E));
		EXPECT_FALSE(bS->hasLink(D));
	}

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory.addLink(Link(D, B, Link::kGlobalClosure, Transform(3.0f, 0, 0, 0, 0, 0), infMatrix)));

	const int reducedTo = memory.reduceNode(D);
	EXPECT_EQ(reducedTo, B);

	// D is gone.
	EXPECT_EQ(memory.getSignature(D), nullptr);

	const Signature * bS = memory.getSignature(B);
	const Signature * m2S = memory.getSignature(M2);
	const Signature * eS = memory.getSignature(E);
	ASSERT_NE(bS, nullptr);
	ASSERT_NE(m2S, nullptr);
	ASSERT_NE(eS, nullptr);

	// Existing kNeighbor links on B (to A and M1) are untouched.
	EXPECT_TRUE(bS->hasLink(A, Link::kNeighbor));
	EXPECT_TRUE(bS->hasLink(M1, Link::kNeighbor));

	// The two new merged links are present on B and mirrored on M2 / E.
	EXPECT_TRUE(bS->hasLink(M2, Link::kNeighborMerged));
	EXPECT_TRUE(bS->hasLink(E, Link::kNeighborMerged));
	EXPECT_TRUE(m2S->hasLink(B, Link::kNeighborMerged));
	EXPECT_TRUE(eS->hasLink(B, Link::kNeighborMerged));

	// Everyone's link to D is gone.
	EXPECT_FALSE(bS->hasLink(D));
	EXPECT_FALSE(m2S->hasLink(D));
	EXPECT_FALSE(eS->hasLink(D));

	// M2 keeps its original kNeighbor link to M1 (untouched by the merge).
	EXPECT_TRUE(m2S->hasLink(M1, Link::kNeighbor));

	memory.close(false);
}

TEST(MemoryTest, ReduceNodePropagatesLandmarkObservationToTarget)
{
	// Chain A=1 -- B=2 -- C=3 -- D=4 -- E=5 -- F=6 all in WM (STM=1 + 7 updates).
	// Poses (world): each node sits at x = its position in the chain. So in particular
	// B=2 is at world x=1.0 and D=4 is at world x=3.0.
	// We attach a landmark observation L to D with T_DL = (2, 0, 0) -- i.e. L is 2m
	// ahead of D in D's local frame, which puts L at world x=5. Then we add a loop
	// closure D->B with relative transform T_DB = (-2, 0, 0) (B is 2m behind D).
	// reduceNode(D) must propagate the landmark observation from D onto B. The new
	// link on B has transform T_BL = inv(T_DB) * T_DL = (2, 0, 0) * (2, 0, 0) = (4, 0, 0),
	// which matches the relative position of L in B's frame: world_L - world_B = 4.
	ParametersMap params = defaultMemoryParams(/*stmSize=*/1);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 7; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	const int B = ids[1];
	const int D = ids[3];
	ASSERT_TRUE(memory.isInWM(B));
	ASSERT_TRUE(memory.isInWM(D));

	// Attach a landmark observation on D directly via the underlying Signature.
	// Landmark ids are negative by convention (see Link::kLandmark contract).
	const int landmarkId = -42;
	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	const Transform T_DL(2.0f, 0, 0, 0, 0, 0);
	{
		Signature * dS = const_cast<Signature *>(memory.getSignature(D));
		ASSERT_NE(dS, nullptr);
		dS->addLandmark(Link(D, landmarkId, Link::kLandmark, T_DL, infMatrix));
	}

	// Loop closure D->B with relative transform T_DB = (-2, 0, 0).
	const Transform T_DB(-2.0f, 0, 0, 0, 0, 0);
	ASSERT_TRUE(memory.addLink(Link(D, B, Link::kGlobalClosure, T_DB, infMatrix)));

	// Pre-state: B has no landmark observation.
	{
		const Signature * bS = memory.getSignature(B);
		ASSERT_NE(bS, nullptr);
		EXPECT_EQ(bS->getLandmarks().count(landmarkId), 0u);
	}

	const int reducedTo = memory.reduceNode(D);
	EXPECT_EQ(reducedTo, B);

	// D is removed, landmark observation is now on B.
	EXPECT_EQ(memory.getSignature(D), nullptr);
	const Signature * bS = memory.getSignature(B);
	ASSERT_NE(bS, nullptr);
	ASSERT_EQ(bS->getLandmarks().count(landmarkId), 1u);

	const Link & bLink = bS->getLandmarks().at(landmarkId);
	EXPECT_EQ(bLink.from(), B);
	EXPECT_EQ(bLink.to(), landmarkId);
	EXPECT_EQ(bLink.type(), Link::kLandmark);
	// Propagated transform: inv(T_DB) * T_DL = (2, 0, 0) * (2, 0, 0) = (4, 0, 0).
	// This matches L's position relative to B (world_L=5 minus world_B=1 = 4).
	EXPECT_NEAR(bLink.transform().x(), 4.0f, 1e-4f);
	EXPECT_NEAR(bLink.transform().y(), 0.0f, 1e-4f);
	EXPECT_NEAR(bLink.transform().z(), 0.0f, 1e-4f);

	// _landmarksIndex now references B for this landmark id.
	const std::map<int, std::set<int> > & idx = memory.getLandmarksIndex();
	const auto it = idx.find(landmarkId);
	ASSERT_NE(it, idx.end());
	EXPECT_EQ(it->second.count(B), 1u);
	// D is no longer in the index for this landmark (moveToTrash cleanup).
	EXPECT_EQ(it->second.count(D), 0u);

	// Landmark propagation only flows along non-neighbor links of the reduced node:
	// the landmark branch in reduceNode is nested under the
	//   `iter->second.type() != Link::kNeighbor && iter->second.type() != Link::kUndef`
	// guard, so D's direct kNeighbor peers (C and E) do NOT inherit the observation
	// and the landmark index does not reference them either.
	const int C = ids[2];
	const int E = ids[4];
	const Signature * cS = memory.getSignature(C);
	const Signature * eS = memory.getSignature(E);
	ASSERT_NE(cS, nullptr);
	ASSERT_NE(eS, nullptr);
	EXPECT_EQ(cS->getLandmarks().count(landmarkId), 0u);
	EXPECT_EQ(eS->getLandmarks().count(landmarkId), 0u);
	EXPECT_EQ(it->second.count(C), 0u);
	EXPECT_EQ(it->second.count(E), 0u);

	memory.close(false);
}

TEST(MemoryTest, ReduceNodeRespectsDirection)
{
	// Same WM setup as ReduceNodeMergesAndRemovesNodeViaLoopClosure (STM=1 + 6 updates,
	// chain A-B-C-D-E in WM). Loop closure D->B has to(B=2) < from(D=4):
	//   - direction=1 requires to > from -> rejected.
	//   - direction=-1 requires to < from -> accepted.
	ParametersMap params = defaultMemoryParams(/*stmSize=*/1);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	const int B = ids[1];
	const int D = ids[3];

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory.addLink(Link(D, B, Link::kGlobalClosure, Transform(2.0f, 0, 0, 0, 0, 0), infMatrix)));

	// direction = 1: D->B has to(B=2) < from(D=4), so not accepted.
	EXPECT_EQ(memory.reduceNode(D, /*maxDistance=*/0.0f, /*keepLinkedInDb=*/false, /*direction=*/1), 0);
	EXPECT_NE(memory.getSignature(D), nullptr);

	// direction = -1: 2 < 4, accepted.
	EXPECT_EQ(memory.reduceNode(D, /*maxDistance=*/0.0f, /*keepLinkedInDb=*/false, /*direction=*/-1), B);
	EXPECT_EQ(memory.getSignature(D), nullptr);

	memory.close(false);
}

TEST(MemoryTest, ReduceNodeRespectsMaxDistance)
{
	// Same WM setup. Loop closure D->B with transform norm 5.0. canBeReduced rejects
	// the link when maxDistance=1.0 (link.transform().getNorm() not < 1.0) and accepts
	// it at maxDistance=10.0.
	ParametersMap params = defaultMemoryParams(/*stmSize=*/1);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	const int B = ids[1];
	const int D = ids[3];

	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	ASSERT_TRUE(memory.addLink(Link(D, B, Link::kGlobalClosure, Transform(5.0f, 0, 0, 0, 0, 0), infMatrix)));

	EXPECT_EQ(memory.reduceNode(D, /*maxDistance=*/1.0f), 0);
	EXPECT_NE(memory.getSignature(D), nullptr);

	EXPECT_EQ(memory.reduceNode(D, /*maxDistance=*/10.0f), B);
	EXPECT_EQ(memory.getSignature(D), nullptr);

	memory.close(false);
}

// ---------------------------------------------------------------------------
// Map id
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, IncrementMapIdReturnsNewMapId)
{
	ASSERT_TRUE(update());
	const int firstMap = memory_->getMapId(memory_->getLastSignatureId());
	EXPECT_EQ(firstMap, 0); // first map id is 0
	const int newMap = memory_->incrementMapId();
	EXPECT_EQ(newMap, firstMap + 1);

	ASSERT_TRUE(update());
	const int secondMap = memory_->getMapId(memory_->getLastSignatureId());
	EXPECT_EQ(secondMap, newMap);
}

// ---------------------------------------------------------------------------
// Forget / reactivate (require a database)
// ---------------------------------------------------------------------------

TEST(MemoryTest, ForgetTransfersOldestWmSignaturesToLtm)
{
	// forget() has two branches:
	//   1) When the visual word dictionary is non-empty and FLANN is not incremental,
	//      it transfers signatures until at least _vwd->getNotIndexedWordsCount() words
	//      are unused.
	//   2) Otherwise (e.g. dictionary empty because no features were extracted, or
	//      _vwd->isIncrementalFlann() is true, or the memory is not incremental), it
	//      transfers _signaturesAdded+1 signatures from the working memory regardless
	//      of words.
	// This test exercises branch (2): kKpMaxFeatures=-1 -> no words at all, so forget()
	// still works and transfers the oldest WM signatures.
	ParametersMap params = defaultMemoryParams(2); // STM=2 so the chain quickly fills WM
	Memory memory(params);
	ASSERT_TRUE(memory.init("")); // in-memory database so moveToTrash() has a destination

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	// With STM=2 and 5 updates, STM holds the 2 newest ids (ids[3], ids[4]) and WM
	// holds the 3 oldest (ids[0], ids[1], ids[2]) plus the virtual place entry that
	// init() pushes in (kIdVirtual). ids[2] has a neighbor link to ids[3] (which is
	// in STM), so getRemovableSignatures() excludes it as a rehearsal safeguard.
	// That leaves ids[0] and ids[1] as the only removable nodes.
	ASSERT_EQ((int)memory.getStMem().size(), 2);
	ASSERT_EQ((int)memory.getWorkingMem().size(), 4); // 3 real + 1 virtual
	ASSERT_TRUE(memory.isInWM(ids[0]));
	ASSERT_TRUE(memory.isInWM(ids[1]));
	ASSERT_TRUE(memory.isInWM(ids[2]));

	std::list<int> transferred = memory.forget(std::set<int>());

	// _signaturesAdded is reset by preUpdate() and incremented in addSignatureToStm,
	// so after a single iteration its value is 1; forget() in branch (2) tries to
	// remove _signaturesAdded+1 = 2 signatures and both ids[0] and ids[1] are
	// removable, so they should both be transferred.
	ASSERT_EQ((int)transferred.size(), 2);
	std::set<int> transferredSet(transferred.begin(), transferred.end());
	EXPECT_TRUE(transferredSet.count(ids[0]) > 0);
	EXPECT_TRUE(transferredSet.count(ids[1]) > 0);

	// ids[0] and ids[1] are no longer loaded in WM/STM. They are now in LTM
	// (persisted by the async db driver).
	EXPECT_FALSE(memory.isInWM(ids[0]));
	EXPECT_FALSE(memory.isInWM(ids[1]));
	EXPECT_TRUE(memory.isInLTM(ids[0]));
	EXPECT_TRUE(memory.isInLTM(ids[1]));

	// ids[2] could not be transferred because it is linked to an STM node, so it
	// is still in WM.
	EXPECT_TRUE(memory.isInWM(ids[2]));
	// STM is untouched.
	EXPECT_TRUE(memory.isInSTM(ids[3]));
	EXPECT_TRUE(memory.isInSTM(ids[4]));

	memory.close(false);
}

TEST(MemoryTest, ForgetTransfersBasedOnWordCountInWordRegime)
{
	// Branch (1) of Memory::forget() is gated on:
	//   isIncremental && _vwd->isIncremental && _vwd->getVisualWords().size() && !_vwd->isIncrementalFlann()
	// To populate the dictionary without running real feature extraction, we set
	// kMemUseOdomFeatures=true and feed pre-baked keypoints+descriptors as part of
	// each SensorData (the "odom features" path in Memory::createSignature). We also
	// disable incremental FLANN so the dictionary indexes words immediately and branch
	// (1) is selected.
	ParametersMap params = defaultMemoryParams(2); // STM=2
	params[Parameters::kKpMaxFeatures()] = "10";
	params[Parameters::kKpIncrementalFlann()] = "false";
	params[Parameters::kMemUseOdomFeatures()] = "true";

	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	const int kKeypointsPerFrame = 5;
	const int kFrames = 5;
	// One extra axis reserved for an additional single-keypoint signature we add after
	// forget() to trigger cleanUnusedWords() (see end of the test).
	const int kDescCols = kFrames * kKeypointsPerFrame + 1;
	auto makeData = [&](int frameIndex) {
		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		SensorData data(image);

		std::vector<cv::KeyPoint> kpts(kKeypointsPerFrame, cv::KeyPoint(1.f, 1.f, 1.f));
		std::vector<cv::Point3f> pts3(kKeypointsPerFrame, cv::Point3f(0.f, 0.f, 1.f));
		// Each (frame, row) gets its own one-hot axis in a kFrames*kKeypointsPerFrame
		// dimensional space, so every keypoint in every frame quantizes to its own
		// brand new visual word. This guarantees notIndexedWordsCount > 0 at the time
		// we call forget(), and keeps the dictionary growing on every iteration.
		cv::Mat descriptors = cv::Mat::zeros(kKeypointsPerFrame, kDescCols, CV_32F);
		for(int row = 0; row < kKeypointsPerFrame; ++row)
		{
			const int axis = frameIndex * kKeypointsPerFrame + row;
			descriptors.at<float>(row, axis) = 1000.0f;
		}
		data.setFeatures(kpts, pts3, descriptors);
		return data;
	};

	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < kFrames; ++i)
	{
		ASSERT_TRUE(memory.update(makeData(i), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}

	// Every (frame, keypoint) generates a brand-new visual word, so after kFrames
	// updates the dictionary holds exactly kFrames * kKeypointsPerFrame words and the
	// last iteration left kKeypointsPerFrame of them pending indexing -- that's the
	// quantity the word-count regime tries to "catch up" with.
	ASSERT_EQ((int)memory.getVWDictionary()->getVisualWords().size(),
			kFrames * kKeypointsPerFrame);
	ASSERT_EQ(memory.getVWDictionary()->getNotIndexedWordsCount(), kKeypointsPerFrame);

	ASSERT_EQ((int)memory.getStMem().size(), 2);
	ASSERT_TRUE(memory.isInWM(ids[0]));
	ASSERT_TRUE(memory.isInWM(ids[1]));
	ASSERT_TRUE(memory.isInWM(ids[2]));

	const size_t wordsBefore = memory.getVWDictionary()->getVisualWords().size();

	std::list<int> transferred = memory.forget(std::set<int>());

	// Branch (1) keeps transferring until wordsRemoved >= notIndexedBefore. Each
	// transferred signature releases its kKeypointsPerFrame unique words (their
	// reference count drops to zero), which exactly equals notIndexedBefore -- so
	// the loop transfers exactly one signature.
	ASSERT_EQ((int)transferred.size(), 1);
	const int transferredId = transferred.front();
	// ids[2] is still pinned by its neighbor link to STM (ids[3]); only ids[0] and
	// ids[1] are candidates and getRemovableSignatures returns the oldest first.
	EXPECT_EQ(transferredId, ids[0]);
	EXPECT_FALSE(memory.isInWM(transferredId));
	EXPECT_TRUE(memory.isInLTM(transferredId));
	// The other removable id stays in WM since the word budget was already met.
	EXPECT_TRUE(memory.isInWM(ids[1]));
	// ids[2] is linked to STM and never removable.
	EXPECT_TRUE(memory.isInWM(ids[2]));
	// STM is untouched.
	EXPECT_TRUE(memory.isInSTM(ids[3]));
	EXPECT_TRUE(memory.isInSTM(ids[4]));

	// The transferred signature released its word references but the words themselves
	// are not physically removed until the next preUpdate's cleanUnusedWords(); the
	// dictionary size is therefore unchanged at this point.
	EXPECT_EQ(memory.getVWDictionary()->getVisualWords().size(), wordsBefore);

	// Trigger cleanUnusedWords by running another update(): preUpdate() will physically
	// drop the kKeypointsPerFrame unused words, then addNewWords() will add 1 new word
	// for the single keypoint we provide on a fresh axis. Net change in dictionary size:
	// -kKeypointsPerFrame + 1 = -4 words.
	{
		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		SensorData extraData(image);
		std::vector<cv::KeyPoint> oneKpt(1, cv::KeyPoint(1.f, 1.f, 1.f));
		std::vector<cv::Point3f> onePt3(1, cv::Point3f(0.f, 0.f, 1.f));
		cv::Mat descriptors = cv::Mat::zeros(1, kDescCols, CV_32F);
		// Use the reserved axis (kFrames * kKeypointsPerFrame) so the descriptor is
		// orthogonal to every existing word and creates a brand-new visual word.
		descriptors.at<float>(0, kFrames * kKeypointsPerFrame) = 1000.0f;
		extraData.setFeatures(oneKpt, onePt3, descriptors);
		ASSERT_TRUE(memory.update(extraData, Transform(float(kFrames), 0.0f, 0.0f, 0, 0, 0), covariance));
	}

	// The 5 unused words from the transferred signature are gone (cleanUnusedWords) and
	// one new word was added for the new keypoint: 25 - 5 + 1 = 21.
	EXPECT_EQ((int)memory.getVWDictionary()->getVisualWords().size(),
			kFrames * kKeypointsPerFrame - kKeypointsPerFrame + 1);
	EXPECT_EQ((int)memory.getVWDictionary()->getVisualWords().size(),
			(int)wordsBefore - 4);

	memory.close(false);
}

TEST(MemoryTest, ForgetExcludesWmNodesLinkedToStmTwoVariants)
{
	// getRemovableSignatures rejects a WM candidate via two distinct gates:
	//   (A) lastInSTM->hasLink(memIter->first)        -- the *oldest* STM node has a
	//                                                    link pointing to the candidate.
	//   (B) iterating the candidate's own links and finding any endpoint in STM.
	// With STM=2 and 5 updates, STM={ids[3], ids[4]}, lastInSTM = ids[3] (smallest STM
	// id), and the WM candidates are ids[0], ids[1], ids[2]. ids[2] is *already*
	// rejected by gate (A) via its kNeighbor link to ids[3] -- so the variants below
	// only swap the gate that excludes ids[1] vs. ids[0].
	const cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	auto buildChain = [&](Memory & memory, std::vector<int> & ids) {
		ids.clear();
		for(int i = 0; i < 5; ++i)
		{
			ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
			ids.push_back(memory.getLastSignatureId());
		}
	};

	// --- Variant A: loop closure FROM lastInSTM (oldest STM) TO a WM candidate.
	//     The lastInSTM->hasLink gate fires on ids[1].
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));
	std::vector<int> ids;
	buildChain(memory, ids);
	ASSERT_TRUE(memory.addLink(Link(ids[3], ids[1], Link::kGlobalClosure, Transform::getIdentity(), infMatrix)));

	{
		std::list<int> transferred = memory.forget(std::set<int>());
		// Only ids[0] survives. ids[1] is gated by lastInSTM->hasLink, ids[2] by its
		// kNeighbor link to ids[3] (STM).
		ASSERT_EQ((int)transferred.size(), 1);
		EXPECT_EQ(transferred.front(), ids[0]);
		EXPECT_TRUE(memory.isInLTM(ids[0]));
		EXPECT_TRUE(memory.isInWM(ids[1]));
		EXPECT_TRUE(memory.isInWM(ids[2]));
	}

	// --- Variant B: swap the loop closure. Remove the previous A-style link
	//     ids[3]->ids[1], add a B-style link ids[2]->ids[4]. ids[2] now hits the
	//     inner foundInSTM gate (its loop-closure link points into STM).
	memory.removeLink(ids[3], ids[1]);
	ASSERT_TRUE(memory.addLink(Link(ids[2], ids[4], Link::kGlobalClosure, Transform::getIdentity(), infMatrix)));

	{
		// ids[0] is gone from the previous forget pass; only ids[1] and ids[2] remain
		// in WM. ids[2] now fails the foundInSTM check (its loop closure to ids[4]
		// is a link into STM). ids[1]'s links are: ids[0] (LTM, key>0 still counted in
		// the candidate's own links) and ids[2] (WM) and (now) ids[3] is no longer in
		// its links since we removed the previous loop closure. So ids[1] is the only
		// transferable candidate.
		std::list<int> transferred = memory.forget(std::set<int>());
		ASSERT_EQ((int)transferred.size(), 1);
		EXPECT_EQ(transferred.front(), ids[1]);
		EXPECT_TRUE(memory.isInLTM(ids[1]));
		EXPECT_TRUE(memory.isInWM(ids[2])); // still pinned by its loop-closure link to STM
	}

	memory.close(false);
}

TEST(MemoryTest, ForgetTransfersByWeightOrderAscending)
{
	// WeightAgeIdKey orders candidates by (weight, age, id) ascending, so
	// getRemovableSignatures returns the *lowest-weight* candidate first. With
	// custom weights {ids[0]=5, ids[1]=1, ids[2]=10} (ids[2] is excluded by the
	// STM-link gate anyway), forget pulls ids[1] first (weight=1) and ids[0] second
	// (weight=5), in that order.
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	{
		const_cast<Signature *>(memory.getSignature(ids[0]))->setWeight(5);
		const_cast<Signature *>(memory.getSignature(ids[1]))->setWeight(1);
		const_cast<Signature *>(memory.getSignature(ids[2]))->setWeight(10);
	}

	std::list<int> transferred = memory.forget(std::set<int>());
	// Both ids[1] and ids[0] are removable; ids[2] is gated by its kNeighbor link to
	// STM. forget keeps draining until signaturesRemoved >= _signaturesAdded+1 = 2.
	ASSERT_EQ((int)transferred.size(), 2);
	auto it = transferred.begin();
	EXPECT_EQ(*it++, ids[1]); // weight 1 -- lowest -- comes first
	EXPECT_EQ(*it,   ids[0]); // weight 5 -- next

	memory.close(false);
}

TEST(MemoryTest, ForgetIgnoresIntermediateNodes)
{
	// Intermediate nodes (weight==-1) are excluded by the weight>=0 gate inside
	// Memory::getRemovableSignatures. But Memory::forget also has a follow-up step
	// that drags intermediate *neighbors* of a transferred signature along with it
	// (the intermediate-node propagation loops in both branches of Memory::forget).
	// With STM=2 + 5 updates and ids[0] turned intermediate, ids[1] is the only
	// direct candidate; once forget transfers ids[1] it then traverses ids[1]'s
	// kNeighbor links and drags ids[0] along because it has weight==-1.
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	memory.convertToIntermediate(ids[0]);
	ASSERT_EQ(memory.getSignature(ids[0])->getWeight(), -1);

	std::list<int> transferred = memory.forget(std::set<int>());
	// getRemovableSignatures sees only ids[1] as a candidate (ids[0] gated by weight,
	// ids[2] by the STM-link gate). After ids[1] is transferred, forget then transfers
	// the intermediate neighbor ids[0] in the same call.
	std::set<int> transferredSet(transferred.begin(), transferred.end());
	EXPECT_EQ((int)transferredSet.size(), 2);
	EXPECT_EQ(transferredSet.count(ids[1]), 1u);
	EXPECT_EQ(transferredSet.count(ids[0]), 1u);
	// ids[1] is the "real" candidate -- it should appear before ids[0] in the order
	// reported by forget.
	EXPECT_EQ(transferred.front(), ids[1]);
	EXPECT_TRUE(memory.isInLTM(ids[0]));
	EXPECT_TRUE(memory.isInLTM(ids[1]));
	EXPECT_TRUE(memory.isInWM(ids[2]));

	memory.close(false);
}

TEST(MemoryTest, ForgetHonorsIgnoredIds)
{
	// Same setup as ForgetTransfersOldestWmSignaturesToLtm, but request that ids[0]
	// stays in WM via the ignoredIds set. forget() should then transfer ids[1] only
	// (ids[2] is still pinned by the STM link).
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}

	std::set<int> ignored;
	ignored.insert(ids[0]); // pin the oldest in WM

	std::list<int> transferred = memory.forget(ignored);

	ASSERT_EQ((int)transferred.size(), 1);
	EXPECT_EQ(transferred.front(), ids[1]);
	EXPECT_TRUE(memory.isInWM(ids[0]));
	EXPECT_FALSE(memory.isInWM(ids[1]));
	EXPECT_TRUE(memory.isInWM(ids[2]));

	memory.close(false);
}

// ---------------------------------------------------------------------------
// Save / load 2D map
// ---------------------------------------------------------------------------

TEST(MemoryTest, Save2DMapRoundtripsWhenDatabaseOpen)
{
	const std::string dbPath = uniqueDbPath();
	{
		Memory memory(defaultMemoryParams());
		ASSERT_TRUE(memory.init(dbPath, true));
		cv::Mat map(4, 4, CV_8SC1);
		map.setTo(-1);
		map.at<char>(1, 1) = 0;
		map.at<char>(2, 2) = 100;
		memory.save2DMap(map, -1.0f, -1.0f, 0.5f);

		float xMin = 0.0f, yMin = 0.0f, cellSize = 0.0f;
		cv::Mat loaded = memory.load2DMap(xMin, yMin, cellSize);
		ASSERT_EQ(loaded.rows, 4);
		ASSERT_EQ(loaded.cols, 4);
		EXPECT_NEAR(xMin, -1.0f, 1e-5f);
		EXPECT_NEAR(yMin, -1.0f, 1e-5f);
		EXPECT_NEAR(cellSize, 0.5f, 1e-5f);

		memory.close(true);
	}
	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// Get*Memory / weights
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetWeightsReturnsWmEntriesOnly)
{
	// getWeights() iterates _workingMem and skips STM. With STM=5 (default fixture),
	// the first 2 signatures are still in STM, so WM is empty and so is the result.
	for(int i = 0; i < 2; ++i)
	{
		ASSERT_TRUE(update());
	}
	EXPECT_EQ(memory_->getWeights().size(), 0u);

	// Add enough signatures so the oldest get promoted from STM to WM. After 7
	// updates with STM=5, 2 signatures are in WM.
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(update());
	}
	std::map<int, int> weights = memory_->getWeights();
	EXPECT_EQ((int)weights.size(), (int)memory_->getWorkingMem().size());
	// Without rehearsal merges (kMemRehearsalSimilarity=1.0 in the fixture) every
	// signature keeps its default weight of 0.
	for(auto & p: weights)
	{
		EXPECT_EQ(p.second, 0);
	}
}

TEST_F(MemoryFixture, GetMaxStMemSizeReflectsParameter)
{
	EXPECT_EQ(memory_->getMaxStMemSize(), 5);
}

// ---------------------------------------------------------------------------
// Node info
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetOdomPoseMatchesPassedPose)
{
	const Transform pose(2.5f, -1.0f, 0.5f, 0.1f, 0.2f, 0.3f);
	ASSERT_TRUE(updateWith(pose));
	const int id = memory_->getLastSignatureId();
	const Transform got = memory_->getOdomPose(id);
	EXPECT_NEAR(got.x(), pose.x(), 1e-4f);
	EXPECT_NEAR(got.y(), pose.y(), 1e-4f);
	EXPECT_NEAR(got.z(), pose.z(), 1e-4f);
}

TEST_F(MemoryFixture, GetNodeInfoReturnsTrueForExistingNode)
{
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();

	Transform odomPose, groundTruth;
	int mapId = -1, weight = -1;
	std::string label;
	double stamp = 0.0;
	std::vector<float> velocity;
	GPS gps;
	EnvSensors sensors;
	EXPECT_TRUE(memory_->getNodeInfo(id, odomPose, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors));
	EXPECT_EQ(mapId, 0); // first map id is 0
}

TEST_F(MemoryFixture, GetNodeInfoReturnsFalseForUnknownNode)
{
	Transform odomPose, groundTruth;
	int mapId = -1, weight = -1;
	std::string label;
	double stamp = 0.0;
	std::vector<float> velocity;
	GPS gps;
	EnvSensors sensors;
	EXPECT_FALSE(memory_->getNodeInfo(999, odomPose, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors));
}

// ---------------------------------------------------------------------------
// computeLikelihood
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, ComputeLikelihoodEmptyIdsReturnsEmpty)
{
	ASSERT_TRUE(update());
	const Signature * s = memory_->getSignature(memory_->getLastSignatureId());
	ASSERT_NE(s, nullptr);
	std::list<int> ids;
	std::map<int, float> likelihood = memory_->computeLikelihood(s, ids);
	EXPECT_EQ(likelihood.size(), 0u);
}

// ---------------------------------------------------------------------------
// createSignature (exercised through Memory::update)
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, CreateSignatureRejectsIntermediateAsFirstNode)
{
	// data.id() < 0 marks the input as an intermediate node. The very first node in
	// memory cannot be intermediate; createSignature returns null and update() reports
	// false. Memory must remain empty.
	SensorData data(image_);
	data.setId(-1);
	EXPECT_FALSE(memory_->update(data, Transform(0, 0, 0, 0, 0, 0), covariance_));
	EXPECT_EQ(memory_->getLastSignatureId(), 0);
	EXPECT_EQ(memory_->getStMem().size(), 0u);
}

TEST_F(MemoryFixture, CreateSignatureAcceptsIntermediateNodeAfterFirst)
{
	// Once a normal node exists, a follow-up update with data.id()<0 produces an
	// intermediate signature: it is created with weight==-1, even though
	// kMemGenerateIds=true overwrites the negative id with the next auto-generated one.
	ASSERT_TRUE(update());
	const int firstId = memory_->getLastSignatureId();

	SensorData intermediateData(image_);
	intermediateData.setId(-1);
	ASSERT_TRUE(memory_->update(intermediateData, Transform(1.0f, 0, 0, 0, 0, 0), covariance_));
	const int secondId = memory_->getLastSignatureId();

	EXPECT_EQ(secondId, firstId + 1); // auto-generated, not -1
	const Signature * s = memory_->getSignature(secondId);
	ASSERT_NE(s, nullptr);
	EXPECT_EQ(s->getWeight(), -1); // tagged as intermediate
}

TEST(MemoryTest, CreateSignatureRejectsZeroIdWhenGenerateIdsOff)
{
	// kMemGenerateIds=false: createSignature requires every input to have a positive
	// id (since it never auto-generates). data.id()==0 fails immediately.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemGenerateIds()] = "false";
	Memory memory(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	SensorData data(image);
	data.setId(0);
	EXPECT_FALSE(memory.update(data, Transform(0, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 0);
}

TEST(MemoryTest, CreateSignatureRejectsIdNotStrictlyGreaterWhenGenerateIdsOff)
{
	// kMemGenerateIds=false: subsequent ids must be strictly greater than _idCount.
	// A re-used id, or any id <= _idCount, fails.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemGenerateIds()] = "false";
	Memory memory(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	SensorData first(image);
	first.setId(5);
	ASSERT_TRUE(memory.update(first, Transform(0, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 5);

	// Reject lower id.
	SensorData second(image);
	second.setId(3);
	EXPECT_FALSE(memory.update(second, Transform(1, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 5);

	// Reject equal id.
	SensorData third(image);
	third.setId(5);
	EXPECT_FALSE(memory.update(third, Transform(2, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 5);

	// Accept higher id.
	SensorData fourth(image);
	fourth.setId(7);
	ASSERT_TRUE(memory.update(fourth, Transform(3, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 7);
}

TEST(MemoryTest, CreateSignatureRejectsDepthWithoutCameraModel)
{
	// createSignature aborts when data has a depth/right image but no monocular or
	// stereo calibration AND the odometry pose is non-null. This catches misconfigured
	// pipelines early.
	ParametersMap params = defaultMemoryParams();
	Memory memory(params);

	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	cv::Mat rgb(8, 8, CV_8UC3, cv::Scalar::all(0));
	cv::Mat depth(8, 8, CV_16UC1, cv::Scalar(1000));

	SensorData data;
	data.setRGBDImage(rgb, depth, std::vector<CameraModel>()); // no models
	data.setId(0);

	EXPECT_FALSE(memory.update(data, Transform(0.1f, 0, 0, 0, 0, 0), covariance));
	EXPECT_EQ(memory.getLastSignatureId(), 0);
}

TEST_F(MemoryFixture, CreateSignatureAutoIncrementsIdWhenGenerateIdsOn)
{
	// Default kMemGenerateIds=true: every successful update grabs a new id from
	// getNextId(), regardless of what data.id() contained (so long as it isn't
	// negative -- that means intermediate, handled above).
	SensorData first(image_);
	first.setId(100); // input id is ignored when generating
	ASSERT_TRUE(memory_->update(first, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int id1 = memory_->getLastSignatureId();
	EXPECT_EQ(id1, Memory::kIdStart + 1);

	SensorData second(image_);
	second.setId(42); // again ignored
	ASSERT_TRUE(memory_->update(second, Transform(1, 0, 0, 0, 0, 0), covariance_));
	EXPECT_EQ(memory_->getLastSignatureId(), id1 + 1);
}

TEST(MemoryTest, CreateSignaturePostDecimatesImageWhenPostDecimationGreaterThanOne)
{
	// kMemImagePostDecimation > 1 causes createSignature to downsample the RGB image
	// by that integer factor and scale the CameraModel accordingly (post-decimation
	// block in Memory::createSignature). Verify with a 16x16 image and decimation 2:
	// the signature's stored imageRaw should be 8x8 and the CameraModel scaled to half.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";              // keep raw image
	params[Parameters::kMemImagePostDecimation()] = "2";
	params[Parameters::kRtabmapImagesAlreadyRectified()] = "true"; // skip rectification
	Memory memory(params);

	const cv::Mat image(16, 16, CV_8UC1, cv::Scalar(64));
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	const CameraModel model(10.0, 10.0, 8.0, 8.0,
			CameraModel::opticalRotation(), 0.0, cv::Size(16, 16));

	SensorData data;
	data.setRGBDImage(image, cv::Mat(), std::vector<CameraModel>{model});
	data.setId(0);

	ASSERT_TRUE(memory.update(data, Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();
	const Signature * s = memory.getSignature(id);
	ASSERT_NE(s, nullptr);

	// Image is downsampled by the decimation factor.
	EXPECT_EQ(s->sensorData().imageRaw().rows, 8);
	EXPECT_EQ(s->sensorData().imageRaw().cols, 8);
	// CameraModel is scaled by 1 / kMemImagePostDecimation.
	ASSERT_EQ(s->sensorData().cameraModels().size(), 1u);
	EXPECT_EQ(s->sensorData().cameraModels()[0].imageWidth(), 8);
	EXPECT_EQ(s->sensorData().cameraModels()[0].imageHeight(), 8);
}

// Exhaustive checks of the rotation logic itself (all four roll cases, marker
// positions, dimension swaps) live in test_util2d (RotateImagesUpsideUpIfNecessary*).
// Here we just verify that Memory actually invokes util2d::rotateImagesUpsideUpIfNecessary
// when kMemRotateImagesUpsideUp=true, using a single 90 deg case.
TEST(MemoryTest, CreateSignatureRotatesImageUpsideUpWhenEnabled)
{
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kMemRotateImagesUpsideUp()] = "true";
	params[Parameters::kRtabmapImagesAlreadyRectified()] = "true"; // skip rectification
	Memory memory(params);

	// 8 rows x 16 cols (16 wide, 8 tall). Base gray = 64; marker = 200 at (2, 3).
	// +roll = pi/2 (camera tilted right) -> upright correction is a 90 CW rotation:
	//   transpose then flip(axis=1). The marker at (2, 3) lands at (3, 5) in 16x8.
	const uint8_t kBase = 64;
	const uint8_t kMarker = 200;
	cv::Mat image(8, 16, CV_8UC1, cv::Scalar(kBase));
	image.at<uint8_t>(2, 3) = kMarker;

	// util2d::rotateImagesUpsideUpIfNecessary inspects
	//   model.localTransform() * opticalRotation().inverse()
	// so we set: model.localTransform() = R(roll=+pi/2) * opticalRotation().
	const Transform rolled(0.0f, 0.0f, 0.0f, (float)M_PI / 2.0f, 0.0f, 0.0f);
	const CameraModel model(10.0, 10.0, 8.0, 4.0,
			rolled * CameraModel::opticalRotation(), 0.0, cv::Size(16, 8));

	SensorData data;
	data.setRGBDImage(image, cv::Mat(), std::vector<CameraModel>{model});
	data.setId(0);

	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(data, Transform(0, 0, 0, 0, 0, 0), covariance));
	const Signature * s = memory.getSignature(memory.getLastSignatureId());
	ASSERT_NE(s, nullptr);

	const cv::Mat & rotated = s->sensorData().imageRaw();
	EXPECT_EQ(rotated.rows, 16);
	EXPECT_EQ(rotated.cols, 8);
	ASSERT_EQ(s->sensorData().cameraModels().size(), 1u);
	EXPECT_EQ(s->sensorData().cameraModels()[0].imageWidth(),  8);
	EXPECT_EQ(s->sensorData().cameraModels()[0].imageHeight(), 16);
	EXPECT_EQ(rotated.at<uint8_t>(3, 5), kMarker);
}

TEST_F(MemoryFixture, CreateSignatureStoresPoseOnSignature)
{
	// The pose passed to update() is stored on the signature.
	SensorData data(image_);
	data.setId(0);
	const Transform pose(2.5f, -1.0f, 0.5f, 0.0f, 0.0f, 0.3f);
	ASSERT_TRUE(memory_->update(data, pose, covariance_));
	const int id = memory_->getLastSignatureId();

	Transform odomPose, groundTruth;
	int mapId = -1, weight = -1;
	std::string label;
	double stamp = 0.0;
	std::vector<float> velocity;
	GPS gps;
	EnvSensors sensors;
	ASSERT_TRUE(memory_->getNodeInfo(id, odomPose, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors));
	EXPECT_NEAR(odomPose.x(), pose.x(), 1e-4f);
	EXPECT_NEAR(odomPose.y(), pose.y(), 1e-4f);
	EXPECT_NEAR(odomPose.z(), pose.z(), 1e-4f);
	float r, p, y;
	odomPose.getEulerAngles(r, p, y);
	EXPECT_NEAR(y, 0.3f, 1e-4f);
	// First node gets the default map id 0 (no incrementMapId() yet).
	EXPECT_EQ(mapId, 0);
}

// ---------------------------------------------------------------------------
// getNodeData
// ---------------------------------------------------------------------------

TEST(MemoryTest, GetNodeDataReturnsEmptyForUnknownIdWithoutDb)
{
	// No init() called -> no DB driver. Unknown id falls through both branches
	// (signature is null, _dbDriver is null) and returns a default-constructed
	// SensorData with no image / scan / user data / occupancy grid.
	Memory memory(defaultMemoryParams());
	SensorData r = memory.getNodeData(99999, true, true, true, true);
	EXPECT_EQ(r.imageCompressed().rows, 0);
	EXPECT_EQ(r.imageRaw().rows, 0);
	EXPECT_EQ(r.laserScanCompressed().size(), 0);
	EXPECT_EQ(r.userDataCompressed().rows, 0);
	EXPECT_FLOAT_EQ(r.gridCellSize(), 0.0f);
}

TEST(MemoryTest, GetNodeDataReturnsInMemoryPayloadsWhenSignatureNotSaved)
{
	// kMemBinDataKept=true so update() leaves the compressed image in the signature.
	// The signature is not saved (no _dbDriver call), so getNodeData takes the
	// in-memory branch and returns the compressed image when images=true.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Memory memory(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	const Signature * s = memory.getSignature(id);
	ASSERT_NE(s, nullptr);
	ASSERT_FALSE(s->isSaved());
	ASSERT_EQ(s->sensorData().imageCompressed().rows, 1); // compressed image populated by update

	SensorData r = memory.getNodeData(id, /*images=*/true, false, false, false);
	EXPECT_EQ(r.imageCompressed().rows, 1);
	EXPECT_EQ(r.imageCompressed().cols, s->sensorData().imageCompressed().cols);
}

TEST(MemoryTest, GetNodeDataMasksFieldsThatWereNotRequested)
{
	// Even when a signature has all payloads populated, getNodeData must clear the
	// fields the caller didn't request before returning.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Memory memory(params);

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();
	ASSERT_EQ(memory.getSignature(id)->sensorData().imageCompressed().rows, 1);

	// Ask for nothing: every field is masked to empty.
	SensorData masked = memory.getNodeData(id, /*images=*/false, /*scan=*/false, /*userData=*/false, /*occupancyGrid=*/false);
	EXPECT_EQ(masked.imageRaw().rows, 0);
	EXPECT_EQ(masked.imageCompressed().rows, 0);
	EXPECT_EQ(masked.laserScanCompressed().size(), 0);
	EXPECT_EQ(masked.userDataCompressed().rows, 0);
	EXPECT_FLOAT_EQ(masked.gridCellSize(), 0.0f);

	// Ask only for images: image fields populated, others remain empty.
	SensorData imagesOnly = memory.getNodeData(id, /*images=*/true, /*scan=*/false, /*userData=*/false, /*occupancyGrid=*/false);
	EXPECT_EQ(imagesOnly.imageCompressed().rows, 1);
	EXPECT_EQ(imagesOnly.laserScanCompressed().size(), 0);
	EXPECT_EQ(imagesOnly.userDataCompressed().rows, 0);
	EXPECT_FLOAT_EQ(imagesOnly.gridCellSize(), 0.0f);
}

TEST(MemoryTest, GetNodeDataLoadsEachPayloadTypeFromDatabase)
{
	// When a signature is no longer in WM/STM (transferred to LTM by forget()),
	// getSignature() returns null and getNodeData falls back to _dbDriver->getNodeData().
	// We verify the DB branch returns each of the four selectable payload types
	// (images, scan, userData, occupancyGrid) and that selective masking still works
	// after a round-trip through the database.
	ParametersMap params = defaultMemoryParams(/*stmSize=*/2);
	params[Parameters::kMemBinDataKept()] = "true";
	// Memory drops user-supplied occupancy grids unless RGBD/CreateOccupancyGrid is on
	// (the occupancy-grid block in Memory::createSignature is gated on it). Enable it
	// so our grid survives update().
	params[Parameters::kRGBDCreateOccupancyGrid()] = "true";
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	const cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	const cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	// Build the SensorData for the *first* frame with all four payloads populated.
	// LaserScan, user data and occupancy grid pass through Memory unchanged when
	// laser-related downsampling/voxelization parameters are at their defaults (the
	// fixture doesn't set kMemLaserScanDownsampleStepSize / kMemLaserScanVoxelSize).
	SensorData seeded(image);
	// Scan: a minimal 3D point cloud, 1 row of 4 XYZ points.
	cv::Mat scanData(1, 4, CV_32FC3);
	for(int i = 0; i < 4; ++i)
	{
		scanData.at<cv::Vec3f>(0, i) = cv::Vec3f(float(i), 0.0f, 0.0f);
	}
	const LaserScan scan(scanData, /*maxPoints=*/4, /*maxRange=*/10.0f, LaserScan::kXYZ);
	seeded.setLaserScan(scan);
	// User data: 4 floats.
	cv::Mat userData = (cv::Mat_<float>(1, 4) << 1.0f, 2.0f, 3.0f, 4.0f);
	seeded.setUserData(userData);
	// Occupancy grid: a few obstacle cells.
	cv::Mat obstacles(1, 3, CV_32FC3);
	for(int i = 0; i < 3; ++i)
	{
		obstacles.at<cv::Vec3f>(0, i) = cv::Vec3f(float(i) * 0.1f, 0.0f, 0.0f);
	}
	const float kCellSize = 0.05f;
	seeded.setOccupancyGrid(cv::Mat(), obstacles, cv::Mat(), kCellSize, cv::Point3f(0, 0, 0));

	ASSERT_TRUE(memory.update(seeded, Transform(0.0f, 0.0f, 0.0f, 0, 0, 0), covariance));
	const int targetId = memory.getLastSignatureId();

	// Add more frames so the target signature ends up in WM and becomes a candidate
	// for forget(). STM=2 so the first frame leaves STM after the third update.
	for(int i = 1; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0.0f, 0.0f, 0, 0, 0), covariance));
	}

	std::list<int> transferred = memory.forget(std::set<int>());
	const auto it = std::find(transferred.begin(), transferred.end(), targetId);
	ASSERT_NE(it, transferred.end()); // target was transferred to LTM
	ASSERT_EQ(memory.getSignature(targetId), nullptr);

	// Wait for the async db writer to flush so subsequent reads see the persisted row.
	memory.emptyTrash();

	// LaserScan::size() on a compressed scan is the byte count of the zlib payload,
	// which depends on the source data and isn't a useful exact-equality assertion.
	// "Loaded" -> non-empty (data was returned) AND metadata (maxPoints, format)
	// matches what we seeded. "Empty / masked" -> isEmpty() is true.
	auto expectScanLoaded = [](const LaserScan & s, int expectedMaxPoints) {
		EXPECT_FALSE(s.isEmpty());
		EXPECT_EQ(s.maxPoints(), expectedMaxPoints);
		EXPECT_EQ(s.format(), LaserScan::kXYZ);
	};
	auto expectScanEmpty = [](const LaserScan & s) {
		EXPECT_TRUE(s.isEmpty());
	};

	// (1) images only.
	{
		SensorData r = memory.getNodeData(targetId, /*images=*/true, /*scan=*/false, /*userData=*/false, /*occupancyGrid=*/false);
		EXPECT_EQ(r.imageCompressed().rows, 1);          // image returned
		expectScanEmpty(r.laserScanCompressed());        // scan masked
		EXPECT_EQ(r.userDataCompressed().rows, 0);       // user data masked
		EXPECT_FLOAT_EQ(r.gridCellSize(), 0.0f);         // grid masked
	}

	// (2) scan only.
	{
		SensorData r = memory.getNodeData(targetId, /*images=*/false, /*scan=*/true, /*userData=*/false, /*occupancyGrid=*/false);
		EXPECT_EQ(r.imageCompressed().rows, 0);
		expectScanLoaded(r.laserScanCompressed(), 4);
		EXPECT_EQ(r.userDataCompressed().rows, 0);
		EXPECT_FLOAT_EQ(r.gridCellSize(), 0.0f);
	}

	// (3) userData only.
	{
		SensorData r = memory.getNodeData(targetId, /*images=*/false, /*scan=*/false, /*userData=*/true, /*occupancyGrid=*/false);
		EXPECT_EQ(r.imageCompressed().rows, 0);
		expectScanEmpty(r.laserScanCompressed());
		EXPECT_EQ(r.userDataCompressed().rows, 1);
		EXPECT_FLOAT_EQ(r.gridCellSize(), 0.0f);
	}

	// (4) occupancy grid only.
	{
		SensorData r = memory.getNodeData(targetId, /*images=*/false, /*scan=*/false, /*userData=*/false, /*occupancyGrid=*/true);
		EXPECT_EQ(r.imageCompressed().rows, 0);
		expectScanEmpty(r.laserScanCompressed());
		EXPECT_EQ(r.userDataCompressed().rows, 0);
		EXPECT_FLOAT_EQ(r.gridCellSize(), kCellSize);
	}

	// All four together.
	{
		SensorData r = memory.getNodeData(targetId, /*images=*/true, /*scan=*/true, /*userData=*/true, /*occupancyGrid=*/true);
		EXPECT_EQ(r.imageCompressed().rows, 1);
		expectScanLoaded(r.laserScanCompressed(), 4);
		EXPECT_EQ(r.userDataCompressed().rows, 1);
		EXPECT_FLOAT_EQ(r.gridCellSize(), kCellSize);
	}

	memory.close(false);
}

// ---------------------------------------------------------------------------
// reactivateSignatures
// ---------------------------------------------------------------------------

TEST(MemoryTest, ReactivateSignaturesIsNoOpWhenAllInputsAreEmptyOrInMemory)
{
	// Empty input -> empty result. IDs already in WM/STM are skipped (they don't
	// hit the database, and they don't appear in the returned set).
	Memory memory(defaultMemoryParams(2));
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int inMemId = memory.getLastSignatureId();
	ASSERT_TRUE(memory.isInSTM(inMemId) || memory.isInWM(inMemId));

	{
		double timeDb = -1.0;
		std::set<int> result = memory.reactivateSignatures(std::list<int>(), 0, timeDb);
		EXPECT_EQ(result.size(), 0u);
	}
	{
		double timeDb = -1.0;
		std::set<int> result = memory.reactivateSignatures(std::list<int>{inMemId}, 0, timeDb);
		EXPECT_EQ(result.size(), 0u);
		EXPECT_TRUE(memory.isInSTM(inMemId) || memory.isInWM(inMemId));
	}

	memory.close(false);
}

TEST(MemoryTest, ReactivateSignaturesLoadsForgottenSignatureIntoWm)
{
	// Setup mirrors ForgetTransfersOldestWmSignaturesToLtm: STM=2, 5 updates,
	// forget() transfers ids[0] and ids[1] to LTM. reactivateSignatures must
	// bring ids[0] back into WM while ids[1] remains in LTM.
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0, 0, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	ASSERT_EQ(memory.forget(std::set<int>()).size(), 2u);
	ASSERT_TRUE(memory.isInLTM(ids[0]));
	ASSERT_TRUE(memory.isInLTM(ids[1]));

	double timeDb = -1.0;
	std::set<int> result = memory.reactivateSignatures(std::list<int>{ids[0]}, 0, timeDb);
	EXPECT_EQ(result.size(), 1u);
	EXPECT_EQ(result.count(ids[0]), 1u);
	EXPECT_TRUE(memory.isInWM(ids[0]));
	EXPECT_TRUE(memory.isInLTM(ids[1]));
	// The reloaded signature is fully queryable through getSignature().
	EXPECT_NE(memory.getSignature(ids[0]), nullptr);

	memory.close(false);
}

TEST(MemoryTest, ReactivateSignaturesRespectsMaxLoadedCapWithNormalNodes)
{
	// Three normal (non-intermediate) signatures in LTM. Asking to reactivate all
	// three with maxLoaded=2 must load exactly 2 and leave the third in LTM (the
	// recursive queue load only fires when intermediate nodes are loaded).
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0, 0, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	// First forget() removes 2 oldest (ids[0], ids[1]); second forget() removes ids[2].
	// ids[3] stays in WM (linked to ids[4] in STM and excluded).
	memory.forget(std::set<int>());
	memory.forget(std::set<int>());
	ASSERT_TRUE(memory.isInLTM(ids[0]));
	ASSERT_TRUE(memory.isInLTM(ids[1]));
	ASSERT_TRUE(memory.isInLTM(ids[2]));

	double timeDb = -1.0;
	std::set<int> result = memory.reactivateSignatures(
			std::list<int>{ids[0], ids[1], ids[2]}, 2, timeDb);
	EXPECT_EQ(result.size(), 2u);
	EXPECT_EQ(result.count(ids[0]), 1u);
	EXPECT_EQ(result.count(ids[1]), 1u);
	EXPECT_EQ(result.count(ids[2]), 0u);
	EXPECT_TRUE(memory.isInWM(ids[0]));
	EXPECT_TRUE(memory.isInWM(ids[1]));
	EXPECT_TRUE(memory.isInLTM(ids[2]));

	memory.close(false);
}

TEST(MemoryTest, ReactivateSignaturesRecursesWhenIntermediateNodesLoaded)
{
	// When an intermediate node is loaded as part of the first batch, it does not
	// count against maxLoaded -- the function recursively pulls intermediateNodesLoaded
	// extra signatures from the queue. Here ids[1] is converted to intermediate before
	// forget(), so with maxLoaded=2 and intermediate first in the list, the recursive
	// call loads ids[0] from the queue and all 3 end up in WM.
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 6; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0, 0, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	memory.convertToIntermediate(ids[1]);
	memory.forget(std::set<int>());
	memory.forget(std::set<int>());
	ASSERT_TRUE(memory.isInLTM(ids[0]));
	ASSERT_TRUE(memory.isInLTM(ids[1]));
	ASSERT_TRUE(memory.isInLTM(ids[2]));

	// Intermediate (ids[1]) and ids[2] go into the first load batch (cap=2);
	// ids[0] goes to the queue. ids[1] being intermediate triggers a recursive call
	// that loads ids[0].
	double timeDb = -1.0;
	std::set<int> result = memory.reactivateSignatures(
			std::list<int>{ids[1], ids[2], ids[0]}, 2, timeDb);
	EXPECT_EQ(result.size(), 3u);
	EXPECT_EQ(result.count(ids[0]), 1u);
	EXPECT_EQ(result.count(ids[1]), 1u);
	EXPECT_EQ(result.count(ids[2]), 1u);
	EXPECT_TRUE(memory.isInWM(ids[0]));
	EXPECT_TRUE(memory.isInWM(ids[1]));
	EXPECT_TRUE(memory.isInWM(ids[2]));
	// The reloaded intermediate keeps weight=-1.
	ASSERT_NE(memory.getSignature(ids[1]), nullptr);
	EXPECT_EQ(memory.getSignature(ids[1])->getWeight(), -1);

	memory.close(false);
}

TEST(MemoryTest, ReactivateSignaturesRepopulatesLandmarkIndex)
{
	// moveToTrash() (called via forget()) removes the landmark observer from
	// _landmarksIndex; when the sole observer is removed, the landmark entry
	// disappears entirely. reactivateSignatures() must rebuild that index entry.
	ParametersMap params = defaultMemoryParams(2);
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	const int landmarkPositiveId = 9;
	const int landmarkInternalId = -landmarkPositiveId;
	const cv::Mat lmCov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	SensorData first(image);
	Landmarks landmarks;
	landmarks.insert(std::make_pair(
			landmarkPositiveId,
			Landmark(landmarkPositiveId, 0.1f, Transform(1.0f, 0, 0, 0, 0, 0), lmCov)));
	first.setLandmarks(landmarks);
	ASSERT_TRUE(memory.update(first, Transform(0, 0, 0, 0, 0, 0), covariance));
	const int landmarkObserver = memory.getLastSignatureId();

	// Four more updates with no landmark, just to fill STM/WM so forget() can
	// transfer the landmark observer to LTM.
	for(int i = 1; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0, 0, 0, 0, 0), covariance));
	}

	// Pre-forget: landmark index has landmarkObserver.
	{
		const std::map<int, std::set<int> > & idx = memory.getLandmarksIndex();
		ASSERT_EQ(idx.count(landmarkInternalId), 1u);
		EXPECT_EQ(idx.at(landmarkInternalId).count(landmarkObserver), 1u);
	}

	ASSERT_EQ(memory.forget(std::set<int>()).size(), 2u);
	ASSERT_TRUE(memory.isInLTM(landmarkObserver));

	// After forget: sole observer is gone, so the landmark entry is erased.
	{
		const std::map<int, std::set<int> > & idx = memory.getLandmarksIndex();
		EXPECT_EQ(idx.count(landmarkInternalId), 0u);
	}

	double timeDb = -1.0;
	std::set<int> result = memory.reactivateSignatures(std::list<int>{landmarkObserver}, 0, timeDb);
	EXPECT_EQ(result.size(), 1u);
	EXPECT_TRUE(memory.isInWM(landmarkObserver));

	// After reactivate: index entry is back with the observer.
	{
		const std::map<int, std::set<int> > & idx = memory.getLandmarksIndex();
		ASSERT_EQ(idx.count(landmarkInternalId), 1u);
		EXPECT_EQ(idx.at(landmarkInternalId).size(), 1u);
		EXPECT_EQ(idx.at(landmarkInternalId).count(landmarkObserver), 1u);
	}

	memory.close(false);
}

// ---------------------------------------------------------------------------
// getMetricConstraints
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetMetricConstraintsLinearChainReturnsUniqueLinks)
{
	// A-B-C-D chain. All 4 ids requested -> 4 poses, 3 unique neighbor links.
	// Each underlying neighbor link is stored once (the dedup uses graph::findLink).
	std::vector<int> ids;
	for(int i = 0; i < 4; ++i)
	{
		ASSERT_TRUE(update());
		ids.push_back(memory_->getLastSignatureId());
	}
	std::set<int> requested(ids.begin(), ids.end());

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints(requested, poses, links);

	ASSERT_EQ(poses.size(), 4u);
	for(int id : ids) EXPECT_EQ(poses.count(id), 1u);
	// 3 unique edges in the chain.
	ASSERT_EQ(links.size(), 3u);
	for(const auto & kv : links)
	{
		EXPECT_EQ(kv.second.type(), Link::kNeighbor);
	}
}

TEST_F(MemoryFixture, GetMetricConstraintsSubsetExcludesLinksToNonIncludedNodes)
{
	// A-B-C-D, but request only {A, C}. A's only neighbor is B (not requested);
	// C's neighbors are B and D (neither requested). So 2 poses, 0 links.
	std::vector<int> ids;
	for(int i = 0; i < 4; ++i)
	{
		ASSERT_TRUE(update());
		ids.push_back(memory_->getLastSignatureId());
	}
	std::set<int> requested = {ids[0], ids[2]};

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints(requested, poses, links);

	ASSERT_EQ(poses.size(), 2u);
	EXPECT_EQ(poses.count(ids[0]), 1u);
	EXPECT_EQ(poses.count(ids[2]), 1u);
	EXPECT_EQ(links.size(), 0u);
}

TEST_F(MemoryFixture, GetMetricConstraintsSkipsUnknownIds)
{
	// Two real signatures + a non-existent id. The unknown id is silently dropped
	// because getOdomPose returns a null transform for it.
	ASSERT_TRUE(update());
	const int A = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int B = memory_->getLastSignatureId();
	const int unknown = 9999;
	std::set<int> requested = {A, unknown, B};

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints(requested, poses, links);

	EXPECT_EQ(poses.count(A), 1u);
	EXPECT_EQ(poses.count(B), 1u);
	EXPECT_EQ(poses.count(unknown), 0u);
	ASSERT_EQ(poses.size(), 2u);
	ASSERT_EQ(links.size(), 1u);
}

TEST_F(MemoryFixture, GetMetricConstraintsMergesThroughIntermediateWhenNotLookingInDb)
{
	// A-B-C with B intermediate. lookInDatabase=false -> B is excluded from poses
	// and A's neighbor link to B is merged into a single A->C link.
	ASSERT_TRUE(update());
	const int A = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int B = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int C = memory_->getLastSignatureId();
	memory_->convertToIntermediate(B);

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints({A, B, C}, poses, links, /*lookInDatabase=*/false, /*landmarksAdded=*/false);

	ASSERT_EQ(poses.size(), 2u);
	EXPECT_EQ(poses.count(A), 1u);
	EXPECT_EQ(poses.count(B), 0u);
	EXPECT_EQ(poses.count(C), 1u);
	// One merged link from A to C (the chain through B).
	ASSERT_EQ(links.size(), 1u);
	const Link & merged = links.begin()->second;
	EXPECT_EQ(merged.from(), A);
	EXPECT_EQ(merged.to(), C);
}

TEST_F(MemoryFixture, GetMetricConstraintsKeepsIntermediateWhenLookingInDb)
{
	// Same setup as above but with lookInDatabase=true: the intermediate-merge
	// branch is bypassed, so B is kept in poses and both A-B and B-C links are
	// returned individually.
	ASSERT_TRUE(update());
	const int A = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int B = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int C = memory_->getLastSignatureId();
	memory_->convertToIntermediate(B);

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints({A, B, C}, poses, links, /*lookInDatabase=*/true, /*landmarksAdded=*/false);

	ASSERT_EQ(poses.size(), 3u);
	EXPECT_EQ(poses.count(A), 1u);
	EXPECT_EQ(poses.count(B), 1u);
	EXPECT_EQ(poses.count(C), 1u);
	ASSERT_EQ(links.size(), 2u);
	for(const auto & kv : links)
	{
		EXPECT_EQ(kv.second.type(), Link::kNeighbor);
	}
}

TEST_F(MemoryFixture, GetMetricConstraintsIncludesLoopClosureLinkExactlyOnce)
{
	// A-B-C chain plus a loop closure A<->C. Result must contain the loop link
	// exactly once (not twice) thanks to the graph::findLink dedup.
	ASSERT_TRUE(update());
	const int A = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int B = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int C = memory_->getLastSignatureId();
	ASSERT_TRUE(memory_->addLink(Link(A, C, Link::kGlobalClosure,
			Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1))));

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	memory_->getMetricConstraints({A, B, C}, poses, links);

	EXPECT_EQ(poses.size(), 3u);
	ASSERT_EQ(links.size(), 3u);
	int neighbors = 0, loops = 0;
	for(const auto & kv : links)
	{
		if(kv.second.type() == Link::kNeighbor) ++neighbors;
		else if(kv.second.type() == Link::kGlobalClosure) ++loops;
	}
	EXPECT_EQ(neighbors, 2);
	EXPECT_EQ(loops, 1);
}

TEST_F(MemoryFixture, GetMetricConstraintsExcludesLandmarkUnlessRequested)
{
	// A signature with a landmark observation. landmarksAdded=false -> no landmark
	// in poses, no landmark link. landmarksAdded=true -> landmark pose included
	// and the inverted link is added (key = landmark id).
	const int landmarkPositiveId = 11;
	const int landmarkInternalId = -landmarkPositiveId;
	const cv::Mat lmCov = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;

	SensorData data(image_);
	Landmarks landmarks;
	landmarks.insert(std::make_pair(
			landmarkPositiveId,
			Landmark(landmarkPositiveId, 0.1f, Transform(1.0f, 0, 0, 0, 0, 0), lmCov)));
	data.setLandmarks(landmarks);
	ASSERT_TRUE(memory_->update(data, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int A = memory_->getLastSignatureId();

	{
		SCOPED_TRACE("landmarksAdded=false");
		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		memory_->getMetricConstraints({A}, poses, links, /*lookInDatabase=*/false, /*landmarksAdded=*/false);
		ASSERT_EQ(poses.size(), 1u);
		EXPECT_EQ(poses.count(A), 1u);
		EXPECT_EQ(poses.count(landmarkInternalId), 0u);
		EXPECT_EQ(links.size(), 0u);
	}
	{
		SCOPED_TRACE("landmarksAdded=true");
		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		memory_->getMetricConstraints({A}, poses, links, /*lookInDatabase=*/false, /*landmarksAdded=*/true);
		ASSERT_EQ(poses.size(), 2u);
		EXPECT_EQ(poses.count(A), 1u);
		EXPECT_EQ(poses.count(landmarkInternalId), 1u);
		ASSERT_EQ(links.size(), 1u);
		EXPECT_EQ(links.begin()->first, landmarkInternalId);
		EXPECT_EQ(links.begin()->second.type(), Link::kLandmark);
	}
}

// ---------------------------------------------------------------------------
// cleanup
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, CleanupReturnsZeroWhenNoLastSignature)
{
	// Fresh memory, no update() yet -> _lastSignature is null.
	EXPECT_EQ(memory_->cleanup(), 0);
}

TEST_F(MemoryFixture, CleanupReturnsZeroForLastSignatureInDefaultMode)
{
	// Default params: kMemBadSignaturesIgnored=false -> the bad-signature branch
	// is disabled even though the last signature has no words (kKpMaxFeatures=-1).
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	EXPECT_EQ(memory_->cleanup(), 0);
	// Signature still in STM.
	EXPECT_TRUE(memory_->isInSTM(id));
}

TEST(MemoryTest, CleanupRemovesBadSignatureWhenBadSignaturesIgnored)
{
	// kKpMaxFeatures=-1 -> every signature has 0 words -> isBadSignature() == true.
	// kMemBadSignaturesIgnored=true gates the cleanup branch on. The non-intermediate
	// last signature is removed and its id is returned.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBadSignaturesIgnored()] = "true";
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	EXPECT_EQ(memory.cleanup(), id);
	EXPECT_EQ(memory.getSignature(id), nullptr);
	memory.close(false);
}

TEST(MemoryTest, CleanupRemovesLastSignatureInLocalizationMode)
{
	// !_incrementalMemory -> cleanup() always removes the last signature regardless
	// of bad-signature status.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemIncrementalMemory()] = "false";
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();
	ASSERT_NE(memory.getSignature(id), nullptr);

	EXPECT_EQ(memory.cleanup(), id);
	EXPECT_EQ(memory.getSignature(id), nullptr);
	memory.close(false);
}

// ---------------------------------------------------------------------------
// removeRawData
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, RemoveRawDataClearsImage)
{
	// Default registration pipeline (kReg=0, RegistrationVis) does not require scan or
	// userData; reextractLoopClosureFeatures defaults to false, so image clearing is
	// also unconditional. removeRawData(id, true, true, true, true) should clear the
	// signature's imageRaw and depthRaw.
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();
	ASSERT_FALSE(memory_->getSignature(id)->sensorData().imageRaw().empty());

	memory_->removeRawData(id, /*image=*/true, /*scan=*/true, /*userData=*/true, /*occupancyGrid=*/true);

	EXPECT_TRUE(memory_->getSignature(id)->sensorData().imageRaw().empty());
}

// ---------------------------------------------------------------------------
// updateAge
// ---------------------------------------------------------------------------

TEST(MemoryTest, UpdateAgeReordersForgetCandidatesByTimestamp)
{
	// Memory::forget() orders removal candidates by weight, then age (timestamp in
	// _workingMem). Touching ids[0] with updateAge() makes it the freshest, so the
	// next forget() picks ids[1] (next-oldest) instead of ids[0].
	ParametersMap params = defaultMemoryParams(1); // STM=1
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	std::vector<int> ids;
	for(int i = 0; i < 5; ++i)
	{
		ASSERT_TRUE(memory.update(SensorData(image), Transform(float(i), 0, 0, 0, 0, 0), covariance));
		ids.push_back(memory.getLastSignatureId());
	}
	// WM = {virtual, ids[0..3]}, STM = {ids[4]}. ids[3] linked to STM -> excluded.
	// Removable in age order: ids[0], ids[1], ids[2].

	// Refresh ids[0]'s timestamp so it becomes the freshest in WM. forget() should
	// now pick ids[1] and ids[2] (next two oldest), leaving ids[0] alone.
	memory.updateAge(ids[0]);

	std::list<int> transferred = memory.forget(std::set<int>());
	std::set<int> transferredSet(transferred.begin(), transferred.end());
	EXPECT_EQ(transferredSet.count(ids[0]), 0u); // refreshed, not picked
	EXPECT_EQ(transferredSet.count(ids[1]), 1u);
	EXPECT_TRUE(memory.isInWM(ids[0]));
	EXPECT_TRUE(memory.isInLTM(ids[1]));

	memory.close(false);
}

// ---------------------------------------------------------------------------
// saveLocationData
// ---------------------------------------------------------------------------

TEST(MemoryTest, SaveLocationDataFlushesSignatureToOnDiskDatabase)
{
	// saveLocationData gates on _dbDriver, !isInMemory(), incrementalMemory, !isSaved
	// and id>0. With an on-disk database, the call must: (1) flip isSaved() to true,
	// (2) clear the in-memory compressed payload, and (3) leave the signature still
	// reachable in WM/STM (getSignature stays non-null).
	const std::string dbPath = uniqueDbPath();
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true"; // keep image compressed in memory
	{
		Memory memory(params);
		ASSERT_TRUE(memory.init(dbPath, true));

		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
		const int id = memory.getLastSignatureId();

		const Signature * sBefore = memory.getSignature(id);
		ASSERT_NE(sBefore, nullptr);
		ASSERT_FALSE(sBefore->isSaved());
		ASSERT_FALSE(sBefore->sensorData().imageCompressed().empty());

		memory.saveLocationData(id);

		const Signature * sAfter = memory.getSignature(id);
		ASSERT_NE(sAfter, nullptr);
		EXPECT_TRUE(sAfter->isSaved());
		EXPECT_TRUE(sAfter->sensorData().imageCompressed().empty());

		memory.close(false);
	}
	UFile::erase(dbPath.c_str());
}

// ---------------------------------------------------------------------------
// removeVirtualLinks (per-id)
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, RemoveVirtualLinksClearsBothSidesOfTheLink)
{
	// Add a virtual link between A and C, then call removeVirtualLinks(A): both A
	// and C must lose the link. The non-virtual neighbor link between A and B is
	// untouched.
	ASSERT_TRUE(update());
	const int A = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int B = memory_->getLastSignatureId();
	ASSERT_TRUE(update());
	const int C = memory_->getLastSignatureId();

	ASSERT_TRUE(memory_->addLink(Link(A, C, Link::kVirtualClosure,
			Transform::getIdentity(), cv::Mat::eye(6, 6, CV_64FC1))));
	ASSERT_NE(memory_->getSignature(A)->getLinks().find(C), memory_->getSignature(A)->getLinks().end());
	ASSERT_NE(memory_->getSignature(C)->getLinks().find(A), memory_->getSignature(C)->getLinks().end());

	memory_->removeVirtualLinks(A);

	// Virtual link gone from both endpoints.
	EXPECT_EQ(memory_->getSignature(A)->getLinks().find(C), memory_->getSignature(A)->getLinks().end());
	EXPECT_EQ(memory_->getSignature(C)->getLinks().find(A), memory_->getSignature(C)->getLinks().end());
	// Neighbor A-B link is preserved.
	EXPECT_NE(memory_->getSignature(A)->getLinks().find(B), memory_->getSignature(A)->getLinks().end());
}

// ---------------------------------------------------------------------------
// getMapId
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetMapIdReturnsZeroForFirstMap)
{
	// Fresh memory uses mapId=0 for the first update. incrementMapId bumps it.
	ASSERT_TRUE(update());
	const int id1 = memory_->getLastSignatureId();
	EXPECT_EQ(memory_->getMapId(id1), 0);

	memory_->incrementMapId();
	ASSERT_TRUE(update());
	const int id2 = memory_->getLastSignatureId();
	EXPECT_EQ(memory_->getMapId(id2), 1);
}

TEST_F(MemoryFixture, GetMapIdReturnsMinusOneForUnknownId)
{
	EXPECT_EQ(memory_->getMapId(9999), -1);
}

// ---------------------------------------------------------------------------
// getImageCompressed
// ---------------------------------------------------------------------------

TEST(MemoryTest, GetImageCompressedReturnsInMemoryCompressedImage)
{
	// kMemBinDataKept=true keeps imageCompressed populated on the in-memory signature.
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";
	Memory memory(params);
	ASSERT_TRUE(memory.init(""));

	cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
	cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
	ASSERT_TRUE(memory.update(SensorData(image), Transform(0, 0, 0, 0, 0, 0), covariance));
	const int id = memory.getLastSignatureId();

	cv::Mat compressed = memory.getImageCompressed(id);
	EXPECT_FALSE(compressed.empty());
	memory.close(false);
}

TEST_F(MemoryFixture, GetImageCompressedReturnsEmptyForUnknownId)
{
	EXPECT_TRUE(memory_->getImageCompressed(9999).empty());
}

// ---------------------------------------------------------------------------
// getNodeWordsAndGlobalDescriptors / getNodeCalibration
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetNodeCalibrationReturnsAttachedCameraModel)
{
	// Attach a CameraModel to the SensorData and verify getNodeCalibration returns it.
	const CameraModel camera(500.0, 500.0, 4.0, 4.0,
			CameraModel::opticalRotation(), 0.0, cv::Size(8, 8));
	SensorData data(image_, camera);
	data.setId(0);
	ASSERT_TRUE(memory_->update(data, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int id = memory_->getLastSignatureId();

	std::vector<CameraModel> models;
	std::vector<StereoCameraModel> stereoModels;
	memory_->getNodeCalibration(id, models, stereoModels);

	ASSERT_EQ(models.size(), 1u);
	EXPECT_EQ(models[0].imageWidth(), 8);
	EXPECT_EQ(models[0].imageHeight(), 8);
	EXPECT_DOUBLE_EQ(models[0].fx(), 500.0);
	EXPECT_DOUBLE_EQ(models[0].fy(), 500.0);
	EXPECT_EQ(stereoModels.size(), 0u);
}

TEST_F(MemoryFixture, GetNodeCalibrationIsNoOpForUnknownId)
{
	std::vector<CameraModel> models;
	std::vector<StereoCameraModel> stereoModels;
	memory_->getNodeCalibration(9999, models, stereoModels);
	EXPECT_EQ(models.size(), 0u);
	EXPECT_EQ(stereoModels.size(), 0u);
}

TEST_F(MemoryFixture, GetNodeWordsReturnsSignaturesWords)
{
	// With kKpMaxFeatures=-1, no features are extracted by the pipeline, so the
	// signature has no words. The accessor still returns successfully with empty
	// outputs.
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();

	std::multimap<int, int> words;
	std::vector<cv::KeyPoint> wordsKpts;
	std::vector<cv::Point3f> words3;
	cv::Mat wordsDescriptors;
	std::vector<GlobalDescriptor> globalDescriptors;
	memory_->getNodeWordsAndGlobalDescriptors(id, words, wordsKpts, words3, wordsDescriptors, globalDescriptors);

	EXPECT_EQ(words.size(), 0u);
	EXPECT_EQ(wordsKpts.size(), 0u);
	EXPECT_EQ(words3.size(), 0u);
	EXPECT_EQ(wordsDescriptors.cols, 0);
	EXPECT_EQ(globalDescriptors.size(), 0u);
}

// ---------------------------------------------------------------------------
// getGroundTruthPose
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetGroundTruthPoseReturnsAttachedGroundTruth)
{
	const Transform gt(1.5f, -2.0f, 0.25f, 0.0f, 0.0f, 0.3f);
	SensorData data(image_);
	data.setGroundTruth(gt);
	data.setId(0);
	ASSERT_TRUE(memory_->update(data, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int id = memory_->getLastSignatureId();

	const Transform retrieved = memory_->getGroundTruthPose(id);
	EXPECT_TRUE(retrieved.getDistance(gt) < 1e-5f);
}

TEST_F(MemoryFixture, GetGroundTruthPoseReturnsNullForUnknownId)
{
	EXPECT_TRUE(memory_->getGroundTruthPose(9999).isNull());
}

// ---------------------------------------------------------------------------
// getGPS
// ---------------------------------------------------------------------------

TEST_F(MemoryFixture, GetGpsReturnsAttachedGps)
{
	// GPS attached directly to the signature -> returned verbatim, offsetENU stays
	// identity.
	SensorData data(image_);
	const GPS attached(/*stamp=*/1234.5, /*lon=*/-73.5, /*lat=*/45.5, /*alt=*/30.0, /*err=*/2.0, /*bearing=*/0.0);
	data.setGPS(attached);
	data.setId(0);
	ASSERT_TRUE(memory_->update(data, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int id = memory_->getLastSignatureId();

	GPS gps;
	Transform offsetENU;
	memory_->getGPS(id, gps, offsetENU, /*lookInDatabase=*/false, /*maxGraphDepth=*/0);

	EXPECT_DOUBLE_EQ(gps.stamp(), attached.stamp());
	EXPECT_DOUBLE_EQ(gps.longitude(), attached.longitude());
	EXPECT_DOUBLE_EQ(gps.latitude(), attached.latitude());
	EXPECT_TRUE(offsetENU.isIdentity());
}

TEST_F(MemoryFixture, GetGpsSearchesNeighborsWhenSignatureHasNoGps)
{
	// Two-node chain A-B. GPS is only attached to B. Calling getGPS on A with
	// maxGraphDepth=2 must walk the neighbor graph to B and return B's GPS.
	const GPS bGps(/*stamp=*/4567.8, /*lon=*/-71.0, /*lat=*/46.8, /*alt=*/10.0, /*err=*/1.0, /*bearing=*/0.0);

	SensorData dataA(image_);
	ASSERT_TRUE(memory_->update(dataA, Transform(0, 0, 0, 0, 0, 0), covariance_));
	const int A = memory_->getLastSignatureId();

	SensorData dataB(image_);
	dataB.setGPS(bGps);
	ASSERT_TRUE(memory_->update(dataB, Transform(1, 0, 0, 0, 0, 0), covariance_));

	GPS gps;
	Transform offsetENU;
	memory_->getGPS(A, gps, offsetENU, /*lookInDatabase=*/false, /*maxGraphDepth=*/2);

	EXPECT_DOUBLE_EQ(gps.stamp(), bGps.stamp());
	EXPECT_DOUBLE_EQ(gps.longitude(), bGps.longitude());
	EXPECT_DOUBLE_EQ(gps.latitude(), bGps.latitude());
}

TEST_F(MemoryFixture, GetGpsReturnsDefaultWhenNoNeighborHasGps)
{
	// Nobody has a GPS fix -> the function returns the default GPS (stamp=0) and
	// identity offset.
	ASSERT_TRUE(update());
	const int id = memory_->getLastSignatureId();

	GPS gps;
	Transform offsetENU;
	memory_->getGPS(id, gps, offsetENU, /*lookInDatabase=*/false, /*maxGraphDepth=*/0);

	EXPECT_DOUBLE_EQ(gps.stamp(), 0.0);
	EXPECT_TRUE(offsetENU.isIdentity());
}

// ---------------------------------------------------------------------------
// cleanupLocalGrids
// ---------------------------------------------------------------------------

TEST(MemoryTest, CleanupLocalGridsReturnsErrorWithoutDatabase)
{
	// Without init() there is no _dbDriver -> the function logs an error and
	// returns -1 regardless of inputs.
	Memory memory(defaultMemoryParams());
	std::map<int, Transform> poses{{1, Transform::getIdentity()}};
	cv::Mat map(4, 4, CV_8UC1, cv::Scalar(0));
	EXPECT_EQ(memory.cleanupLocalGrids(poses, map, 0.0f, 0.0f, 0.1f), -1);
}

TEST(MemoryTest, CleanupLocalGridsReturnsErrorWhenPosesEmpty)
{
	Memory memory(defaultMemoryParams());
	ASSERT_TRUE(memory.init(""));
	cv::Mat map(4, 4, CV_8UC1, cv::Scalar(0));
	EXPECT_EQ(memory.cleanupLocalGrids({}, map, 0.0f, 0.0f, 0.1f), -1);
	memory.close(false);
}

TEST(MemoryTest, CleanupLocalGridsReturnsErrorWhenMapEmpty)
{
	Memory memory(defaultMemoryParams());
	ASSERT_TRUE(memory.init(""));
	std::map<int, Transform> poses{{1, Transform::getIdentity()}};
	EXPECT_EQ(memory.cleanupLocalGrids(poses, cv::Mat(), 0.0f, 0.0f, 0.1f), -1);
	memory.close(false);
}

TEST(MemoryTest, CleanupLocalGridsFiltersObstaclesAgainstMap)
{
	// Setup: a signature with 3 obstacle cells at world positions
	//   (0.0, 0.0), (0.1, 0.0), (10.0, 10.0)
	// and a 4x4 map with cellSize=1, xMin=-2, yMin=-2 where only the cell at
	// world (0, 0) is marked as obstacle (=100). All other cells are free (=0).
	// The first two obstacles fall on cells that map to the (0,0) area; the third
	// falls outside the map and is dropped. Final obstacle count: 2 -> 2 (no
	// change), or filtered depending on exact cell mapping; we assert the function
	// runs and returns a non-negative count.
	const std::string dbPath = uniqueDbPath();
	ParametersMap params = defaultMemoryParams();
	params[Parameters::kMemBinDataKept()] = "true";
	params[Parameters::kRGBDCreateOccupancyGrid()] = "true";
	{
		Memory memory(params);
		ASSERT_TRUE(memory.init(dbPath, true));

		// Build an obstacles point cloud: 1 row, 3 cols, CV_32FC2 (2D points).
		cv::Mat obstacles(1, 3, CV_32FC2);
		obstacles.at<cv::Vec2f>(0, 0) = cv::Vec2f(0.0f, 0.0f);
		obstacles.at<cv::Vec2f>(0, 1) = cv::Vec2f(0.1f, 0.0f);
		obstacles.at<cv::Vec2f>(0, 2) = cv::Vec2f(10.0f, 10.0f);
		const float kCellSize = 1.0f;

		cv::Mat image(8, 8, CV_8UC1, cv::Scalar(128));
		SensorData data(image);
		data.setOccupancyGrid(/*ground=*/cv::Mat(), obstacles, /*empty=*/cv::Mat(), kCellSize, cv::Point3f(0, 0, 0));
		data.setId(0);
		cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.01;
		ASSERT_TRUE(memory.update(data, Transform(0, 0, 0, 0, 0, 0), covariance));
		const int id = memory.getLastSignatureId();

		// Map: 4x4 cells, only (col=2, row=2) marked as obstacle (covers world (0,0)).
		cv::Mat map(4, 4, CV_8UC1, cv::Scalar(0));
		map.at<unsigned char>(2, 2) = 100;

		// Pose: identity. xMin=-2, yMin=-2, cellSize=1 -> world (0,0) hits cell (2,2).
		std::map<int, Transform> poses{{id, Transform::getIdentity()}};
		const int modified = memory.cleanupLocalGrids(poses, map, /*xMin=*/-2.0f, /*yMin=*/-2.0f, kCellSize, /*cropRadius=*/0, /*filterScans=*/false);
		// Two of the three obstacles land on the marked cell (kept); the (10,10)
		// obstacle falls outside the map and is dropped. modified=1 grid touched.
		EXPECT_EQ(modified, 1);

		memory.close(false);
	}
	UFile::erase(dbPath.c_str());
}
