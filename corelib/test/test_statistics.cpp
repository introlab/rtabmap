#include <gtest/gtest.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>
#include <cmath>

using namespace rtabmap;

// Constructor Tests

TEST(StatisticsTest, DefaultConstructor)
{
    Statistics stats;
    
    EXPECT_FALSE(stats.extended());
    EXPECT_EQ(stats.refImageId(), 0);
    EXPECT_EQ(stats.refImageMapId(), -1);
    EXPECT_EQ(stats.loopClosureId(), 0);
    EXPECT_EQ(stats.loopClosureMapId(), -1);
    EXPECT_EQ(stats.proximityDetectionId(), 0);
    EXPECT_EQ(stats.proximityDetectionMapId(), -1);
    EXPECT_DOUBLE_EQ(stats.stamp(), 0.0);
    EXPECT_EQ(stats.currentGoalId(), 0);
    EXPECT_TRUE(stats.data().empty());
    EXPECT_TRUE(stats.getSignaturesData().empty());
    EXPECT_TRUE(stats.poses().empty());
    EXPECT_TRUE(stats.constraints().empty());
}

// Static Methods Tests

TEST(StatisticsTest, DefaultData)
{
    // Create a Statistics object to initialize default data
    Statistics stats;
    
    const auto& defaultData = Statistics::defaultData();
    
    // Should contain all predefined statistics
    EXPECT_FALSE(defaultData.empty());
    
    // Check some known statistics exist
    EXPECT_NE(defaultData.find(Statistics::kTimingTotal()), defaultData.end());
    EXPECT_NE(defaultData.find(Statistics::kLoopId()), defaultData.end());
    EXPECT_NE(defaultData.find(Statistics::kMemoryWorking_memory_size()), defaultData.end());
    
    // All default values should be 0.0
    for(const auto& pair : defaultData)
    {
        EXPECT_FLOAT_EQ(pair.second, 0.0f) << "Statistic: " << pair.first;
    }
}

TEST(StatisticsTest, SerializeData)
{
    std::map<std::string, float> data;
    data["Timing/Total/ms"] = 500.0f;
    data["Loop/Id/"] = 42.0f;
    data["Memory/Working_memory_size/"] = 100.0f;
    
    std::string serialized = Statistics::serializeData(data);
    
    // Should contain all keys and values
    EXPECT_NE(serialized.find("Timing/Total/ms:500"), std::string::npos);
    EXPECT_NE(serialized.find("Loop/Id/:42"), std::string::npos);
    EXPECT_NE(serialized.find("Memory/Working_memory_size/:100"), std::string::npos);
    
    // Should use semicolons as separators
    EXPECT_NE(serialized.find(";"), std::string::npos);
    
    // Should use dots for decimal separators (not commas)
    data["Test/Value/"] = 3.14159f;
    serialized = Statistics::serializeData(data);
    EXPECT_NE(serialized.find("3.14159"), std::string::npos);
    EXPECT_EQ(serialized.find("3,14159"), std::string::npos);
}

TEST(StatisticsTest, SerializeDataEmpty)
{
    std::map<std::string, float> emptyData;
    std::string serialized = Statistics::serializeData(emptyData);
    EXPECT_TRUE(serialized.empty());
}

TEST(StatisticsTest, DeserializeData)
{
    std::string serialized = "Timing/Total/ms:500.0;Loop/Id/:42;Memory/Working_memory_size/:100.5";
    
    auto deserialized = Statistics::deserializeData(serialized);
    
    EXPECT_EQ(deserialized.size(), 3u);
    EXPECT_FLOAT_EQ(deserialized["Timing/Total/ms"], 500.0f);
    EXPECT_FLOAT_EQ(deserialized["Loop/Id/"], 42.0f);
    EXPECT_FLOAT_EQ(deserialized["Memory/Working_memory_size/"], 100.5f);
}

TEST(StatisticsTest, DeserializeDataEmpty)
{
    auto deserialized = Statistics::deserializeData("");
    EXPECT_TRUE(deserialized.empty());
}

TEST(StatisticsTest, DeserializeDataInvalid)
{
    // Invalid entries should be skipped
    std::string serialized = "Valid/Key/:123.0;InvalidEntry;Another/Valid/:456.0;MissingColon";
    
    auto deserialized = Statistics::deserializeData(serialized);
    
    EXPECT_EQ(deserialized.size(), 2u);
    EXPECT_FLOAT_EQ(deserialized["Valid/Key/"], 123.0f);
    EXPECT_FLOAT_EQ(deserialized["Another/Valid/"], 456.0f);
}

TEST(StatisticsTest, SerializeDeserializeRoundTrip)
{
    std::map<std::string, float> original;
    original["Timing/Total/ms"] = 500.0f;
    original["Loop/Id/"] = 42.0f;
    original["Memory/Working_memory_size/"] = 100.5f;
    original["Test/Float/"] = 3.14159f;
    original["Test/Negative/"] = -10.5f;
    
    std::string serialized = Statistics::serializeData(original);
    auto deserialized = Statistics::deserializeData(serialized);
    
    EXPECT_EQ(deserialized.size(), original.size());
    for(const auto& pair : original)
    {
        EXPECT_NE(deserialized.find(pair.first), deserialized.end());
        EXPECT_FLOAT_EQ(deserialized[pair.first], pair.second);
    }
}

// addStatistic Tests

TEST(StatisticsTest, AddStatistic)
{
    Statistics stats;
    
    stats.addStatistic("Timing/Total/ms", 500.0f);
    stats.addStatistic("Loop/Id/", 42.0f);
    
    const auto& data = stats.data();
    EXPECT_EQ(data.size(), 2u);
    EXPECT_FLOAT_EQ(data.at("Timing/Total/ms"), 500.0f);
    EXPECT_FLOAT_EQ(data.at("Loop/Id/"), 42.0f);
}

TEST(StatisticsTest, AddStatisticOverwrite)
{
    Statistics stats;
    
    stats.addStatistic("Timing/Total/ms", 500.0f);
    stats.addStatistic("Timing/Total/ms", 750.0f); // Overwrite
    
    const auto& data = stats.data();
    EXPECT_EQ(data.size(), 1u);
    EXPECT_FLOAT_EQ(data.at("Timing/Total/ms"), 750.0f);
}

TEST(StatisticsTest, AddStatisticWithRTABMAP_STATS)
{
    Statistics stats;
    
    // Use the generated static methods
    stats.addStatistic(Statistics::kTimingTotal(), 500.0f);
    stats.addStatistic(Statistics::kLoopId(), 42.0f);
    stats.addStatistic(Statistics::kMemoryWorking_memory_size(), 100.0f);
    
    const auto& data = stats.data();
    EXPECT_EQ(data.size(), 3u);
    EXPECT_FLOAT_EQ(data.at(Statistics::kTimingTotal()), 500.0f);
    EXPECT_FLOAT_EQ(data.at(Statistics::kLoopId()), 42.0f);
    EXPECT_FLOAT_EQ(data.at(Statistics::kMemoryWorking_memory_size()), 100.0f);
}

// Extended Mode Tests

TEST(StatisticsTest, ExtendedMode)
{
    Statistics stats;
    
    EXPECT_FALSE(stats.extended());
    
    stats.setExtended(true);
    EXPECT_TRUE(stats.extended());
    
    stats.setExtended(false);
    EXPECT_FALSE(stats.extended());
}

// Setter/Getter Tests

TEST(StatisticsTest, RefImageId)
{
    Statistics stats;
    
    stats.setRefImageId(123);
    EXPECT_EQ(stats.refImageId(), 123);
    
    stats.setRefImageId(456);
    EXPECT_EQ(stats.refImageId(), 456);
}

TEST(StatisticsTest, RefImageMapId)
{
    Statistics stats;
    
    stats.setRefImageMapId(1);
    EXPECT_EQ(stats.refImageMapId(), 1);
    
    stats.setRefImageMapId(-1);
    EXPECT_EQ(stats.refImageMapId(), -1);
}

TEST(StatisticsTest, LoopClosureId)
{
    Statistics stats;
    
    stats.setLoopClosureId(789);
    EXPECT_EQ(stats.loopClosureId(), 789);
    
    stats.setLoopClosureId(0);
    EXPECT_EQ(stats.loopClosureId(), 0);
}

TEST(StatisticsTest, LoopClosureMapId)
{
    Statistics stats;
    
    stats.setLoopClosureMapId(2);
    EXPECT_EQ(stats.loopClosureMapId(), 2);
    
    stats.setLoopClosureMapId(-1);
    EXPECT_EQ(stats.loopClosureMapId(), -1);
}

TEST(StatisticsTest, ProximityDetectionId)
{
    Statistics stats;
    
    stats.setProximityDetectionId(101);
    EXPECT_EQ(stats.proximityDetectionId(), 101);
}

TEST(StatisticsTest, ProximityDetectionMapId)
{
    Statistics stats;
    
    stats.setProximityDetectionMapId(3);
    EXPECT_EQ(stats.proximityDetectionMapId(), 3);
}

TEST(StatisticsTest, Stamp)
{
    Statistics stats;
    
    stats.setStamp(12345.678);
    EXPECT_DOUBLE_EQ(stats.stamp(), 12345.678);
    
    stats.setStamp(0.0);
    EXPECT_DOUBLE_EQ(stats.stamp(), 0.0);
}

// Signature Data Tests

TEST(StatisticsTest, AddSignatureData)
{
    Statistics stats;
    
    Signature sig1(1, 0, 0, 100.0);
    Signature sig2(2, 0, 0, 200.0);
    
    stats.addSignatureData(sig1);
    stats.addSignatureData(sig2);
    
    const auto& signatures = stats.getSignaturesData();
    EXPECT_EQ(signatures.size(), 2u);
    EXPECT_NE(signatures.find(1), signatures.end());
    EXPECT_NE(signatures.find(2), signatures.end());
    EXPECT_EQ(signatures.at(1).id(), 1);
    EXPECT_EQ(signatures.at(2).id(), 2);
}

TEST(StatisticsTest, AddSignatureDataOverwrite)
{
    Statistics stats;
    
    Signature sig1(1, 0, 0, 100.0);
    Signature sig2(1, 0, 0, 200.0); // Same ID
    
    stats.addSignatureData(sig1);
    stats.addSignatureData(sig2); // Should overwrite
    
    const auto& signatures = stats.getSignaturesData();
    EXPECT_EQ(signatures.size(), 1u);
    EXPECT_DOUBLE_EQ(signatures.at(1).getStamp(), 200.0);
}

TEST(StatisticsTest, SetSignaturesData)
{
    Statistics stats;
    
    std::map<int, Signature> sigs;
    sigs[1] = Signature(1, 0, 0, 100.0);
    sigs[2] = Signature(2, 0, 0, 200.0);
    sigs[3] = Signature(3, 0, 0, 300.0);
    
    stats.setSignaturesData(sigs);
    
    const auto& signatures = stats.getSignaturesData();
    EXPECT_EQ(signatures.size(), 3u);
    EXPECT_NE(signatures.find(1), signatures.end());
    EXPECT_NE(signatures.find(2), signatures.end());
    EXPECT_NE(signatures.find(3), signatures.end());
}

TEST(StatisticsTest, GetLastSignatureData)
{
    Statistics stats;
    
    // Empty should return dummy
    const auto& empty = stats.getLastSignatureData();
    EXPECT_EQ(empty.id(), 0);
    
    Signature sig1(1, 0, 0, 100.0);
    Signature sig2(2, 0, 0, 200.0);
    Signature sig3(3, 0, 0, 300.0);
    
    stats.addSignatureData(sig1);
    stats.addSignatureData(sig2);
    stats.addSignatureData(sig3);
    
    // Should return the last one (highest ID)
    const auto& last = stats.getLastSignatureData();
    EXPECT_EQ(last.id(), 3);
    EXPECT_DOUBLE_EQ(last.getStamp(), 300.0);
}

// Poses and Constraints Tests

TEST(StatisticsTest, SetPoses)
{
    Statistics stats;
    
    std::map<int, Transform> poses;
    poses[1] = Transform(1.0f, 0.0f, 0.0f, 0, 0, 0);
    poses[2] = Transform(2.0f, 0.0f, 0.0f, 0, 0, 0);
    poses[3] = Transform(3.0f, 0.0f, 0.0f, 0, 0, 0);
    
    stats.setPoses(poses);
    
    const auto& retrievedPoses = stats.poses();
    EXPECT_EQ(retrievedPoses.size(), 3u);
    EXPECT_NE(retrievedPoses.find(1), retrievedPoses.end());
    EXPECT_NE(retrievedPoses.find(2), retrievedPoses.end());
    EXPECT_NE(retrievedPoses.find(3), retrievedPoses.end());
    EXPECT_FLOAT_EQ(retrievedPoses.at(1).x(), 1.0f);
    EXPECT_FLOAT_EQ(retrievedPoses.at(2).x(), 2.0f);
    EXPECT_FLOAT_EQ(retrievedPoses.at(3).x(), 3.0f);
}

TEST(StatisticsTest, SetConstraints)
{
    Statistics stats;
    
    std::multimap<int, Link> constraints;
    constraints.insert(std::make_pair(1, Link(1, 2, Link::kNeighbor, Transform::getIdentity())));
    constraints.insert(std::make_pair(1, Link(1, 3, Link::kNeighbor, Transform::getIdentity())));
    constraints.insert(std::make_pair(2, Link(2, 3, Link::kGlobalClosure, Transform::getIdentity())));
    
    stats.setConstraints(constraints);
    
    const auto& retrievedConstraints = stats.constraints();
    EXPECT_EQ(retrievedConstraints.size(), 3u);
    
    // Count constraints for node 1
    auto range = retrievedConstraints.equal_range(1);
    int count = std::distance(range.first, range.second);
    EXPECT_EQ(count, 2);
}

// Transform Tests

TEST(StatisticsTest, MapCorrection)
{
    Statistics stats;
    
    Transform correction(1.0f, 2.0f, 3.0f, 0, 0, 0);
    stats.setMapCorrection(correction);
    
    const auto& retrieved = stats.mapCorrection();
    EXPECT_FLOAT_EQ(retrieved.x(), 1.0f);
    EXPECT_FLOAT_EQ(retrieved.y(), 2.0f);
    EXPECT_FLOAT_EQ(retrieved.z(), 3.0f);
}

TEST(StatisticsTest, LoopClosureTransform)
{
    Statistics stats;
    
    Transform transform(0.5f, 0.5f, 0.5f, 0, 0, 0);
    stats.setLoopClosureTransform(transform);
    
    const auto& retrieved = stats.loopClosureTransform();
    EXPECT_FLOAT_EQ(retrieved.x(), 0.5f);
    EXPECT_FLOAT_EQ(retrieved.y(), 0.5f);
    EXPECT_FLOAT_EQ(retrieved.z(), 0.5f);
}

TEST(StatisticsTest, LocalizationCovariance)
{
    Statistics stats;
    
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.1;
    stats.setLocalizationCovariance(covariance);
    
    const auto& retrieved = stats.localizationCovariance();
    EXPECT_FALSE(retrieved.empty());
    EXPECT_EQ(retrieved.rows, 6);
    EXPECT_EQ(retrieved.cols, 6);
    EXPECT_DOUBLE_EQ(retrieved.at<double>(0, 0), 0.1);
}

// Labels and Weights Tests

TEST(StatisticsTest, Labels)
{
    Statistics stats;
    
    std::map<int, std::string> labels;
    labels[1] = "room1";
    labels[2] = "room2";
    labels[3] = "corridor";
    
    stats.setLabels(labels);
    
    const auto& retrieved = stats.labels();
    EXPECT_EQ(retrieved.size(), 3u);
    EXPECT_EQ(retrieved.at(1), "room1");
    EXPECT_EQ(retrieved.at(2), "room2");
    EXPECT_EQ(retrieved.at(3), "corridor");
}

TEST(StatisticsTest, Weights)
{
    Statistics stats;
    
    std::map<int, int> weights;
    weights[1] = 10;
    weights[2] = 20;
    weights[3] = 30;
    
    stats.setWeights(weights);
    
    const auto& retrieved = stats.weights();
    EXPECT_EQ(retrieved.size(), 3u);
    EXPECT_EQ(retrieved.at(1), 10);
    EXPECT_EQ(retrieved.at(2), 20);
    EXPECT_EQ(retrieved.at(3), 30);
}

// Likelihood and Posterior Tests

TEST(StatisticsTest, Likelihood)
{
    Statistics stats;
    
    std::map<int, float> likelihood;
    likelihood[1] = 0.8f;
    likelihood[2] = 0.6f;
    likelihood[3] = 0.4f;
    
    stats.setLikelihood(likelihood);
    
    const auto& retrieved = stats.likelihood();
    EXPECT_EQ(retrieved.size(), 3u);
    EXPECT_FLOAT_EQ(retrieved.at(1), 0.8f);
    EXPECT_FLOAT_EQ(retrieved.at(2), 0.6f);
    EXPECT_FLOAT_EQ(retrieved.at(3), 0.4f);
}

TEST(StatisticsTest, RawLikelihood)
{
    Statistics stats;
    
    std::map<int, float> rawLikelihood;
    rawLikelihood[1] = 100.0f;
    rawLikelihood[2] = 50.0f;
    
    stats.setRawLikelihood(rawLikelihood);
    
    const auto& retrieved = stats.rawLikelihood();
    EXPECT_EQ(retrieved.size(), 2u);
    EXPECT_FLOAT_EQ(retrieved.at(1), 100.0f);
    EXPECT_FLOAT_EQ(retrieved.at(2), 50.0f);
}

TEST(StatisticsTest, Posterior)
{
    Statistics stats;
    
    std::map<int, float> posterior;
    posterior[1] = 0.9f;
    posterior[2] = 0.1f;
    
    stats.setPosterior(posterior);
    
    const auto& retrieved = stats.posterior();
    EXPECT_EQ(retrieved.size(), 2u);
    EXPECT_FLOAT_EQ(retrieved.at(1), 0.9f);
    EXPECT_FLOAT_EQ(retrieved.at(2), 0.1f);
}

// Path and Goal Tests

TEST(StatisticsTest, LocalPath)
{
    Statistics stats;
    
    std::vector<int> path = {1, 2, 3, 4, 5};
    stats.setLocalPath(path);
    
    const auto& retrieved = stats.localPath();
    EXPECT_EQ(retrieved.size(), 5u);
    EXPECT_EQ(retrieved[0], 1);
    EXPECT_EQ(retrieved[1], 2);
    EXPECT_EQ(retrieved[2], 3);
    EXPECT_EQ(retrieved[3], 4);
    EXPECT_EQ(retrieved[4], 5);
}

TEST(StatisticsTest, CurrentGoalId)
{
    Statistics stats;
    
    stats.setCurrentGoalId(42);
    EXPECT_EQ(stats.currentGoalId(), 42);
    
    stats.setCurrentGoalId(0);
    EXPECT_EQ(stats.currentGoalId(), 0);
}

// Reduced IDs Tests

TEST(StatisticsTest, ReducedIds)
{
    Statistics stats;
    
    std::map<int, int> reducedIds;
    reducedIds[100] = 1;
    reducedIds[200] = 2;
    reducedIds[300] = 3;
    
    stats.setReducedIds(reducedIds);
    
    const auto& retrieved = stats.reducedIds();
    EXPECT_EQ(retrieved.size(), 3u);
    EXPECT_EQ(retrieved.at(100), 1);
    EXPECT_EQ(retrieved.at(200), 2);
    EXPECT_EQ(retrieved.at(300), 3);
}

// Working Memory State Tests

TEST(StatisticsTest, WmState)
{
    Statistics stats;
    
    std::vector<int> wmState = {10, 20, 30, 40};
    stats.setWmState(wmState);
    
    const auto& retrieved = stats.wmState();
    EXPECT_EQ(retrieved.size(), 4u);
    EXPECT_EQ(retrieved[0], 10);
    EXPECT_EQ(retrieved[1], 20);
    EXPECT_EQ(retrieved[2], 30);
    EXPECT_EQ(retrieved[3], 40);
}

// Odometry Cache Tests

TEST(StatisticsTest, OdomCachePoses)
{
    Statistics stats;
    
    std::map<int, Transform> odomPoses;
    odomPoses[1] = Transform(1.0f, 0.0f, 0.0f, 0, 0, 0);
    odomPoses[2] = Transform(2.0f, 0.0f, 0.0f, 0, 0, 0);
    
    stats.setOdomCachePoses(odomPoses);
    
    const auto& retrieved = stats.odomCachePoses();
    EXPECT_EQ(retrieved.size(), 2u);
    EXPECT_FLOAT_EQ(retrieved.at(1).x(), 1.0f);
    EXPECT_FLOAT_EQ(retrieved.at(2).x(), 2.0f);
}

TEST(StatisticsTest, OdomCacheConstraints)
{
    Statistics stats;
    
    std::multimap<int, Link> odomConstraints;
    odomConstraints.insert(std::make_pair(1, Link(1, 2, Link::kNeighbor, Transform::getIdentity())));
    odomConstraints.insert(std::make_pair(2, Link(2, 3, Link::kNeighbor, Transform::getIdentity())));
    
    stats.setOdomCacheConstraints(odomConstraints);
    
    const auto& retrieved = stats.odomCacheConstraints();
    EXPECT_EQ(retrieved.size(), 2u);
}

// Comprehensive Test

TEST(StatisticsTest, ComprehensiveUsage)
{
    Statistics stats;
    
    // Set basic fields
    stats.setExtended(true);
    stats.setRefImageId(100);
    stats.setRefImageMapId(1);
    stats.setLoopClosureId(50);
    stats.setLoopClosureMapId(1);
    stats.setStamp(12345.678);
    
    // Add statistics
    stats.addStatistic(Statistics::kTimingTotal(), 500.0f);
    stats.addStatistic(Statistics::kLoopId(), 50.0f);
    stats.addStatistic(Statistics::kMemoryWorking_memory_size(), 100.0f);
    
    // Add signature data
    Signature sig(100, 1, 0, 12345.678);
    stats.addSignatureData(sig);
    
    // Set poses
    std::map<int, Transform> poses;
    poses[100] = Transform(1.0f, 2.0f, 3.0f, 0, 0, 0);
    stats.setPoses(poses);
    
    // Set constraints
    std::multimap<int, Link> constraints;
    constraints.insert(std::make_pair(100, Link(100, 50, Link::kGlobalClosure, Transform::getIdentity())));
    stats.setConstraints(constraints);
    
    // Set transforms
    stats.setMapCorrection(Transform(0.1f, 0.2f, 0.3f, 0, 0, 0));
    stats.setLoopClosureTransform(Transform(0.5f, 0.5f, 0.5f, 0, 0, 0));
    
    // Set likelihood and posterior
    std::map<int, float> likelihood;
    likelihood[50] = 0.9f;
    stats.setLikelihood(likelihood);
    
    std::map<int, float> posterior;
    posterior[50] = 0.95f;
    stats.setPosterior(posterior);
    
    // Verify everything
    EXPECT_TRUE(stats.extended());
    EXPECT_EQ(stats.refImageId(), 100);
    EXPECT_EQ(stats.loopClosureId(), 50);
    EXPECT_EQ(stats.data().size(), 3u);
    EXPECT_EQ(stats.getSignaturesData().size(), 1u);
    EXPECT_EQ(stats.poses().size(), 1u);
    EXPECT_EQ(stats.constraints().size(), 1u);
    EXPECT_FLOAT_EQ(stats.likelihood().at(50), 0.9f);
    EXPECT_FLOAT_EQ(stats.posterior().at(50), 0.95f);
}

