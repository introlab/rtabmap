#include <gtest/gtest.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cmath>

using namespace rtabmap;

// Constructor Tests

TEST(SignatureTest, DefaultConstructor)
{
    Signature sig;
    
    EXPECT_EQ(sig.id(), 0);
    EXPECT_EQ(sig.mapId(), -1);
    EXPECT_DOUBLE_EQ(sig.getStamp(), 0.0);
    EXPECT_EQ(sig.getWeight(), 0);
    EXPECT_TRUE(sig.getLabel().empty());
    EXPECT_TRUE(sig.getPose().isNull());
    EXPECT_TRUE(sig.getGroundTruthPose().isNull());
    EXPECT_FALSE(sig.isSaved());
    EXPECT_TRUE(sig.isModified());
    EXPECT_TRUE(sig.isLinksModified());
    EXPECT_FALSE(sig.isEnabled());
    EXPECT_TRUE(sig.getWords().empty());
    EXPECT_TRUE(sig.getWordsKpts().empty());
    EXPECT_TRUE(sig.getWords3().empty());
    EXPECT_TRUE(sig.getWordsDescriptors().empty());
    EXPECT_TRUE(sig.getLinks().empty());
    EXPECT_TRUE(sig.getLandmarks().empty());
    EXPECT_TRUE(sig.isBadSignature());
}

TEST(SignatureTest, ParameterizedConstructor)
{
    int id = 100;
    int mapId = 1;
    int weight = 5;
    double stamp = 12345.678;
    std::string label = "test_location";
    Transform pose(1.0f, 2.0f, 3.0f, 0, 0, 0);
    Transform groundTruth(1.1f, 2.1f, 3.1f, 0, 0, 0);
    
    SensorData sensorData;
    sensorData.setId(id);
    
    Signature sig(id, mapId, weight, stamp, label, pose, groundTruth, sensorData);
    
    EXPECT_EQ(sig.id(), id);
    EXPECT_EQ(sig.mapId(), mapId);
    EXPECT_EQ(sig.getWeight(), weight);
    EXPECT_DOUBLE_EQ(sig.getStamp(), stamp);
    EXPECT_EQ(sig.getLabel(), label);
    EXPECT_FLOAT_EQ(sig.getPose().x(), 1.0f);
    EXPECT_FLOAT_EQ(sig.getPose().y(), 2.0f);
    EXPECT_FLOAT_EQ(sig.getPose().z(), 3.0f);
    EXPECT_FLOAT_EQ(sig.getGroundTruthPose().x(), 1.1f);
    EXPECT_FALSE(sig.isSaved());
    EXPECT_TRUE(sig.isModified());
    EXPECT_FALSE(sig.isEnabled());
}

TEST(SignatureTest, ParameterizedConstructorDefaults)
{
    Signature sig(42);
    
    EXPECT_EQ(sig.id(), 42);
    EXPECT_EQ(sig.mapId(), -1);
    EXPECT_EQ(sig.getWeight(), 0);
    EXPECT_DOUBLE_EQ(sig.getStamp(), 0.0);
    EXPECT_TRUE(sig.getLabel().empty());
    EXPECT_TRUE(sig.getPose().isNull());
    EXPECT_TRUE(sig.getGroundTruthPose().isNull());
}

TEST(SignatureTest, ConstructorFromSensorData)
{
    SensorData sensorData;
    sensorData.setId(200);
    sensorData.setStamp(54321.0);
    Transform gt(5.0f, 6.0f, 7.0f, 0, 0, 0);
    sensorData.setGroundTruth(gt);
    
    Signature sig(sensorData);
    
    EXPECT_EQ(sig.id(), 200);
    EXPECT_EQ(sig.mapId(), -1);
    EXPECT_DOUBLE_EQ(sig.getStamp(), 54321.0);
    EXPECT_TRUE(sig.getPose().isIdentity());
    EXPECT_FLOAT_EQ(sig.getGroundTruthPose().x(), 5.0f);
    EXPECT_FALSE(sig.isSaved());
    EXPECT_TRUE(sig.isModified());
}

// Basic Getters/Setters Tests

TEST(SignatureTest, SetWeight)
{
    Signature sig(1);
    
    EXPECT_TRUE(sig.isModified());
    sig.setModified(false);
    
    sig.setWeight(10);
    EXPECT_EQ(sig.getWeight(), 10);
    EXPECT_TRUE(sig.isModified());
    
    sig.setWeight(10); // Same value
    EXPECT_FALSE(sig.isModified());
}

TEST(SignatureTest, SetLabel)
{
    Signature sig(1);
    
    EXPECT_TRUE(sig.isModified());
    sig.setModified(false);
    
    sig.setLabel("room1");
    EXPECT_EQ(sig.getLabel(), "room1");
    EXPECT_TRUE(sig.isModified());
    
    sig.setLabel("room1"); // Same label
    EXPECT_FALSE(sig.isModified());
    
    sig.setLabel("room2");
    EXPECT_EQ(sig.getLabel(), "room2");
    EXPECT_TRUE(sig.isModified());
}

// Link Management Tests

TEST(SignatureTest, AddLink)
{
    Signature sig(1);
    
    Link link(1, 2, Link::kNeighbor, Transform::getIdentity());
    sig.addLink(link);
    
    const auto& links = sig.getLinks();
    EXPECT_EQ(links.size(), 1u);
    EXPECT_NE(links.find(2), links.end());
    EXPECT_EQ(links.find(2)->second.type(), Link::kNeighbor);
    EXPECT_TRUE(sig.isLinksModified());
}

TEST(SignatureTest, AddLinksFromList)
{
    Signature sig(1);
    
    std::list<Link> links;
    links.push_back(Link(1, 2, Link::kNeighbor, Transform::getIdentity()));
    links.push_back(Link(1, 3, Link::kNeighbor, Transform::getIdentity()));
    links.push_back(Link(1, 4, Link::kGlobalClosure, Transform::getIdentity()));
    
    sig.addLinks(links);
    
    const auto& retrievedLinks = sig.getLinks();
    EXPECT_EQ(retrievedLinks.size(), 3u);
    EXPECT_NE(retrievedLinks.find(2), retrievedLinks.end());
    EXPECT_NE(retrievedLinks.find(3), retrievedLinks.end());
    EXPECT_NE(retrievedLinks.find(4), retrievedLinks.end());
    EXPECT_TRUE(sig.isLinksModified());
}

TEST(SignatureTest, AddLinksFromMap)
{
    Signature sig(1);
    
    std::map<int, Link> links;
    links[2] = Link(1, 2, Link::kNeighbor, Transform::getIdentity());
    links[3] = Link(1, 3, Link::kNeighbor, Transform::getIdentity());
    
    sig.addLinks(links);
    
    const auto& retrievedLinks = sig.getLinks();
    EXPECT_EQ(retrievedLinks.size(), 2u);
    EXPECT_NE(retrievedLinks.find(2), retrievedLinks.end());
    EXPECT_NE(retrievedLinks.find(3), retrievedLinks.end());
}

TEST(SignatureTest, HasLink)
{
    Signature sig(1);
    
    Link link1(1, 2, Link::kNeighbor, Transform::getIdentity());
    Link link2(1, 3, Link::kGlobalClosure, Transform::getIdentity());
    sig.addLink(link1);
    sig.addLink(link2);
    
    EXPECT_TRUE(sig.hasLink(2));
    EXPECT_TRUE(sig.hasLink(3));
    EXPECT_FALSE(sig.hasLink(4));
    
    EXPECT_TRUE(sig.hasLink(2, Link::kNeighbor));
    EXPECT_FALSE(sig.hasLink(2, Link::kGlobalClosure));
    EXPECT_TRUE(sig.hasLink(3, Link::kGlobalClosure));
    
    EXPECT_TRUE(sig.hasLink(0, Link::kNeighbor)); // Check for any neighbor link
    EXPECT_TRUE(sig.hasLink(0, Link::kGlobalClosure)); // Check for any global closure
}

TEST(SignatureTest, ChangeLinkIds)
{
    Signature sig(1);
    
    Link link1(1, 10, Link::kNeighbor, Transform::getIdentity());
    Link link2(1, 10, Link::kGlobalClosure, Transform::getIdentity());
    sig.addLink(link1);
    EXPECT_THROW(sig.addLink(link2), UException); // cannot have two links pointing on same signature
    
    Link link3(1, 11, Link::kGlobalClosure, Transform::getIdentity());
    sig.addLink(link3);
    EXPECT_EQ(sig.getLinks().size(), 2u);
    EXPECT_NE(sig.getLinks().find(11), sig.getLinks().end());
    
    sig.changeLinkIds(11, 20);
    
    const auto& links = sig.getLinks();
    EXPECT_EQ(links.find(11), links.end());
    EXPECT_NE(links.find(20), links.end());
    EXPECT_EQ(links.count(20), 1u);
    EXPECT_TRUE(sig.isLinksModified());
}

TEST(SignatureTest, RemoveLink)
{
    Signature sig(1);
    
    sig.addLink(Link(1, 2, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 3, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 4, Link::kGlobalClosure, Transform::getIdentity()));
    
    EXPECT_EQ(sig.getLinks().size(), 3u);
    
    sig.removeLink(3);
    
    EXPECT_EQ(sig.getLinks().size(), 2u);
    EXPECT_NE(sig.getLinks().find(2), sig.getLinks().end());
    EXPECT_EQ(sig.getLinks().find(3), sig.getLinks().end());
    EXPECT_NE(sig.getLinks().find(4), sig.getLinks().end());
    EXPECT_TRUE(sig.isLinksModified());
}

TEST(SignatureTest, RemoveLinks)
{
    Signature sig(1);
    
    sig.addLink(Link(1, 2, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 3, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 1, Link::kPosePrior, Transform::getIdentity())); // Self-referring
    
    EXPECT_EQ(sig.getLinks().size(), 3u);
    
    sig.removeLinks(false); // Remove all, including self-referring
    
    EXPECT_TRUE(sig.getLinks().empty());
    
    // Add links again
    sig.addLink(Link(1, 2, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 1, Link::kPosePrior, Transform::getIdentity()));
    
    sig.removeLinks(true); // Keep self-referring
    
    EXPECT_EQ(sig.getLinks().size(), 1u);
    EXPECT_NE(sig.getLinks().find(1), sig.getLinks().end());
}

TEST(SignatureTest, RemoveVirtualLinks)
{
    Signature sig(1);
    
    sig.addLink(Link(1, 2, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(1, 3, Link::kVirtualClosure, Transform::getIdentity()));
    sig.addLink(Link(1, 4, Link::kGlobalClosure, Transform::getIdentity()));
    
    EXPECT_EQ(sig.getLinks().size(), 3u);
    
    sig.removeVirtualLinks();
    
    EXPECT_EQ(sig.getLinks().size(), 2u);
    EXPECT_NE(sig.getLinks().find(2), sig.getLinks().end());
    EXPECT_EQ(sig.getLinks().find(3), sig.getLinks().end());
    EXPECT_NE(sig.getLinks().find(4), sig.getLinks().end());
}

// Landmark Tests

TEST(SignatureTest, AddLandmark)
{
    Signature sig(1);
    
    Link landmark(1, -10, Link::kLandmark, Transform(1.0f, 0.0f, 0.0f, 0, 0, 0));
    sig.addLandmark(landmark);
    
    const auto& landmarks = sig.getLandmarks();
    EXPECT_EQ(landmarks.size(), 1u);
    EXPECT_NE(landmarks.find(-10), landmarks.end());
}

TEST(SignatureTest, RemoveLandmark)
{
    Signature sig(1);
    
    sig.addLandmark(Link(1, -10, Link::kLandmark, Transform::getIdentity()));
    sig.addLandmark(Link(1, -20, Link::kLandmark, Transform::getIdentity()));
    
    EXPECT_EQ(sig.getLandmarks().size(), 2u);
    
    sig.removeLandmark(-10);
    
    EXPECT_EQ(sig.getLandmarks().size(), 1u);
    EXPECT_EQ(sig.getLandmarks().find(-10), sig.getLandmarks().end());
    EXPECT_NE(sig.getLandmarks().find(-20), sig.getLandmarks().end());
}

TEST(SignatureTest, RemoveLandmarks)
{
    Signature sig(1);
    
    sig.addLandmark(Link(1, -10, Link::kLandmark, Transform::getIdentity()));
    sig.addLandmark(Link(1, -20, Link::kLandmark, Transform::getIdentity()));
    
    EXPECT_EQ(sig.getLandmarks().size(), 2u);
    
    sig.removeLandmarks();
    
    EXPECT_TRUE(sig.getLandmarks().empty());
}

// Modification Tracking Tests

TEST(SignatureTest, SetSaved)
{
    Signature sig(1);
    
    EXPECT_FALSE(sig.isSaved());
    
    sig.setSaved(true);
    EXPECT_TRUE(sig.isSaved());
    
    sig.setSaved(false);
    EXPECT_FALSE(sig.isSaved());
}

TEST(SignatureTest, SetModified)
{
    Signature sig(1);
    
    EXPECT_TRUE(sig.isModified());
    EXPECT_TRUE(sig.isLinksModified());
    
    sig.setModified(false);
    EXPECT_FALSE(sig.isModified());
    EXPECT_FALSE(sig.isLinksModified());
    
    sig.setModified(true);
    EXPECT_TRUE(sig.isModified());
    EXPECT_TRUE(sig.isLinksModified());
}

// Visual Words Tests

TEST(SignatureTest, SetWords)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(2, 1));
    words.insert(std::make_pair(3, 2));
    
    std::vector<cv::KeyPoint> keypoints;
    keypoints.push_back(cv::KeyPoint(10.0f, 20.0f, 1.0f));
    keypoints.push_back(cv::KeyPoint(30.0f, 40.0f, 1.0f));
    keypoints.push_back(cv::KeyPoint(50.0f, 60.0f, 1.0f));
    
    std::vector<cv::Point3f> words3;
    words3.push_back(cv::Point3f(0.1f, 0.2f, 0.3f));
    words3.push_back(cv::Point3f(0.4f, 0.5f, 0.6f));
    words3.push_back(cv::Point3f(0.7f, 0.8f, 0.9f));
    
    cv::Mat descriptors = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_EQ(sig.getWords().size(), 3u);
    EXPECT_EQ(sig.getWordsKpts().size(), 3u);
    EXPECT_EQ(sig.getWords3().size(), 3u);
    EXPECT_EQ(sig.getWordsDescriptors().rows, 3);
    EXPECT_FALSE(sig.isEnabled());
    EXPECT_EQ(sig.getInvalidWordsCount(), 0);
    EXPECT_FALSE(sig.isBadSignature());
}

TEST(SignatureTest, SetWordsWithInvalidWords)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(0, 1)); // Invalid word (ID <= 0)
    words.insert(std::make_pair(-1, 2)); // Invalid word (ID <= 0)
    words.insert(std::make_pair(2, 3));
    
    std::vector<cv::KeyPoint> keypoints(4);
    std::vector<cv::Point3f> words3(4);
    cv::Mat descriptors = cv::Mat::zeros(4, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_EQ(sig.getWords().size(), 4u);
    EXPECT_EQ(sig.getInvalidWordsCount(), 2);
    EXPECT_FALSE(sig.isBadSignature()); // Has 2 valid words
}

TEST(SignatureTest, SetWordsAllInvalid)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(0, 0));
    words.insert(std::make_pair(-1, 1));
    
    std::vector<cv::KeyPoint> keypoints(2);
    std::vector<cv::Point3f> words3(2);
    cv::Mat descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_EQ(sig.getInvalidWordsCount(), 2);
    EXPECT_TRUE(sig.isBadSignature()); // No valid words
}

TEST(SignatureTest, SetWordsEmpty)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point3f> words3;
    cv::Mat descriptors;
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_TRUE(sig.getWords().empty());
    EXPECT_TRUE(sig.isBadSignature());
}

TEST(SignatureTest, RemoveAllWords)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(2, 1));
    
    std::vector<cv::KeyPoint> keypoints(2);
    std::vector<cv::Point3f> words3(2);
    cv::Mat descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    EXPECT_EQ(sig.getWords().size(), 2u);
    
    sig.removeAllWords();
    
    EXPECT_TRUE(sig.getWords().empty());
    EXPECT_TRUE(sig.getWordsKpts().empty());
    EXPECT_TRUE(sig.getWords3().empty());
    EXPECT_TRUE(sig.getWordsDescriptors().empty());
    EXPECT_TRUE(sig.isBadSignature());
}

TEST(SignatureTest, ChangeWordsRef)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(10, 0));
    words.insert(std::make_pair(10, 1)); // Duplicate word ID
    words.insert(std::make_pair(20, 2));
    
    std::vector<cv::KeyPoint> keypoints(3);
    std::vector<cv::Point3f> words3(3);
    cv::Mat descriptors = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_EQ(sig.getWords().count(10), 2u);
    EXPECT_EQ(sig.getWords().count(20), 1u);
    
    sig.changeWordsRef(10, 30);
    
    EXPECT_EQ(sig.getWords().count(10), 0u);
    EXPECT_EQ(sig.getWords().count(30), 2u);
    EXPECT_EQ(sig.getWords().count(20), 1u);
    
    const auto& wordsChanged = sig.getWordsChanged();
    EXPECT_NE(wordsChanged.find(10), wordsChanged.end());
    EXPECT_EQ(wordsChanged.at(10), 30);
}

TEST(SignatureTest, ChangeWordsRefInvalid)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(0, 0)); // Invalid word
    words.insert(std::make_pair(1, 1));
    
    std::vector<cv::KeyPoint> keypoints(2);
    std::vector<cv::Point3f> words3(2);
    cv::Mat descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    EXPECT_EQ(sig.getInvalidWordsCount(), 1);
    
    sig.changeWordsRef(0, 10);
    
    EXPECT_EQ(sig.getWords().count(0), 0u);
    EXPECT_EQ(sig.getWords().count(10), 1u);
    EXPECT_EQ(sig.getInvalidWordsCount(), 0);
}

TEST(SignatureTest, SetEnabled)
{
    Signature sig(1);
    
    EXPECT_FALSE(sig.isEnabled());
    
    sig.setEnabled(true);
    EXPECT_TRUE(sig.isEnabled());
    
    sig.setEnabled(false);
    EXPECT_FALSE(sig.isEnabled());
}

TEST(SignatureTest, SetWordsDisablesSignature)
{
    Signature sig(1);
    
    sig.setEnabled(true);
    EXPECT_TRUE(sig.isEnabled());
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    std::vector<cv::KeyPoint> keypoints(1);
    std::vector<cv::Point3f> words3(1);
    cv::Mat descriptors = cv::Mat::zeros(1, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    EXPECT_FALSE(sig.isEnabled()); // Should be disabled after setWords
}

TEST(SignatureTest, SetWordsDescriptors)
{
    Signature sig(1);
    
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(2, 1));
    
    std::vector<cv::KeyPoint> keypoints(2);
    std::vector<cv::Point3f> words3(2);
    cv::Mat descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    
    cv::Mat newDescriptors = cv::Mat::ones(2, 32, CV_8UC1) * 128;
    sig.setWordsDescriptors(newDescriptors);
    
    const auto& retrieved = sig.getWordsDescriptors();
    EXPECT_EQ(retrieved.rows, 2);
    EXPECT_EQ(retrieved.cols, 32);
    EXPECT_EQ(retrieved.at<uchar>(0, 0), 128);
}

// Pose and Velocity Tests

TEST(SignatureTest, SetPose)
{
    Signature sig(1);
    
    Transform pose(1.0f, 2.0f, 3.0f, 0, 0, 0);
    sig.setPose(pose);
    
    const auto& retrieved = sig.getPose();
    EXPECT_FLOAT_EQ(retrieved.x(), 1.0f);
    EXPECT_FLOAT_EQ(retrieved.y(), 2.0f);
    EXPECT_FLOAT_EQ(retrieved.z(), 3.0f);
}

TEST(SignatureTest, SetGroundTruthPose)
{
    Signature sig(1);
    
    Transform gt(10.0f, 20.0f, 30.0f, 0, 0, 0);
    sig.setGroundTruthPose(gt);
    
    const auto& retrieved = sig.getGroundTruthPose();
    EXPECT_FLOAT_EQ(retrieved.x(), 10.0f);
    EXPECT_FLOAT_EQ(retrieved.y(), 20.0f);
    EXPECT_FLOAT_EQ(retrieved.z(), 30.0f);
}

TEST(SignatureTest, SetVelocity)
{
    Signature sig(1);
    
    sig.setVelocity(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    
    const auto& velocity = sig.getVelocity();
    EXPECT_EQ(velocity.size(), 6u);
    EXPECT_FLOAT_EQ(velocity[0], 1.0f); // vx
    EXPECT_FLOAT_EQ(velocity[1], 2.0f); // vy
    EXPECT_FLOAT_EQ(velocity[2], 3.0f); // vz
    EXPECT_FLOAT_EQ(velocity[3], 0.1f); // vroll
    EXPECT_FLOAT_EQ(velocity[4], 0.2f); // vpitch
    EXPECT_FLOAT_EQ(velocity[5], 0.3f); // vyaw
}

TEST(SignatureTest, GetPoseCovariance)
{
    Signature sig(1);
    
    // Add a link to compute covariance
    Link link(1, 2, Link::kNeighbor, Transform::getIdentity());
    cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1) * 10.0; // High information = low covariance
    link.setInfMatrix(infMatrix);
    sig.addLink(link);
    
    cv::Mat covariance = sig.getPoseCovariance();
    
    EXPECT_FALSE(covariance.empty());
    EXPECT_EQ(covariance.rows, 6);
    EXPECT_EQ(covariance.cols, 6);
}

// Sensor Data Tests

TEST(SignatureTest, SensorDataAccess)
{
    Signature sig(1);
    
    SensorData& mutableData = sig.sensorData();
    mutableData.setId(1);
    mutableData.setStamp(100.0);
    
    const auto& constData = sig.sensorData();
    EXPECT_EQ(constData.id(), 1);
    EXPECT_DOUBLE_EQ(constData.stamp(), 100.0);
}

// Comparison Tests

TEST(SignatureTest, CompareTo)
{
    Signature sig1(1);
    Signature sig2(2);
    
    // Empty signatures should have 0 similarity
    float similarity = sig1.compareTo(sig2);
    EXPECT_GE(similarity, 0.0f);
    EXPECT_LE(similarity, 1.0f);
    
    // Add some words to sig1
    std::multimap<int, int> words1;
    words1.insert(std::make_pair(1, 0));
    words1.insert(std::make_pair(2, 1));
    words1.insert(std::make_pair(3, 2));
    
    std::vector<cv::KeyPoint> keypoints1(3);
    std::vector<cv::Point3f> words3_1(3);
    cv::Mat descriptors1 = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig1.setWords(words1, keypoints1, words3_1, descriptors1);
    
    // Add overlapping words to sig2
    std::multimap<int, int> words2;
    words2.insert(std::make_pair(1, 0));
    words2.insert(std::make_pair(2, 1));
    words2.insert(std::make_pair(4, 2)); // Different word
    
    std::vector<cv::KeyPoint> keypoints2(3);
    std::vector<cv::Point3f> words3_2(3);
    cv::Mat descriptors2 = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig2.setWords(words2, keypoints2, words3_2, descriptors2);
    
    similarity = sig1.compareTo(sig2);
    EXPECT_GE(similarity, 0.0f);
    EXPECT_LE(similarity, 1.0f);
    // Should have some similarity due to overlapping words (1 and 2)
    EXPECT_GT(similarity, 0.0f);
}

TEST(SignatureTest, CompareToIdentical)
{
    Signature sig1(1);
    Signature sig2(2);
    
    // Add identical words
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(2, 1));
    words.insert(std::make_pair(3, 2));
    
    std::vector<cv::KeyPoint> keypoints(3);
    std::vector<cv::Point3f> words3(3);
    cv::Mat descriptors = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig1.setWords(words, keypoints, words3, descriptors);
    sig2.setWords(words, keypoints, words3, descriptors);
    
    float similarity = sig1.compareTo(sig2);
    EXPECT_GE(similarity, 0.0f);
    EXPECT_LE(similarity, 1.0f);
    // Identical words should have high similarity
    EXPECT_GT(similarity, 0.5f);
}

// Memory Usage Tests

TEST(SignatureTest, GetMemoryUsed)
{
    Signature sig(1);
    
    unsigned long memory1 = sig.getMemoryUsed(false);
    unsigned long memory2 = sig.getMemoryUsed(true);
    
    EXPECT_GT(memory1, 0u);
    EXPECT_GE(memory2, memory1); // With sensor data should be >= without
}

// Comprehensive Test

TEST(SignatureTest, ComprehensiveUsage)
{
    Signature sig(100, 1, 10, 12345.678, "test_location");
    
    // Set pose and velocity
    sig.setPose(Transform(1.0f, 2.0f, 3.0f, 0, 0, 0));
    sig.setGroundTruthPose(Transform(1.1f, 2.1f, 3.1f, 0, 0, 0));
    sig.setVelocity(0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f);
    
    // Add links
    sig.addLink(Link(100, 101, Link::kNeighbor, Transform::getIdentity()));
    sig.addLink(Link(100, 102, Link::kGlobalClosure, Transform::getIdentity()));
    
    // Add landmark
    sig.addLandmark(Link(100, -10, Link::kLandmark, Transform(5.0f, 0.0f, 0.0f, 0, 0, 0)));
    
    // Add words
    std::multimap<int, int> words;
    words.insert(std::make_pair(1, 0));
    words.insert(std::make_pair(2, 1));
    words.insert(std::make_pair(3, 2));
    
    std::vector<cv::KeyPoint> keypoints(3);
    std::vector<cv::Point3f> words3(3);
    cv::Mat descriptors = cv::Mat::zeros(3, 32, CV_8UC1);
    
    sig.setWords(words, keypoints, words3, descriptors);
    sig.setEnabled(true);
    
    // Verify everything
    EXPECT_EQ(sig.id(), 100);
    EXPECT_EQ(sig.getWeight(), 10);
    EXPECT_EQ(sig.getLinks().size(), 2u);
    EXPECT_EQ(sig.getLandmarks().size(), 1u);
    EXPECT_EQ(sig.getWords().size(), 3u);
    EXPECT_TRUE(sig.isEnabled());
    EXPECT_FALSE(sig.isBadSignature());
    EXPECT_FLOAT_EQ(sig.getPose().x(), 1.0f);
    EXPECT_FLOAT_EQ(sig.getVelocity()[0], 0.1f);
}

