#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
#include <opencv2/gpu/gpu.hpp>
#endif
#else
#include <opencv2/core/cuda.hpp>
#endif
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/UFile.h"
#include <vector>
#include <list>
#include <set>
#include <iterator>
#include <fstream>
#include <cstdio>

using namespace rtabmap;

namespace {
// Spelled out rather than taken from VWDictionary: the point is to check the
// strategies that are expected to build an index, not to agree with whatever
// the implementation classifies as one.
bool hasFlannIndex(VWDictionary::NNStrategy strategy)
{
    return strategy == VWDictionary::kNNFlannNaive ||
           strategy == VWDictionary::kNNFlannKdTree ||
           strategy == VWDictionary::kNNFlannLSH ||
           strategy == VWDictionary::kNNNanoFlannKdTree ||
           strategy == VWDictionary::kNNFlannKdTreeSingle;
}
} // namespace

class VWDictionaryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a dictionary with default parameters
        dict = new VWDictionary();
    }

    void TearDown() override {
        delete dict;
    }

    VWDictionary* dict;
};

TEST_F(VWDictionaryTest, Constructor)
{
    EXPECT_TRUE(dict != nullptr);
    EXPECT_TRUE(dict->isIncremental());
    EXPECT_EQ(dict->getVisualWords().size(), 0u);
    EXPECT_EQ(dict->getTotalActiveReferences(), 0);
    EXPECT_EQ(dict->getIndexedWordsCount(), 0u);
}

TEST_F(VWDictionaryTest, AddNewWordsIncremental)
{
    // Test incremental mode - NNDR is applied, new words created if NNDR fails
    // Test with all NNStrategy values
    VWDictionary::NNStrategy strategies[] = {
        VWDictionary::kNNFlannNaive,
        VWDictionary::kNNFlannKdTree,
        VWDictionary::kNNFlannLSH,
        VWDictionary::kNNBruteForce,
        VWDictionary::kNNBruteForceGPU,
        VWDictionary::kNNNanoFlannKdTree,
        VWDictionary::kNNFlannKdTreeSingle
    };

    // That will mke logic below works with numbers chosen
    ParametersMap params;
    params.insert(ParametersPair(Parameters::kKpNndrRatio(), "0.4"));
    dict->parseParameters(params);

    for(VWDictionary::NNStrategy strategy : strategies)
    {
        if(strategy == VWDictionary::kNNBruteForceGPU)
        {
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
            if(cv::gpu::getCudaEnabledDeviceCount() <= 0)
            {
                strategy = VWDictionary::kNNBruteForce;
            }
#else
            strategy = VWDictionary::kNNBruteForce;
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
            if(cv::cuda::getCudaEnabledDeviceCount() <= 0)
            {
                strategy = VWDictionary::kNNBruteForce;
            }
#else
            strategy = VWDictionary::kNNBruteForce;
#endif
#endif
        }

        // Reset dictionary for each strategy
        dict->clear();
        dict->setNNStrategy(strategy);

        EXPECT_TRUE(dict->isIncremental());
        EXPECT_EQ(dict->getNNStrategy(), strategy);
        
        // Add initial words to dictionary (2D descriptors)
        // Word 1: (0, 0)
        // Word 2: (15, 0)
        // Word 3: (0, 255)
        // Using dimension 8 to support LSH
        cv::Mat initialDescriptors = (cv::Mat_<float>(3, 8) << 
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 255.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        // Convert to binary if using LSH strategy
        if(strategy == VWDictionary::kNNFlannLSH)
        {
            // Convert float descriptors to binary
            initialDescriptors = VWDictionary::convert32FToBin(initialDescriptors, true);
            std::cout << initialDescriptors << std::endl;
        }

        std::list<int> addedIds = dict->addNewWords(initialDescriptors, 1);
        
        dict->update();
        
        unsigned int initialWordCount = dict->getVisualWords().size();

        EXPECT_FALSE(addedIds.empty()) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        EXPECT_EQ(initialWordCount, 3u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        EXPECT_EQ(addedIds.back(), dict->getVisualWords().rbegin()->first) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // Get the maximum initial word ID
        int maxInitialId = dict->getVisualWords().rbegin()->first;
        
        // Create query descriptors with known distances
        // Query 1: (1, 0) - very close to Word 1 (0,0), far from others
        //   Distance to Word 1: sqrt(1^2 + 0^2) ≈ 1 (LSH 1)
        //   Distance to Word 2: sqrt(6^2 + 0^2) ≈ 6 (LSH 4)
        //   Ratio: 1 / 6 ≈ 0.16 < NNDR threshold (typically 0.4) - should PASS NNDR
        //   LSH Ratio: 1/4 = 0.25 < NNDR
        //
        // Query 2: (9, 0) - "equidistant" from Word 1 and Word 2
        //   Distance to Word 1: 9.0 (LSH 1)
        //   Distance to Word 2: 6.0 (LSH 2)
        //   Ratio: 36 / 81 = 0.44 > NNDR threshold - should FAIL NNDR (new word created)
        //   LSH Ratio: 1/2 = 0.5 > NNDR
        cv::Mat queryDescriptors = (cv::Mat_<float>(2, 8) << 
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,  // Should match Word 1 (passes NNDR)
            9.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // Should create new word (fails NNDR)
       
        // Convert to binary if using LSH strategy
        if(strategy == VWDictionary::kNNFlannLSH)
        {
            // Convert float descriptors to binary
            queryDescriptors = VWDictionary::convert32FToBin(queryDescriptors, true);
            std::cout << queryDescriptors << std::endl;
        }
        
        int signatureId = 2;
        std::list<int> wordIds = dict->addNewWords(queryDescriptors, signatureId);
        
        // In incremental mode, valid word IDs are returned only if NNDR validation passes
        // Otherwise, new words are created
        EXPECT_EQ(wordIds.size(), 2u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // First query should match existing word (NNDR passed)
        int firstId = *wordIds.begin();
        EXPECT_EQ(firstId, VWDictionary::ID_START) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        EXPECT_NE(dict->getWord(firstId), nullptr) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        
        // Second query should create a new word (NNDR failed)
        int secondId = *std::next(wordIds.begin());
        EXPECT_GT(secondId, maxInitialId) << "Strategy: " << VWDictionary::nnStrategyName(strategy); // Should be a new word ID
        EXPECT_NE(dict->getWord(secondId), nullptr) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        
        // Total words should increase by 1 (one new word created)
        EXPECT_EQ(dict->getVisualWords().size(), initialWordCount + 1) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
    }
}

TEST_F(VWDictionaryTest, AddNewWordsFixed)
{
    // Test fixed mode - NNDR is not applied, closest match is always returned
    
    // Create a temporary dictionary file
    std::string dictFile = "test_vwdictionary_fixed_dict.txt";
    
    // Write dictionary file in expected format:
    // First line: dimension
    // Subsequent lines: word_id descriptor_value1 descriptor_value2 ...
    std::ofstream file(dictFile);
    ASSERT_TRUE(file.is_open());
    
    // Write header with dimension
    file << "2" << std::endl;
    
    // Write words: Word 1: (0, 0), Word 2: (10, 5), Word 3: (0, 100)
    file << "1 0.0 0.0" << std::endl;
    file << "2 10 0.0" << std::endl;
    file << "3 0.0 100.0" << std::endl;
    
    file.close();
    
    // Load fixed dictionary from file
    dict->setFixedDictionary(dictFile);
    EXPECT_FALSE(dict->isIncremental());
    
    dict->update();
    
    unsigned int initialWordCount = dict->getVisualWords().size();
    EXPECT_EQ(initialWordCount, 3u);
    
    // Create query descriptors with known distances
    // Query 1: (0.5, 0.5) - closest to Word 1 (0,0), distance ≈ 0.707
    // Query 2: (5.1, 0) - slighlty closer to Word 2 (10,5) than Word 1 (0,0), with distances 4.9 and 5.0 respectively
    cv::Mat queryDescriptors = (cv::Mat_<float>(2, 2) << 
        0.5f, 0.5f,  // Closest to Word 1
        5.1f, 0.0f); // Closest to Word2 but would not pass NNDR (4.9/5 = 0.98 > 0.8 default NNDR)
    
    int signatureId = 2;
    std::list<int> wordIds = dict->addNewWords(queryDescriptors, signatureId);
    
    // In fixed mode, closest visual word ID is always returned (no NNDR check)
    EXPECT_EQ(wordIds.size(), 2u);
    
    // First query should match Word 1 (closest match: distance 0.707 to Word 1 vs 9.513 to Word 2)
    int firstId = *wordIds.begin();
    EXPECT_EQ(firstId, VWDictionary::ID_START);
    EXPECT_NE(dict->getWord(firstId), nullptr);
    
    // Second query should also match an existing word (closest match, no NNDR check)
    // Query (5.1, 0) is slighlty closer to Word 2
    int secondId = *std::next(wordIds.begin());
    EXPECT_EQ(secondId, 2); 
    EXPECT_NE(dict->getWord(secondId), nullptr);
    
    // In fixed mode, no new words should be created
    EXPECT_EQ(dict->getVisualWords().size(), initialWordCount);
    
    // Cleanup: remove temporary dictionary file
    UFile::erase(dictFile);
}

TEST_F(VWDictionaryTest, AddWord)
{
    cv::Mat descriptor = (cv::Mat_<float>(1, 64) << 
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
    
    VisualWord* word = new VisualWord(100, descriptor, 10);
    dict->addWord(word);
    
    EXPECT_EQ(dict->getVisualWords().size(), 1u);
    const VisualWord* retrieved = dict->getWord(100);
    EXPECT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->id(), 100);
}

TEST_F(VWDictionaryTest, FindNNIncremental)
{
    // Test incremental mode - NNDR is applied
    EXPECT_TRUE(dict->isIncremental());
    
    // Add initial words to dictionary (2D descriptors)
    // Word 1: (0, 0)
    // Word 2: (10, 0)
    // Word 3: (0, 100)
    cv::Mat initialDescriptors = (cv::Mat_<float>(3, 2) << 
        0.0f, 0.0f,
        10.0f, 0.0f,
        0.0f, 100.0f);
    dict->addNewWords(initialDescriptors, 1);
    dict->update();
    
    // Create query descriptors with known distances
    // Query 1: (0.5, 0.5) - very close to Word 1 (0,0), far from others
    //   Distance to Word 1: sqrt(0.5^2 + 0.5^2) ≈ 0.707
    //   Distance to Word 2: sqrt(9.5^2 + 0.5^2) ≈ 9.513
    //   Ratio: 0.707 / 9.513 ≈ 0.074 < NNDR threshold (typically 0.8) - should PASS NNDR
    //
    // Query 2: (5, 0) - equidistant from Word 1 and Word 2
    //   Distance to Word 1: 5.0
    //   Distance to Word 2: 5.0
    //   Ratio: 5.0 / 5.0 = 1.0 > NNDR threshold - should FAIL NNDR (return ID_INVALID)
    cv::Mat queryDescriptors = (cv::Mat_<float>(2, 2) << 
        0.5f, 0.5f,  // Should match Word 1 (passes NNDR)
        5.0f, 0.0f); // Should fail NNDR (returns ID_INVALID)
    
    std::vector<int> matches = dict->findNN(queryDescriptors);
    EXPECT_EQ(matches.size(), 2u);
    
    // First query should match existing word (NNDR passed)
    int firstId = matches[0];
    EXPECT_EQ(firstId, VWDictionary::ID_START);
    EXPECT_NE(dict->getWord(firstId), nullptr);
    
    // Second query should fail NNDR (returns ID_INVALID)
    int secondId = matches[1];
    EXPECT_EQ(secondId, VWDictionary::ID_INVALID);
}

TEST_F(VWDictionaryTest, FindNNFixed)
{
    // Test fixed mode - NNDR is not applied, closest match is always returned
    
    // Create a temporary dictionary file
    std::string dictFile = "test_vwdictionary_fixed_dict_findnn.txt";
    
    // Write dictionary file in expected format:
    // First line: dimension
    // Subsequent lines: word_id descriptor_value1 descriptor_value2 ...
    std::ofstream file(dictFile);
    ASSERT_TRUE(file.is_open());
    
    // Write header with dimension
    file << "2" << std::endl;
    
    // Write words: Word 1: (0, 0), Word 2: (10, 0), Word 3: (0, 100)
    file << "1 0.0 0.0" << std::endl;
    file << "2 10 0.0" << std::endl;
    file << "3 0.0 100.0" << std::endl;
    
    file.close();
    
    // Load fixed dictionary from file
    dict->setFixedDictionary(dictFile);
    EXPECT_FALSE(dict->isIncremental());
    
    dict->update();
    
    unsigned int initialWordCount = dict->getVisualWords().size();
    EXPECT_EQ(initialWordCount, 3u);
    
    // Create query descriptors with known distances
    // Query 1: (0.5, 0.5) - closest to Word 1 (0,0), distance ≈ 0.707
    // Query 2: (5.1, 0) - slightly closer to Word 2 (10,0) than Word 1 (0,0), with distances 4.9 and 5.0 respectively
    cv::Mat queryDescriptors = (cv::Mat_<float>(2, 2) << 
        0.5f, 0.5f,  // Closest to Word 1
        5.1f, 0.0f); // Closest to Word 2 but would not pass NNDR (4.9/5 = 0.98 > 0.8 default NNDR)
    
    std::vector<int> matches = dict->findNN(queryDescriptors);
    EXPECT_EQ(matches.size(), 2u);
    
    // First query should match Word 1 (closest match: distance 0.707 to Word 1 vs 9.513 to Word 2)
    int firstId = matches[0];
    EXPECT_EQ(firstId, VWDictionary::ID_START);
    EXPECT_NE(dict->getWord(firstId), nullptr);
    
    // Second query should also match an existing word (closest match, no NNDR check)
    // Query (5.1, 0) is slightly closer to Word 2
    int secondId = matches[1];
    EXPECT_EQ(secondId, 2);
    EXPECT_NE(dict->getWord(secondId), nullptr);
    
    // Cleanup: remove temporary dictionary file
    UFile::erase(dictFile);
}

TEST_F(VWDictionaryTest, AddWordRef)
{
    cv::Mat descriptor = cv::Mat::ones(1, 32, CV_32F);
    std::list<int> wordIds = dict->addNewWords(descriptor, 1);
    ASSERT_EQ(wordIds.size(), 1u);
    
    int wordId = wordIds.front();
    dict->addWordRef(wordId, 2);
    dict->addWordRef(wordId, 3);
    
    EXPECT_EQ(dict->getTotalActiveReferences(), 3); // 1 from addNewWords + 2 from addWordRef
    
    const VisualWord* word = dict->getWord(wordId);
    EXPECT_NE(word, nullptr);
    EXPECT_EQ(word->getTotalReferences(), 3);
}

TEST_F(VWDictionaryTest, RemoveAllWordRef)
{
    cv::Mat descriptor = cv::Mat::ones(1, 32, CV_32F);
    std::list<int> wordIds = dict->addNewWords(descriptor, 1);
    ASSERT_EQ(wordIds.size(), 1u);
    
    int wordId = wordIds.front();
    dict->addWordRef(wordId, 2);
    dict->addWordRef(wordId, 3);
    
    EXPECT_EQ(dict->getTotalActiveReferences(), 3);
    
    dict->removeAllWordRef(wordId, 1);
    EXPECT_EQ(dict->getTotalActiveReferences(), 2);
    
    dict->removeAllWordRef(wordId, 2);
    dict->removeAllWordRef(wordId, 3);
    
    // Word should now be unused
    EXPECT_EQ(dict->getTotalActiveReferences(), 0);
    EXPECT_EQ(dict->getUnusedWordsSize(), 1u);
}

TEST_F(VWDictionaryTest, GetWord)
{
    cv::Mat descriptor = cv::Mat::ones(1, 32, CV_32F);
    std::list<int> wordIds = dict->addNewWords(descriptor, 1);
    ASSERT_EQ(wordIds.size(), 1u);
    
    int wordId = wordIds.front();
    const VisualWord* word = dict->getWord(wordId);
    
    EXPECT_NE(word, nullptr);
    EXPECT_EQ(word->id(), wordId);
    EXPECT_EQ(word->getDescriptor().cols, 32);
    
    // Test with invalid ID
    const VisualWord* invalid = dict->getWord(99999);
    EXPECT_EQ(invalid, nullptr);
}

TEST_F(VWDictionaryTest, GetUnusedWords)
{
    cv::Mat descriptor = cv::Mat::ones(1, 32, CV_32F);
    std::list<int> wordIds = dict->addNewWords(descriptor, 1);
    ASSERT_EQ(wordIds.size(), 1u);
    
    int wordId = wordIds.front();
    
    // Initially word has a reference, so it's not unused
    EXPECT_EQ(dict->getUnusedWordsSize(), 0u);
    
    // Remove all references
    dict->removeAllWordRef(wordId, 1);
    EXPECT_EQ(dict->getUnusedWordsSize(), 1u);
    
    std::vector<VisualWord*> unused = dict->getUnusedWords();
    EXPECT_EQ(unused.size(), 1u);
    EXPECT_EQ(unused[0]->id(), wordId);
    
    std::vector<int> unusedIds = dict->getUnusedWordIds();
    EXPECT_EQ(unusedIds.size(), 1u);
    EXPECT_EQ(unusedIds[0], wordId);
}

TEST_F(VWDictionaryTest, ConvertBinTo32FByteToFloat)
{
    // Test byteToFloat = true (simple conversion)
    cv::Mat input(2, 10, CV_8UC1);
    cv::randu(input, cv::Scalar(0), cv::Scalar(255));
    
    cv::Mat output = VWDictionary::convertBinTo32F(input, true);
    
    EXPECT_EQ(output.type(), CV_32FC1);
    EXPECT_EQ(output.rows, 2);
    EXPECT_EQ(output.cols, 10); // Same dimensions
}

TEST_F(VWDictionaryTest, ConvertBinTo32FBitExpansion)
{
    // Test byteToFloat = false (bit expansion)
    cv::Mat input(1, 4, CV_8UC1);
    input.at<unsigned char>(0, 0) = 0b10101010; // 170
    input.at<unsigned char>(0, 1) = 0b01010101; // 85
    input.at<unsigned char>(0, 2) = 0b11110000; // 240
    input.at<unsigned char>(0, 3) = 0b00001111; // 15
    
    cv::Mat output = VWDictionary::convertBinTo32F(input, false);
    
    EXPECT_EQ(output.type(), CV_32FC1);
    EXPECT_EQ(output.rows, 1);
    EXPECT_EQ(output.cols, 32); // 4 bytes * 8 bits = 32 floats
    
    // Check first byte expansion (10101010)
    EXPECT_FLOAT_EQ(output.at<float>(0, 0), 0.0f); // bit 0
    EXPECT_FLOAT_EQ(output.at<float>(0, 1), 1.0f); // bit 1
    EXPECT_FLOAT_EQ(output.at<float>(0, 2), 0.0f); // bit 2
    EXPECT_FLOAT_EQ(output.at<float>(0, 3), 1.0f); // bit 3
}

TEST_F(VWDictionaryTest, Convert32FToBinByteToFloat)
{
    // Test byteToFloat = true (simple conversion)
    cv::Mat input(2, 10, CV_32FC1);
    cv::randu(input, cv::Scalar(0), cv::Scalar(255));
    
    cv::Mat output = VWDictionary::convert32FToBin(input, true);
    
    EXPECT_EQ(output.type(), CV_8UC1);
    EXPECT_EQ(output.rows, 2);
    EXPECT_EQ(output.cols, 10); // Same dimensions
}

TEST_F(VWDictionaryTest, Convert32FToBinBitPacking)
{
    // Test byteToFloat = false (bit packing)
    cv::Mat input(1, 32, CV_32FC1);
    // Set first 8 floats to represent 10101010
    input.at<float>(0, 0) = 0.0f; // bit 0
    input.at<float>(0, 1) = 1.0f; // bit 1
    input.at<float>(0, 2) = 0.0f; // bit 2
    input.at<float>(0, 3) = 1.0f; // bit 3
    input.at<float>(0, 4) = 0.0f; // bit 4
    input.at<float>(0, 5) = 1.0f; // bit 5
    input.at<float>(0, 6) = 0.0f; // bit 6
    input.at<float>(0, 7) = 1.0f; // bit 7
    // Rest set to 0
    for(int i = 8; i < 32; ++i) {
        input.at<float>(0, i) = 0.0f;
    }
    
    cv::Mat output = VWDictionary::convert32FToBin(input, false);
    
    EXPECT_EQ(output.type(), CV_8UC1);
    EXPECT_EQ(output.rows, 1);
    EXPECT_EQ(output.cols, 4); // 32 floats / 8 = 4 bytes
    
    EXPECT_EQ(output.at<unsigned char>(0, 0), 0b10101010);
}

TEST_F(VWDictionaryTest, ConvertRoundTrip)
{
    // Test round trip conversion with bit expansion
    cv::Mat original(1, 4, CV_8UC1);
    cv::randu(original, cv::Scalar(0), cv::Scalar(255));
    
    cv::Mat expanded = VWDictionary::convertBinTo32F(original, false);
    cv::Mat packed = VWDictionary::convert32FToBin(expanded, false);
    
    EXPECT_EQ(packed.rows, original.rows);
    EXPECT_EQ(packed.cols, original.cols);
    EXPECT_EQ(packed.type(), original.type());
    
    for(int i = 0; i < original.cols; ++i) {
        EXPECT_EQ(packed.at<unsigned char>(0, i), original.at<unsigned char>(0, i));
    }
}

TEST_F(VWDictionaryTest, SetNNStrategy)
{
    // Test resetting same strategy
    bool reinit = dict->setNNStrategy(dict->getNNStrategy());
    EXPECT_FALSE(reinit);

    // Test changing strategy
    reinit = dict->setNNStrategy(VWDictionary::kNNBruteForce);
    EXPECT_TRUE(reinit);
    
    // Add some words and index them
    cv::Mat descriptors(5, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    dict->update();
    
    // Change strategy should reinitialize
    reinit = dict->setNNStrategy(VWDictionary::kNNFlannKdTree);
    EXPECT_TRUE(reinit);
}

TEST_F(VWDictionaryTest, NNStrategyName)
{
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNFlannNaive), "FLANN NAIVE");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNFlannKdTree), "FLANN KD-TREE");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNFlannLSH), "FLANN LSH");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNBruteForce), "BRUTE FORCE");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNBruteForceGPU), "BRUTE FORCE GPU");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNNanoFlannKdTree), "NANOFLANN KD-TREE");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNFlannKdTreeSingle), "FLANN KD-TREE SINGLE");
    EXPECT_EQ(VWDictionary::nnStrategyName(VWDictionary::kNNUndef), "Unknown");
}

TEST_F(VWDictionaryTest, IncrementalDictionary)
{
    EXPECT_TRUE(dict->isIncremental());
    
    dict->setIncrementalDictionary();
    EXPECT_TRUE(dict->isIncremental());
}

TEST_F(VWDictionaryTest, GetNndrRatio)
{
    float ratio = dict->getNndrRatio();
    EXPECT_GT(ratio, 0.0f);
    EXPECT_LE(ratio, 1.0f);
}

TEST_F(VWDictionaryTest, Update)
{
    // Add words without updating index
    cv::Mat descriptors(3, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    
    unsigned int notIndexed = dict->getNotIndexedWordsCount();
    EXPECT_GT(notIndexed, 0u);
    
    dict->update();
    
    // After update, words should be indexed
    EXPECT_GT(dict->getIndexedWordsCount(), 0u);
}

TEST_F(VWDictionaryTest, Clear)
{
    // Add some words
    cv::Mat descriptors(5, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    
    EXPECT_GT(dict->getVisualWords().size(), 0u);
    
    dict->clear();
    
    EXPECT_EQ(dict->getVisualWords().size(), 0u);
    EXPECT_EQ(dict->getTotalActiveReferences(), 0);
    EXPECT_EQ(dict->getIndexedWordsCount(), 0u);
}

TEST_F(VWDictionaryTest, MemoryUsed)
{
    unsigned long memBefore = dict->getMemoryUsed();
    
    // Add some words
    cv::Mat descriptors(10, 128, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    dict->update();
    
    unsigned long memAfter = dict->getMemoryUsed();
    EXPECT_GT(memAfter, memBefore);
    
    unsigned int indexMem = dict->getIndexMemoryUsed();
    EXPECT_GT(indexMem, 0u);
}

TEST_F(VWDictionaryTest, SerializeDeserializeIndex)
{
    // Test with all NNStrategy values
    VWDictionary::NNStrategy strategies[] = {
        VWDictionary::kNNFlannNaive,
        VWDictionary::kNNFlannKdTree,
        VWDictionary::kNNFlannLSH,
        VWDictionary::kNNBruteForce,
        VWDictionary::kNNBruteForceGPU,
        VWDictionary::kNNNanoFlannKdTree,
        VWDictionary::kNNFlannKdTreeSingle
    };

    for(VWDictionary::NNStrategy strategy : strategies)
    {
        if(strategy == VWDictionary::kNNBruteForceGPU)
        {
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
            if(cv::gpu::getCudaEnabledDeviceCount() <= 0)
            {
                continue; // Skip if no GPU available
            }
#else
            continue; // Skip if GPU support not compiled
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
            if(cv::cuda::getCudaEnabledDeviceCount() <= 0)
            {
                continue; // Skip if no GPU available
            }
#else
            continue; // Skip if GPU support not compiled
#endif
#endif
        }

        // Reset dictionary for each strategy
        dict->clear();
        dict->setNNStrategy(strategy);

        // Add words and build index
        cv::Mat descriptors(5, 32, CV_32F);
        cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
        
        // Convert to binary if using LSH strategy
        if(strategy == VWDictionary::kNNFlannLSH)
        {
            // Convert float descriptors to binary
            descriptors = VWDictionary::convert32FToBin(descriptors, true);
        }
        
        dict->addNewWords(descriptors, 1);
        dict->update();
        
        // Serialize
        std::vector<unsigned char> data = dict->serializeIndex();
#ifdef _WIN32
        // The rtflann serialization needs fmemopen, which Windows doesn't have
        // (see FlannIndex::serializeIndex()): there, only the nanoflann index
        // gives data back, the others are left out of the round trip below.
        if(strategy != VWDictionary::kNNNanoFlannKdTree)
        {
            EXPECT_EQ(data.size(), 0u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
            continue;
        }
#endif
        if(hasFlannIndex(strategy))
        {
            EXPECT_GT(data.size(), 0u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        }
        else {
            // brute force strategies have no index to serialize
            EXPECT_EQ(data.size(), 0u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        }
        
        // Create new dictionary and deserialize
        VWDictionary dict2;
        dict2.setNNStrategy(strategy);
        cv::Mat descriptors2(5, 32, CV_32F);
        cv::randu(descriptors2, cv::Scalar(0), cv::Scalar(1));
        
        // Convert to binary if using LSH strategy
        if(strategy == VWDictionary::kNNFlannLSH)
        {
            descriptors2 = VWDictionary::convert32FToBin(descriptors2, true);
        }
        
        dict2.addNewWords(descriptors2, 1);
        
        // Deserialize should fail because we are not using the same descriptors
        bool success = dict2.deserializeIndex(data);
        EXPECT_FALSE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        success = dict2.deserializeIndex(data.data(), data.size());
        EXPECT_FALSE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // Same descriptors
        VWDictionary dict3;
        dict3.setNNStrategy(strategy);
        dict3.addNewWords(descriptors, 1);
        success = dict3.deserializeIndex(data);
        if(hasFlannIndex(strategy))
        {
            EXPECT_TRUE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

            // Index should be loaded
            EXPECT_GT(dict3.getIndexedWordsCount(), 0u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
        }
        else
        {
            EXPECT_FALSE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
            continue;
        }

        // Should fail if index is already built
        success = dict3.deserializeIndex(data);
        EXPECT_FALSE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // raw bytes
        VWDictionary dict4;
        dict4.setNNStrategy(strategy);
        dict4.addNewWords(descriptors, 1);
        success = dict4.deserializeIndex(data.data(), data.size());
        EXPECT_TRUE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // Index should be loaded
        EXPECT_GT(dict4.getIndexedWordsCount(), 0u) << "Strategy: " << VWDictionary::nnStrategyName(strategy);

        // Should fail if index is already built
        success = dict4.deserializeIndex(data.data(), data.size());
        EXPECT_FALSE(success) << "Strategy: " << VWDictionary::nnStrategyName(strategy);
    }
}

namespace {

// One-hot descriptor, so every word is far from every other one and the
// checksum over the search data changes as soon as two words are swapped.
cv::Mat oneHotDescriptor(int hotIndex, int dim = 8)
{
    cv::Mat descriptor = cv::Mat::zeros(1, dim, CV_32F);
    descriptor.at<float>(0, hotIndex) = 1000.0f;
    return descriptor;
}

} // namespace

TEST_F(VWDictionaryTest, RebuildIndexReordersIndexToWordIdOrder)
{
    // A dictionary loaded from a database gets its words back in word-id order
    // (std::map), and deserializeIndex() rebuilds the search data in that same
    // order. So a serialized index is only reusable if it was built in word-id
    // order too.
    //
    // update() with incremental FLANN appends the not-yet-indexed words at the
    // end of the existing index, whatever their id. That is what Memory does
    // when it repairs a dictionary that is missing words: the repaired words
    // usually have ids *lower* than the ones already indexed, so the index ends
    // up in an order the next load cannot reproduce. rebuildIndex() re-indexes
    // everything from scratch, which restores the word-id order.
    dict->setNNStrategy(VWDictionary::kNNFlannKdTree);
    ASSERT_TRUE(dict->isIncrementalFlann()) << "The out-of-order index only happens with incremental FLANN";

    dict->addWord(new VisualWord(1, oneHotDescriptor(0)));
    dict->addWord(new VisualWord(3, oneHotDescriptor(2)));
    dict->update();
    ASSERT_EQ(dict->getIndexedWordsCount(), 2u);

    // Word 2 is indexed after word 3: index order (1, 3, 2) != id order (1, 2, 3).
    dict->addWord(new VisualWord(2, oneHotDescriptor(1)));
    dict->update();
    ASSERT_EQ(dict->getIndexedWordsCount(), 3u);

    std::vector<unsigned char> staleData = dict->serializeIndex();
#ifdef _WIN32
    // FlannIndex::serializeIndex() is not implemented on Windows
    // (see corelib/src/FlannIndex.cpp), so there is nothing to round-trip.
    EXPECT_EQ(staleData.size(), 0u);
#else
    ASSERT_GT(staleData.size(), 0u);

    // Simulates the next load: same words, added in id order like DBDriver does.
    {
        VWDictionary reloaded;
        reloaded.setNNStrategy(VWDictionary::kNNFlannKdTree);
        reloaded.addWord(new VisualWord(1, oneHotDescriptor(0)));
        reloaded.addWord(new VisualWord(2, oneHotDescriptor(1)));
        reloaded.addWord(new VisualWord(3, oneHotDescriptor(2)));
        EXPECT_FALSE(reloaded.deserializeIndex(staleData))
            << "An index built out of word-id order should be rejected on load";
    }

    // Same words, same content, but re-indexed from scratch.
    dict->rebuildIndex();
    EXPECT_EQ(dict->getIndexedWordsCount(), 3u);
    EXPECT_EQ(dict->getNotIndexedWordsCount(), 0u);

    std::vector<unsigned char> rebuiltData = dict->serializeIndex();
    ASSERT_GT(rebuiltData.size(), 0u);

    {
        VWDictionary reloaded;
        reloaded.setNNStrategy(VWDictionary::kNNFlannKdTree);
        reloaded.addWord(new VisualWord(1, oneHotDescriptor(0)));
        reloaded.addWord(new VisualWord(2, oneHotDescriptor(1)));
        reloaded.addWord(new VisualWord(3, oneHotDescriptor(2)));
        EXPECT_TRUE(reloaded.deserializeIndex(rebuiltData));
        EXPECT_EQ(reloaded.getIndexedWordsCount(), 3u);
        // A deserialized index doesn't need to be saved back.
        EXPECT_FALSE(reloaded.isModified());
    }
#endif
}

TEST_F(VWDictionaryTest, RebuildIndexKeepsWordsAndSearchResults)
{
    dict->setNNStrategy(VWDictionary::kNNFlannKdTree);

    cv::Mat descriptors = (cv::Mat_<float>(3, 2) <<
        0.0f, 0.0f,
        10.0f, 0.0f,
        0.0f, 100.0f);
    std::list<int> wordIds = dict->addNewWords(descriptors, 1);
    dict->update();
    ASSERT_EQ(wordIds.size(), 3u);

    cv::Mat query = (cv::Mat_<float>(2, 2) <<
        0.5f, 0.5f,   // matches the first word
        0.0f, 99.0f); // matches the third word
    const std::vector<int> before = dict->findNN(query);
    ASSERT_EQ(before.size(), 2u);
    ASSERT_EQ(before[0], wordIds.front());
    ASSERT_EQ(before[1], wordIds.back());

    dict->rebuildIndex();

    // Re-indexing doesn't touch the words themselves, only the search index.
    EXPECT_EQ(dict->getVisualWords().size(), 3u);
    EXPECT_EQ(dict->getIndexedWordsCount(), 3u);
    EXPECT_EQ(dict->getNotIndexedWordsCount(), 0u);
    EXPECT_EQ(dict->findNN(query), before);

    // The index changed, so it has to be saved back (Memory::saveFlannIndex()
    // only serializes a modified dictionary).
    EXPECT_TRUE(dict->isModified());
}

TEST_F(VWDictionaryTest, RebuildIndexOnEmptyDictionaryIsSafe)
{
    dict->setNNStrategy(VWDictionary::kNNFlannKdTree);

    dict->rebuildIndex();

    EXPECT_EQ(dict->getVisualWords().size(), 0u);
    EXPECT_EQ(dict->getIndexedWordsCount(), 0u);
    EXPECT_EQ(dict->getNotIndexedWordsCount(), 0u);
    EXPECT_TRUE(dict->serializeIndex().empty());
}

TEST_F(VWDictionaryTest, ByteToFloatChangeRebuildsIndex)
{
    // parseParameters() re-indexes through rebuildIndex() when the binary to
    // float conversion changes, because the descriptors fed to the kd-tree
    // change dimension (1 float per byte vs 1 float per bit).
    dict->setNNStrategy(VWDictionary::kNNFlannKdTree);

    cv::Mat descriptors = cv::Mat::zeros(3, 4, CV_8U);
    for(int row = 0; row < descriptors.rows; ++row)
    {
        descriptors.at<unsigned char>(row, row) = 255;
    }
    dict->addNewWords(descriptors, 1);
    dict->update();
    ASSERT_EQ(dict->getIndexedWordsCount(), 3u);

    ParametersMap params;
    params.insert(ParametersPair(Parameters::kKpByteToFloat(), "true"));
    dict->parseParameters(params);

    EXPECT_EQ(dict->getVisualWords().size(), 3u);
    EXPECT_EQ(dict->getIndexedWordsCount(), 3u);
    EXPECT_EQ(dict->getNotIndexedWordsCount(), 0u);

    // The re-indexed dictionary is still searchable, with the smaller
    // byte-to-float descriptors this time.
    const std::vector<int> matches = dict->findNN(descriptors.row(0));
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_NE(matches[0], VWDictionary::ID_INVALID);
}

TEST_F(VWDictionaryTest, IsModified)
{
    EXPECT_TRUE(dict->isModified());
    
    // Adding words should keep it modified
    cv::Mat descriptors(3, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    
    EXPECT_TRUE(dict->isModified());
}

TEST_F(VWDictionaryTest, SetLastWordId)
{
    dict->setLastWordId(100);
    
    // Add a word - should get ID > 100
    cv::Mat descriptor = cv::Mat::ones(1, 32, CV_32F);
    std::list<int> wordIds = dict->addNewWords(descriptor, 1);
    
    ASSERT_EQ(wordIds.size(), 1u);
    EXPECT_GT(wordIds.front(), 100);
}

TEST_F(VWDictionaryTest, DeleteUnusedWords)
{
    // Add words (orthogonal descriptors so each gets its own visual word)
    cv::Mat descriptors(3, 32, CV_32F, cv::Scalar(0));
    for(int i = 0; i < 3; ++i)
    {
        descriptors.at<float>(i, i) = 1.0f;
    }
    std::list<int> wordIds = dict->addNewWords(descriptors, 1);
    
    ASSERT_EQ(wordIds.size(), 3u);
    std::set<int> uniqueWordIds(wordIds.begin(), wordIds.end());
    ASSERT_EQ(uniqueWordIds.size(), 3u);
    
    // Remove references to make words unused
    for(int id : uniqueWordIds) {
        dict->removeAllWordRef(id, 1);
    }
    
    EXPECT_EQ(dict->getUnusedWordsSize(), 3u);
    
    // Delete unused words
    dict->deleteUnusedWords();
    
    EXPECT_EQ(dict->getUnusedWordsSize(), 0u);
    EXPECT_EQ(dict->getVisualWords().size(), 0u);
}

TEST_F(VWDictionaryTest, FindNNWithVisualWords)
{
    // Add words
    cv::Mat descriptors(5, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    std::list<int> wordIds = dict->addNewWords(descriptors, 1);
    dict->update();
    
    // Create list of visual words to match (create new words with same descriptors)
    std::list<VisualWord*> vws;
    for(int id : wordIds) {
        const VisualWord* word = dict->getWord(id);
        // Create a new VisualWord with the same descriptor for testing
        VisualWord* testWord = new VisualWord(id + 1000, word->getDescriptor().clone());
        vws.push_back(testWord);
    }

    // That will force to accept close matches
    ParametersMap params;
    params.insert(ParametersPair(Parameters::kKpNndrRatio(), "1"));

    dict->parseParameters(params);
    std::vector<int> matches = dict->findNN(vws);
    EXPECT_EQ(matches.size(), vws.size());
    
    // Each word should match itself
    int i =0;
    for(VisualWord * vw: vws) {
        EXPECT_EQ(vw->id()-1000, matches[i++]);
    }
    
    // Cleanup
    for(VisualWord* vw : vws) {
        delete vw;
    }
}


// What Kp/ByteToFloat costs in matching quality. Both conversions feed the same
// exact index, so any difference comes from the distance they induce: expanding
// each bit to a float keeps the L1 distance equal to the Hamming distance,
// while converting each byte to a float doesn't, one flipped bit moving a byte
// by 1 or by 128 depending on which bit it is.
TEST_F(VWDictionaryTest, ByteToFloatMatchingQuality)
{
    const int words = 200;
    const int bytes = 32; // ORB
    const int queries = 100;

    cv::RNG rng(42);
    cv::Mat descriptors(words, bytes, CV_8U);
    rng.fill(descriptors, cv::RNG::UNIFORM, 0, 256);

    int totalBitExpansion = 0;
    int totalByteToFloat = 0;
    // From a query a few bits away from its word, which any distance finds, to
    // one almost as far as the others are from each other (two random 256 bits
    // descriptors differ by about 128).
    for(int flippedBits: {8, 32, 64, 96})
    {
        // Queries are indexed descriptors with bits flipped, the way the same
        // feature looks when seen again.
        cv::Mat queryDescriptors(queries, bytes, CV_8U);
        for(int i=0; i<queries; ++i)
        {
            descriptors.row(rng.uniform(0, words)).copyTo(queryDescriptors.row(i));
            for(int b=0; b<flippedBits; ++b)
            {
                queryDescriptors.at<unsigned char>(i, rng.uniform(0, bytes)) ^= (1 << rng.uniform(0, 8));
            }
        }

        // Ground truth: the closest descriptor in Hamming distance.
        std::vector<int> hammingNN(queries);
        for(int i=0; i<queries; ++i)
        {
            int best = -1;
            double bestDistance = -1;
            for(int w=0; w<words; ++w)
            {
                const double distance = cv::norm(queryDescriptors.row(i), descriptors.row(w), cv::NORM_HAMMING);
                if(best < 0 || distance < bestDistance)
                {
                    best = w;
                    bestDistance = distance;
                }
            }
            hammingNN[i] = best;
        }

        int correct[2] = {0, 0};
        for(int byteToFloat=0; byteToFloat<2; ++byteToFloat)
        {
            VWDictionary dictionary;
            ParametersMap params;
            // An exact index, so that only the distance the conversion induces
            // can change what is found.
            params.insert(ParametersPair(Parameters::kKpNNStrategy(),
                    uNumber2Str((int)VWDictionary::kNNFlannKdTreeSingle)));
            params.insert(ParametersPair(Parameters::kKpIncrementalFlann(), "false"));
            params.insert(ParametersPair(Parameters::kKpByteToFloat(), byteToFloat?"true":"false"));
            params.insert(ParametersPair(Parameters::kKpNndrRatio(), "0.8"));
            // One word per descriptor: without this, two of the random ones
            // that happen to be close are merged as they are added.
            params.insert(ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
            dictionary.parseParameters(params);

            const std::list<int> addedIds = dictionary.addNewWords(descriptors, 1);
            ASSERT_EQ(addedIds.size(), (size_t)words) << "byteToFloat=" << byteToFloat;
            dictionary.update();
            ASSERT_EQ(dictionary.getVisualWords().size(), (size_t)words) << "byteToFloat=" << byteToFloat;

            const std::vector<int> ids(addedIds.begin(), addedIds.end());
            const std::vector<int> matched = dictionary.findNN(queryDescriptors);
            ASSERT_EQ(matched.size(), (size_t)queries) << "byteToFloat=" << byteToFloat;
            for(int i=0; i<queries; ++i)
            {
                if(matched[i] == ids[hammingNN[i]])
                {
                    ++correct[byteToFloat];
                }
            }
        }

        std::cerr << "[          ] " << flippedBits << "/" << bytes*8
                  << " bits flipped: " << Parameters::kKpByteToFloat() << "=false found "
                  << correct[0] << "/" << queries << " of the Hamming nearest neighbors, =true found "
                  << correct[1] << "/" << queries << "\n";

        // Expanding the bits keeps the Hamming ordering, so it finds what an
        // exhaustive Hamming search finds; converting the bytes cannot do
        // better than that.
        EXPECT_EQ(correct[0], queries) << "flippedBits=" << flippedBits;
        EXPECT_LE(correct[1], correct[0]) << "flippedBits=" << flippedBits;
        totalBitExpansion += correct[0];
        totalByteToFloat += correct[1];
    }

    // The distortion is what the parameter trades for its smaller descriptors:
    // it shows once the true match is not much closer than the others.
    EXPECT_LT(totalByteToFloat, totalBitExpansion);
}
