#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UFile.h"
#include <vector>
#include <list>
#include <iterator>
#include <fstream>
#include <cstdio>

using namespace rtabmap;

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

TEST_F(VWDictionaryTest, AddNewWords_Incremental)
{
    // Test incremental mode - NNDR is applied, new words created if NNDR fails
    // Test with all NNStrategy values
    VWDictionary::NNStrategy strategies[] = {
        VWDictionary::kNNFlannNaive,
        VWDictionary::kNNFlannKdTree,
        VWDictionary::kNNFlannLSH,
        VWDictionary::kNNBruteForce,
        VWDictionary::kNNBruteForceGPU
    };

    // That will mke logic below works with numbers chosen
    ParametersMap params;
    params.insert(ParametersPair(Parameters::kKpNndrRatio(), "0.4"));
    dict->parseParameters(params);

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);

    for(VWDictionary::NNStrategy strategy : strategies)
    {
        // Reset dictionary for each strategy
        dict->clear();
        dict->setNNStrategy(strategy);
        
        EXPECT_TRUE(dict->isIncremental());

        if(strategy == VWDictionary::kNNBruteForceGPU)
        {
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
            if(!cv::gpu::getCudaEnabledDeviceCount())
            {
                strategy = VWDictionary::kNNBruteForce;
            }
#else
            strategy = VWDictionary::kNNBruteForce;
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
            if(!cv::cuda::getCudaEnabledDeviceCount())
            {
                strategy = VWDictionary::kNNBruteForce;
            }
#else
            strategy = VWDictionary::kNNBruteForce;
#endif
#endif
        }
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

TEST_F(VWDictionaryTest, AddNewWords_Fixed)
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

TEST_F(VWDictionaryTest, FindNN_Incremental)
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

TEST_F(VWDictionaryTest, FindNN_Fixed)
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

TEST_F(VWDictionaryTest, ConvertBinTo32F_ByteToFloat)
{
    // Test byteToFloat = true (simple conversion)
    cv::Mat input(2, 10, CV_8UC1);
    cv::randu(input, cv::Scalar(0), cv::Scalar(255));
    
    cv::Mat output = VWDictionary::convertBinTo32F(input, true);
    
    EXPECT_EQ(output.type(), CV_32FC1);
    EXPECT_EQ(output.rows, 2);
    EXPECT_EQ(output.cols, 10); // Same dimensions
}

TEST_F(VWDictionaryTest, ConvertBinTo32F_BitExpansion)
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

TEST_F(VWDictionaryTest, Convert32FToBin_ByteToFloat)
{
    // Test byteToFloat = true (simple conversion)
    cv::Mat input(2, 10, CV_32FC1);
    cv::randu(input, cv::Scalar(0), cv::Scalar(255));
    
    cv::Mat output = VWDictionary::convert32FToBin(input, true);
    
    EXPECT_EQ(output.type(), CV_8UC1);
    EXPECT_EQ(output.rows, 2);
    EXPECT_EQ(output.cols, 10); // Same dimensions
}

TEST_F(VWDictionaryTest, Convert32FToBin_BitPacking)
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
    // Add words and build index
    cv::Mat descriptors(5, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    dict->addNewWords(descriptors, 1);
    dict->update();
    
    // Serialize
    std::vector<unsigned char> data = dict->serializeIndex();
    EXPECT_GT(data.size(), 0u);
    
    // Create new dictionary and deserialize
    VWDictionary dict2;
    cv::Mat descriptors2(5, 32, CV_32F);
    cv::randu(descriptors2, cv::Scalar(0), cv::Scalar(1));
    dict2.addNewWords(descriptors2, 1);
    
    // Deserialize should fail because we are not using the same descriptors
    bool success = dict2.deserializeIndex(data);
    EXPECT_FALSE(success);
    success = dict2.deserializeIndex(data.data(), data.size());
    EXPECT_FALSE(success);

    // Same descriptors
    VWDictionary dict3;
    dict3.addNewWords(descriptors, 1);
    success = dict3.deserializeIndex(data);
    EXPECT_TRUE(success);

    // Index should be loaded
    EXPECT_GT(dict3.getIndexedWordsCount(), 0u);

    // Should fail if index is already built
    success = dict3.deserializeIndex(data);
    EXPECT_FALSE(success);

    // raw bytes
    VWDictionary dict4;
    dict4.addNewWords(descriptors, 1);
    success = dict4.deserializeIndex(data.data(), data.size());
    EXPECT_TRUE(success);

    // Index should be loaded
    EXPECT_GT(dict4.getIndexedWordsCount(), 0u);

    // Should fail if index is already built
    success = dict4.deserializeIndex(data.data(), data.size());
    EXPECT_FALSE(success);
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
    // Add words
    cv::Mat descriptors(3, 32, CV_32F);
    cv::randu(descriptors, cv::Scalar(0), cv::Scalar(1));
    std::list<int> wordIds = dict->addNewWords(descriptors, 1);
    
    ASSERT_EQ(wordIds.size(), 3u);
    
    // Remove references to make words unused
    for(int id : wordIds) {
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

