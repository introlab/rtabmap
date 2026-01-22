#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include "rtabmap/core/VisualWord.h"
#include <map>

using namespace rtabmap;

TEST(VisualWordTest, ConstructorAndAccessors)
{
    cv::Mat descriptor = (cv::Mat_<float>(1, 3) << 1.0, 2.0, 3.0);
    int wordId = 123;
    int signatureId = 10;

    VisualWord word(wordId, descriptor, signatureId);

    EXPECT_EQ(word.id(), wordId);
    EXPECT_EQ(word.getTotalReferences(), 1);
    EXPECT_EQ(word.getDescriptor().cols, descriptor.cols);
    EXPECT_EQ(word.getDescriptor().rows, descriptor.rows);

    auto refs = word.getReferences();
    ASSERT_EQ(refs.size(), 1);
    EXPECT_EQ(refs.at(signatureId), 1);
}

TEST(VisualWordTest, AddMultipleReferences)
{
    VisualWord word(1, cv::Mat::ones(1, 5, CV_32F));

    word.addRef(100);
    word.addRef(100); // same signature
    word.addRef(200);

    EXPECT_EQ(word.getTotalReferences(), 3);

    auto refs = word.getReferences();
    EXPECT_EQ(refs.size(), 2);
    EXPECT_EQ(refs.at(100), 2);
    EXPECT_EQ(refs.at(200), 1);
}

TEST(VisualWordTest, RemoveReferences)
{
    VisualWord word(1, cv::Mat::zeros(1, 3, CV_8U));
    word.addRef(10);
    word.addRef(10);
    word.addRef(20);

    EXPECT_EQ(word.getTotalReferences(), 3);

    int removed = word.removeAllRef(10);
    EXPECT_EQ(removed, 2);
    EXPECT_EQ(word.getTotalReferences(), 1);

    auto refs = word.getReferences();
    EXPECT_EQ(refs.size(), 1);
    EXPECT_TRUE(refs.find(10) == refs.end());
    EXPECT_EQ(refs.at(20), 1);
}

TEST(VisualWordTest, SavedFlagBehavior)
{
    VisualWord word(1, cv::Mat());
    EXPECT_FALSE(word.isSaved());

    word.setSaved(true);
    EXPECT_TRUE(word.isSaved());

    word.setSaved(false);
    EXPECT_FALSE(word.isSaved());
}

TEST(VisualWordTest, MemoryUsedEstimation)
{
    cv::Mat descriptor = cv::Mat::ones(1, 128, CV_32FC1); // simulate SIFT descriptor
    VisualWord word(42, descriptor);

    unsigned long mem = word.getMemoryUsed();

    // Should be at least size of descriptor
    EXPECT_GE(mem, 128ul*sizeof(float));
}