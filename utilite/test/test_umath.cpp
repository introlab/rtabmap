#include "gtest/gtest.h"
#include "rtabmap/utilite/UMath.h"
#include <cmath>
#include <limits>

TEST(UMathTest, uIsNan)
{
    float nanValue = std::numeric_limits<float>::quiet_NaN();
    float normalValue = 3.14f;
    
    EXPECT_TRUE(uIsNan(nanValue));
    EXPECT_FALSE(uIsNan(normalValue));
    
    double nanDouble = std::numeric_limits<double>::quiet_NaN();
    double normalDouble = 3.14;
    EXPECT_TRUE(uIsNan(nanDouble));
    EXPECT_FALSE(uIsNan(normalDouble));
}

TEST(UMathTest, uIsFinite)
{
    float infValue = std::numeric_limits<float>::infinity();
    float normalValue = 3.14f;
    float nanValue = std::numeric_limits<float>::quiet_NaN();
    
    EXPECT_FALSE(uIsFinite(infValue));
    EXPECT_TRUE(uIsFinite(normalValue));
    EXPECT_FALSE(uIsFinite(nanValue));
}

TEST(UMathTest, uMin3)
{
    EXPECT_EQ(uMin3(3, 1, 2), 1);
    EXPECT_EQ(uMin3(1, 2, 3), 1);
    EXPECT_EQ(uMin3(2, 3, 1), 1);
    EXPECT_EQ(uMin3(1.5f, 2.5f, 0.5f), 0.5f);
}

TEST(UMathTest, uMax3)
{
    EXPECT_EQ(uMax3(3, 1, 2), 3);
    EXPECT_EQ(uMax3(1, 2, 3), 3);
    EXPECT_EQ(uMax3(2, 3, 1), 3);
    EXPECT_EQ(uMax3(1.5f, 2.5f, 0.5f), 2.5f);
}

TEST(UMathTest, uMaxArray)
{
    int arr[] = {1, 5, 3, 9, 2};
    unsigned int index;
    int max = uMax(arr, 5, index);
    EXPECT_EQ(max, 9);
    EXPECT_EQ(index, 3u);
    
    int max2 = uMax(arr, 5);
    EXPECT_EQ(max2, 9);
}

TEST(UMathTest, uMaxVector)
{
    std::vector<int> v = {1, 5, 3, 9, 2};
    unsigned int index;
    int max = uMax(v, index);
    EXPECT_EQ(max, 9);
    EXPECT_EQ(index, 3u);
    
    int max2 = uMax(v);
    EXPECT_EQ(max2, 9);
}

TEST(UMathTest, uMinArray)
{
    int arr[] = {5, 1, 3, 9, 2};
    unsigned int index;
    int min = uMin(arr, 5, index);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(index, 1u);
    
    int min2 = uMin(arr, 5);
    EXPECT_EQ(min2, 1);
}

TEST(UMathTest, uMinVector)
{
    std::vector<int> v = {5, 1, 3, 9, 2};
    unsigned int index;
    int min = uMin(v, index);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(index, 1u);
    
    int min2 = uMin(v);
    EXPECT_EQ(min2, 1);
}

TEST(UMathTest, uMinMaxArray)
{
    int arr[] = {5, 1, 3, 9, 2};
    int min, max;
    unsigned int indexMin, indexMax;
    uMinMax(arr, 5, min, max, indexMin, indexMax);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(max, 9);
    EXPECT_EQ(indexMin, 1u);
    EXPECT_EQ(indexMax, 3u);
}

TEST(UMathTest, uMinMaxVector)
{
    std::vector<int> v = {5, 1, 3, 9, 2};
    int min, max;
    unsigned int indexMin, indexMax;
    uMinMax(v, min, max, indexMin, indexMax);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(max, 9);
    EXPECT_EQ(indexMin, 1u);
    EXPECT_EQ(indexMax, 3u);
}

TEST(UMathTest, uMinMaxWithoutIndices)
{
    int arr[] = {5, 1, 3, 9, 2};
    int min, max;
    uMinMax(arr, 5, min, max);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(max, 9);
    
    std::vector<int> v = {5, 1, 3, 9, 2};
    uMinMax(v, min, max);
    EXPECT_EQ(min, 1);
    EXPECT_EQ(max, 9);
}

TEST(UMathTest, uSign)
{
    EXPECT_EQ(uSign(5), 1);
    EXPECT_EQ(uSign(-5), -1);
    EXPECT_EQ(uSign(0), 1);
    EXPECT_EQ(uSign(3.14f), 1);
    EXPECT_EQ(uSign(-3.14f), -1);
}

TEST(UMathTest, uSumList)
{
    std::list<int> list = {1, 2, 3, 4, 5};
    int sum = uSum(list);
    EXPECT_EQ(sum, 15);
}

TEST(UMathTest, uSumArray)
{
    int arr[] = {1, 2, 3, 4, 5};
    int sum = uSum(arr, 5);
    EXPECT_EQ(sum, 15);
}

TEST(UMathTest, uSumVector)
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    int sum = uSum(v);
    EXPECT_EQ(sum, 15);
}

TEST(UMathTest, uSumSquared)
{
    int arr[] = {1, 2, 3};
    int sum = uSumSquared(arr, 3);
    EXPECT_EQ(sum, 14); // 1*1 + 2*2 + 3*3 = 1 + 4 + 9 = 14
    
    int sum2 = uSumSquared(arr, 3, 2); // subtract 2
    EXPECT_EQ(sum2, 2); // (-1)*(-1) + 0*0 + 1*1 = 1 + 0 + 1 = 2
}

TEST(UMathTest, uMean)
{
    int arr[] = {1, 2, 3, 4, 5};
    float mean = uMean(arr, 5);
    EXPECT_FLOAT_EQ(mean, 3.0f);
    
    std::vector<int> v = {1, 2, 3, 4, 5};
    float mean2 = uMean(v);
    EXPECT_FLOAT_EQ(mean2, 3.0f);
    
    std::list<int> list = {1, 2, 3, 4, 5};
    float mean3 = uMean(list);
    EXPECT_FLOAT_EQ(mean3, 3.0f);
}

TEST(UMathTest, uMeanSquaredError)
{
    float x[] = {1.0f, 2.0f, 3.0f};
    float y[] = {1.1f, 2.1f, 3.1f};
    float mse = uMeanSquaredError(x, 3, y, 3);
    EXPECT_NEAR(mse, 0.01f, 0.001f); // (0.1^2 + 0.1^2 + 0.1^2) / 3 = 0.01
    
    std::vector<float> vx = {1.0f, 2.0f, 3.0f};
    std::vector<float> vy = {1.1f, 2.1f, 3.1f};
    float mse2 = uMeanSquaredError(vx, vy);
    EXPECT_NEAR(mse2, 0.01f, 0.001f);
}

TEST(UMathTest, uVariance)
{
    float arr[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float mean = uMean(arr, 5);
    float variance = uVariance(arr, 5, mean);
    EXPECT_NEAR(variance, 2.5f, 0.1f); // Variance of 1,2,3,4,5 is 2.5
    
    float variance2 = uVariance(arr, 5);
    EXPECT_NEAR(variance2, 2.5f, 0.1f);
}

TEST(UMathTest, uNormSquared)
{
    std::vector<float> v = {3.0f, 4.0f};
    float normSq = uNormSquared(v);
    EXPECT_FLOAT_EQ(normSq, 25.0f); // 3*3 + 4*4 = 9 + 16 = 25
    
    float normSq2 = uNormSquared(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(normSq2, 25.0f);
    
    float normSq3 = uNormSquared(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(normSq3, 14.0f); // 1*1 + 2*2 + 3*3 = 1 + 4 + 9 = 14
}

TEST(UMathTest, uNorm)
{
    std::vector<float> v = {3.0f, 4.0f};
    float norm = uNorm(v);
    EXPECT_FLOAT_EQ(norm, 5.0f); // sqrt(3*3 + 4*4) = sqrt(25) = 5
    
    float norm2 = uNorm(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(norm2, 5.0f);
    
    float norm3 = uNorm(1.0f, 2.0f, 3.0f);
    EXPECT_NEAR(norm3, 3.741657f, 0.001f); // sqrt(14)
}

TEST(UMathTest, uNormalize)
{
    std::vector<float> v = {3.0f, 4.0f};
    std::vector<float> normalized = uNormalize(v);
    float norm = uNorm(normalized);
    EXPECT_NEAR(norm, 1.0f, 0.001f);
    
    std::vector<float> zero = {0.0f, 0.0f};
    std::vector<float> normalizedZero = uNormalize(zero);
    EXPECT_EQ(normalizedZero.size(), 2u);
    EXPECT_FLOAT_EQ(normalizedZero[0], 0.0f);
    EXPECT_FLOAT_EQ(normalizedZero[1], 0.0f);
}

TEST(UMathTest, uLocalMaxima)
{
    float arr[] = {1.0f, 3.0f, 2.0f, 5.0f, 4.0f, 6.0f, 3.0f};
    std::list<unsigned int> maxima = uLocalMaxima(arr, 7);
    // Should find maxima at indices 1, 3, 5
    EXPECT_GT(maxima.size(), 0u);
    
    std::vector<float> v = {1.0f, 3.0f, 2.0f, 5.0f, 4.0f};
    std::list<unsigned int> maxima2 = uLocalMaxima(v);
    EXPECT_GT(maxima2.size(), 0u);
}

TEST(UMathTest, uHamming)
{
    std::vector<float> window = uHamming(10);
    EXPECT_EQ(window.size(), 10u);
    // Hamming window values should be between 0 and 1
    for(float w : window)
    {
        EXPECT_GE(w, 0.0f);
        EXPECT_LE(w, 1.0f);
    }
}

TEST(UMathTest, uIsInBounds)
{
    EXPECT_TRUE(uIsInBounds(5, 0, 10));
    EXPECT_FALSE(uIsInBounds(10, 0, 10));
    EXPECT_FALSE(uIsInBounds(-1, 0, 10));
    EXPECT_FALSE(uIsInBounds(std::numeric_limits<float>::quiet_NaN(), 0.0f, 10.0f));
    EXPECT_FALSE(uIsInBounds(std::numeric_limits<float>::infinity(), 0.0f, 10.0f));
}

