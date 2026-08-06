#include "gtest/gtest.h"
#include "rtabmap/utilite/UProcessInfo.h"

TEST(UProcessInfoTest, Constructor)
{
    UProcessInfo info;
    // Should be constructible
    SUCCEED();
}

TEST(UProcessInfoTest, GetMemoryUsage)
{
    long int memory = UProcessInfo::getMemoryUsage();
    // Memory usage should be positive
    EXPECT_GT(memory, 0);
    // Should be reasonable (less than 1TB)
    EXPECT_LT(memory, 1024LL * 1024 * 1024 * 1024);
}

TEST(UProcessInfoTest, GetMemoryUsageMultipleCalls)
{
    long int memory1 = UProcessInfo::getMemoryUsage();
    long int memory2 = UProcessInfo::getMemoryUsage();
    // Both calls should return valid values
    EXPECT_GT(memory1, 0);
    EXPECT_GT(memory2, 0);
    // Values should be similar (might vary slightly)
    EXPECT_NEAR(memory1, memory2, memory1 * 0.1); // Allow 10% variation
}

