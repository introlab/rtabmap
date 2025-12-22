#include "gtest/gtest.h"
#include "rtabmap/utilite/UVariant.h"

TEST(UVariantTest, DefaultConstructor)
{
    UVariant v;
    EXPECT_TRUE(v.isUndef());
    EXPECT_EQ(v.type(), UVariant::kUndef);
}

TEST(UVariantTest, BoolConstructor)
{
    UVariant v(true);
    EXPECT_TRUE(v.isBool());
    EXPECT_EQ(v.type(), UVariant::kBool);
    EXPECT_TRUE(v.toBool());
    
    UVariant v2(false);
    EXPECT_FALSE(v2.toBool());
}

TEST(UVariantTest, CharConstructor)
{
    signed char value = 42;
    UVariant v(value);
    EXPECT_TRUE(v.isChar());
    EXPECT_EQ(v.type(), UVariant::kChar);
    bool ok = false;
    EXPECT_EQ(v.toChar(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, UCharConstructor)
{
    unsigned char value = 200;
    UVariant v(value);
    EXPECT_TRUE(v.isUChar());
    EXPECT_EQ(v.type(), UVariant::kUChar);
    bool ok = false;
    EXPECT_EQ(v.toUChar(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, ShortConstructor)
{
    short value = -100;
    UVariant v(value);
    EXPECT_TRUE(v.isShort());
    EXPECT_EQ(v.type(), UVariant::kShort);
    bool ok = false;
    EXPECT_EQ(v.toShort(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, UShortConstructor)
{
    unsigned short value = 500;
    UVariant v(value);
    EXPECT_TRUE(v.isUShort());
    EXPECT_EQ(v.type(), UVariant::kUShort);
    bool ok = false;
    EXPECT_EQ(v.toUShort(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, IntConstructor)
{
    int value = -1000;
    UVariant v(value);
    EXPECT_TRUE(v.isInt());
    EXPECT_EQ(v.type(), UVariant::kInt);
    bool ok = false;
    EXPECT_EQ(v.toInt(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, UIntConstructor)
{
    unsigned int value = 100000;
    UVariant v(value);
    EXPECT_TRUE(v.isUInt());
    EXPECT_EQ(v.type(), UVariant::kUInt);
    bool ok = false;
    EXPECT_EQ(v.toUInt(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, FloatConstructor)
{
    float value = 3.14f;
    UVariant v(value);
    EXPECT_TRUE(v.isFloat());
    EXPECT_EQ(v.type(), UVariant::kFloat);
    bool ok = false;
    EXPECT_NEAR(v.toFloat(&ok), value, 0.001f);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, DoubleConstructor)
{
    double value = 3.14159;
    UVariant v(value);
    EXPECT_TRUE(v.isDouble());
    EXPECT_EQ(v.type(), UVariant::kDouble);
    bool ok = false;
    EXPECT_NEAR(v.toDouble(&ok), value, 0.00001);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, StringConstructor)
{
    std::string value = "Hello World";
    UVariant v(value);
    EXPECT_TRUE(v.isStr());
    EXPECT_EQ(v.type(), UVariant::kStr);
    bool ok = false;
    EXPECT_EQ(v.toStr(&ok), value);
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, CStringConstructor)
{
    const char* value = "Hello World";
    UVariant v(value);
    EXPECT_TRUE(v.isStr());
    EXPECT_EQ(v.type(), UVariant::kStr);
    bool ok = false;
    EXPECT_EQ(v.toStr(&ok), std::string(value));
    EXPECT_TRUE(ok);
}

TEST(UVariantTest, CharArrayConstructor)
{
    std::vector<signed char> value = {1, 2, 3, -4};
    UVariant v(value);
    EXPECT_TRUE(v.isCharArray());
    EXPECT_EQ(v.type(), UVariant::kCharArray);
    bool ok = false;
    std::vector<signed char> result = v.toCharArray(&ok);
    EXPECT_TRUE(ok);
    EXPECT_EQ(result.size(), value.size());
    for(size_t i = 0; i < value.size(); ++i)
    {
        EXPECT_EQ(result[i], value[i]);
    }
}

TEST(UVariantTest, UCharArrayConstructor)
{
    std::vector<unsigned char> value = {1, 2, 3, 4};
    UVariant v(value);
    EXPECT_TRUE(v.isUCharArray());
    EXPECT_EQ(v.type(), UVariant::kUCharArray);
    bool ok = false;
    std::vector<unsigned char> result = v.toUCharArray(&ok);
    EXPECT_TRUE(ok);
    EXPECT_EQ(result.size(), value.size());
    for(size_t i = 0; i < value.size(); ++i)
    {
        EXPECT_EQ(result[i], value[i]);
    }
}

TEST(UVariantTest, IntArrayConstructor)
{
    std::vector<int> value = {1, -2, 3, -4, 5};
    UVariant v(value);
    EXPECT_TRUE(v.isIntArray());
    EXPECT_EQ(v.type(), UVariant::kIntArray);
    bool ok = false;
    std::vector<int> result = v.toIntArray(&ok);
    EXPECT_TRUE(ok);
    EXPECT_EQ(result.size(), value.size());
    for(size_t i = 0; i < value.size(); ++i)
    {
        EXPECT_EQ(result[i], value[i]);
    }
}

TEST(UVariantTest, FloatArrayConstructor)
{
    std::vector<float> value = {1.1f, 2.2f, 3.3f};
    UVariant v(value);
    EXPECT_TRUE(v.isFloatArray());
    EXPECT_EQ(v.type(), UVariant::kFloatArray);
    bool ok = false;
    std::vector<float> result = v.toFloatArray(&ok);
    EXPECT_TRUE(ok);
    EXPECT_EQ(result.size(), value.size());
    for(size_t i = 0; i < value.size(); ++i)
    {
        EXPECT_NEAR(result[i], value[i], 0.001f);
    }
}

TEST(UVariantTest, TypeChecks)
{
    UVariant v(42);
    EXPECT_TRUE(v.isInt());
    EXPECT_FALSE(v.isBool());
    EXPECT_FALSE(v.isFloat());
    EXPECT_FALSE(v.isStr());
    EXPECT_FALSE(v.isUndef());
}

TEST(UVariantTest, ConversionFailure)
{
    UVariant v(42); // int
    bool ok = true;
    v.toFloat(&ok);
    EXPECT_FALSE(ok); // Should fail to convert int to float
}

