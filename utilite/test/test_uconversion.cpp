#include "gtest/gtest.h"
#include "rtabmap/utilite/UConversion.h"

TEST(UConversionTest, uReplaceCharChar)
{
    std::string str = "Hello";
    std::string result = uReplaceChar(str, 'l', 'p');
    EXPECT_EQ(result, "Heppo");
}

TEST(UConversionTest, uReplaceCharString)
{
    std::string str = "Hello";
    std::string result = uReplaceChar(str, 'o', "oween");
    EXPECT_EQ(result, "Helloween");
}

TEST(UConversionTest, uToUpperCase)
{
    std::string str = "hello!";
    std::string result = uToUpperCase(str);
    EXPECT_EQ(result, "HELLO!");
}

TEST(UConversionTest, uToLowerCase)
{
    std::string str = "HELLO!";
    std::string result = uToLowerCase(str);
    EXPECT_EQ(result, "hello!");
}

TEST(UConversionTest, uNumber2StrUnsignedInt)
{
    unsigned int number = 42;
    std::string result = uNumber2Str(number);
    EXPECT_EQ(result, "42");
}

TEST(UConversionTest, uNumber2StrInt)
{
    int number = -42;
    std::string result = uNumber2Str(number);
    EXPECT_EQ(result, "-42");
}

TEST(UConversionTest, uNumber2StrFloat)
{
    float number = 0.14159f;
    std::string result = uNumber2Str(number, 2, false);
    EXPECT_STREQ(result.c_str(), "0.14");

    number = 3.14159f;
    result = uNumber2Str(number, 2, false);
    EXPECT_STREQ(result.c_str(), "3.1");
}

TEST(UConversionTest, uNumber2StrDouble)
{
    double number = 0.14159;
    std::string result = uNumber2Str(number, 2, false);
    EXPECT_STREQ(result.c_str(), "0.14");

    number = 3.14159;
    result = uNumber2Str(number, 2, false);
    EXPECT_STREQ(result.c_str(), "3.1");
}

TEST(UConversionTest, uStr2Int)
{
    std::string str = "42";
    int result = uStr2Int(str);
    EXPECT_EQ(result, 42);
}

TEST(UConversionTest, uStr2IntNegative)
{
    std::string str = "-42";
    int result = uStr2Int(str);
    EXPECT_EQ(result, -42);
}

TEST(UConversionTest, uStr2Float)
{
    std::string str = "3.14";
    float result = uStr2Float(str);
    EXPECT_NEAR(result, 3.14f, 0.001f);
}

TEST(UConversionTest, uStr2FloatComma)
{
    std::string str = "3,14";
    float result = uStr2Float(str);
    EXPECT_NEAR(result, 3.14f, 0.001f);
}

TEST(UConversionTest, uStr2Double)
{
    std::string str = "3.14159";
    double result = uStr2Double(str);
    EXPECT_NEAR(result, 3.14159, 0.00001);
}

TEST(UConversionTest, uStr2DoubleComma)
{
    std::string str = "3,14159";
    double result = uStr2Double(str);
    EXPECT_NEAR(result, 3.14159, 0.00001);
}

TEST(UConversionTest, uBool2Str)
{
    std::string resultTrue = uBool2Str(true);
    std::string resultFalse = uBool2Str(false);
    EXPECT_EQ(resultTrue, "true");
    EXPECT_EQ(resultFalse, "false");
}

TEST(UConversionTest, uStr2Bool)
{
    EXPECT_TRUE(uStr2Bool("true"));
    EXPECT_TRUE(uStr2Bool("TRUE"));
    EXPECT_TRUE(uStr2Bool("True"));
    EXPECT_FALSE(uStr2Bool("false"));
    EXPECT_FALSE(uStr2Bool("FALSE"));
    EXPECT_FALSE(uStr2Bool("0"));
    EXPECT_TRUE(uStr2Bool("1"));
    EXPECT_TRUE(uStr2Bool("anything"));
}

TEST(UConversionTest, uStr2Bytes)
{
    std::string str = "Hello";
    std::vector<unsigned char> bytes = uStr2Bytes(str);
    EXPECT_EQ(bytes.size(), 6); // "Hello" + null terminator
    EXPECT_EQ(bytes[0], 'H');
    EXPECT_EQ(bytes[4], 'o');
    EXPECT_EQ(bytes[5], '\0');
}

TEST(UConversionTest, uBytes2Str)
{
    std::vector<unsigned char> bytes = {'H', 'e', 'l', 'l', 'o', '\0'};
    std::string result = uBytes2Str(bytes);
    EXPECT_EQ(result, "Hello");
}

TEST(UConversionTest, uBytes2Hex)
{
    char bytes[] = {0x3F, 0x1A};
    std::string result = uBytes2Hex(bytes, 2);
    EXPECT_EQ(result, "3F1A");
}

TEST(UConversionTest, uHex2Bytes)
{
    std::string hex = "1f3B";
    std::vector<char> bytes = uHex2Bytes(hex);
    EXPECT_EQ(bytes.size(), 2);
    EXPECT_EQ(bytes[0], 0x1F);
    EXPECT_EQ(bytes[1], 0x3B);
}

TEST(UConversionTest, uHex2BytesWithLength)
{
    std::string hex = "1f3B";
    std::vector<char> bytes = uHex2Bytes(hex.c_str(), 4);
    EXPECT_EQ(bytes.size(), 2);
    EXPECT_EQ(bytes[0], 0x1F);
    EXPECT_EQ(bytes[1], 0x3B);
}

TEST(UConversionTest, uHex2Str)
{
    std::string hex = "48656C6C6F21";
    std::string result = uHex2Str(hex);
    EXPECT_EQ(result, "Hello!");
}

TEST(UConversionTest, uHex2Ascii)
{
    unsigned char c = 0xFA;
    unsigned char left = uHex2Ascii(c, false);
    unsigned char right = uHex2Ascii(c, true);
    EXPECT_EQ(left, 'F');
    EXPECT_EQ(right, 'A');
}

TEST(UConversionTest, uAscii2Hex)
{
    unsigned char result = uAscii2Hex('F');
    EXPECT_EQ(result, 0x0F);
    result = uAscii2Hex('a');
    EXPECT_EQ(result, 0x0A);
}

TEST(UConversionTest, uFormat)
{
    std::string result = uFormat("Hello %s %d", "world", 42);
    EXPECT_NE(result.find("Hello"), std::string::npos);
    EXPECT_NE(result.find("world"), std::string::npos);
    EXPECT_NE(result.find("42"), std::string::npos);
}

