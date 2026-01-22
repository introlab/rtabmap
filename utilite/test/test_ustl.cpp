#include "gtest/gtest.h"
#include "rtabmap/utilite/UStl.h"
#include <map>

TEST(UStlTest, uUniqueKeys)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({1, "b"});
    mm.insert({2, "c"});
    mm.insert({3, "d"});
    mm.insert({3, "e"});
    
    std::list<int> uniqueKeys = uUniqueKeys(mm);
    EXPECT_EQ(uniqueKeys.size(), 3u);
    EXPECT_EQ(uniqueKeys.front(), 1);
    EXPECT_EQ(uniqueKeys.back(), 3);
}

TEST(UStlTest, uKeysMultimap)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({2, "b"});
    mm.insert({3, "c"});
    
    std::vector<int> keys = uKeys(mm);
    EXPECT_EQ(keys.size(), 3u);
    
    std::list<int> keysList = uKeysList(mm);
    EXPECT_EQ(keysList.size(), 3u);
}

TEST(UStlTest, uValuesMultimap)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({1, "b"});
    mm.insert({2, "c"});
    
    std::vector<std::string> values = uValues(mm);
    EXPECT_EQ(values.size(), 3u);
    
    std::list<std::string> valuesList = uValuesList(mm);
    EXPECT_EQ(valuesList.size(), 3u);
}

TEST(UStlTest, uValuesMultimapForKey)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({1, "b"});
    mm.insert({2, "c"});
    
    std::list<std::string> values = uValues(mm, 1);
    EXPECT_EQ(values.size(), 2u);
    EXPECT_EQ(values.front(), "a");
    EXPECT_EQ(values.back(), "b");
}

TEST(UStlTest, uKeysMap)
{
    std::map<int, std::string> m;
    m[1] = "a";
    m[2] = "b";
    m[3] = "c";
    
    std::vector<int> keys = uKeys(m);
    EXPECT_EQ(keys.size(), 3u);
    
    std::list<int> keysList = uKeysList(m);
    EXPECT_EQ(keysList.size(), 3u);
    
    std::set<int> keysSet = uKeysSet(m);
    EXPECT_EQ(keysSet.size(), 3u);
}

TEST(UStlTest, uValuesMap)
{
    std::map<int, std::string> m;
    m[1] = "a";
    m[2] = "b";
    m[3] = "c";
    
    std::vector<std::string> values = uValues(m);
    EXPECT_EQ(values.size(), 3u);
    
    std::list<std::string> valuesList = uValuesList(m);
    EXPECT_EQ(valuesList.size(), 3u);
}

TEST(UStlTest, uValueMap)
{
    std::map<int, std::string> m;
    m[1] = "a";
    m[2] = "b";
    
    std::string value = uValue(m, 1, std::string("default"));
    EXPECT_EQ(value, "a");
    
    std::string defaultValue = uValue(m, 99, std::string("default"));
    EXPECT_EQ(defaultValue, "default");
}

TEST(UStlTest, uTake)
{
    std::map<int, std::string> m;
    m[1] = "a";
    m[2] = "b";
    
    std::string value = uTake(m, 1, std::string("default"));
    EXPECT_EQ(value, "a");
    EXPECT_EQ(m.size(), 1u);
    EXPECT_EQ(m.find(1), m.end());
}

TEST(UStlTest, uIteratorAtList)
{
    std::list<int> list = {1, 2, 3, 4, 5};
    auto iter = uIteratorAt(list, 2);
    EXPECT_EQ(*iter, 3);
    
    const std::list<int>& constList = list;
    auto constIter = uIteratorAt(constList, 2);
    EXPECT_EQ(*constIter, 3);
}

TEST(UStlTest, uIteratorAtSet)
{
    std::set<int> s = {1, 2, 3, 4, 5};
    auto iter = uIteratorAt(s, 2);
    EXPECT_NE(iter, s.end());
}

TEST(UStlTest, uIteratorAtVector)
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    auto iter = uIteratorAt(v, 2);
    EXPECT_EQ(*iter, 3);
    
    const std::vector<int>& constV = v;
    auto constIter = uIteratorAt(constV, 2);
    EXPECT_EQ(*constIter, 3);
}

TEST(UStlTest, uValueAtList)
{
    std::list<int> list = {1, 2, 3, 4, 5};
    int& value = uValueAt(list, 2);
    EXPECT_EQ(value, 3);
    value = 10;
    EXPECT_EQ(uValueAt(list, 2), 10);
    
    const std::list<int>& constList = list;
    const int& constValue = uValueAt(constList, 2);
    EXPECT_EQ(constValue, 10);
}

TEST(UStlTest, uContainsList)
{
    std::list<int> list = {1, 2, 3, 4, 5};
    EXPECT_TRUE(uContains(list, 3));
    EXPECT_FALSE(uContains(list, 10));
}

TEST(UStlTest, uContainsMap)
{
    std::map<int, std::string> m;
    m[1] = "a";
    m[2] = "b";
    EXPECT_TRUE(uContains(m, 1));
    EXPECT_FALSE(uContains(m, 10));
}

TEST(UStlTest, uContainsMultimap)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({2, "b"});
    EXPECT_TRUE(uContains(mm, 1));
    EXPECT_FALSE(uContains(mm, 10));
}

TEST(UStlTest, uInsert)
{
    std::map<int, std::string> m;
    uInsert(m, std::make_pair(1, std::string("a")));
    EXPECT_EQ(m[1], "a");
    
    uInsert(m, std::make_pair(1, std::string("b")));
    EXPECT_EQ(m[1], "b"); // Should replace
    
    std::map<int, std::string> m2;
    m2[2] = "c";
    m2[3] = "d";
    uInsert(m, m2);
    EXPECT_EQ(m.size(), 3u);
}

TEST(UStlTest, uListToVector)
{
    std::list<int> list = {1, 2, 3, 4, 5};
    std::vector<int> v = uListToVector(list);
    EXPECT_EQ(v.size(), 5u);
    EXPECT_EQ(v[0], 1);
    EXPECT_EQ(v[4], 5);
}

TEST(UStlTest, uVectorToList)
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    std::list<int> list = uVectorToList(v);
    EXPECT_EQ(list.size(), 5u);
    EXPECT_EQ(list.front(), 1);
    EXPECT_EQ(list.back(), 5);
}

TEST(UStlTest, uMultimapToMap)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({1, "b"});
    mm.insert({2, "c"});
    
    std::map<int, std::string> m = uMultimapToMap(mm);
    EXPECT_EQ(m.size(), 2u); // All entries
}

TEST(UStlTest, uMultimapToMapUnique)
{
    std::multimap<int, std::string> mm;
    mm.insert({1, "a"});
    mm.insert({1, "b"});
    mm.insert({2, "c"});
    
    std::map<int, std::string> m = uMultimapToMapUnique(mm);
    EXPECT_EQ(m.size(), 1u); // Only unique keys with single value
    EXPECT_EQ(m[2], "c");
}

TEST(UStlTest, uAppend)
{
    std::list<int> list1 = {1, 2, 3};
    std::list<int> list2 = {4, 5, 6};
    uAppend(list1, list2);
    EXPECT_EQ(list1.size(), 6u);
    EXPECT_EQ(list1.back(), 6);
}

TEST(UStlTest, uIndexOf)
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    EXPECT_EQ(uIndexOf(v, 3), 2);
    EXPECT_EQ(uIndexOf(v, 10), -1);
}

TEST(UStlTest, uSplit)
{
    std::list<std::string> result = uSplit("Hello the world!", ' ');
    EXPECT_EQ(result.size(), 3u);
    EXPECT_EQ(result.front(), "Hello");
    EXPECT_EQ(result.back(), "world!");
    
    std::list<std::string> result2 = uSplit("a,b,c", ',');
    EXPECT_EQ(result2.size(), 3u);
}

TEST(UStlTest, uJoin)
{
    std::list<std::string> strings;
    strings.push_back("Hello");
    strings.push_back("world!");
    std::string joined = uJoin(strings, " ");
    EXPECT_EQ(joined, "Hello world!");
    
    std::string joined2 = uJoin(strings, "");
    EXPECT_EQ(joined2, "Helloworld!");
}

TEST(UStlTest, uIsDigit)
{
    EXPECT_TRUE(uIsDigit('0'));
    EXPECT_TRUE(uIsDigit('5'));
    EXPECT_TRUE(uIsDigit('9'));
    EXPECT_FALSE(uIsDigit('a'));
    EXPECT_FALSE(uIsDigit('A'));
}

TEST(UStlTest, uIsInteger)
{
    EXPECT_TRUE(uIsInteger("123"));
    EXPECT_TRUE(uIsInteger("-123"));
    EXPECT_FALSE(uIsInteger("12.3"));
    EXPECT_FALSE(uIsInteger("abc"));
    
    EXPECT_TRUE(uIsInteger("123", true));
    EXPECT_TRUE(uIsInteger("123", false));
    EXPECT_FALSE(uIsInteger("-123", false));
}

TEST(UStlTest, uIsNumber)
{
    EXPECT_TRUE(uIsNumber("123"));
    EXPECT_TRUE(uIsNumber("12.3"));
    EXPECT_TRUE(uIsNumber("-12.3"));
    EXPECT_FALSE(uIsNumber("abc"));
    EXPECT_FALSE(uIsNumber("12.3.4"));
}

TEST(UStlTest, uSplitNumChar)
{
    std::list<std::string> result = uSplitNumChar("Hello 03 my 65 world!");
    EXPECT_GT(result.size(), 0u);
    // Should contain numbers and characters separately
}

TEST(UStlTest, uStrNumCmp)
{
    EXPECT_LT(uStrNumCmp("Image9.jpg", "Image10.jpg"), 0);
    EXPECT_GT(uStrNumCmp("Image10.jpg", "Image9.jpg"), 0);
    EXPECT_EQ(uStrNumCmp("Image10.jpg", "Image10.jpg"), 0);
}

TEST(UStlTest, uStrContains)
{
    EXPECT_TRUE(uStrContains("Hello World", "World"));
    EXPECT_TRUE(uStrContains("Hello World", "Hello"));
    EXPECT_FALSE(uStrContains("Hello World", "xyz"));
}

TEST(UStlTest, uCompareVersion)
{
    EXPECT_GT(uCompareVersion("2.0.0", 1, 0, 0), 0);
    EXPECT_EQ(uCompareVersion("1.0.0", 1, 0, 0), 0);
    EXPECT_LT(uCompareVersion("0.9.0", 1, 0, 0), 0);
}

TEST(UStlTest, uPad)
{
    std::string result = uPad("Hello", 10);
    EXPECT_GE(result.size(), 10u);
    EXPECT_EQ(result.substr(0, 5), "Hello");
    
    std::string result2 = uPad("Hello", 3);
    EXPECT_EQ(result2, "Hello"); // Should not truncate
}

