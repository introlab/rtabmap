#include "gtest/gtest.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UDirectory.h"
#include <fstream>
#include <cstdio>

TEST(UFileTest, Exists)
{
    // Create a temporary file
    std::string testFile = "test_file_exists.txt";
    std::ofstream file(testFile);
    file << "test content";
    file.close();
    
    EXPECT_TRUE(UFile::exists(testFile));
    EXPECT_FALSE(UFile::exists("nonexistent_file.txt"));
    
    // Cleanup
    std::remove(testFile.c_str());
}

TEST(UFileTest, Length)
{
    std::string testFile = "test_file_length.txt";
    std::string content = "test content";
    std::ofstream file(testFile);
    file << content;
    file.close();
    
    long length = UFile::length(testFile);
    EXPECT_EQ(length, static_cast<long>(content.size()));
    
    long lengthNonexistent = UFile::length("nonexistent_file.txt");
    EXPECT_EQ(lengthNonexistent, 0);
    
    // Cleanup
    std::remove(testFile.c_str());
}

TEST(UFileTest, Erase)
{
    std::string testFile = "test_file_erase.txt";
    std::ofstream file(testFile);
    file << "test content";
    file.close();
    
    EXPECT_TRUE(UFile::exists(testFile));
    int result = UFile::erase(testFile);
    EXPECT_EQ(result, 0);
    EXPECT_FALSE(UFile::exists(testFile));
}

TEST(UFileTest, Rename)
{
    std::string oldFile = "test_file_old.txt";
    std::string newFile = "test_file_new.txt";
    std::ofstream file(oldFile);
    file << "test content";
    file.close();
    
    int result = UFile::rename(oldFile, newFile);
    EXPECT_EQ(result, 0);
    EXPECT_FALSE(UFile::exists(oldFile));
    EXPECT_TRUE(UFile::exists(newFile));
    
    // Cleanup
    std::remove(newFile.c_str());
}

TEST(UFileTest, GetName)
{
    std::string path = "/path/to/file.txt";
    std::string name = UFile::getName(path);
    EXPECT_EQ(name, "file.txt");
    
    std::string path2 = "file.txt";
    std::string name2 = UFile::getName(path2);
    EXPECT_EQ(name2, "file.txt");
}

TEST(UFileTest, GetExtension)
{
    std::string path = "/path/to/file.txt";
    std::string ext = UFile::getExtension(path);
    EXPECT_EQ(ext, "txt");
    
    std::string path2 = "file";
    std::string ext2 = UFile::getExtension(path2);
    EXPECT_STREQ(ext2.c_str(), "");
}

TEST(UFileTest, Copy)
{
    std::string sourceFile = "test_file_source.txt";
    std::string destFile = "test_file_dest.txt";
    std::string content = "test content";
    
    std::ofstream file(sourceFile);
    file << content;
    file.close();
    
    UFile::copy(sourceFile, destFile);
    EXPECT_TRUE(UFile::exists(destFile));
    EXPECT_EQ(UFile::length(destFile), static_cast<long>(content.size()));
    
    // Cleanup
    std::remove(sourceFile.c_str());
    std::remove(destFile.c_str());
}

TEST(UFileTest, InstanceMethods)
{
    std::string testFile = "test_file_instance.txt";
    std::ofstream file(testFile);
    file << "test content";
    file.close();
    
    UFile ufile(testFile);
    EXPECT_TRUE(ufile.isValid());
    EXPECT_TRUE(ufile.exists());
    EXPECT_GT(ufile.length(), 0);
    EXPECT_EQ(ufile.getName(), "test_file_instance.txt");
    EXPECT_EQ(ufile.getExtension(), "txt");
    
    // Cleanup
    std::remove(testFile.c_str());
}

TEST(UFileTest, InstanceRename)
{
    std::string testFile = "test_file_rename_old.txt";
    std::ofstream file(testFile);
    file << "test content";
    file.close();
    
    UFile ufile(testFile);
    int result = ufile.rename("test_file_rename_new");
    EXPECT_EQ(result, 0);
    EXPECT_FALSE(UFile::exists(testFile));
    EXPECT_TRUE(UFile::exists("test_file_rename_new.txt"));
    
    // Cleanup
    std::remove("test_file_rename_new.txt");
}

