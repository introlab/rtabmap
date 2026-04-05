#include "gtest/gtest.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include <cstdio>
#include <fstream>

TEST(UDirectoryTest, Exists)
{
    // Test with current directory (should exist)
    EXPECT_TRUE(UDirectory::exists("."));
    
    // Test with nonexistent directory
    EXPECT_FALSE(UDirectory::exists("nonexistent_directory_xyz123"));
}

TEST(UDirectoryTest, GetDir)
{
    std::string filePath = "/path/to/file.txt";
    std::string dir = UDirectory::getDir(filePath);
    EXPECT_EQ(dir, "/path/to");

    std::string filePath2 = "/path";
    std::string dir2 = UDirectory::getDir(filePath2);
    EXPECT_EQ(dir2, "/");
    
    std::string filePath3 = "file.txt";
    std::string dir3 = UDirectory::getDir(filePath3);
    EXPECT_EQ(dir3, ".");

    std::string filePath4 = "./file.txt";
    std::string dir4 = UDirectory::getDir(filePath4);
    EXPECT_EQ(dir4, ".");
}

TEST(UDirectoryTest, CurrentDir)
{
    std::string current = UDirectory::currentDir(false);
    EXPECT_GT(current.size(), 0);
    
    std::string currentWithSep = UDirectory::currentDir(true);
    EXPECT_GT(currentWithSep.size(), 0);
    // Should end with separator if trailingSeparator is true
    std::string sep = UDirectory::separator();
    if(currentWithSep.size() > 0)
    {
        EXPECT_EQ(currentWithSep.back(), sep[0]);
    }
}

TEST(UDirectoryTest, MakeDir)
{
    std::string testDir = "test_make_dir";
    // Remove if exists
    if(UDirectory::exists(testDir))
    {
        UDirectory::removeDir(testDir);
    }
    
    bool result = UDirectory::makeDir(testDir);
    EXPECT_TRUE(result);
    EXPECT_TRUE(UDirectory::exists(testDir));
    
    // Cleanup
    UDirectory::removeDir(testDir);
}

TEST(UDirectoryTest, RemoveDir)
{
    std::string testDir = "test_remove_dir";
    // Create if doesn't exist
    if(!UDirectory::exists(testDir))
    {
        UDirectory::makeDir(testDir);
    }
    
    bool result = UDirectory::removeDir(testDir);
    EXPECT_TRUE(result);
    EXPECT_FALSE(UDirectory::exists(testDir));
}

TEST(UDirectoryTest, HomeDir)
{
    std::string home = UDirectory::homeDir();
    EXPECT_GT(home.size(), 0);
}

TEST(UDirectoryTest, Separator)
{
    std::string sep = UDirectory::separator();
    EXPECT_EQ(sep.size(), 1);
#ifdef _WIN32
    EXPECT_EQ(sep, "\\");
#else
    EXPECT_EQ(sep, "/");
#endif
}

TEST(UDirectoryTest, Constructor)
{
    std::string testDir = ".";
    UDirectory dir(testDir);
    EXPECT_TRUE(dir.isValid());
}

TEST(UDirectoryTest, SetPath)
{
    UDirectory dir;
    dir.setPath(".");
    EXPECT_TRUE(dir.isValid());
}

TEST(UDirectoryTest, GetNextFileName)
{
    // Create a test directory with files
    std::string testDir = "test_dir_files";
    if(!UDirectory::exists(testDir))
    {
        UDirectory::makeDir(testDir);
    }
    
    // Create some test files
    std::ofstream file1(testDir + UDirectory::separator() + "file1.txt");
    file1 << "content1";
    file1.close();
    
    std::ofstream file2(testDir + UDirectory::separator() + "file2.txt");
    file2 << "content2";
    file2.close();
    
    UDirectory dir(testDir);
    std::string name1 = dir.getNextFileName();
    std::string name2 = dir.getNextFileName();
    
    EXPECT_STREQ(name1.c_str(), "file1.txt");
    EXPECT_STREQ(name2.c_str(), "file2.txt");
    
    // Cleanup
    UFile::erase(testDir + UDirectory::separator() + "file1.txt");
    UFile::erase(testDir + UDirectory::separator() + "file2.txt");
    UDirectory::removeDir(testDir);
}

TEST(UDirectoryTest, GetNextFilePath)
{
    std::string testDir = "test_dir_paths";
    if(!UDirectory::exists(testDir))
    {
        UDirectory::makeDir(testDir);
    }
    
    std::ofstream file(testDir + UDirectory::separator() + "test.txt");
    file << "content";
    file.close();
    
    UDirectory dir(testDir);
    std::string path = dir.getNextFilePath();
    EXPECT_GT(path.size(), 0);
    EXPECT_NE(path.find("test.txt"), std::string::npos);
    
    // Cleanup
    UFile::erase(testDir + UDirectory::separator() + "test.txt");
    UDirectory::removeDir(testDir);
}

TEST(UDirectoryTest, GetFileNames)
{
    std::string testDir = "test_dir_list";
    if(!UDirectory::exists(testDir))
    {
        UDirectory::makeDir(testDir);
    }
    
    std::ofstream file1(testDir + UDirectory::separator() + "file1.txt");
    file1 << "content1";
    file1.close();
    
    std::ofstream file2(testDir + UDirectory::separator() + "file2.txt");
    file2 << "content2";
    file2.close();
    
    UDirectory dir(testDir);
    const std::list<std::string>& fileNames = dir.getFileNames();
    EXPECT_GE(fileNames.size(), 2);
    EXPECT_STREQ(fileNames.front().c_str(), "file1.txt");
    EXPECT_STREQ(fileNames.back().c_str(), "file2.txt");
    
    // Cleanup
    UFile::erase(testDir + UDirectory::separator() + "file1.txt");
    UFile::erase(testDir + UDirectory::separator() + "file2.txt");
    UDirectory::removeDir(testDir);
}

TEST(UDirectoryTest, Rewind)
{
    std::string testDir = "test_dir_rewind";
    if(UDirectory::exists(testDir))
    {
        UDirectory::removeDir(testDir);
    }
    UDirectory::makeDir(testDir);
    std::ofstream file1(testDir + UDirectory::separator() + "file1.txt");
    file1 << "content1";
    file1.close();
    UDirectory dir(testDir);
    std::string name1 = dir.getNextFileName();
    EXPECT_FALSE(name1.empty());
    dir.rewind();
    std::string name2 = dir.getNextFileName();
    // Should get the same file name after rewind
    EXPECT_EQ(name1, name2);
}

TEST(UDirectoryTest, ExtensionsFilter)
{
    std::string testDir = "test_dir_ext";
    if(!UDirectory::exists(testDir))
    {
        UDirectory::makeDir(testDir);
    }
    
    std::ofstream file1(testDir + UDirectory::separator() + "file1.txt");
    file1 << "content1";
    file1.close();
    
    std::ofstream file2(testDir + UDirectory::separator() + "file2.jpg");
    file2 << "content2";
    file2.close();
    
    UDirectory dir(testDir, "txt");
    const std::list<std::string>& fileNames = dir.getFileNames();
    // Should only get .txt files
    for(const std::string& name : fileNames)
    {
        EXPECT_NE(name.find(".txt"), std::string::npos);
    }
    
    // Cleanup
    UFile::erase(testDir + UDirectory::separator() + "file1.txt");
    UFile::erase(testDir + UDirectory::separator() + "file2.jpg");
    UDirectory::removeDir(testDir);
}

