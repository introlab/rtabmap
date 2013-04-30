/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rtabmap/utilite/UtiLite.h"

#include <fstream>
#include <iostream>
#include <string.h>

void showUsage()
{
	printf("Usage:\n"
			"uresourcegenerator.exe [option] \"file1\" \"file2\" ... \n"
			"  Create a file named \"file\".h with string\n"
			"  variable named \"file\" which contains the data of the file.\n"
			"  Warning, it overwrites the target file\n"
			"  Options:\n"
			"     -n \"namespace\"      namespace used\n"
			"     -p \"targetPath\"     target path where the file is created\n"
			"     -v                    version of the UtiLite library\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}
	else if(argc == 2 && strcmp(argv[1], "-v") == 0)
	{
		printf("%s\n", UTILITE_VERSION);
		exit(0);
	}

	std::string targetDir = UDirectory::currentDir(); // By default, use the current directory
	std::string nspace; // namespace

	int k;
	for(k=1; k<(argc-1); ++k)
	{
		if(strcmp(argv[k], "-n") == 0)
		{
			if(!(k+1<(argc-1)))
			{
				showUsage();
			}
			nspace = argv[k+1];
			printf(" Using namespace=%s\n", nspace.c_str());
			++k;
		}
		else if(strcmp(argv[k], "-p") == 0)
		{
			if(!(k+1<(argc-1)))
			{
				showUsage();
			}
			targetDir = argv[k+1];
			printf(" Using target directory=%s\n", targetDir.c_str());
			++k;
		}
		else
		{
			break;
		}
	}

	while(k < argc)
	{
		std::string filePath = argv[k];
		std::string varName = UFile::getName(argv[k]);
		// replace '_'
		for(unsigned int i=0; i<varName.size(); ++i)
		{
			if(!((varName[i] >= '0' && varName[i] <= '9') ||
				(varName[i] >= 'A' && varName[i] <= 'Z') ||
				(varName[i] >= 'a' && varName[i] <= 'z')))
			{
				varName[i] = '_';
			}
		}
		std::string targetFileName = varName + ".h";
		// upper case
		for(unsigned int i=0; i<varName.size(); ++i)
		{
			if(varName[i] >= 'a' && varName[i] <= 'z')
			{
				varName[i] -= 32; // upper case
			}
		}

		std::fstream outFile;
		std::fstream inFile;
		outFile.open(((targetDir + "/") + targetFileName).c_str(), std::fstream::out);
		inFile.open(filePath.c_str(), std::fstream::in | std::fstream::binary);

		printf("Input file \"%s\" size = %ld bytes\n", filePath.c_str(), UFile::length(filePath));
		if(outFile.is_open() && inFile.is_open())
		{
			outFile << "/*This is a generated file...*/\n\n";
			outFile << "#ifndef " << varName << "_H\n";
			outFile << "#define " << varName << "_H\n\n";

			if(!nspace.empty())
			{
				outFile << "namespace " << nspace.c_str() << "\n{\n\n";
			}

			outFile << "static const char * " << varName.c_str() << " = ";

			if(!inFile.good())
			{
				outFile << "\"\""; //empty string
			}
			else
			{
				std::string startLine = "\n   \"";
				std::string endLine = "\"";
				std::vector<char> buffer(1024);
				while(inFile.good())
				{
					inFile.read(buffer.data(), 1024);
					std::streamsize count = inFile.gcount();
					if(count)
					{
						outFile.write(startLine.c_str(), startLine.size());

						std::string hex = uBytes2Hex(buffer.data(), count);
						outFile.write(hex.c_str(), hex.size());

						outFile.write(endLine.c_str(), endLine.size());
					}
				}
			}

			std::string endOfVar = ";\n\n";
			outFile.write(endOfVar.c_str(), endOfVar.size());

			if(!nspace.empty())
			{
				outFile << "}\n\n";
			}

			outFile << "#endif //" << varName << "_H\n\n";
		}

		outFile.close();
		inFile.close();

		printf("Output file \"%s\" size = %ld bytes\n", ((targetDir + "/") + targetFileName).c_str(), UFile::length(((targetDir + "/") + targetFileName).c_str()));
		++k;
	}

}
