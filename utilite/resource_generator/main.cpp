/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
