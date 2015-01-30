/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
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

#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void showUsage()
{
	printf("Usage:\n"
			"imagesJoiner.exe [option] path\n"
			"  Options:\n"
			"    -inv       option for copying odd images on the right\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	bool inv = false;
	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "-inv") == 0)
		{
			inv = true;
			printf(" Inversing option activated...\n");
			continue;
		}
		showUsage();
		printf(" Not recognized option: \"%s\"\n", argv[i]);
	}

	std::string path = argv[argc-1];
	printf(" Path = %s\n", path.c_str());

	UDirectory dir(path, "jpg bmp png tiff jpeg");
	if(!dir.isValid())
	{
		printf("Path invalid!\n");
		exit(-1);
	}

	std::string targetDirectory = path+"_joined";
	UDirectory::makeDir(targetDirectory);
	printf(" Creating directory \"%s\"\n", targetDirectory.c_str());


	std::string fileNameA = dir.getNextFilePath();
	std::string fileNameB = dir.getNextFilePath();

	int i=1;
	while(!fileNameA.empty() && !fileNameB.empty())
	{
		if(inv)
		{
			std::string tmp = fileNameA;
			fileNameA = fileNameB;
			fileNameB = tmp;
		}

		std::string ext = UFile::getExtension(fileNameA);

		std::string targetFilePath = targetDirectory+UDirectory::separator()+uNumber2Str(i++)+"."+ext;

		IplImage * imageA = cvLoadImage(fileNameA.c_str(), CV_LOAD_IMAGE_COLOR);
		IplImage * imageB = cvLoadImage(fileNameB.c_str(), CV_LOAD_IMAGE_COLOR);

		fileNameA.clear();
		fileNameB.clear();

		if(imageA && imageB)
		{
			CvSize sizeA = cvGetSize(imageA);
			CvSize sizeB = cvGetSize(imageB);
			CvSize targetSize = {0};
			targetSize.width = sizeA.width + sizeB.width;
			targetSize.height = sizeA.height > sizeB.height ? sizeA.height : sizeB.height;
			IplImage* targetImage = cvCreateImage(targetSize, imageA->depth, imageA->nChannels);
			if(targetImage)
			{
				cvSetImageROI( targetImage, cvRect( 0, 0, sizeA.width, sizeA.height ) );
				cvCopy( imageA, targetImage );
				cvSetImageROI( targetImage, cvRect( sizeA.width, 0, sizeB.width, sizeB.height ) );
				cvCopy( imageB, targetImage );
				cvResetImageROI( targetImage );

				if(!cvSaveImage(targetFilePath.c_str(), targetImage))
				{
					printf("Error : saving to \"%s\" goes wrong...\n", targetFilePath.c_str());
				}
				else
				{
					printf("Saved \"%s\" \n", targetFilePath.c_str());
				}

				cvReleaseImage(&targetImage);

				fileNameA = dir.getNextFilePath();
				fileNameB = dir.getNextFilePath();
			}
			else
			{
				printf("Error : can't allocated the target image with size (%d,%d)\n", targetSize.width, targetSize.height);
			}
		}
		else
		{
			printf("Error: loading images failed!\n");
		}

		if(imageA)
		{
			cvReleaseImage(&imageA);
		}
		if(imageB)
		{
			cvReleaseImage(&imageB);
		}
	}
	printf("%d files processed\n", i-1);

	return 0;
}
