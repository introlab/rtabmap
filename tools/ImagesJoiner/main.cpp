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
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void showUsage()
{
	printf("Usage:\n"
			"imagesJoiner.exe \"# (see below)\" \"type\" [option]\n"
			"  # :      Name pattern is how the file names are formatted (in number).\n"
			"           Examples: 4 for pictures with 0001.jpg, 0002.jpg, ..., 0010.jpg, 0100.jpg, 1000.jpg, ...\n"
			"                     1 for pictures with 1.jpg, 2.jpg, ..., 10.jpg, 100.jpg, 1000.jpg, ...\n"
			"  type :   is the extension (jpg, bmp, png...)\n"
			"  Options:\n"
			"    -inv       option for copying odd images on the right\n"
			"    -d #       destination filename size\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 3)
	{
		showUsage();
	}

	bool inv = false;
	unsigned int sizeFileName = std::atoi(argv[1]);
	int sizeTargetFileName = sizeFileName;

	std::string type = argv[2];
	for(int i=3; i<argc; ++i)
	{
		if(strcmp(argv[i], "-inv") == 0)
		{
			inv = true;
			printf(" Inversing option activated...\n");
		}
		if(strcmp(argv[i], "-d") == 0 && i+1<argc)
		{
			sizeTargetFileName = std::atoi(argv[i+1]);
			if(sizeTargetFileName < 0)
			{
				showUsage();
			}
			printf(" Target size file name option activated=%d\n",sizeTargetFileName);
			++i;
		}
	}

	printf(" Format = %s\n", argv[1]);
	printf(" Type = %s\n", argv[2]);

	std::string targetDirectory = "imagesJoined/";
	UDirectory::makeDir((UDirectory::currentDir(true) + "imagesJoined").c_str());

	int counterJoined = 1;
	int counterImages = 1;
	bool imagesExist = true;

	std::string fileNameA;
	std::string fileNameB;

	while(imagesExist)
	{
		std::string fileNameTarget = uNumber2Str(counterJoined);
		if(inv)
		{
			fileNameA =  uNumber2Str(counterImages+1);
			fileNameB =  uNumber2Str(counterImages);
		}
		else
		{
			fileNameA =  uNumber2Str(counterImages);
			fileNameB =  uNumber2Str(counterImages+1);
		}

		while(fileNameA.size() < sizeFileName)
		{
			fileNameA.insert(0, "0");
		}

		while(fileNameB.size() < sizeFileName)
		{
			fileNameB.insert(0, "0");
		}

		while(fileNameTarget.size() < (unsigned int)sizeTargetFileName)
		{
			fileNameTarget.insert(0, "0");
		}

		(fileNameTarget.insert(0, targetDirectory) += ".") += type;
		(fileNameA += ".") += type;
		(fileNameB += ".") += type;

		IplImage * imageA = cvLoadImage(fileNameA.c_str(), CV_LOAD_IMAGE_COLOR);
		IplImage * imageB = cvLoadImage(fileNameB.c_str(), CV_LOAD_IMAGE_COLOR);

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

				if(!cvSaveImage(fileNameTarget.c_str(), targetImage))
				{
					printf("Error : saving to \"%s\" goes wrong...\n", fileNameTarget.c_str());
				}
				else
				{
					printf("Saved \"%s\" \n", fileNameTarget.c_str());
				}

				cvReleaseImage(&targetImage);
			}
			else
			{
				printf("Error : can't allocated the target image with size (%d,%d)\n", targetSize.width, targetSize.height);
				imagesExist = false;
			}
		}
		else
		{
			imagesExist = false;
		}

		if(imageA)
		{
			cvReleaseImage(&imageA);
		}
		if(imageB)
		{
			cvReleaseImage(&imageB);
		}

		counterJoined++;
		counterImages += 2;
	}




	return 0;
}
