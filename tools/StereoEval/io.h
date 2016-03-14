/*
 * pfmReader.h
 *
 *  Created on: Dec 4, 2015
 *      Author: mathieu
 */

#ifndef PFMREADER_H_
#define PFMREADER_H_

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

// taken from http://vision.middlebury.edu/stereo/ evaluation tool

void skipComment(FILE *fp)
{
    // skip comment lines in the headers of pnm files

    char c;
    while ((c=getc(fp)) == '#')
        while (getc(fp) != '\n') ;
    ungetc(c, fp);
}

void skipSpace(FILE *fp)
{
    // skip white space in the headers or pnm files

    char c;
    do {
        c = getc(fp);
    } while (c == '\n' || c == ' ' || c == '\t' || c == '\r');
    ungetc(c, fp);
}

bool readHeader(FILE *fp, const char *imtype, char c1, char c2,
                 int *width, int *height, int *nbands, int thirdArg)
{
    // read the header of a pnmfile and initialize width and height

    char c;

	if (getc(fp) != c1 || getc(fp) != c2)
	{
		printf("ReadFilePGM: wrong magic code for %s file\n", imtype);
		return false;
	}
	skipSpace(fp);
	skipComment(fp);
	skipSpace(fp);
	int r = fscanf(fp, "%d", width);
	if(r==0)
	{
		return false;
	}
	skipSpace(fp);
	r = fscanf(fp, "%d", height);
	if(r==0)
	{
		return false;
	}
	if (thirdArg) {
		skipSpace(fp);
		r = fscanf(fp, "%d", nbands);
		if(r==0)
		{
			return false;
		}
	}
    // skip SINGLE newline character after reading image height (or third arg)
	c = getc(fp);
    if (c == '\r')      // <cr> in some files before newline
        c = getc(fp);
    if (c != '\n') {
        if (c == ' ' || c == '\t' || c == '\r')
        {
            printf("newline expected in file after image height\n");
            return false;
        }
        else
        {
        	printf("whitespace expected in file after image height\n");
        	return false;
        }
  }

    return true;
}

int littleendian()
{
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

cv::Mat readPFM(const char* filename)
{
	cv::Mat disp;

    // Open the file and read the header
    FILE *fp = fopen(filename, "rb");
    if (fp == 0)
    {
        printf("ReadFilePFM: could not open %s\n", filename);
        return cv::Mat();
    }

    int width, height, nBands;
    readHeader(fp, "PFM", 'P', 'f', &width, &height, &nBands, 0);

    skipSpace(fp);

    float scalef;
    int r = fscanf(fp, "%f", &scalef);  // scale factor (if negative, little endian)
    if(r==0)
	{
    	return cv::Mat();
	}

    // skip SINGLE newline character after reading third arg
    char c = getc(fp);
    if (c == '\r')      // <cr> in some files before newline
        c = getc(fp);
    if (c != '\n') {
        if (c == ' ' || c == '\t' || c == '\r')
        {
            printf("newline expected in file after scale factor\n");
            return cv::Mat();
        }
        else
        {
            printf("whitespace expected in file after scale factor\n");
            return cv::Mat();
        }
    }

    // Set the image shape
    disp = cv::Mat(height, width, CV_32FC1);

    int littleEndianFile = (scalef < 0);
    int littleEndianMachine = littleendian();
    int needSwap = (littleEndianFile != littleEndianMachine);
    //printf("endian file = %d, endian machine = %d, need swap = %d\n",
    //       littleEndianFile, littleEndianMachine, needSwap);

    for (int y = height-1; y >= 0; y--) { // PFM stores rows top-to-bottom!!!!
	int n = width;
	float* ptr = (float *) disp.row(y).data;
	if ((int)fread(ptr, sizeof(float), n, fp) != n)
	{
	    printf("ReadFilePFM(%s): file is too short\n", filename);
	    return cv::Mat();
	}

	if (needSwap) { // if endianness doesn't agree, swap bytes
	    uchar* ptr = (uchar *) disp.row(y).data;
	    int x = 0;
	    uchar tmp = 0;
	    while (x < n) {
		tmp = ptr[0]; ptr[0] = ptr[3]; ptr[3] = tmp;
		tmp = ptr[1]; ptr[1] = ptr[2]; ptr[2] = tmp;
		ptr += 4;
		x++;
	    }
	}
    }
    if (fclose(fp))
    {
        printf("ReadFilePGM(%s): error closing file\n", filename);
        return cv::Mat();
    }

    return disp;
}


#endif /* PFMREADER_H_ */
