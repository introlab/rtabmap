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

#ifndef UCV2QT_H_
#define UCV2QT_H_

#include <QtGui/QImage>
#include <QtGui/QColor>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UThread.h>
#include <stdio.h>

enum uCvQtDepthColorMap{
	uCvQtDepthWhiteToBlack,
	uCvQtDepthBlackToWhite,
	uCvQtDepthRedToBlue,
	uCvQtDepthBlueToRed
};

/**
 * Convert a cv::Mat image to a QImage. Support 
 * depth (float32, uint16) image and RGB/BGR 8bits images.
 * @param image the cv::Mat image (can be 1 channel [CV_8U, CV_16U or CV_32F] or 3 channels [CV_U8])
 * @param isBgr if 3 channels, it is BGR or RGB order.
 * @param colorMap gradient of color to use to visualize depth or monochrome images
 * @param depthMin fixed minimum range (m) of the depth gradient (if depthMax<=depthMin, max/min are computed based on data in depth image)
 * @param depthMax fixed maximum range (m) of the depth gradient (if depthMax<=depthMin, max/min are computed based on data in depth image)
 * @return the QImage
 */
inline QImage uCvMat2QImage(
		const cv::Mat & image,
		bool isBgr = true,
		uCvQtDepthColorMap colorMap = uCvQtDepthBlackToWhite,
		float depthMin = 0,
		float depthMax = 0)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U)
	{
		if(image.channels()==3)
		{
			qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
#if Q_BYTE_ORDER == Q_LITTLE_ENDIAN
			// Format_RGB32 pixels (0xffRRGGBB) are stored as B,G,R,255 bytes: convert directly in the QImage buffer.
			cv::Mat bgra(image.rows, image.cols, CV_8UC4, qtemp.bits(), qtemp.bytesPerLine());
			cv::cvtColor(image, bgra, isBgr?cv::COLOR_BGR2BGRA:cv::COLOR_RGB2BGRA);
#else
			for(int y = 0; y < image.rows; ++y)
			{
				const unsigned char * data = image.ptr<unsigned char>(y);
				QRgb * line = (QRgb*)qtemp.scanLine(y);
				for(int x = 0; x < image.cols; ++x)
				{
					if(isBgr)
					{
						line[x] = qRgb(data[x * 3+2], data[x * 3+1], data[x * 3]);
					}
					else
					{
						line[x] = qRgb(data[x * 3], data[x * 3+1], data[x * 3+2]);
					}
				}
			}
#endif
		}
		else if(image.channels() == 1)
		{
			// mono grayscale
			qtemp = QImage(image.data, image.cols, image.rows, image.step, QImage::Format_Indexed8).copy();

			QVector<QRgb> my_table;
			my_table.reserve(256);
			if(colorMap == uCvQtDepthRedToBlue || colorMap == uCvQtDepthBlueToRed)
			{
				if(colorMap == uCvQtDepthBlueToRed) {
					for(int i = 0; i < 256; i++)
						my_table.push_back(QColor::fromHsv(255-i, 255, 255, 255).rgb());
				}
				else {
					for(int i = 0; i < 256; i++)
						my_table.push_back(QColor::fromHsv(i, 255, 255, 255).rgb());
				}
			}
			else if(colorMap == uCvQtDepthBlackToWhite)
			{
				for(int i = 0; i < 256; i++)
					my_table.push_back(qRgb(i,i,i));
			}
			else // uCvQtDepthWhiteToBlack
			{
				for(int i = 0; i < 256; i++)
					my_table.push_back(qRgb(255-i,255-i,255-i));
			}
			qtemp.setColorTable(my_table);
		}
		else
		{
			printf("Wrong image format, must have 1 or 3 channels\n");
		}
	}
	else if(image.depth() == CV_32F && image.channels()==1)
    {
		// Assume depth image (float in meters)
		const float * data = (const float *)image.data;
		float min,max;
		if(depthMin != 0 && depthMax != 0 && depthMax > depthMin)
		{
			min = depthMin;
			max = depthMax;
		}
		else
		{
			// range of valid pixels only
			min = max = 0;
			bool found = false;
			for(int y = 0; y < image.rows; ++y)
			{
				const float * row = image.ptr<float>(y);
				for(int x = 0; x < image.cols; ++x)
				{
					if(uIsFinite(row[x]) && row[x] != 0)
					{
						if(!found || row[x]<min)
						{
							min = row[x];
						}
						if(!found || row[x]>max)
						{
							max = row[x];
						}
						found = true;
					}
				}
			}
			if(depthMax != 0 && depthMax > depthMin)
			{
				max = depthMax;
			}
			if(depthMin != 0 && (depthMin < depthMax || depthMin < max))
			{
				min = depthMin;
			}
		}

		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y)
		{
			data = image.ptr<float>(y);
			uchar * line = qtemp.scanLine(y);
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = line + x;
				if(!uIsFinite(data[x]) || max == min || data[x] == 0)
				{
					*p = 0;
				}
				else if(data[x] < min)
				{
					*p = 255;
				}
				else if(data[x] > max)
				{
					*p=1;
				}
				else
				{
					*p = uchar(std::max(1.0f, std::min(255.0f, 255.0f - ((data[x]-min)*255.0f)/(max-min))));
				}
				if(*p!=0 && (colorMap == uCvQtDepthBlackToWhite || colorMap == uCvQtDepthRedToBlue))
				{
					*p = 255-*p+1;
				}
			}
		}

		QVector<QRgb> my_table;
		my_table.reserve(256);
		if(colorMap == uCvQtDepthRedToBlue || colorMap == uCvQtDepthBlueToRed)
		{
			my_table.push_back(qRgb(0,0,0));
			for(int i = 1; i < 256; i++)
				my_table.push_back(QColor::fromHsv(i, 255, 255, 255).rgb());
		}
		else
		{
			for(int i = 0; i < 256; i++)
				my_table.push_back(qRgb(i,i,i));
		}
		qtemp.setColorTable(my_table);
    }
	else if(image.depth() == CV_16U && image.channels()==1)
	{
		// Assume depth image (unsigned short in mm)
		const unsigned short * data = (const unsigned short *)image.data;
		unsigned short min,max;
		if(depthMin != 0 && depthMax != 0 && depthMax > depthMin)
		{
			min = depthMin*1000;
			max = depthMax*1000;
		}
		else
		{
			// range of valid pixels only
			min = max = 0;
			bool found = false;
			for(int y = 0; y < image.rows; ++y)
			{
				const unsigned short * row = image.ptr<unsigned short>(y);
				for(int x = 0; x < image.cols; ++x)
				{
					if(uIsFinite(row[x]) && row[x] != 0)
					{
						if(!found || row[x]<min)
						{
							min = row[x];
						}
						if(!found || row[x]>max)
						{
							max = row[x];
						}
						found = true;
					}
				}
			}
			if(depthMax != 0 && depthMax > depthMin)
			{
				max = depthMax*1000;
			}
			if(depthMin != 0 && (depthMin < depthMax || depthMin*1000 < max))
			{
				min = depthMin*1000;
			}
		}

		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y)
		{
			data = image.ptr<unsigned short>(y);
			uchar * line = qtemp.scanLine(y);
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = line + x;
				if(!uIsFinite(data[x]) || max == min || data[x]==0)
				{
					*p = 0;
				}
				else if(data[x] < min)
				{
					*p = 255;
				}
				else if(data[x] > max)
				{
					*p = 1;
				}
				else
				{
					*p = uchar(std::max(1.0f, std::min(255.0f, 255.0f - (float(data[x]-min)/float(max-min))*255.0f)));
				}
				if(*p!=0 && (colorMap == uCvQtDepthBlackToWhite || colorMap == uCvQtDepthRedToBlue))
				{
					*p = 255-*p+1;
				}
			}
		}

		QVector<QRgb> my_table;
		my_table.reserve(256);
		if(colorMap == uCvQtDepthRedToBlue || colorMap == uCvQtDepthBlueToRed)
		{
			my_table.push_back(qRgb(0,0,0));
			for(int i = 1; i < 256; i++)
				my_table.push_back(QColor::fromHsv(i, 255, 255, 255).rgb());
		}
		else
		{
			for(int i = 0; i < 256; i++)
				my_table.push_back(qRgb(i,i,i));
		}
		qtemp.setColorTable(my_table);
	}
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits/3channels or (depth) 32bitsFloat/1channel, 16bits/1channel\n");
	}
	return qtemp;
}

class UCvMat2QImageThread : public UThread
{
public:
	UCvMat2QImageThread(const cv::Mat & image, bool isBgr = true) :
		image_(image),
		isBgr_(isBgr) {}
	QImage & getQImage() {return qtImage_;}
protected:
	virtual void mainLoop()
	{
		qtImage_ = uCvMat2QImage(image_, isBgr_);
		this->kill();
	}
private:
	cv::Mat image_;
	bool isBgr_;
	QImage qtImage_;
};

#endif /* UCV2QT_H_ */
