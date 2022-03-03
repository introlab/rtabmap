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
 * @param colorMap gradient of color to use to visualize depth
 * @param depthMin fixed minimum range (m) of the depth gradient (if depthMax<=depthMin, max/min are computed based on data in depth image)
 * @param depthMax fixed maximum range (m) of the depth gradient (if depthMax<=depthMin, max/min are computed based on data in depth image)
 * @return the QImage
 */
inline QImage uCvMat2QImage(
		const cv::Mat & image,
		bool isBgr = true,
		uCvQtDepthColorMap colorMap = uCvQtDepthWhiteToBlack,
		float depthMin = 0,
		float depthMax = 0)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U)
	{
		if(image.channels()==3)
		{
			const unsigned char * data = image.data;
			if(image.channels() == 3)
			{
				qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
				for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
				{
					for(int x = 0; x < image.cols; ++x)
					{
						QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
						if(isBgr)
						{
							*p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
						}
						else
						{
							*p = qRgb(data[x * image.channels()], data[x * image.channels()+1], data[x * image.channels()+2]);
						}
					}
				}
			}
		}
		else if(image.channels() == 1)
		{
			// mono grayscale
			qtemp = QImage(image.data, image.cols, image.rows, image.cols, QImage::Format_Indexed8).copy();
			QVector<QRgb> my_table;
			for(int i = 0; i < 256; i++)
				my_table.push_back(qRgb(i,i,i));
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
		float min=depthMax>depthMin?depthMin:data[0], max=depthMax>depthMin?depthMax:data[0];
		if(depthMax <= depthMin)
		{
			for(unsigned int i=1; i<image.total(); ++i)
			{
				if(uIsFinite(data[i]) && data[i] > 0)
				{
					if(!uIsFinite(min) || (data[i] > 0 && data[i]<min))
					{
						min = data[i];
					}
					if(!uIsFinite(max) || (data[i] > 0 && data[i]>max))
					{
						max = data[i];
					}
				}
			}
		}

		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y, data += image.cols)
		{
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = qtemp.scanLine (y) + x;
				if(data[x] < min || data[x] > max || !uIsFinite(data[x]) || max == min)
				{
					*p = 0;
				}
				else
				{
					*p = uchar(std::max(0.0f, std::min(255.0f, 255.0f - ((data[x]-min)*255.0f)/(max-min))));
					if(*p == 255)
					{
						*p = 0;
					}
				}
				if(*p!=0 && (colorMap == uCvQtDepthBlackToWhite || colorMap == uCvQtDepthRedToBlue))
				{
					*p = 255-*p;
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
		unsigned short min=depthMax>depthMin?(unsigned short)(depthMin*1000):data[0], max=depthMax>depthMin?(unsigned short)(depthMax*1000):data[0];
		if(depthMax<=depthMin)
		{
			for(unsigned int i=1; i<image.total(); ++i)
			{
				if(uIsFinite(data[i]) && data[i] > 0)
				{
					if(!uIsFinite(min) || (data[i] > 0 && data[i]<min))
					{
						min = data[i];
					}
					if(!uIsFinite(max) || (data[i] > 0 && data[i]>max))
					{
						max = data[i];
					}
				}
			}
		}

		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y, data += image.cols)
		{
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = qtemp.scanLine (y) + x;
				if(data[x] < min || data[x] > max || !uIsFinite(data[x]) || max == min)
				{
					*p = 0;
				}
				else
				{
					*p = uchar(std::max(0.0f, std::min(255.0f, 255.0f - (float(data[x]-min)/float(max-min))*255.0f)));
					if(*p == 255)
					{
						*p = 0;
					}
				}
				if(*p!=0 && (colorMap == uCvQtDepthBlackToWhite || colorMap == uCvQtDepthRedToBlue))
				{
					*p = 255-*p;
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
