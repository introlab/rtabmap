/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtabmap/core/SMMemory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "utilite/UtiLite.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "utilite/UStl.h"
#include "utilite/UConversion.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <set>
#include <iostream>
#include <sstream>
#include <string>
#include "rtabmap/core/ColorTable.h"

namespace rtabmap {


SMMemory::SMMemory(const ParametersMap & parameters) :
	Memory(parameters),
	_useLogPolar(Parameters::defaultSMLogPolarUsed()),
	_colorTable(0),
	_useMotionMask(Parameters::defaultSMMotionMaskUsed()),
	_dBThreshold(Parameters::defaultSMAudioDBThreshold()),
	_dBIndexing(Parameters::defaultSMAudioDBIndexing()),
	_magnitudeInvariant(Parameters::defaultSMMagnitudeInvariant())
{
	this->parseParameters(parameters);
	if(!_colorTable)
	{
		// index 0 = 8, index 1 = 16...
		if(Parameters::defaultSMColorTable() == 8)
		{
			setColorTable(65536);
		}
		else
		{
			int i=1;
			setColorTable(i<<(Parameters::defaultSMColorTable() + 3));
		}
	}
}

SMMemory::~SMMemory()
{
	ULOGGER_DEBUG("");
	if(this->memoryChanged())
	{
		this->clear();
	}
	delete _colorTable;
}

void SMMemory::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSMLogPolarUsed())) != parameters.end())
	{
		_useLogPolar = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSMMotionMaskUsed())) != parameters.end())
	{
		_useMotionMask = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSMAudioDBThreshold())) != parameters.end())
	{
		_dBThreshold = atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSMAudioDBIndexing())) != parameters.end())
	{
		_dBIndexing = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSMMagnitudeInvariant())) != parameters.end())
	{
		_magnitudeInvariant = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSMColorTable())) != parameters.end())
	{
		// index 0 = 8, index 1 = 16...
		if(atoi((*iter).second.c_str()) == 8)
		{
			setColorTable(65536);
		}
		else
		{
			int i=1;
			setColorTable(i<<(atoi((*iter).second.c_str()) + 3));
		}
	}

	Memory::parseParameters(parameters);
}

void SMMemory::setColorTable(int size)
{
	if(_colorTable)
	{
		if(_colorTable->size() != size)
		{
			delete _colorTable;
			_colorTable = new ColorTable(size);
		}
	}
	else
	{
		_colorTable = new ColorTable(size);
	}
}

void SMMemory::copyData(const Signature * from, Signature * to)
{
	// The signatures must be SMSignature
	const SMSignature * sFrom = dynamic_cast<const SMSignature *>(from);
	SMSignature * sTo = dynamic_cast<SMSignature *>(to);
	UTimer timer;
	timer.start();
	if(sFrom && sTo)
	{
		sTo->setSensors(sFrom->getData());
	}
	else
	{
		ULOGGER_ERROR("Can't merge the signatures because there are not same type.");
	}
	ULOGGER_DEBUG("Merging time = %fs", timer.ticks());
}

Signature * SMMemory::createSignature(int id, const std::list<Sensor> & rawSensors, bool keepRawData)
{
	if(_useMotionMask)
	{
		UWARN("Using motion mask TODO");
	}
	UDEBUG("");
	UTimer timer;
	timer.start();
	UTimer timerDetails;
	timerDetails.start();
	std::list<std::vector<int> > postData;
	//const SMSignature * previousSignature = dynamic_cast<const SMSignature *>(this->getLastSignature());

	// Process all sensors
	for(std::list<Sensor>::const_iterator iter = rawSensors.begin(); iter!=rawSensors.end(); ++iter)
	{
		if(iter->type() == Sensor::kTypeImage)
		{
			UASSERT(iter->data().type() == CV_8UC3 && iter->data().channels() == 3);

			const cv::Mat & image = iter->data();
			UDEBUG("depth=%d, width=%d, height=%d, nChannels=%d, imageSize=%d,", image.type(), image.cols, image.rows, image.channels(), image.total());

			if(_useLogPolar)
			{
				// Log-polar transform
				int radius = image.rows < image.cols ? image.rows/2: image.cols/2;
				CvSize polarSize = cvSize(64, 128);
				float M = polarSize.width/std::log(radius);
				IplImage * polar = cvCreateImage( polarSize, 8, 3 );
				IplImage iplImg = image;
				cvLogPolar(&iplImg, polar, cvPoint2D32f(image.cols/2,image.rows/2), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );

				UDEBUG("polar size= %d, %d, time=%fs", polar->width, polar->height, timerDetails.ticks());

				// IND transform
				unsigned char * data = (unsigned char *)polar->imageData;
				int k=0;
				std::vector<int> sensors(polar->width*polar->height);
				for(int i=0; i<polar->height; ++i)
				{
					for(int j=0; j<polar->width; ++j)
					{
						unsigned char & b = data[i*polar->widthStep+j*3+0];
						unsigned char & g = data[i*polar->widthStep+j*3+1];
						unsigned char & r = data[i*polar->widthStep+j*3+2];
						int index = (int)_colorTable->getIndex(r, g, b);
						sensors[k] = index;
						++k;
					}
				}
				postData.push_back(sensors);
				cvReleaseImage(&polar);

				UDEBUG("indexing time = %fs", timerDetails.ticks());
			}
			else
			{
				// IND transform
				int k=0;
				std::vector<int> sensors(image.cols*image.rows);
				int sum=0;
				for(int i=0; i<image.rows; ++i)
				{
					cv::Mat row = image.row(i); // DON'T modify row! (it refers to const data)
					for(int j=0; j<row.cols; j+=3)
					{
						unsigned char b = row.at<unsigned char>(j+0);
						unsigned char g = row.at<unsigned char>(j+1);
						unsigned char r = row.at<unsigned char>(j+2);
						if(b && g && r)
						{
							sensors[k] = (int)_colorTable->getIndex(r, g, b); // index
						}
						else
						{
							sensors[k] = 0; // null, will be ignored on likelihood computation
						}
						++k;
					}
				}
				postData.push_back(sensors);

				UDEBUG("sum=%d, indexing time = %fs", sum, timerDetails.ticks());
			}
		} // end kTypeImage
		else if(iter->type() == Sensor::kTypeAudioFreqSqrdMagn)
		{
			UASSERT(iter->data().type() == CV_32FC1);

			const cv::Mat & data = iter->data();
			int k = 0;
			std::vector<int> sensors(data.cols, 0);
			unsigned int index;
			float max = uMax((float*)data.data, data.cols, index);
			int maxLimit = -1; // FIXME Must be not hard coded
			float minDB = -1000;// FIXME Must be not hard coded

			UDEBUG("data.rows=%d, data.cols=%d, data.type=%d, max=%f at %d", data.rows, data.cols, data.type(), max, index);

			if(_dBThreshold > 0)
			{
				maxLimit = max / std::pow(10.0f, _dBThreshold/10);
			}
			for(int i=0; i<data.cols; ++i)
			{
				float val = data.at<float>(0, i);
				if(_dBIndexing && max)
				{
					if(val>=0.001f)
					{
						val = 10*std::log(val/max);// transform to dB
					}
					else
					{
						val = minDB;
					}

				}
				if(!_dBIndexing && val <= maxLimit)
				{
					val = 0;
				}
				else if(_dBIndexing)
				{
					if(val <= minDB || (_dBThreshold && val <= -_dBThreshold))
					{
						val = 0;
					}
					else if(max)
					{
						if(_magnitudeInvariant)
						{
							val = -1; // ignore magnitude, just set it not null to say this frequency is here
						}
						else
						{
							val -= 1; // make sure high values are not null
						}
					}
				}
				sensors[k] = int(val);
				if((!_dBIndexing && sensors[k]<0) || (_dBIndexing && sensors[k]>0))
				{
					UERROR("sensors[%d]=%d %f", k, sensors[k], data.at<float>(0,i));
				}

				++k;
			}
			postData.push_back(sensors);
		} // end kTypeAudioFreqSqrdMagn
		else if(iter->type() == Sensor::kTypeTwist)
		{
			UASSERT(iter->data().type() == CV_32FC1);

			const cv::Mat & data = iter->data();
			std::vector<int> sensors(data.cols);
			for(int i=0; i<data.cols; ++i)
			{
				sensors[i] = (int)(data.at<float>(0, i)*100.0f);
			}
			postData.push_back(sensors);
		} //end kTypeTwist
		else
		{
			UWARN("Sensor type (%d) not handled!", iter->type());
		}
	}

	ULOGGER_DEBUG("time new signature (id=%d) %fs", id, timer.ticks());
	if(keepRawData)
	{
		return new SMSignature(postData, id, rawSensors);
	}
	else
	{
		return new SMSignature(postData, id);
	}
}

std::set<int> SMMemory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess)
{
	// get the signatures, if not in the working memory, they
	// will be loaded from the database in an more efficient way
	// than how it is done in the Memory

	ULOGGER_DEBUG("");
	UTimer timer;
	std::list<int> idsToLoad;
	std::map<int, int>::iterator wmIter;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!this->getSignature(*i) && !uContains(idsToLoad, *i))
		{
			if(!maxLoaded || idsToLoad.size() < maxLoaded)
			{
				idsToLoad.push_back(*i);
			}
		}
	}

	ULOGGER_DEBUG("idsToLoad = %d", idsToLoad.size());

	std::list<Signature *> reactivatedSigns;
	if(_dbDriver)
	{
		_dbDriver->loadSMSignatures(idsToLoad, reactivatedSigns);
	}
	timeDbAccess = timer.getElapsedTime();
	for(std::list<Signature *>::iterator i=reactivatedSigns.begin(); i!=reactivatedSigns.end(); ++i)
	{
		//append to working memory
		this->addSignatureToWm(*i);
	}
	ULOGGER_DEBUG("time = %fs", timer.ticks());
	return std::set<int>(idsToLoad.begin(), idsToLoad.end());
}

} // namespace rtabmap
