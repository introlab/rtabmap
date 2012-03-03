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

#include "SMMemory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "utilite/UtiLite.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SMState.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "utilite/UStl.h"
#include "utilite/UConversion.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <set>
#include <iostream>
#include <sstream>
#include <string>
#include "ColorTable.h"

namespace rtabmap {


SMMemory::SMMemory(const ParametersMap & parameters) :
	Memory(parameters),
	_useLogPolar(Parameters::defaultSMLogPolarUsed()),
	_useVotingScheme(Parameters::defaultSMVotingSchemeUsed()),
	_colorTable(0),
	_useMotionMask(Parameters::defaultSMMotionMaskUsed())
{
	this->parseParameters(parameters);
	if(!_colorTable)
	{
		int i=1;
		this->setColorTable(i<<(Parameters::defaultSMColorTable() + 3));
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
	if((iter=parameters.find(Parameters::kSMVotingSchemeUsed())) != parameters.end())
	{
		this->setVotingScheme(uStr2Bool((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kSMMotionMaskUsed())) != parameters.end())
	{
		_useMotionMask = uStr2Bool((*iter).second.c_str());
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

void SMMemory::setVotingScheme(bool useVotingScheme)
{
	_useVotingScheme = useVotingScheme;
	_dictionary.clear();
	if(_useVotingScheme)
	{
		const std::map<int, Signature *> & signatures = this->getSignatures();
		for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			this->updateDictionary(i->second);
		}
	}
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
		sTo->setSensors(sFrom->getSensors());
		sTo->setMotionMask(sFrom->getMotionMask());
	}
	else
	{
		ULOGGER_ERROR("Can't merge the signatures because there are not same type.");
	}
	ULOGGER_DEBUG("Merging time = %fs", timer.ticks());
}

Signature * SMMemory::createSignature(int id, const SMState * smState, bool keepRawData)
{
	UDEBUG("");
	UTimer timer;
	timer.start();
	UTimer timerDetails;
	timerDetails.start();
	std::vector<int> sensors;
	const std::vector<int> * sensorsPrevious = 0;
	std::vector<unsigned char> motionMask;
	const IplImage * image = 0;
	IplImage * polar = 0;
	IplImage * indexed = 0;
	const SMSignature * previousSignature = dynamic_cast<const SMSignature *>(this->getLastSignature());
	if(previousSignature)
	{
		UDEBUG("");
		sensorsPrevious = &previousSignature->getSensors();
	}
	if(smState)
	{
		image = smState->getImage();

		// sensors
		if(!smState->getSensors().empty() == 0 && image && image->imageSize)
		{
			if(image->depth != IPL_DEPTH_8U && image->nChannels != 3)
			{
				UFATAL("Only IplImage depth of IPL_DEPTH_8U and 3 channels (BGR) is supported.");
			}

			UDEBUG("depth=%d, alpha=%d, widthStep=%d, width=%d, height=%d, nChannels=%d, imageSize=%d,", image->depth, image->alphaChannel, image->widthStep, image->width, image->height, image->nChannels, image->imageSize);

			if(_useLogPolar)
			{
				// Log-polar transform
				int radius = image->height < image->width ? image->height/2: image->width/2;
				CvSize polarSize = cvSize(64, 128);
				float M = polarSize.width/std::log(radius);
				polar = cvCreateImage( polarSize, 8, 3 );
				cvLogPolar( image, polar, cvPoint2D32f(image->width/2,image->height/2), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );

				UDEBUG("polar size= %d, %d, time=%fs", polar->width, polar->height, timerDetails.ticks());

				// IND transform
				unsigned char * data = (unsigned char *)polar->imageData;
				sensors = std::vector<int>(polar->width*polar->height);
				if(_useVotingScheme && (_dictionary.empty() || _dictionary.size() != sensors.size()))
				{
					_dictionary = std::vector<std::map<int, std::set<int> > >(sensors.size());
				}
				if(_useMotionMask)
				{
					motionMask = std::vector<unsigned char>(sensors.size(), 0);
				}
				bool updateMask = sensorsPrevious && sensorsPrevious->size() == motionMask.size();
				int k=0;
				for(int i=0; i<polar->height; ++i)
				{
					for(int j=0; j<polar->width; ++j)
					{
						unsigned char & b = data[i*polar->widthStep+j*3+0];
						unsigned char & g = data[i*polar->widthStep+j*3+1];
						unsigned char & r = data[i*polar->widthStep+j*3+2];
						int index = (int)_colorTable->getIndex(r, g, b);
						sensors[k] = index;
						_colorTable->getRgb(index, r, g , b);

						if(_useMotionMask && updateMask && sensorsPrevious->at(k) != sensors[k])
						{
							motionMask[k] = 1;
						}

						if(!_dictionary.empty())
						{
							std::set<int> sensorId;
							sensorId.insert(id);
							std::pair<std::map<int, std::set<int> >::iterator, bool> ret;
							ret = _dictionary[k].insert(std::make_pair(sensors[k], sensorId));
							if(ret.second == false)
							{
								ret.first->second.insert(id);
							}
						}

						++k;
					}
				}

				UDEBUG("indexing time = %fs", timerDetails.ticks());

				//cv::Mat indPolar;
				//fromIndPolar = cvCreateImage(cvGetSize(image), 8, 3);
				//cvLogPolar(polar, fromIndPolar, cvPoint2D32f(image->width/2,image->height/2), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );
				//UDEBUG("back from polar time = %fs", timerDetails());

				//image = polar;
			}
			else
			{
				// IND transform
				indexed = cvCloneImage(image);
				unsigned char * data = (unsigned char *)indexed->imageData;
				sensors = std::vector<int>(indexed->width*indexed->height);
				if(_useVotingScheme && (_dictionary.empty() || _dictionary.size() != sensors.size()))
				{
					_dictionary = std::vector<std::map<int, std::set<int> > >(sensors.size());
				}
				if(_useMotionMask)
				{
					motionMask = std::vector<unsigned char>(sensors.size(), 0);
				}
				bool updateMask = sensorsPrevious && sensorsPrevious->size() == motionMask.size();
				int k=0;
				int sum=0;
				for(int i=0; i<indexed->height; ++i)
				{
					for(int j=0; j<indexed->width; ++j)
					{
						unsigned char & b = data[i*indexed->widthStep+j*3+0];
						unsigned char & g = data[i*indexed->widthStep+j*3+1];
						unsigned char & r = data[i*indexed->widthStep+j*3+2];
						int index = (int)_colorTable->getIndex(r, g, b);
						sensors[k] = index;
						_colorTable->getRgb(index, r, g , b);

						if(_useMotionMask && updateMask && sensorsPrevious->at(k) != sensors[k])
						{
							motionMask[k] = 1;
							++sum;
						}

						if(!_dictionary.empty())
						{
							std::set<int> sensorId;
							sensorId.insert(id);
							std::pair<std::map<int, std::set<int> >::iterator, bool> ret;
							ret = _dictionary[k].insert(std::make_pair(sensors[k], sensorId));
							if(ret.second == false)
							{
								ret.first->second.insert(id);
							}
						}

						++k;
					}
				}
				image = indexed;

				UDEBUG("sum=%d, indexing time = %fs", sum, timerDetails.ticks());
			}
		}
		else
		{
			std::vector<float> sensorsMerged;
			int buf;
			smState->getSensorsMerged(sensorsMerged, buf);
			sensors = std::vector<int>(sensorsMerged.size());
			if(_useVotingScheme && (_dictionary.empty() || _dictionary.size() != sensorsMerged.size()))
			{
				_dictionary = std::vector<std::map<int, std::set<int> > >(sensorsMerged.size());
			}
			if(_useMotionMask)
			{
				motionMask = std::vector<unsigned char>(sensors.size(), 0);
			}
			bool updateMask = sensorsPrevious && sensorsPrevious->size() == motionMask.size();
			for(unsigned int i=0; i<sensorsMerged.size(); ++i)
			{
				if(sensorsMerged[i]>0 && sensorsMerged[i]<1)
				{
					UWARN("Conversion from float to int may lost precision...");
				}
				sensors[i] = (int)sensorsMerged[i];
				if(_useMotionMask && updateMask && sensorsPrevious->at(i) != sensors[i])
				{
					motionMask[i] = 1;
				}

				if(!_dictionary.empty())
				{
					std::set<int> sensorId;
					sensorId.insert(id);
					std::pair<std::map<int, std::set<int> >::iterator, bool> ret;
					ret = _dictionary[i].insert(std::make_pair(sensors[i], sensorId));
					if(ret.second == false)
					{
						ret.first->second.insert(id);
					}
				}
			}
		}
	}
	SMSignature * s = new SMSignature(sensors, motionMask, id, image, keepRawData);

	if(polar)
	{
		cvReleaseImage(&polar);
	}
	if(indexed)
	{
		cvReleaseImage(&indexed);
	}

	ULOGGER_DEBUG("time new signature (id=%d) %fs", id, timer.ticks());
	return s;
}

std::map<int, float> SMMemory::computeLikelihood(const Signature * signature, const std::list<int> & ids, float & maximumScore)
{
	if(!_useVotingScheme)
	{
		return Memory::computeLikelihood(signature, ids, maximumScore);
	}
	else
	{
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;
		maximumScore = 0;

		const SMSignature * query = dynamic_cast<const SMSignature *>(signature);
		if(!query)
		{
			ULOGGER_ERROR("The signature is not a SMSignature");
			return likelihood; // Must be a SMSignature *
		}
		else if(ids.empty())
		{
			UWARN("ids list is empty");
			return likelihood;
		}

		UDEBUG("Likelihood for %d", query->id());

		const std::vector<int> & sensors = query->getSensors();
		if(_dictionary.size() != sensors.size())
		{
			UERROR("Dictionary (%d) and sensor (%d) are not the same size!", (int)_dictionary.size(), (int)sensors.size());
			return likelihood;
		}
		const std::vector<unsigned char> & mask = query->getMotionMask();
		bool maskUsed = false;
		if(mask.size() != 0 && mask.size() != sensors.size())
		{
			UWARN("mask's size (%d) and sensor's size (%d) are not equal", (int)mask.size(), (int)sensors.size());
		}
		else if(mask.size())
		{
			maskUsed = true;
		}

		// prepare likelihood
		for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			likelihood.insert(likelihood.end(), std::make_pair(*iter, 0.0f));
		}

		//float nwi; // nwi is the number of a specific word referenced by a place
		//float ni; // ni is the total of words referenced by a place
		float nw; // nw is the number of places referenced by a specific word
		float N; // N is the total number of places
		float logNnw;

		N = this->getSignatures().size();

		if(N)
		{
			for(unsigned int i=0; i<sensors.size(); ++i)
			{
				if(!maskUsed || mask[i])
				{
					// "Inverted index"
					std::map<int, std::set<int> >::iterator iter = _dictionary[i].find(sensors[i]);
					if(iter == _dictionary[i].end())
					{
						UERROR("Sensor %d not found in dictionary ?!?", sensors[i]);
					}
					else
					{
						nw = iter->second.size();
						if(nw)
						{
							if(nw > N)
							{
								for(std::set<int>::iterator jter = iter->second.begin(); jter!=iter->second.end(); ++jter)
								{
									UERROR("sensor pos %d, refid = %d", (int)i, *jter);
								}

								UFATAL("id=%d, N = %f, nw=%f", signature->id(), N, nw);
							}
							logNnw = log10(N/nw);
							if(logNnw)
							{
								for(std::set<int>::iterator jter = iter->second.begin(); jter!=iter->second.end(); ++jter)
								{
									std::map<int, float>::iterator kter = likelihood.find(*jter);
									if(kter != likelihood.end())
									{
										kter->second += logNnw;
									}
								}
							}
						}
					}
				}
			}
		}

		if(sensors.size())
		{
			maximumScore = log(N) * float(sensors.size());
		}

		ULOGGER_DEBUG("compute likelihood, maximumScore=%f... %f s", maximumScore, timer.ticks());
		return likelihood;
	}
}

void SMMemory::moveToTrash(Signature * s)
{
	if(_useVotingScheme)
	{
		UTimer timer;
		SMSignature * sm = dynamic_cast<SMSignature *>(s);
		if(sm && sm->id() > 0)
		{
			const std::vector<int> & sensors = sm->getSensors();
			if(sensors.size() == _dictionary.size())
			{
				for(unsigned int i=0; i<sensors.size(); ++i)
				{
					std::map<int, std::set<int> >::iterator iter = _dictionary[i].find(sensors[i]);
					if(iter != _dictionary[i].end())
					{
						if(!iter->second.erase(sm->id()))
						{
							UWARN("Sensor id %d not found in dictionary at pos %d", sm->id(), (int)i);
						}
					}
					else
					{
						UWARN("Sensor value %d at sensor pos %d is not found in dictionary", sensors[i], (int)i);
					}
				}
			}
			else
			{
				UWARN("Dictionary size (%d) is not the same as the sensor (%d), signId=%d", (int)_dictionary.size(), (int)sensors.size(), sm->id());
			}
		}
		UDEBUG("time=%fs", timer.ticks());
	}
	Memory::moveToTrash(s);
}

Signature * SMMemory::getSignatureLtMem(int id)
{
	Signature * s = Memory::getSignatureLtMem(id);
	if(_useVotingScheme && s)
	{
		this->updateDictionary(s);
	}
	return s;
}

bool SMMemory::init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters)
{
	UDEBUG("");
	bool success = Memory::init(dbDriverName, dbUrl, dbOverwritten, parameters);

	if(_useVotingScheme)
	{
		// Update sensory dictionary
		const std::map<int, Signature *> & signatures = this->getSignatures();
		for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
		{
			this->updateDictionary(i->second);
		}
	}

	return success;
}

void SMMemory::updateDictionary(const Signature * s)
{
	if(s)
	{
		const SMSignature * sm = dynamic_cast<const SMSignature *>(s);
		if(sm)
		{
			const std::vector<int> & sensors = sm->getSensors();
			if(_dictionary.empty())
			{
				_dictionary = std::vector<std::map<int, std::set<int> > >(sensors.size());
			}
			if(sensors.size() == _dictionary.size())
			{
				for(unsigned int i=0; i<sensors.size(); ++i)
				{
					std::set<int> sensorId;
					sensorId.insert(sm->id());
					std::pair<std::map<int, std::set<int> >::iterator, bool> ret;
					ret = _dictionary[i].insert(std::make_pair(sensors[i], sensorId));
					if(ret.second == false)
					{
						ret.first->second.insert(sm->id());
					}
				}
			}
			else if(_dictionary.size())
			{
				UWARN("Loaded signature %d with size (%d) doesn't have the same size as the dicitonary (%d)", sm->id(), (int)sensors.size(), (int)_dictionary.size());
			}
		}
	}
	else
	{
		UFATAL("Signature must not be null!");
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
