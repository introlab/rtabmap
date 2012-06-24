/*
 * Micro.cpp
 *
 *  Created on: Mar 5, 2012
 *      Author: MatLab
 */

#include "rtabmap/core/Micro.h"

#include "utilite/UAudioRecorderMic.h"
#include "utilite/UAudioRecorderFile.h"
#include <utilite/UEventsManager.h>
#include <utilite/UFile.h>
#include <utilite/UMath.h>
#include <fftw3.h>

namespace rtabmap {

Micro::Micro(MicroEvent::Type eventType,
			int deviceId,
			int fs,
			int frameLength,
			int channels,
			int bytesPerSample,
			int id) :
	_eventType(eventType),
	_recorder(0),
	_simulateFreq(false),
	_out(0),
	_id(id)
{
	UASSERT(eventType == MicroEvent::kTypeFrame || eventType == MicroEvent::kTypeFrameFreq || eventType == MicroEvent::kTypeFrameFreqSqrdMagn);
	UASSERT(deviceId >= 0);
	UASSERT(frameLength > 0 && frameLength % 2 == 0);

	_recorder = new UAudioRecorderMic(deviceId, fs, frameLength, bytesPerSample, channels);
}

Micro::Micro(MicroEvent::Type eventType,
			const std::string & path,
			bool simulateFrameRate,
			int frameLength,
			int id,
			bool playWhileRecording) :
	_eventType(eventType),
	_recorder(0),
	_simulateFreq(simulateFrameRate),
	_out(0),
	_id(id)
{
	UASSERT(eventType == MicroEvent::kTypeFrame || eventType == MicroEvent::kTypeFrameFreq || eventType == MicroEvent::kTypeFrameFreqSqrdMagn);
	UASSERT(frameLength > 0 && frameLength % 2 == 0);

	if(playWhileRecording)
	{
		simulateFrameRate = false;
	}
	_recorder = new UAudioRecorderFile(path, playWhileRecording, frameLength);
}

Micro::~Micro()
{
	UDEBUG("");
	join(true);
	if(_recorder)
	{
		delete _recorder;
	}

	if(_out)
	{
		fftwf_destroy_plan((fftwf_plan)_p);
		fftwf_free(_out);
		_out = 0;
	}
}

bool Micro::init()
{
	if(!_recorder->init())
	{
		UERROR("Recorder initialization failed!");
		return false;
	}

	// init FFTW stuff
	if(_out)
	{
		fftwf_destroy_plan((fftwf_plan)_p);
		fftwf_free(_out);
		_out = 0;
		_in.clear();
	}
	int N = _recorder->frameLength();
	_in.resize(N);
	_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
	_p = fftwf_plan_dft_r2c_1d(N, _in.data(), _out, 0);
	_window = uHamming(N);

	return true;
}

void Micro::stop()
{
	if(this->isRunning())
	{
		this->kill();
	}
	else if(_recorder && _recorder->isRunning())
	{
		_recorder->join(true);
	}
}

void Micro::startRecorder()
{
	if(_recorder)
	{
		_recorder->start();
		_timer.start();
	}
}

void Micro::mainLoopBegin()
{
	this->startRecorder();
}

void Micro::mainLoop()
{
	if(!_recorder)
	{
		UERROR("Recorder not initialized");
		this->kill();
		return;
	}

	if(this->isRunning())
	{
		bool noMoreFrames = true;
		if(_eventType == MicroEvent::kTypeFrame)
		{
			UDEBUG("");
			cv::Mat data = this->getFrame();
			if(!data.empty())
			{
				noMoreFrames = false;
				UEventsManager::post(new MicroEvent(data, 2, _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else if(_eventType == MicroEvent::kTypeFrameFreq)
		{
			UDEBUG("");
			cv::Mat freq;
			cv::Mat data = this->getFrame(freq, false);
			if(!data.empty())
			{
				noMoreFrames = false;
				UEventsManager::post(new MicroEvent(MicroEvent::kTypeFrameFreq, freq, _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else if(_eventType == MicroEvent::kTypeFrameFreqSqrdMagn)
		{
			UDEBUG("");
			cv::Mat freq;
			cv::Mat data = this->getFrame(freq, true);
			if(!data.empty())
			{
				noMoreFrames = false;
				UEventsManager::post(new MicroEvent(MicroEvent::kTypeFrameFreqSqrdMagn, freq, _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else
		{
			UFATAL("Not supposed to be here...");
		}

		if(noMoreFrames)
		{
			if(this->isRunning())
			{
				UEventsManager::post(new MicroEvent(_id));
			}
			this->kill();
		}
	}
}

void Micro::mainLoopKill()
{
	if(_recorder)
	{
		_recorder->join(true);
	}
}

cv::Mat Micro::getFrame()
{
	cv::Mat data;
	std::vector<char> frame;
	if(!_recorder)
	{
		UERROR("Micro is not initialized...");
		return data;
	}
	int frameLength = _recorder->frameLength();
	int fs = _recorder->fs();
	int channels = _recorder->channels();
	int bytesPerSample = _recorder->bytesPerSample();

	if(_simulateFreq && fs)
	{
		int sleepTime = ((double(frameLength)/double(fs) - _timer.getElapsedTime())  * 1000.0) + 0.5;
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}
		// Add precision at the cost of a small overhead
		while(_timer.getElapsedTime() < double(frameLength)/double(fs)-0.000001)
		{
			//
		}
		double slept = _timer.getElapsedTime();
		_timer.start();
		UDEBUG("slept=%fs vs target=%fs", slept, double(frameLength)/double(fs));
	}

	if(_recorder->getNextFrame(frame, true) && int(frame.size()) == frameLength * channels * bytesPerSample)
	{
		UASSERT(bytesPerSample == 1 || bytesPerSample == 2 || bytesPerSample == 4);
		if(bytesPerSample == 1)
		{
			data = cv::Mat(channels, frameLength, CV_8S);
			// Split channels in rows
			for(unsigned int i = 0; i<frame.size(); i+=channels*bytesPerSample)
			{
				for(unsigned int j=0; j<(unsigned int)channels; ++j)
				{
					data.at<char>(j, i/(channels*bytesPerSample)) = *((char*)&frame[i + j*bytesPerSample]);
				}
			}
		}
		else if(bytesPerSample == 2)
		{
			data = cv::Mat(channels, frameLength, CV_16S);
			// Split channels in rows
			for(unsigned int i = 0; i<frame.size(); i+=channels*bytesPerSample)
			{
				for(unsigned int j=0; j<(unsigned int)channels; ++j)
				{
					data.at<short>(j, i/(channels*bytesPerSample)) = *((short*)&frame[i + j*bytesPerSample]);
				}
			}
		}
		else if(bytesPerSample == 4)
		{
			data = cv::Mat(channels, frameLength, CV_32S);
			// Split channels in rows
			for(unsigned int i = 0; i<frame.size(); i+=channels*bytesPerSample)
			{
				for(unsigned int j=0; j<(unsigned int)channels; ++j)
				{
					data.at<int>(j, i/(channels*bytesPerSample)) = *((int*)&frame[i + j*bytesPerSample]);
				}
			}
		}
	}
	else
	{
		UDEBUG("No more frames...");
	}
	return data;
}

cv::Mat Micro::getFrame(cv::Mat & frameFreq, bool sqrdMagn)
{
	cv::Mat frame = this->getFrame();

	if(!frame.empty())
	{
		UASSERT(frame.depth() == CV_8S || frame.depth() == CV_16S || frame.depth() == CV_32S);
		cv::Mat timeSample(frame.rows, frame.cols, CV_32F);
		for(int i=0; i<frame.cols; ++i)
		{
			// for each channels
			for(int j=0; j<frame.rows; ++j)
			{
				if(frame.depth() == CV_8S)
				{
					timeSample.at<float>(j, i) = _window[i] * (float)(frame.at<char>(j, i)) / float(1<<7); // between 0 and 1
				}
				else if(frame.depth() == CV_16S)
				{
					timeSample.at<float>(j, i) = _window[i] * (float)(frame.at<short>(j, i)) / float(1<<15); // between 0 and 1
				}
				else if(frame.depth() == CV_32S)
				{
					timeSample.at<float>(j, i) = _window[i] * (float)(frame.at<int>(j, i)) / float(1<<31); // between 0 and 1
				}
			}
		}

		int size = timeSample.cols/2+1;
		if(sqrdMagn)
		{
			frameFreq = cv::Mat(timeSample.rows, size, CV_32F);
		}
		else
		{
			frameFreq = cv::Mat(timeSample.rows, size * 2, CV_32F); // [re, im, re, im, ...]
		}

		// for each channels
		for(int j=0; j<timeSample.rows; ++j)
		{
			cv::Mat row = timeSample.row(j);
			cv::Mat rowFreq = frameFreq.row(j);
			memcpy(_in.data(), row.data, row.cols*sizeof(float));
			fftwf_execute((fftwf_plan)_p); /* repeat as needed */

			float re;
			float im;
			for(int i=0; i<size; ++i)
			{
				re = float(_out[i][0]);
				im = float(_out[i][1]);
				if(sqrdMagn)
				{
					frameFreq.at<float>(0, i) = re*re+im*im; // squared magnitude
				}
				else
				{
					frameFreq.at<float>(0, i*2) = re;
					frameFreq.at<float>(0, i*2+1) = im;
				}
			}
		}
	}

	return frame;
}

int Micro::fs()
{
	int fs = 0;
	if(_recorder)
	{
		fs = _recorder->fs();
	}
	return fs;
}

int Micro::bytesPerSample()
{
	int bytes = 0;
	if(_recorder)
	{
		bytes = _recorder->bytesPerSample();
	}
	return bytes;
}

int Micro::channels()
{
	int channels = 0;
	if(_recorder)
	{
		channels = _recorder->channels();
	}
	return channels;
}

int Micro::nfft()
{
	int n = 0;
	if(_recorder)
	{
		n = _recorder->frameLength();
	}
	return n?n/2+1:0;
}

}
