/*
 * Micro.h
 *
 *  Created on: Mar 5, 2012
 *      Author: MatLab
 */

#ifndef MICRO_H_
#define MICRO_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <utilite/UThreadNode.h>
#include <utilite/UTimer.h>
#include <utilite/UEvent.h>
#include <utilite/ULogger.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class UAudioRecorder;

namespace rtabmap {

class MicroEvent :
	public UEvent
{
public:
	enum Type {
		kTypeFrame,
		kTypeFrameFreq,
		kTypeFrameFreqSqrdMagn,
		kTypeNoMoreFrames
	};

public:
	// kTypeNoMoreFrames constructor
	MicroEvent(int microId = 0) :
		UEvent(kTypeNoMoreFrames),
		_sampleSize(0),
		_microId(microId)
	{
	}
	// kTypeFrame constructor
	MicroEvent(const cv::Mat & frame,
			int sampleSize,
			int fs,
			int channels,
			int microId = 0) :
		UEvent(kTypeFrame),
		_frame(frame),
		_sampleSize(sampleSize),
		_microId(microId)
	{
	}
	// kTypeFrameFreq and kTypeFrameFreqSqrdMagn constructors
	MicroEvent(Type frameType,
			const cv::Mat & frameFreq,
			int fs,
			int channels,
			int microId = 0) :
		UEvent(frameType),
		_frame(frameFreq),
		_sampleSize(sizeof(float)),
		_microId(microId)
	{
		UASSERT(frameType == kTypeFrameFreqSqrdMagn || frameType == kTypeFrameFreq);
	}

	int type() const {return this->getCode();}
	const cv::Mat & frame() const {return _frame;}
	int sampleSize() const {return _sampleSize;}
	int microId() const {return _microId;}
	virtual ~MicroEvent() {}
	virtual std::string getClassName() const {return std::string("MicroEvent");}

private:
	cv::Mat _frame;
	int _sampleSize; // bytes
	int _fs; //sampling rate
	int _microId;
};

class RTABMAP_EXP Micro : public UThreadNode
{
	typedef float fftwf_complex[2];
public:
	Micro(MicroEvent::Type eventType,
			int deviceId,
			int fs,
			int frameLength,
			int channels,
			int bytesPerSample,
			int id = 0);
	Micro(MicroEvent::Type eventType,
			const std::string & path,
			bool simulateFrameRate,
			int frameLength,
			int id = 0,
			bool playWhileRecording = false);
	virtual ~Micro();

	bool init();
	void stop(); // same as kill() but handle the case where underlying recorder is running and not the micro.
	void startRecorder(); // must only be used if Micro::start() is not used

	cv::Mat getFrame();
	cv::Mat getFrame(cv::Mat & frameFreq, bool sqrdMagn = false);

	int fs();
	int bytesPerSample();
	int channels();
	int nfft();

protected:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	virtual void mainLoopKill();

private:
	MicroEvent::Type _eventType;
	UAudioRecorder* _recorder;
	bool _simulateFreq;
	UTimer _timer;
	std::vector<float> _window;
	std::vector<float> _in;
	fftwf_complex * _out; // fftwf_complex
	void * _p; // fftwf_plan
	int _id;
};

}

#endif /* MICRO_H_ */
