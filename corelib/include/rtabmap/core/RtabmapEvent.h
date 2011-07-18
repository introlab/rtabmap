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

#ifndef RTABMAPEVENT_H_
#define RTABMAPEVENT_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "utilite/UEvent.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utilite/ULogger.h"
#include <list>
#include <vector>

namespace rtabmap
{

#define RTABMAP_STATS(PREFIX, NAME, UNIT) \
	public: \
		static std::string k##PREFIX##NAME() {return #PREFIX "/" #NAME "/" #UNIT;} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {if(!_defaultDataInitialized)_defaultData.insert(std::pair<std::string, float>(#PREFIX "/" #NAME "/" #UNIT, 0.0f));} \
		}; \
		Dummy##PREFIX##NAME dummy##PREFIX##NAME;

class RTABMAP_EXP Statistics
{
	RTABMAP_STATS(Loop, Closure_id,);
	RTABMAP_STATS(Loop, RejectedHypothesis,);
	RTABMAP_STATS(Loop, Highest_hypothesis_id,);
	RTABMAP_STATS(Loop, Highest_hypothesis_value,);
	RTABMAP_STATS(Loop, Vp_likelihood,);
	RTABMAP_STATS(Loop, ReactivateId,);
	RTABMAP_STATS(Loop, Hypothesis_ratio,);
	RTABMAP_STATS(Loop, Retrieval_margin,)

	RTABMAP_STATS(Memory, Working_memory_size,);
	RTABMAP_STATS(Memory, Short_time_memory_size,);
	RTABMAP_STATS(Memory, Database_size, MB);
	RTABMAP_STATS(Memory, Process_memory_used, MB);
	RTABMAP_STATS(Memory, Signatures_removed,);
	RTABMAP_STATS(Memory, Signatures_reactivated,);
	RTABMAP_STATS(Memory, Images_buffered,);

	RTABMAP_STATS(Timing, Memory_update, ms);
	RTABMAP_STATS(Timing, Cleaning_neighbors, ms);
	RTABMAP_STATS(Timing, Reactivation, ms);
	RTABMAP_STATS(Timing, Likelihood_computation, ms);
	RTABMAP_STATS(Timing, Posterior_computation, ms);
	RTABMAP_STATS(Timing, Hypotheses_creation, ms);
	RTABMAP_STATS(Timing, Hypotheses_validation, ms);
	RTABMAP_STATS(Timing, Statistics_creation, ms);
	RTABMAP_STATS(Timing, Memory_cleanup, ms);
	RTABMAP_STATS(Timing, Total, ms);
	RTABMAP_STATS(Timing, Forgetting, ms);
	RTABMAP_STATS(Timing, Emptying_memory_trash, ms);

	RTABMAP_STATS(, Parent_id,);
	RTABMAP_STATS(, Hypothesis_reactivated,);

	RTABMAP_STATS(Keypoint, Dictionary_size, words);
	RTABMAP_STATS(Keypoint, Response_threshold,);

public:
	static const std::map<std::string, float> & defaultData();

public:
	Statistics();
	Statistics(const Statistics & s);
	virtual ~Statistics();

	// name format = "Grp/Name/unit"
	void addStatistic(const std::string & name, float value);

	// setters
	void setExtended(bool extended) {_extended = extended;}
	void setRefImageId(int refImageId) {_refImageId = refImageId;}
	void setLoopClosureId(int loopClosureId) {_loopClosureId = loopClosureId;}
	void setActions(const std::list<std::vector<float> > & actions) {_actions = actions;}
	void setRefImage(IplImage ** refImage);
	void setRefImage(const IplImage * refImage);
	void setLoopClosureImage(IplImage ** loopClosureImage);
	void setLoopClosureImage(const IplImage * loopClosureImage);
	void setWeights(const std::map<int, int> & weights) {_weights = weights;}
	void setPosterior(const std::map<int, float> & posterior) {_posterior = posterior;}
	void setLikelihood(const std::map<int, float> & likelihood) {_likelihood = likelihood;}
	void setRefWords(const std::multimap<int, cv::KeyPoint> & refWords) {_refWords = refWords;}
	void setLoopWords(const std::multimap<int, cv::KeyPoint> & loopWords) {_loopWords = loopWords;}

	// getters
	bool extended() const {return _extended;}
	int refImageId() const {return _refImageId;}
	int loopClosureId() const {return _loopClosureId;}
	const std::list<std::vector<float> > & getActions() const {return _actions;}
	const IplImage * refImage() const {return _refImage;}
	const IplImage * loopClosureImage() const {return _loopClosureImage;}
	const std::map<int, int> & weights() const {return _weights;}
	const std::map<int, float> & posterior() const {return _posterior;}
	const std::map<int, float> & likelihood() const {return _likelihood;}
	const std::multimap<int, cv::KeyPoint> & refWords() const {return _refWords;}
	const std::multimap<int, cv::KeyPoint> & loopWords() const {return _loopWords;}
	const std::map<std::string, float> & data() const {return _data;}


	Statistics & operator=(const Statistics & s);

private:
	int _extended; // 0 -> only loop closure and last signature ID fields are filled

	int _refImageId;
	int _loopClosureId;

	std::list<std::vector<float> > _actions;

	// extended data start here...
	IplImage * _refImage; // Released by the event destructor
	IplImage * _loopClosureImage; // Released by the event destructor

	std::map<int, int> _weights;
	std::map<int, float> _posterior;
	std::map<int, float> _likelihood;

	//surf
	std::multimap<int, cv::KeyPoint> _refWords;
	std::multimap<int, cv::KeyPoint> _loopWords;

	// Format for statistics (Plottable statistics must go in that map) :
	// {"Group/Name/Unit", value}
	// Example : {"Timing/Total time/ms", 500.0f}
	std::map<std::string, float> _data;
	static std::map<std::string, float> _defaultData;
	static bool _defaultDataInitialized;
	// end extended data
};


////////// The RtabmapEvent class //////////////
class RtabmapEvent : public UEvent
{
public:
	RtabmapEvent(Statistics ** stats) :
		UEvent(0),
		_stats(*stats) {}

	virtual ~RtabmapEvent() {if(_stats) delete _stats;}
	const Statistics & getStats() const {return *_stats;}
	virtual std::string getClassName() const {return std::string("RtabmapEvent");}

private:
	Statistics * _stats;
};

class RtabmapEventCmd : public UEvent
{
public:
	enum Cmd {
		kCmdResetMemory,
		kCmdDumpMemory,
		kCmdDumpPrediction,
		kCmdGenerateGraph,
		kCmdDeleteMemory};
public:
	RtabmapEventCmd(Cmd cmd) :
		UEvent(0),
		_cmd(cmd) {}

	virtual ~RtabmapEventCmd() {}
	Cmd getCmd() const {return _cmd;}
	void setStr(const std::string & str) {_str = str;}
	const std::string & getStr() const {return _str;}
	virtual std::string getClassName() const {return std::string("RtabmapEventCmd");}

private:
	Cmd _cmd;
	std::string _str;
};

class RtabmapEventInit : public UEvent
{
public:
	enum Status {
		kInitializing,
		kInitialized,
		kInfo,
		kError
	};

public:
	RtabmapEventInit(Status status, const std::string & info = std::string()) :
		UEvent(0),
		_status(status),
		_info(info)
	{}

	// for convenience
	RtabmapEventInit(const std::string & info) :
		UEvent(0),
		_status(kInfo),
		_info(info)
	{}

	Status getStatus() const {return _status;}
	const std::string & getInfo() const {return _info;}

	virtual ~RtabmapEventInit() {}
	virtual std::string getClassName() const {return std::string("RtabmapEventInit");}
private:
	Status _status;
	std::string _info; // "Loading signatures", "Loading words" ...
};

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
