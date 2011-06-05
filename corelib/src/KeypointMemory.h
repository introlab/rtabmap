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

#ifndef KEYPOINTMEMORY_H_
#define KEYPOINTMEMORY_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "Memory.h"

namespace rtabmap {

class VWDictionary;
class VisualWord;
class KeypointDetector;
class KeypointDescriptor;

class RTABMAP_EXP KeypointMemory : public Memory
{
public:
	enum DetectorStrategy {kDetectorSurf, kDetectorStar, kDetectorSift, kDetectorUndef};
	enum DescriptorStrategy {kDescriptorSurf, kDescriptorColorSurf, kDescriptorLaplacianSurf, kDescriptorSift, kDescriptorHueSurf, kDescriptorUndef};

public:
	KeypointMemory(const ParametersMap & parameters = ParametersMap());
	virtual ~KeypointMemory();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual bool init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten = false, const ParametersMap & parameters = ParametersMap());
	virtual std::map<int, float> computeLikelihood(const Signature * signature, const std::set<int> & signatureIds = std::set<int>()) const;
	virtual int forget(const std::list<int> & ignoredIds = std::list<int>());
	virtual int reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, unsigned int maxTouched);
	virtual void dumpMemory(std::string directory) const;
	virtual void dumpSignatures(const char * fileNameSign) const;

	void dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const;

	const KeypointDetector * getKeypointDetector() const {return _keypointDetector;}
	const KeypointDescriptor * getKeypointDescriptor() const {return _keypointDescriptor;}
	const VWDictionary * getVWD() const {return _vwd;}
	DetectorStrategy detectorStrategy() const;

protected:
	virtual Signature * getSignatureLtMem(int id);
	virtual void addSignatureToStm(Signature * signature, const std::list<std::vector<float> > & actions = std::list<std::vector<float> >());
	virtual void clear();
	virtual void moveToTrash(Signature * s);
	virtual void preUpdate();
	virtual void merge(const Signature * from, Signature * to, MergingStrategy s);

private:
	virtual Signature * createSignature(int id, const SMState * rawData, bool keepRawData=false);
	void disableWordsRef(int signatureId);
	void enableWordsRef(const std::list<int> & signatureIds);
	void cleanUnusedWords();
	int getNi(int signatureId) const;

private:
	std::list<int> _commonWords;
	VWDictionary * _vwd;
	KeypointDetector * _keypointDetector;
	KeypointDescriptor * _keypointDescriptor;
	//std::map<int, int> _wordRefsToChange;
	bool _reactivatedWordsComparedToNewWords;
	float _badSignRatio;;
	bool _tfIdfLikelihoodUsed;
	bool _parallelized;
	bool _tfIdfNormalized;
};

}

#endif /* KEYPOINTMEMORY_H_ */
