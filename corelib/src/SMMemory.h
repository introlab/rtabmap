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

#ifndef SIMPLEMEMORY_H_
#define SIMPLEMEMORY_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "Memory.h"

namespace rtabmap {

class ColorTable;
class SMSignature;

class RTABMAP_EXP SMMemory : public Memory
{
public:
	SMMemory(const ParametersMap & parameters = ParametersMap());
	virtual ~SMMemory();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual bool init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten = false, const ParametersMap & parameters = ParametersMap());
	virtual std::map<int, float> computeLikelihood(const Signature * signature, const std::list<int> & ids, float & maximumScore);
	virtual std::set<int> reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess);
	void setRoi(const std::string & roi);
	void setVotingScheme(bool useVotingScheme);
	void setColorTable(int size);

protected:
	virtual void moveToTrash(Signature * s);
	virtual Signature * getSignatureLtMem(int id);

private:
	virtual void copyData(const Signature * from, Signature * to);
	virtual Signature * createSignature(int id, const SMState * rawData, bool keepRawData=false);
	void updateDictionary(const Signature * s);

private:
	bool _useLogPolar;
	bool _useVotingScheme;
	ColorTable * _colorTable;
	bool _useMotionMask;
	std::vector<std::map<int, std::set<int> > > _dictionary;
};

}

#endif /* KEYPOINTMEMORY_H_ */
