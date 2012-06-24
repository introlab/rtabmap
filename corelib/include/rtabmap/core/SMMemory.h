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

#include "rtabmap/core/Memory.h"

namespace rtabmap {

class ColorTable;
class SMSignature;

class RTABMAP_EXP SMMemory : public Memory
{
public:
	SMMemory(const ParametersMap & parameters = ParametersMap());
	virtual ~SMMemory();
	virtual void parseParameters(const ParametersMap & parameters);
	virtual std::set<int> reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess);
	void setRoi(const std::string & roi);
	void setColorTable(int size);

private:
	virtual void copyData(const Signature * from, Signature * to);
	virtual Signature * createSignature(int id, const std::list<Sensor> & sensors, bool keepRawData=false);

private:
	bool _useLogPolar;
	ColorTable * _colorTable;
	bool _useMotionMask;
	float _dBThreshold;
	bool _dBIndexing;
	bool _magnitudeInvariant;
};

}

#endif /* KEYPOINTMEMORY_H_ */
