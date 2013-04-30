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

#include "VisualWord.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap
{

VisualWord::VisualWord(int id, const float * descriptor, int dim, int signatureId) :
	_id(id),
	_saved(false),
	_totalReferences(0)
{
	_descriptor = new float[dim];
	if(_descriptor && descriptor)
	{
		memcpy(_descriptor, descriptor, dim*sizeof(float));
	}
	else
	{
		ULOGGER_ERROR("not enough memory to create the descriptor...");
	}
	_dim = dim;

	if(signatureId)
	{
		addRef(signatureId);
	}
}

VisualWord::~VisualWord()
{
	if(_descriptor)
	{
		delete [] _descriptor;
	}
}

void VisualWord::addRef(int signatureId)
{
	std::map<int, int>::iterator iter = _references.find(signatureId);
	if(iter != _references.end())
	{
		(*iter).second += 1;
	}
	else
	{
		_references.insert(std::pair<int, int>(signatureId, 1));
	}
	++_totalReferences;
}

int VisualWord::removeAllRef(int signatureId)
{
	int removed = uTake(_references, signatureId, 0);
	_totalReferences -= removed;
	return removed;
}

} // namespace rtabmap
