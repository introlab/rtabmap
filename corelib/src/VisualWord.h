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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>

namespace rtabmap
{

class SignatureSurf;

class RTABMAP_EXP VisualWord
{
public:
	VisualWord(int id, const float * descriptor, int dim, int signatureId = 0);
	~VisualWord();

	void addRef(int signatureId);
	int removeAllRef(int signatureId);

	int getTotalReferences() const {return _totalReferences;}
	int id() const {return _id;}
	const float * getDescriptor() const {return _descriptor;}
	int getDim() const {return _dim;}
	const std::map<int, int> & getReferences() const {return _references;} // (signature id , occurrence in the signature)

	bool isSaved() const {return _saved;}
	void setSaved(bool saved) {_saved = saved;}

private:
	int _id;
	float * _descriptor;
	int _dim;
	bool _saved; // If it's saved to db

	int _totalReferences;
	std::map<int, int> _references; // (signature id , occurrence in the signature)
	std::map<int, int> _oldReferences; // (signature id , occurrence in the signature)
};

} // namespace rtabmap
