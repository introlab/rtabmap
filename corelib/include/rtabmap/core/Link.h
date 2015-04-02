/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LINK_H_
#define LINK_H_

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

namespace rtabmap {

class Link
{
public:
	enum Type {kNeighbor, kGlobalClosure, kLocalSpaceClosure, kLocalTimeClosure, kUserClosure, kVirtualClosure, kUndef};
	Link() :
		from_(0),
		to_(0),
		type_(kUndef),
		rotVariance_(1.0f),
		transVariance_(1.0f)
	{
	}
	Link(int from, int to, Type type, const Transform & transform, float rotVariance, float transVariance) :
		from_(from),
		to_(to),
		transform_(transform),
		type_(type),
		rotVariance_(rotVariance),
		transVariance_(transVariance)
	{
		UASSERT_MSG(uIsFinite(rotVariance) && rotVariance>0 && uIsFinite(transVariance) && transVariance>0, "Rotational and transitional variances should not be null! (set to 1 if unknown)");
	}

	bool isValid() const {return from_ > 0 && to_ > 0 && !transform_.isNull() && type_!=kUndef;}

	int from() const {return from_;}
	int to() const {return to_;}
	const Transform & transform() const {return transform_;}
	Type type() const {return type_;}
	float rotVariance() const {return rotVariance_;}
	float transVariance() const {return transVariance_;}

	void setFrom(int from) {from_ = from;}
	void setTo(int to) {to_ = to;}
	void setTransform(const Transform & transform) {transform_ = transform;}
	void setType(Type type) {type_ = type;}
	void setVariance(float rotVariance, float transVariance) {
		UASSERT_MSG(uIsFinite(rotVariance) && rotVariance>0 && uIsFinite(transVariance) && transVariance>0, "Rotational and transitional variances should not be null! (set to 1 if unknown)");
		rotVariance_ = rotVariance;
		transVariance_ = transVariance;
	}

private:
	int from_;
	int to_;
	Transform transform_;
	Type type_;
	float rotVariance_;
	float transVariance_;
};

}


#endif /* LINK_H_ */
