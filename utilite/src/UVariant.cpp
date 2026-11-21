/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
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

#include "rtabmap/utilite/UVariant.h"
#include "rtabmap/utilite/UConversion.h"
#include <limits>
#include <string.h>

UVariant::UVariant() :
	type_(kUndef)
{
}
UVariant::UVariant(const bool & value) :
	type_(kBool),
	data_(1)
{
	data_[0] = value?1:0;
}
UVariant::UVariant(const signed char & value) :
	type_(kChar),
	data_(sizeof(signed char))
{
	memcpy(data_.data(), &value, sizeof(signed char));
}
UVariant::UVariant(const unsigned char & value) :
	type_(kUChar),
	data_(sizeof(unsigned char))
{
	memcpy(data_.data(), &value, sizeof(unsigned char));
}
UVariant::UVariant(const short & value) :
	type_(kShort),
	data_(sizeof(short))
{
	memcpy(data_.data(), &value, sizeof(short));
}
UVariant::UVariant(const unsigned short & value) :
	type_(kUShort),
	data_(sizeof(unsigned short))
{
	memcpy(data_.data(), &value, sizeof(unsigned short));
}
UVariant::UVariant(const int & value) :
	type_(kInt),
	data_(sizeof(int))
{
	memcpy(data_.data(), &value, sizeof(int));
}
UVariant::UVariant(const unsigned int & value) :
	type_(kUInt),
	data_(sizeof(unsigned int))
{
	memcpy(data_.data(), &value, sizeof(unsigned int));
}
UVariant::UVariant(const float & value) :
	type_(kFloat),
	data_(sizeof(float))
{
	memcpy(data_.data(), &value, sizeof(float));
}
UVariant::UVariant(const double & value) :
	type_(kDouble),
	data_(sizeof(double))
{
	memcpy(data_.data(), &value, sizeof(double));
}
UVariant::UVariant(const char * value) :
	type_(kStr)
{
	std::string str(value);
	data_.resize(str.size()+1);
	memcpy(data_.data(), str.data(), str.size()+1);
}
UVariant::UVariant(const std::string & value) :
	type_(kStr),
	data_(value.size()+1) // with null character
{
	memcpy(data_.data(), value.data(), value.size()+1);
}
UVariant::UVariant(const std::vector<signed char> & value) :
	type_(kCharArray),
	data_(sizeof(signed char)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(signed char)*value.size());
}
UVariant::UVariant(const std::vector<unsigned char> & value) :
	type_(kUCharArray),
	data_(sizeof(unsigned char)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(unsigned char)*value.size());
}
UVariant::UVariant(const std::vector<short> & value) :
	type_(kShortArray),
	data_(sizeof(short)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(short)*value.size());
}
UVariant::UVariant(const std::vector<unsigned short> & value) :
	type_(kUShortArray),
	data_(sizeof(unsigned short)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(unsigned short)*value.size());
}
UVariant::UVariant(const std::vector<int> & value) :
	type_(kIntArray),
	data_(sizeof(int)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(int)*value.size());
}
UVariant::UVariant(const std::vector<unsigned int> & value) :
	type_(kUIntArray),
	data_(sizeof(unsigned int)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(unsigned int)*value.size());
}
UVariant::UVariant(const std::vector<float> & value) :
	type_(kFloatArray),
	data_(sizeof(float)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(float)*value.size());
}
UVariant::UVariant(const std::vector<double> & value) :
	type_(kDoubleArray),
	data_(sizeof(double)*value.size())
{
	memcpy(data_.data(), value.data(), sizeof(double)*value.size());
}

bool UVariant::toBool() const
{
	if(type_ ==kStr)
	{
		return uStr2Bool(toStr().c_str());
	}
	else if(data_.size())
	{
		return memcmp(data_.data(), std::vector<unsigned char>(data_.size(), 0).data(), data_.size()) != 0;
	}
	return false;
}

signed char UVariant::toChar(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	signed char v = 0;
	if(type_ == kChar)
	{
		memcpy(&v, data_.data(), sizeof(signed char));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUChar)
	{
		unsigned char tmp = toUChar();
		if(tmp <= std::numeric_limits<signed char>::max())
		{
			v = (signed char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kShort)
	{
		short tmp = toShort();
		if(tmp >= std::numeric_limits<signed char>::min() && tmp <= std::numeric_limits<signed char>::max())
		{
			v = (signed char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUShort)
	{
		unsigned short tmp = toUShort();
		if(tmp <= std::numeric_limits<signed char>::max())
		{
			v = (signed char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= std::numeric_limits<signed char>::min() && tmp <= std::numeric_limits<signed char>::max())
		{
			v = (signed char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= (unsigned int)std::numeric_limits<signed char>::max())
		{
			v = (signed char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
unsigned char UVariant::toUChar(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	unsigned char v = 0;
	if(type_ == kUChar)
	{
		memcpy(&v, data_.data(), sizeof(unsigned char));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		char tmp = toChar();
		if(tmp >= std::numeric_limits<unsigned char>::min())
		{
			v = (unsigned char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kShort)
	{
		short tmp = toShort();
		if(tmp >= std::numeric_limits<unsigned char>::min() && tmp <= std::numeric_limits<unsigned char>::max())
		{
			v = (unsigned char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUShort)
	{
		unsigned short tmp = toUShort();
		if(tmp >= std::numeric_limits<unsigned char>::min() && tmp <= std::numeric_limits<unsigned char>::max())
		{
			v = (unsigned char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= std::numeric_limits<unsigned char>::min() && tmp <= std::numeric_limits<unsigned char>::max())
		{
			v = (unsigned char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= std::numeric_limits<unsigned char>::max())
		{
			v = (unsigned char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
short UVariant::toShort(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	short v = 0;
	if(type_ == kShort)
	{
		memcpy(&v, data_.data(), sizeof(short));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		v = (short)toChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUChar)
	{
		v = (short)toUChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUShort)
	{
		unsigned short tmp = toUShort();
		if(tmp <= std::numeric_limits<short>::max())
		{
			v = (short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= std::numeric_limits<short>::min() && tmp <= std::numeric_limits<short>::max())
		{
			v = (short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= (unsigned int)std::numeric_limits<short>::max())
		{
			v = (short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
unsigned short UVariant::toUShort(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	unsigned short v = 0;
	if(type_ == kUShort)
	{
		memcpy(&v, data_.data(), sizeof(unsigned short));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		signed char tmp = toChar();
		if(tmp >= 0)
		{
			v = (unsigned short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUChar)
	{
		v = (unsigned short)toUChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kShort)
	{
		short tmp = toShort();
		if(tmp >= std::numeric_limits<unsigned short>::min())
		{
			v = (unsigned short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= std::numeric_limits<unsigned short>::min() && tmp <= std::numeric_limits<unsigned short>::max())
		{
			v = (unsigned short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= std::numeric_limits<unsigned short>::max())
		{
			v = (unsigned short)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
int UVariant::toInt(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	int v = 0;
	if(type_ == kInt)
	{
		memcpy(&v, data_.data(), sizeof(int));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		v = (int)toChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUChar)
	{
		v = (int)toUChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kShort)
	{
		v = (int)toShort();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUShort)
	{
		v = (int)toUShort();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= (unsigned int)std::numeric_limits<int>::max())
		{
			v = (int)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
unsigned int UVariant::toUInt(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	unsigned int v = 0;
	if(type_ == kUInt)
	{
		memcpy(&v, data_.data(), sizeof(unsigned int));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		signed char tmp = toChar();
		if(tmp >= 0)
		{
			v = (unsigned int)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUChar)
	{
		v = (unsigned int)toUChar();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kShort)
	{
		short tmp = toShort();
		if(tmp >= 0)
		{
			v = (unsigned int)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUShort)
	{
		v = (unsigned int)toUShort();
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= 0)
		{
			v = (unsigned int)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
float UVariant::toFloat(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	float v = 0;
	if(type_ == kFloat)
	{
		memcpy(&v, data_.data(), sizeof(float));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kDouble)
	{
		double tmp = toDouble();
		if(tmp >= std::numeric_limits<float>::min() && tmp <= std::numeric_limits<float>::max())
		{
			v = (float)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	return v;
}
double UVariant::toDouble(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	double v = 0;
	if(type_ == kDouble)
	{
		memcpy(&v, data_.data(), sizeof(double));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kFloat)
	{
		v = (double)toFloat(ok);
	}
	return v;
}
std::string UVariant::toStr(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	std::string v;
	if(type_ == kStr)
	{
		v = std::string((const char *)data_.data());
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kBool)
	{
		v = toBool()?"true":"false";
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kChar)
	{
		v = " ";
		v.at(0) = toChar(ok);
	}
	else if(type_ == kUChar)
	{
		v = uNumber2Str(toUChar(ok));
	}
	else if(type_ == kShort)
	{
		v = uNumber2Str(toShort(ok));
	}
	else if(type_ == kUShort)
	{
		v = uNumber2Str(toUShort(ok));
	}
	else if(type_ == kInt)
	{
		v = uNumber2Str(toInt(ok));
	}
	else if(type_ == kUInt)
	{
		v = uNumber2Str(toUInt(ok));
	}
	else if(type_ == kFloat)
	{
		v = uNumber2Str(toFloat(ok));
	}
	else if(type_ == kDouble)
	{
		v = uNumber2Str(toDouble(ok));
	}
	return v;
}

std::vector<signed char> UVariant::toCharArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<signed char> v;
	if(type_ == kCharArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(signed char));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}

std::vector<unsigned char> UVariant::toUCharArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<unsigned char> v;
	if(type_ == kUCharArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(unsigned char));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}

std::vector<short> UVariant::toShortArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<short> v;
	if(type_ == kShortArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(short));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
std::vector<unsigned short> UVariant::toUShortArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<unsigned short> v;
	if(type_ == kUShortArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(unsigned short));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
std::vector<int> UVariant::toIntArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<int> v;
	if(type_ == kIntArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(int));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
std::vector<unsigned int> UVariant::toUIntArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<unsigned int> v;
	if(type_ == kUIntArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(unsigned int));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
std::vector<float> UVariant::toFloatArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<float> v;
	if(type_ == kFloatArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(float));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
std::vector<double> UVariant::toDoubleArray(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}

	std::vector<double> v;
	if(type_ == kDoubleArray)
	{
		if(ok)
		{
			*ok = true;
		}
		if(data_.size())
		{
			v.resize(data_.size() / sizeof(double));
			memcpy(v.data(), data_.data(), data_.size());
		}
	}
	return v;
}
