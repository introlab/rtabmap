/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
UVariant::UVariant(const char & value) :
	type_(kChar),
	data_(sizeof(char))
{
	memcpy(data_.data(), &value, sizeof(char));
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

char UVariant::toChar(bool * ok) const
{
	if(ok)
	{
		*ok = false;
	}
	char v = 0;
	if(type_ == kChar)
	{
		memcpy(&v, data_.data(), sizeof(char));
		if(ok)
		{
			*ok = true;
		}
	}
	else if(type_ == kUChar)
	{
		unsigned char tmp = toUChar();
		if(tmp <= std::numeric_limits<char>::max())
		{
			v = (char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kShort)
	{
		short tmp = toShort();
		if(tmp >= std::numeric_limits<char>::min() && tmp <= std::numeric_limits<char>::max())
		{
			v = (char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUShort)
	{
		unsigned short tmp = toUShort();
		if(tmp <= std::numeric_limits<char>::max())
		{
			v = (char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kInt)
	{
		int tmp = toInt();
		if(tmp >= std::numeric_limits<char>::min() && tmp <= std::numeric_limits<char>::max())
		{
			v = (char)tmp;
			if(ok)
			{
				*ok = true;
			}
		}
	}
	else if(type_ == kUInt)
	{
		unsigned int tmp = toUInt();
		if(tmp <= (unsigned int)std::numeric_limits<char>::max())
		{
			v = (char)tmp;
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
		char tmp = toChar();
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
		char tmp = toChar();
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
