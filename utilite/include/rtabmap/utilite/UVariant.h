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

#ifndef UVARIANT_H
#define UVARIANT_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines
#include <string>
#include <vector>

/**
 * Experimental class...
 */
class UTILITE_EXPORT UVariant
{
public:
	enum Type{
		kBool,
		kChar,
		kUChar,
		kShort,
		kUShort,
		kInt,
		kUInt,
		kFloat,
		kDouble,
		kStr,
		kCharArray,
		kUCharArray,
		kShortArray,
		kUShortArray,
		kIntArray,
		kUIntArray,
		kFloatArray,
		kDoubleArray,
		kUndef
		};
public:
	UVariant();
	UVariant(const bool & value);
	UVariant(const char & value);
	UVariant(const unsigned char & value);
	UVariant(const short & value);
	UVariant(const unsigned short & value);
	UVariant(const int & value);
	UVariant(const unsigned int & value);
	UVariant(const float & value);
	UVariant(const double & value);
	UVariant(const char * value);
	UVariant(const std::string & value);
	UVariant(const std::vector<char> & value);
	UVariant(const std::vector<unsigned char> & value);
	UVariant(const std::vector<short> & value);
	UVariant(const std::vector<unsigned short> & value);
	UVariant(const std::vector<int> & value);
	UVariant(const std::vector<unsigned int> & value);
	UVariant(const std::vector<float> & value);
	UVariant(const std::vector<double> & value);

	Type type() const {return type_;}

	bool isUndef() const {return type_ == kUndef;}
	bool isBool() const {return type_ == kBool;}
	bool isChar() const {return type_ == kChar;}
	bool isUChar() const {return type_ == kUChar;}
	bool isShort() const {return type_ == kShort;}
	bool isUShort() const {return type_ == kUShort;}
	bool isInt() const {return type_ == kInt;}
	bool isUInt() const {return type_ == kUInt;}
	bool isFloat() const {return type_ == kFloat;}
	bool isDouble() const {return type_ == kDouble;}
	bool isStr() const {return type_ == kStr;}
	bool isCharArray() const {return type_ == kCharArray;}
	bool isUCharArray() const {return type_ == kUCharArray;}
	bool isShortArray() const {return type_ == kShortArray;}
	bool isUShortArray() const {return type_ == kUShortArray;}
	bool isIntArray() const {return type_ == kIntArray;}
	bool isUIntArray() const {return type_ == kUIntArray;}
	bool isFloatArray() const {return type_ == kFloatArray;}
	bool isDoubleArray() const {return type_ == kDoubleArray;}

	bool toBool() const;
	char toChar(bool * ok = 0) const;
	unsigned char toUChar(bool * ok = 0) const;
	short toShort(bool * ok = 0) const;
	unsigned short toUShort(bool * ok = 0) const;
	int toInt(bool * ok = 0) const;
	unsigned int toUInt(bool * ok = 0) const;
	float toFloat(bool * ok = 0) const;
	double toDouble(bool * ok = 0) const;
	std::string toStr(bool * ok = 0) const;
	std::vector<char> toCharArray(bool * ok = 0) const;
	std::vector<unsigned char> toUCharArray(bool * ok = 0) const;
	std::vector<short> toShortArray(bool * ok = 0) const;
	std::vector<unsigned short> toUShortArray(bool * ok = 0) const;
	std::vector<int> toIntArray(bool * ok = 0) const;
	std::vector<unsigned int> toUIntArray(bool * ok = 0) const;
	std::vector<float> toFloatArray(bool * ok = 0) const;
	std::vector<double> toDoubleArray(bool * ok = 0) const;

	virtual ~UVariant() {}

private:
	Type type_;
	std::vector<unsigned char> data_;
};

#endif /* UVARIANT_H */
