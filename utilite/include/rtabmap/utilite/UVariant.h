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
 * \file UVariant.h
 * \brief UVariant class for storing and converting between different data types
 *
 * This class provides a type-safe way to store values of different types
 * and convert between them. It supports basic types (bool, char, int, float, etc.)
 * and arrays of these types.
 *
 * Example:
 * @code
 * UVariant v(42);
 * if(v.isInt())
 * {
 *     int value = v.toInt();
 * }
 * @endcode
 */

/**
 * \class UVariant
 * \brief A variant type that can hold values of different types
 *
 * UVariant is a type-safe container that can store values of various types
 * including booleans, integers, floats, strings, and arrays. It provides
 * methods to check the stored type and convert to other types.
 */
class UTILITE_EXPORT UVariant
{
public:
	/**
	 * \enum Type
	 * \brief Enumeration of supported data types
	 */
	enum Type{
		kBool,        /**< Boolean type */
		kChar,        /**< Signed char type */
		kUChar,       /**< Unsigned char type */
		kShort,       /**< Short integer type */
		kUShort,      /**< Unsigned short integer type */
		kInt,         /**< Integer type */
		kUInt,        /**< Unsigned integer type */
		kFloat,       /**< Float type */
		kDouble,      /**< Double type */
		kStr,         /**< String type */
		kCharArray,   /**< Array of signed char */
		kUCharArray,  /**< Array of unsigned char */
		kShortArray,  /**< Array of short */
		kUShortArray, /**< Array of unsigned short */
		kIntArray,    /**< Array of int */
		kUIntArray,   /**< Array of unsigned int */
		kFloatArray,  /**< Array of float */
		kDoubleArray, /**< Array of double */
		kUndef        /**< Undefined type (default) */
		};
public:
	/**
	 * \brief Default constructor
	 * Creates an undefined variant
	 */
	UVariant();
	
	/**
	 * \brief Constructor from bool
	 * @param value the boolean value
	 */
	UVariant(const bool & value);
	
	/**
	 * \brief Constructor from signed char
	 * @param value the signed char value
	 */
	UVariant(const signed char & value);
	
	/**
	 * \brief Constructor from unsigned char
	 * @param value the unsigned char value
	 */
	UVariant(const unsigned char & value);
	
	/**
	 * \brief Constructor from short
	 * @param value the short value
	 */
	UVariant(const short & value);
	
	/**
	 * \brief Constructor from unsigned short
	 * @param value the unsigned short value
	 */
	UVariant(const unsigned short & value);
	
	/**
	 * \brief Constructor from int
	 * @param value the integer value
	 */
	UVariant(const int & value);
	
	/**
	 * \brief Constructor from unsigned int
	 * @param value the unsigned integer value
	 */
	UVariant(const unsigned int & value);
	
	/**
	 * \brief Constructor from float
	 * @param value the float value
	 */
	UVariant(const float & value);
	
	/**
	 * \brief Constructor from double
	 * @param value the double value
	 */
	UVariant(const double & value);
	
	/**
	 * \brief Constructor from C string
	 * @param value the C string value
	 */
	UVariant(const char * value);
	
	/**
	 * \brief Constructor from std::string
	 * @param value the string value
	 */
	UVariant(const std::string & value);
	
	/**
	 * \brief Constructor from array of signed char
	 * @param value the array of signed char
	 */
	UVariant(const std::vector<signed char> & value);
	
	/**
	 * \brief Constructor from array of unsigned char
	 * @param value the array of unsigned char
	 */
	UVariant(const std::vector<unsigned char> & value);
	
	/**
	 * \brief Constructor from array of short
	 * @param value the array of short
	 */
	UVariant(const std::vector<short> & value);
	
	/**
	 * \brief Constructor from array of unsigned short
	 * @param value the array of unsigned short
	 */
	UVariant(const std::vector<unsigned short> & value);
	
	/**
	 * \brief Constructor from array of int
	 * @param value the array of int
	 */
	UVariant(const std::vector<int> & value);
	
	/**
	 * \brief Constructor from array of unsigned int
	 * @param value the array of unsigned int
	 */
	UVariant(const std::vector<unsigned int> & value);
	
	/**
	 * \brief Constructor from array of float
	 * @param value the array of float
	 */
	UVariant(const std::vector<float> & value);
	
	/**
	 * \brief Constructor from array of double
	 * @param value the array of double
	 */
	UVariant(const std::vector<double> & value);

	/**
	 * \brief Get the type of the stored value
	 * @return the Type enum value
	 */
	Type type() const {return type_;}

	/**
	 * \brief Check if the variant is undefined
	 * @return true if type is kUndef
	 */
	bool isUndef() const {return type_ == kUndef;}
	
	/**
	 * \brief Check if the variant is a bool
	 * @return true if type is kBool
	 */
	bool isBool() const {return type_ == kBool;}
	
	/**
	 * \brief Check if the variant is a signed char
	 * @return true if type is kChar
	 */
	bool isChar() const {return type_ == kChar;}
	
	/**
	 * \brief Check if the variant is an unsigned char
	 * @return true if type is kUChar
	 */
	bool isUChar() const {return type_ == kUChar;}
	
	/**
	 * \brief Check if the variant is a short
	 * @return true if type is kShort
	 */
	bool isShort() const {return type_ == kShort;}
	
	/**
	 * \brief Check if the variant is an unsigned short
	 * @return true if type is kUShort
	 */
	bool isUShort() const {return type_ == kUShort;}
	
	/**
	 * \brief Check if the variant is an int
	 * @return true if type is kInt
	 */
	bool isInt() const {return type_ == kInt;}
	
	/**
	 * \brief Check if the variant is an unsigned int
	 * @return true if type is kUInt
	 */
	bool isUInt() const {return type_ == kUInt;}
	
	/**
	 * \brief Check if the variant is a float
	 * @return true if type is kFloat
	 */
	bool isFloat() const {return type_ == kFloat;}
	
	/**
	 * \brief Check if the variant is a double
	 * @return true if type is kDouble
	 */
	bool isDouble() const {return type_ == kDouble;}
	
	/**
	 * \brief Check if the variant is a string
	 * @return true if type is kStr
	 */
	bool isStr() const {return type_ == kStr;}
	
	/**
	 * \brief Check if the variant is an array of signed char
	 * @return true if type is kCharArray
	 */
	bool isCharArray() const {return type_ == kCharArray;}
	
	/**
	 * \brief Check if the variant is an array of unsigned char
	 * @return true if type is kUCharArray
	 */
	bool isUCharArray() const {return type_ == kUCharArray;}
	
	/**
	 * \brief Check if the variant is an array of short
	 * @return true if type is kShortArray
	 */
	bool isShortArray() const {return type_ == kShortArray;}
	
	/**
	 * \brief Check if the variant is an array of unsigned short
	 * @return true if type is kUShortArray
	 */
	bool isUShortArray() const {return type_ == kUShortArray;}
	
	/**
	 * \brief Check if the variant is an array of int
	 * @return true if type is kIntArray
	 */
	bool isIntArray() const {return type_ == kIntArray;}
	
	/**
	 * \brief Check if the variant is an array of unsigned int
	 * @return true if type is kUIntArray
	 */
	bool isUIntArray() const {return type_ == kUIntArray;}
	
	/**
	 * \brief Check if the variant is an array of float
	 * @return true if type is kFloatArray
	 */
	bool isFloatArray() const {return type_ == kFloatArray;}
	
	/**
	 * \brief Check if the variant is an array of double
	 * @return true if type is kDoubleArray
	 */
	bool isDoubleArray() const {return type_ == kDoubleArray;}

	/**
	 * \brief Convert the variant to bool
	 * @return the boolean value
	 */
	bool toBool() const;
	
	/**
	 * \brief Convert the variant to signed char
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the signed char value
	 */
	signed char toChar(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to unsigned char
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the unsigned char value
	 */
	unsigned char toUChar(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to short
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the short value
	 */
	short toShort(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to unsigned short
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the unsigned short value
	 */
	unsigned short toUShort(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to int
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the integer value
	 */
	int toInt(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to unsigned int
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the unsigned integer value
	 */
	unsigned int toUInt(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to float
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the float value
	 */
	float toFloat(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to double
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the double value
	 */
	double toDouble(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to string
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the string value
	 */
	std::string toStr(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of signed char
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of signed char
	 */
	std::vector<signed char> toCharArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of unsigned char
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of unsigned char
	 */
	std::vector<unsigned char> toUCharArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of short
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of short
	 */
	std::vector<short> toShortArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of unsigned short
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of unsigned short
	 */
	std::vector<unsigned short> toUShortArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of int
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of int
	 */
	std::vector<int> toIntArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of unsigned int
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of unsigned int
	 */
	std::vector<unsigned int> toUIntArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of float
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of float
	 */
	std::vector<float> toFloatArray(bool * ok = 0) const;
	
	/**
	 * \brief Convert the variant to array of double
	 * @param ok pointer to bool to indicate success (optional)
	 * @return the array of double
	 */
	std::vector<double> toDoubleArray(bool * ok = 0) const;

	/**
	 * \brief Virtual destructor
	 */
	virtual ~UVariant() {}

private:
	Type type_;                      /**< The type of the stored value */
	std::vector<unsigned char> data_; /**< The stored data */
};

#endif /* UVARIANT_H */
