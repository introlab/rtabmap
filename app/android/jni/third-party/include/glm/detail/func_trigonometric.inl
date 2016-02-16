///////////////////////////////////////////////////////////////////////////////////
/// OpenGL Mathematics (glm.g-truc.net)
///
/// Copyright (c) 2005 - 2014 G-Truc Creation (www.g-truc.net)
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
///
/// @ref core
/// @file glm/core/func_trigonometric.inl
/// @date 2008-08-03 / 2011-06-15
/// @author Christophe Riccio
///////////////////////////////////////////////////////////////////////////////////

#include "_vectorize.hpp"
#include <cmath>
#include <limits>

namespace glm
{
	// radians
	template <typename genType>
	GLM_FUNC_QUALIFIER genType radians
	(
		genType const & degrees
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'radians' only accept floating-point input");

		return degrees * genType(0.01745329251994329576923690768489);
	}

	VECTORIZE_VEC(radians)
	
	// degrees
	template <typename genType>
	GLM_FUNC_QUALIFIER genType degrees
	(
		genType const & radians
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'degrees' only accept floating-point input");

		return radians * genType(57.295779513082320876798154814105);
	}

	VECTORIZE_VEC(degrees)

	// sin
	using std::sin;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType sin
	(
		genType const & angle
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'sin' only accept floating-point input");

		return genType(::std::sin(angle));
	}
*/
	VECTORIZE_VEC(sin)

	// cos
	using std::cos;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType cos(genType const & angle)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'cos' only accept floating-point input");

		return genType(::std::cos(angle));
	}
*/

	VECTORIZE_VEC(cos)

	// tan
	using std::tan;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType tan
	(
		genType const & angle
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'tan' only accept floating-point input");

		return genType(::std::tan(angle));
	}
*/

	VECTORIZE_VEC(tan)

	// asin
	using std::asin;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType asin
	(
		genType const & x
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'asin' only accept floating-point input");

		return genType(::std::asin(x));
	}
*/
	VECTORIZE_VEC(asin)

	// acos
	using std::acos;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType acos
	(
		genType const & x
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'acos' only accept floating-point input");

		return ::std::acos(x);
	}
*/
	VECTORIZE_VEC(acos)

	// atan
	template <typename genType>
	GLM_FUNC_QUALIFIER genType atan
	(
		genType const & y,
		genType const & x
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'atan' only accept floating-point input");

		return ::std::atan2(y, x);
	}

	VECTORIZE_VEC_VEC(atan)

	using std::atan;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType atan
	(
		genType const & x
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'atan' only accept floating-point input");

		return ::std::atan(x);
	}
*/
	VECTORIZE_VEC(atan)

	// sinh
	using std::sinh;
/*
	template <typename genType> 
	GLM_FUNC_QUALIFIER genType sinh
	(
		genType const & angle
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'sinh' only accept floating-point input");

		return ::std::sinh(angle);
	}
*/
	VECTORIZE_VEC(sinh)

	// cosh
	using std::cosh;
/*
	template <typename genType> 
	GLM_FUNC_QUALIFIER genType cosh
	(
		genType const & angle
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'cosh' only accept floating-point input");

		return ::std::cosh(angle);
	}
*/
	VECTORIZE_VEC(cosh)

	// tanh
	using std::tanh;
/*
	template <typename genType>
	GLM_FUNC_QUALIFIER genType tanh
	(
		genType const & angle
	)
	{
		GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'tanh' only accept floating-point input");

		return ::std::tanh(angle);
	}
*/
	VECTORIZE_VEC(tanh)

	// asinh
#	if(GLM_LANG & GLM_LANG_CXX11_FLAG & !ANDROID)
		using std::asinh;
#	else
		template <typename genType> 
		GLM_FUNC_QUALIFIER genType asinh(genType const & x)
		{
			GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'asinh' only accept floating-point input");

			return (x < static_cast<genType>(0) ? static_cast<genType>(-1) : (x > static_cast<genType>(0) ? static_cast<genType>(1) : static_cast<genType>(0))) * log(abs(x) + sqrt(static_cast<genType>(1) + x * x));
		}
#	endif

	VECTORIZE_VEC(asinh)

	// acosh
#	if(GLM_LANG & GLM_LANG_CXX11_FLAG & !ANDROID)
		using std::acosh;
#	else
		template <typename genType> 
		GLM_FUNC_QUALIFIER genType acosh(genType const & x)
		{
			GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'acosh' only accept floating-point input");

			if(x < static_cast<genType>(1))
				return static_cast<genType>(0);
			return log(x + sqrt(x * x - static_cast<genType>(1)));
		}
#	endif

	VECTORIZE_VEC(acosh)

	// atanh
#	if(GLM_LANG & GLM_LANG_CXX11_FLAG & !ANDROID)
		using std::atanh;
#	else
		template <typename genType>
		GLM_FUNC_QUALIFIER genType atanh(genType const & x)
		{
			GLM_STATIC_ASSERT(std::numeric_limits<genType>::is_iec559, "'atanh' only accept floating-point input");
		
			if(abs(x) >= static_cast<genType>(1))
				return 0;
			return static_cast<genType>(0.5) * log((static_cast<genType>(1) + x) / (static_cast<genType>(1) - x));
		}
#	endif

	VECTORIZE_VEC(atanh)

}//namespace glm
