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

#ifndef UMATH_H
#define UMATH_H

/** \file UMath.h
    \brief Basic mathematics functions
*/

#include <cmath>
#include <list>
#include <vector>

#if _MSC_VER
	#undef min
	#undef max
#endif

/**
 * Return true if the number is NAN.
 */
template<class T>
inline bool uIsNan(const T & value)
{
#if _MSC_VER
	return _isnan(value) != 0;
#else
	return std::isnan(value);
#endif
}

/**
 * Return true if the number is finite.
 */
template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
	return _finite(value) != 0;
#else
	return std::isfinite(value);
#endif
}

/**
 * Get the minimum of the 3 values.
 * @return the minimum value
 */
template<class T>
inline T uMin3( const T& a, const T& b, const T& c)
{
	float m=a<b?a:b;
	return m<c?m:c;
}

/**
 * Get the maximum of the 3 values.
 * @return the maximum value
 */
template<class T>
inline T uMax3( const T& a, const T& b, const T& c)
{
	float m=a>b?a:b;
	return m>c?m:c;
}

/**
 * Get the maximum of a vector.
 * @param v the array
 * @param size the size of the array
 * @param index the index of the maximum value in the vector.
 * @return the maximum value of the array
 */
template<class T>
inline T uMax(const T * v, unsigned int size, unsigned int & index)
{
	T max = 0;
	index = 0;
	if(!v || size == 0)
	{
		return max;
	}
	max = v[0];
	for(unsigned int i=1; i<size; ++i)
	{
		if(uIsNan(max) || (max < v[i] && !uIsNan(v[i])))
		{
			max = v[i];
			index = i;
		}
	}

	return max;
}

/**
 * Get the maximum of a vector.
 * @param v the array
 * @param index the index of the maximum value in the vector.
 * @return the maximum value of the array
 */
template<class T>
inline T uMax(const std::vector<T> & v, unsigned int & index)
{
	return uMax(v.data(), v->size(), index);
}

/**
 * Get the maximum of a vector.
 * @param v the array
 * @param size the size of the array
 * @return the maximum value of the array
 */
template<class T>
inline T uMax(const T * v, unsigned int size)
{
	unsigned int index;
	return uMax(v, size, index);
}

/**
 * Get the maximum of a vector.
 * @param v the array
 * @return the maximum value of the array
 */
template<class T>
inline T uMax(const std::vector<T> & v)
{
	return uMax(v.data(), v.size());
}

/**
 * Get the minimum of a vector.
 * @param v the array
 * @param size the size of the array
 * @param index the index of the minimum value in the vector.
 * @return the minimum value of the array
 */
template<class T>
inline T uMin(const T * v, unsigned int size, unsigned int & index)
{
	T min = 0;
	index = 0;
	if(!v || size == 0)
	{
		return min;
	}
	min = v[0];
	for(unsigned int i=1; i<size; ++i)
	{
		if(uIsNan(min) || (min > v[i] && !uIsNan(v[i])))
		{
			min = v[i];
			index = i;
		}
	}

	return min;
}

/**
 * Get the minimum of a vector.
 * @param v the array
 * @param index the index of the minimum value in the vector.
 * @return the minimum value of the array
 */
template<class T>
inline T uMin(const std::vector<T> & v, unsigned int & index)
{
	return uMin(v.data(), v.size(), index);
}

/**
 * Get the minimum of a vector.
 * @param v the array
 * @param size the size of the array
 * @return the minimum value of the array
 */
template<class T>
inline T uMin(const T * v, unsigned int size)
{
	unsigned int index;
	return uMin(v, size, index);
}

/**
 * Get the minimum of a vector.
 * @param v the array
 * @return the minimum value of the array
 */
template<class T>
inline T uMin(const std::vector<T> & v)
{
	return uMin(v.data(), v.size());
}

/**
 * Get the minimum and maximum of a vector.
 * @param v the array
 * @param size the size of the array
 * @param min reference to output minimum
 * @param max reference to output maximum
 * @param min reference to output minimum index
 * @param max reference to output maximum index
 */
template<class T>
inline void uMinMax(const T * v, unsigned int size, T & min, T & max, unsigned int & indexMin, unsigned int & indexMax)
{
	min = 0;
	max = 0;
	indexMin = 0;
	indexMax = 0;
	if(!v || size == 0)
	{
		return;
	}
	min = v[0];
	max = v[0];

	for(unsigned int i=1; i<size; ++i)
	{
		if(uIsNan(min) || (min > v[i] && !uIsNan(v[i])))
		{
			min = v[i];
			indexMin = i;
		}

		if(uIsNan(max) || (max < v[i] && !uIsNan(v[i])))
		{
			max = v[i];
			indexMax = i;
		}
	}
}

/**
 * Get the minimum and maximum of a vector.
 * @param v the array
 * @param min reference to output minimum
 * @param max reference to output maximum
 * @param min reference to output minimum index
 * @param max reference to output maximum index
 */
template<class T>
inline void uMinMax(const std::vector<T> & v, T & min, T & max, unsigned int & indexMin, unsigned int & indexMax)
{
	uMinMax(v.data(), v.size(), min, max, indexMin, indexMax);
}

/**
 * Get the minimum and maximum of a vector.
 * @param v the array
 * @param size the size of the array
 * @param min reference to output minimum
 * @param max reference to output maximum
 */
template<class T>
inline void uMinMax(const T * v, unsigned int size, T & min, T & max)
{
	unsigned int indexMin;
	unsigned int indexMax;
	uMinMax(v, size, min, max, indexMin, indexMax);
}

/**
 * Get the minimum and maximum of a vector.
 * @param v the array
 * @param min reference to output minimum
 * @param max reference to output maximum
 */
template<class T>
inline void uMinMax(const std::vector<T> & v, T & min, T & max)
{
	uMinMax(v.data(), v.size(), min, max);
}

/**
 * Get the sign of value.
 * @param v the value
 * @return -1 if v<0, otherwise 1
 */
template<class T>
inline int uSign(const T & v)
{
	if(v < 0)
	{
		return -1;
	}
	else
	{
		return 1;
	}
}

/**
 * Get the sum of all values contained in a list. Provided for convenience.
 * @param list the list
 * @return the sum of values of the list
 */
template<class T>
inline T uSum(const std::list<T> & list)
{
	T sum = 0;
	for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
	{
		sum += *i;
	}
	return sum;
}

/**
 * Get the sum of all values contained in an array: sum(x).
 * @param v the array
 * @param size the size of the array
 * @return the sum of values of the array
 */
template<class T>
inline T uSum(const T * v, unsigned int size)
{
	T sum = 0;
	if(v && size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			sum += v[i];
		}
	}
	return sum;
}

/**
 * Get the sum of all values contained in a vector. Provided for convenience.
 * @param v the vector
 * @return the sum of values of the vector
 */
template<class T>
inline T uSum(const std::vector<T> & v)
{
	return uSum(v.data(), (int)v.size());
}

/**
 * Get the sum of all squared values contained in an array: sum(x.^2).
 * @param v the array
 * @param size the size of the array
 * @param subtract an optional value to remove to v before squaring v: sum((x-sub).^2)
 * @return the sum of values of the array
 */
template<class T>
inline T uSumSquared(const T * v, unsigned int size, T subtract = T())
{
	T sum = 0;
	if(v && size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			sum += (v[i]-subtract)*(v[i]-subtract);
		}
	}
	return sum;
}

/**
 * Get the sum of all squared values contained in an array: sum(x.^2).
 * @param v the array
 * @param subtract an optional value to remove to v before squaring v: sum((x-sub).^2)
 * @return the sum of values of the array
 */
template<class T>
inline T uSumSquared(const std::vector<T> & v, T subtract = T())
{
	return uSumSquared(v.data(), v.size(), subtract);
}

/**
 * Compute the mean of an array.
 * @param v the array
 * @param size the size of the array
 * @return the mean
 */
template<class T>
inline T uMean(const T * v, unsigned int size)
{
	T buf = 0;
	if(v && size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			buf += v[i];
		}
		buf /= size;
	}
	return buf;
}

/**
 * Get the mean of a list. Provided for convenience.
 * @param list the list
 * @return the mean
 */
template<class T>
inline T uMean(const std::list<T> & list)
{
	T m = 0;
	if(list.size())
	{
		for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
		{
			m += *i;
		}
		m /= list.size();
	}
	return m;
}

/**
 * Get the mean of a vector. Provided for convenience.
 * @param v the vector
 * @return the mean
 */
template<class T>
inline T uMean(const std::vector<T> & v)
{
	return uMean(v.data(), v.size());
}

/**
 * Compute mean squared error between two arrays: mean((x-y).^2).
 * @param x the first array
 * @param sizeX the size of the array x
 * @param y the second array
 * @param sizeY the size of the array y (must be same size as x)
 * @return the mean squared error (return -1 if error cannot be computed)
 */
template<class T>
inline T uMeanSquaredError(const T * x, unsigned int sizeX, const T * y, unsigned int sizeY)
{
	T sum = 0;
	if(x && y && sizeX == sizeY)
	{
		for(unsigned int i=0; i<sizeX; ++i)
		{
			T diff = x[i]-y[i];
			sum += diff*diff;
		}
		return sum/(T)sizeX;
	}
	return (T)-1;
}

/**
 * Compute mean squared error between two arrays: mean((x-y).^2).
 * @param x the first array
 * @param y the second array (must be same size as x)
 * @return the mean squared error (return -1 if error cannot be computed)
 */
template<class T>
inline T uMeanSquaredError(const std::vector<T> & x, const std::vector<T> & y)
{
	return uMeanSquaredError(x.data(), x.size(), y.data(), y.size());
}

/**
 * Compute the variance of an array.
 * @param v the array
 * @param size the size of the array
 * @param meanV the mean of the array
 * @return the variance
 * @see mean()
 */
template<class T>
inline T uVariance(const T * v, unsigned int size, T meanV)
{
	T buf = 0;
	if(v && size>1)
	{
		float sum = 0;
		for(unsigned int i=0; i<size; ++i)
		{
			sum += (v[i]-meanV)*(v[i]-meanV);
		}
		buf = sum/(size-1);
	}
	return buf;
}

/**
 * Get the variance of a list. Provided for convenience.
 * @param list the list
 * @param m the mean of the list
 * @return the variance
 * @see mean()
 */
template<class T>
inline T uVariance(const std::list<T> & list, const T & m)
{
	T buf = 0;
	if(list.size()>1)
	{
		float sum = 0;
		for(typename std::list<T>::const_iterator i=list.begin(); i!=list.end(); ++i)
		{
			sum += (*i-m)*(*i-m);
		}
		buf = sum/(list.size()-1);
	}
	return buf;
}

/**
 * Compute the variance of an array.
 * @param v the array
 * @param size the size of the array
 * @return the variance
 */
template<class T>
inline T uVariance(const T * v, unsigned int size)
{
	T m = uMean(v, size);
	return uVariance(v, size, m);
}

/**
 * Get the variance of a vector. Provided for convenience.
 * @param v the vector
 * @param m the mean of the vector
 * @return the variance
 * @see mean()
 */
template<class T>
inline T uVariance(const std::vector<T> & v, const T & m)
{
	return uVariance(v.data(), v.size(), m);
}

/**
 * Get the squared norm of the vector : return x1*x1 + x2*x2 + x3*x3 + ...
 * @return the squared norm of the vector
 */
template<class T>
inline T uNormSquared(const std::vector<T> & v)
{
	float sum = 0.0f;
	for(unsigned int i=0; i<v.size(); ++i)
	{
		sum += v[i]*v[i];
	}
	return sum;
}

/**
 * Get the norm of the vector : return sqrt(x1*x1 + x2*x2 + x3*x3 + ...)
 * @return the norm of the vector
 */
template<class T>
inline T uNorm(const std::vector<T> & v)
{
	return std::sqrt(uNormSquared(v));
}

/**
 * Get the squared norm of the vector : return x1*x1 + x2*x2
 * @return the squared norm of the vector
 */
template<class T>
inline T uNormSquared(const T & x1, const T & x2)
{
	return x1*x1 + x2*x2;
}

/**
 * Get the norm of the vector : return sqrt(x1*x1 + x2*x2 + x3*x3)
 * @return the norm of the vector
 */
template<class T>
inline T uNorm(const T & x1, const T & x2)
{
	return std::sqrt(uNormSquared(x1, x2));
}

/**
 * Get the squared norm of the vector : return x1*x1 + x2*x2 + x3*x3
 * @return the squared norm of the vector
 */
template<class T>
inline T uNormSquared(const T & x1, const T & x2, const T & x3)
{
	return x1*x1 + x2*x2 + x3*x3;
}

/**
 * Get the norm of the vector : return sqrt(x1*x1 + x2*x2 + x3*x3)
 * @return the norm of the vector
 */
template<class T>
inline T uNorm(const T & x1, const T & x2, const T & x3)
{
	return std::sqrt(uNormSquared(x1, x2, x3));
}

/**
 * Normalize the vector : [x1 x2 x3 ...] ./ uNorm([x1 x2 x3 ...])
 * @return the vector normalized
 */
template<class T>
inline std::vector<T> uNormalize(const std::vector<T> & v)
{
	float norm = uNorm(v);
	if(norm == 0)
	{
		return v;
	}
	else
	{
		std::vector<T> r(v.size());
		for(unsigned int i=0; i<v.size(); ++i)
		{
			r[i] = v[i]/norm;
		}
		return r;
	}
}

/**
 * Find all local maxima.
 */
template<class T>
inline std::list<unsigned int> uLocalMaxima(const T * v, unsigned int size)
{
	std::list<unsigned int> maxima;
	if(size)
	{
		for(unsigned int i=0; i<size; ++i)
		{
			if(i == 0)
			{
				// first item
				if((i+1 < size && v[i] > v[i+1]) ||
					i+1 >= size)
				{
					maxima.push_back(i);
				}
			}
			else if(i == size - 1)
			{
				//last item
				if((i >= 1 && v[i] > v[i-1]) ||
					i == 0)
				{
					maxima.push_back(i);
				}
			}
			else
			{
				//all others, check previous and next
				if(v[i] > v[i-1] && v[i] > v[i+1])
				{
					maxima.push_back(i);
				}
			}
		}
	}
	return maxima;
}

/**
 * Find all local maxima.
 */
template<class T>
inline std::list<unsigned int> uLocalMaxima(const std::vector<T> & v)
{
	return uLocalMaxima(v.data(), v.size());
}

/**
 * Enum of cross matching methods (cross-correlation, cross-covariance) :
 * UXCorrRaw, UXCorrBiased, UXCorrUnbiased, UXCorrCoeff, UXCovRaw, UXCovBiased, UXCovUnbiased, UXCovCoeff.
 */
enum UXMatchMethod{UXCorrRaw, UXCorrBiased, UXCorrUnbiased, UXCorrCoeff, UXCovRaw, UXCovBiased, UXCovUnbiased, UXCovCoeff};

/**
 * Do a full cross-correlation or cross-covariance between 2 arrays.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @param method see UXMatchMethod
 * @return the resulting correlation/covariance vector of size = sizeA + sizeB - 1
 */
template<class T>
inline std::vector<T> uXMatch(const T * vA, const T * vB, unsigned int sizeA, unsigned int sizeB, UXMatchMethod method)
{
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
		return std::vector<T>();
	}

	std::vector<T> result(sizeA + sizeB - 1);

	T meanA = 0;
	T meanB = 0;
	if(method > UXCorrCoeff)
	{
		meanA = uMean(vA, sizeA);
		meanB = uMean(vB, sizeB);
	}

	T den = 1;
	if(method == UXCorrCoeff || method == UXCovCoeff)
	{
		den = std::sqrt(uSumSquared(vA, sizeA, meanA) * uSumSquared(vB, sizeB, meanB));
	}
	else if(method == UXCorrBiased || method == UXCovBiased)
	{
		den = (T)std::max(sizeA, sizeB);
	}

	if(sizeA == sizeB)
	{
		T resultA;
		T resultB;

		int posA;
		int posB;
		unsigned int j;

		// Optimization, filling two results at once
		for(unsigned int i=0; i<sizeA; ++i)
		{
			if(method == UXCorrUnbiased || method == UXCovUnbiased)
			{
				den = 0;
			}

			posA = sizeA - i - 1;
			posB = sizeB - i - 1;
			resultA = 0;
			resultB = 0;
			for(j=0; (j + posB) < sizeB && (j + posA) < sizeA; ++j)
			{
				resultA += (vA[j] - meanA) * (vB[j + posB] - meanB);
				resultB += (vA[j + posA] - meanA) * (vB[j] - meanB);
				if(method == UXCorrUnbiased || method == UXCovUnbiased)
				{
					++den;
				}
			}

			result[i] = resultA / den;
			result[result.size()-1 -i] = resultB / den;
		}
	}
	else
	{
		for(unsigned int i=0; i<result.size(); ++i)
		{
			if(method == UXCorrUnbiased || method == UXCovUnbiased)
			{
				den = 0;
			}

			int posB = sizeB - i - 1;
			T r = 0;
			if(posB >= 0)
			{
				for(unsigned int j=0; (j + posB) < sizeB && j < sizeA; ++j)
				{
					r += (vA[j] - meanA) * (vB[j + posB] - meanB);
					if(method == UXCorrUnbiased || method == UXCovUnbiased)
					{
						++den;
					}
				}
			}
			else
			{
				int posA = posB*-1;
				for(unsigned int i=0; (i+posA) < sizeA && i < sizeB; ++i)
				{
					r += (vA[i+posA] - meanA) * (vB[i] - meanB);
					if(method == UXCorrUnbiased || method == UXCovUnbiased)
					{
						++den;
					}
				}
			}

			result[i] = r / den;
		}
	}

	return result;
}

/**
 * Do a full cross-correlation or cross-covariance between 2 arrays.
 * @param vA the first array
 * @param vB the second array
 * @param method see UXMatchMethod
 * @return the resulting correlation/covariance vector of size = sizeA + sizeB - 1
 */
template<class T>
inline std::vector<T> uXMatch(const std::vector<T> & vA, const std::vector<T> & vB, UXMatchMethod method)
{
	return uXMatch(vA.data(), vB.data(), vA.size(), vB.size(), method);
}

/**
 * Do a cross correlation between 2 arrays at a specified index.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @param index the index to correlate
 * @param method see UXMatchMethod
 * @return the resulting correlation value
 */
template<class T>
inline T uXMatch(const T * vA, const T * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index, UXMatchMethod method)
{
	T result = 0;
	if(!vA || !vB || sizeA == 0 || sizeB == 0)
	{
		return result;
	}

	T meanA = 0;
	T meanB = 0;
	if(method > UXCorrCoeff)
	{
		meanA = uMean(vA, sizeA);
		meanB = uMean(vB, sizeB);
	}
	unsigned int size = sizeA + sizeB - 1;

	T den = 1;
	if(method == UXCorrCoeff || method == UXCovCoeff)
	{
		den = std::sqrt(uSumSquared(vA, sizeA, meanA) * uSumSquared(vB, sizeB, meanB));
	}
	else if(method == UXCorrBiased || method == UXCovBiased)
	{
		den = (T)std::max(sizeA, sizeB);
	}
	else if(method == UXCorrUnbiased || method == UXCovUnbiased)
	{
		den = 0;
	}

	if(index < size)
	{
		int posB = sizeB - index - 1;
		unsigned int i;
		if(posB >= 0)
		{
			for(i=0; (i + posB) < sizeB && i < sizeA; ++i)
			{
				result += (vA[i] - meanA) * (vB[i + posB] - meanB);
				if(method == UXCorrUnbiased || method == UXCovUnbiased)
				{
					++den;
				}
			}
		}
		else
		{
			int posA = posB*-1;
			for(i=0; (i+posA) < sizeA && i < sizeB; ++i)
			{
				result += (vA[i+posA] - meanA) * (vB[i] - meanB);
				if(method == UXCorrUnbiased || method == UXCovUnbiased)
				{
					++den;
				}
			}
		}
	}
	return result / den;
}

/**
 * Do a cross correlation between 2 arrays at a specified index.
 * @param vA the first array
 * @param vB the second array
 * @param sizeA the size of the first array
 * @param sizeB the size of the second array
 * @param index the index to correlate
 * @param method see UXMatchMethod
 * @return the resulting correlation value
 */
template<class T>
inline T uXMatch(const std::vector<T> & vA, const std::vector<T> & vB, unsigned int index, UXMatchMethod method)
{
	return uXMatch(vA.data(), vB.data(), vA.size(), vB.size(), index, method);
}

/**
 * Return Hamming window of length L.
 * @param L the window length
 * @return the Hamming window (values are between 0 and 1)
 */
inline std::vector<float> uHamming(unsigned int L)
{
	std::vector<float> w(L);
	unsigned int N = L-1;
	float pi = 3.14159265f;
	for(unsigned int n=0; n<N; ++n)
	{
		w[n] = 0.54f-0.46f*std::cos(2.0f*pi*float(n)/float(N));
	}
	return w;
}

template <typename T>
bool uIsInBounds(const T& value, const T& low, const T& high)
{
	return uIsFinite(value) && !(value < low) && !(value >= high);
}

#endif // UMATH_H
