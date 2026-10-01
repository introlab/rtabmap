/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 * Copyright 2011-2026  Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

/** \mainpage nanoflann C++ API documentation
 *  nanoflann is a C++ header-only library for building KD-Trees, mostly
 *  optimized for 2D or 3D point clouds.
 *
 *  nanoflann does not require compiling or installing, just an
 *  #include <nanoflann.hpp> in your code.
 *
 *  Macros that can be defined by the user to configure nanoflann:
 *  - NANOFLANN_NO_THREADS: If defined, multithreading is disabled, so the
 *    library can be used without linking against a threads library. Requesting a
 *    multi-threaded index build (`n_thread_build != 1`) then throws.
 *  - NANOFLANN_FIRST_MATCH: If defined, in case of a tie in distances the item
 *    with the smallest index will be returned.
 *  - NANOFLANN_NODE_ALIGNMENT: The memory alignment, in bytes, for kd-tree
 *    nodes. Default: 16.
 *
 *  Macros defined internally by nanoflann (not meant to be set by the user):
 *  - NANOFLANN_RESTRICT: Expands to the compiler-specific `restrict` pointer
 *    qualifier (`__restrict__`, `__restrict`) when available, empty otherwise.
 *  - NANOFLANN_NODISCARD: Expands to `[[nodiscard]]` when the compiler supports
 *    it, empty otherwise.
 *  - NANOFLANN_VERSION: Library version as 0xMmP (M=Major, m=minor, P=patch).
 *
 *  See the [README](https://github.com/jlblancoc/nanoflann#readme) for usage
 *  details and examples.
 *
 *  See:
 *   - [Online README](https://github.com/jlblancoc/nanoflann)
 *   - [C++ API documentation](https://jlblancoc.github.io/nanoflann/)
 */

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>  // std::chrono (async incremental index polling)
#include <cmath>  // for abs()
#include <condition_variable>  // rebuild worker of the async incremental index
#include <cstdint>
#include <cstdio>  // snprintf
#include <cstdlib>  // for abs()
#include <exception>  // std::exception_ptr (async incremental index)
#include <functional>  // std::reference_wrapper
#include <future>
#include <istream>
#include <limits>  // std::numeric_limits
#include <memory>  // std::unique_ptr (async incremental index)
#include <mutex>  // rebuild worker of the async incremental index
#include <new>  // placement new (incremental index node pool)
#include <ostream>
#include <stack>
#include <stdexcept>
#include <thread>
#include <type_traits>  // std::is_trivially_destructible
#include <unordered_map>
#include <vector>

/** Library version as a decimal string "MAJOR.MINOR.PATCH" */
#define NANOFLANN_VERSION_STRING "1.12.1"
/** Library version: 0xMMmmPP (MM=Major, mm=minor, PP=patch) */
#define NANOFLANN_VERSION 0x010C01

// Avoid conflicting declaration of min/max macros in Windows headers
#if !defined(NOMINMAX) && (defined(_WIN32) || defined(_WIN32_) || defined(WIN32) || defined(_WIN64))
#define NOMINMAX
#ifdef max
#undef max
#undef min
#endif
#endif
// Avoid conflicts with X11 headers
#ifdef None
#undef None
#endif

// Handle restricted pointers
#if defined(__GNUC__) || defined(__clang__)
#define NANOFLANN_RESTRICT __restrict__
#elif defined(_MSC_VER)
#define NANOFLANN_RESTRICT __restrict
#else
#define NANOFLANN_RESTRICT
#endif

// [[nodiscard]] support
#if defined(__has_cpp_attribute) && __has_cpp_attribute(nodiscard)
#define NANOFLANN_NODISCARD [[nodiscard]]
#else
#define NANOFLANN_NODISCARD
#endif

// [[fallthrough]] support for intentional switch fall-throughs
#if defined(__has_cpp_attribute) && __has_cpp_attribute(fallthrough)
#define NANOFLANN_FALLTHROUGH [[fallthrough]]
#else
#define NANOFLANN_FALLTHROUGH
#endif

// Memory alignment of KD-tree nodes:
#ifndef NANOFLANN_NODE_ALIGNMENT
#define NANOFLANN_NODE_ALIGNMENT 16
#endif

namespace nanoflann
{
/** @addtogroup nanoflann_grp nanoflann C++ library for KD-trees
 *  @{ */

/** the PI constant (required to avoid MSVC missing symbols) */
template <typename T>
constexpr T pi_const()
{
    return static_cast<T>(3.14159265358979323846);
}

/**
 * Traits if object is resizable and assignable (typically has a resize | assign
 * method)
 */
template <typename T, typename = int>
struct has_resize : std::false_type
{
};

template <typename T>
struct has_resize<T, decltype((void)std::declval<T>().resize(1), 0)> : std::true_type
{
};

template <typename T, typename = int>
struct has_assign : std::false_type
{
};

template <typename T>
struct has_assign<T, decltype((void)std::declval<T>().assign(1, 0), 0)> : std::true_type
{
};

/**
 * Free function to resize a resizable object
 */
template <typename Container>
inline typename std::enable_if<has_resize<Container>::value, void>::type resize(
    Container& c, const size_t nElements)
{
    c.resize(nElements);
}

/**
 * Free function that has no effects on non resizable containers (e.g.
 * std::array) It raises an exception if the expected size does not match
 */
template <typename Container>
inline typename std::enable_if<!has_resize<Container>::value, void>::type resize(
    Container& c, const size_t nElements)
{
    if (nElements != c.size()) throw std::logic_error("Attempt to resize a fixed size container.");
}

/**
 * Free function to assign to a container
 */
template <typename Container, typename T>
inline typename std::enable_if<has_assign<Container>::value, void>::type assign(
    Container& c, const size_t nElements, const T& value)
{
    c.assign(nElements, value);
}

/**
 * Free function to assign to a std::array
 */
template <typename Container, typename T>
inline typename std::enable_if<!has_assign<Container>::value, void>::type assign(
    Container& c, const size_t nElements, const T& value)
{
    for (size_t i = 0; i < nElements; i++) c[i] = value;
}

/** operator "<" for std::sort() */
struct IndexDist_Sorter
{
    /** PairType will be typically: ResultItem<IndexType,DistanceType> */
    template <typename PairType>
    bool operator()(const PairType& p1, const PairType& p2) const
    {
        return p1.second < p2.second;
    }
};

/**
 * Each result element in RadiusResultSet. Note that distances and indices
 * are named `first` and `second` to keep backward-compatibility with the
 * `std::pair<>` type used in the past. In contrast, this structure is ensured
 * to be `std::is_standard_layout` so it can be used in wrappers to other
 * languages.
 * See: https://github.com/jlblancoc/nanoflann/issues/166
 */
template <typename IndexType = size_t, typename DistanceType = double>
struct ResultItem
{
    ResultItem() = default;
    ResultItem(const IndexType index, const DistanceType distance) : first(index), second(distance)
    {
    }

    IndexType    first;  //!< Index of the sample in the dataset
    DistanceType second;  //!< Distance from sample to query point
};

namespace detail
{
/** Insert (dist, index) into a sorted result buffer (dists, indices) of the
 *  given capacity, keeping ascending distance order.  Shared by KNNResultSet
 *  and RKNNResultSet, which are otherwise byte-for-byte identical.
 *  Always returns true (caller should continue searching). */
template <typename DistanceType, typename IndexType, typename CountType>
bool addPointToSortedResultSet(
    DistanceType* dists, IndexType* indices, CountType& count, CountType capacity,
    DistanceType dist, IndexType index)
{
    CountType i;
    for (i = count; i > 0; --i)
    {
#ifdef NANOFLANN_FIRST_MATCH
        if ((dists[i - 1] > dist) || ((dist == dists[i - 1]) && (indices[i - 1] > index)))
        {
#else
        if (dists[i - 1] > dist)
        {
#endif
            if (i < capacity)
            {
                dists[i]   = dists[i - 1];
                indices[i] = indices[i - 1];
            }
        }
        else
            break;
    }
    if (i < capacity)
    {
        dists[i]   = dist;
        indices[i] = index;
    }
    if (count < capacity) count++;
    return true;
}
}  // namespace detail

/** @addtogroup result_sets_grp Result set classes
 *  @{ */

/** Result set for KNN searches (N-closest neighbors) */
template <typename _DistanceType, typename _IndexType = size_t, typename _CountType = size_t>
class KNNResultSet
{
   public:
    using DistanceType = _DistanceType;
    using IndexType    = _IndexType;
    using CountType    = _CountType;

   private:
    IndexType*    indices;
    DistanceType* dists;
    CountType     capacity;
    CountType     count;

   public:
    explicit KNNResultSet(CountType capacity_)
        : indices(nullptr), dists(nullptr), capacity(capacity_), count(0)
    {
    }

    void init(IndexType* indices_, DistanceType* dists_)
    {
        indices = indices_;
        dists   = dists_;
        count   = 0;
    }

    NANOFLANN_NODISCARD CountType size() const noexcept { return count; }
    NANOFLANN_NODISCARD bool      empty() const noexcept { return count == 0; }
    NANOFLANN_NODISCARD bool      full() const noexcept { return count == capacity; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are
     * sufficient
     */
    bool addPoint(DistanceType dist, IndexType index)
    {
        return detail::addPointToSortedResultSet(dists, indices, count, capacity, dist, index);
    }

    //! Returns the worst distance among found solutions if the search result is
    //! full, or the maximum possible distance, if not full yet.
    NANOFLANN_NODISCARD DistanceType worstDist() const noexcept
    {
        return (count < capacity || !count) ? std::numeric_limits<DistanceType>::max()
                                            : dists[count - 1];
    }

    void sort()
    {
        // already sorted
    }
};

/** Result set for RKNN searches (N-closest neighbors with a maximum radius) */
template <typename _DistanceType, typename _IndexType = size_t, typename _CountType = size_t>
class RKNNResultSet
{
   public:
    using DistanceType = _DistanceType;
    using IndexType    = _IndexType;
    using CountType    = _CountType;

   private:
    IndexType*    indices;
    DistanceType* dists;
    CountType     capacity;
    CountType     count;
    DistanceType  maximumSearchDistanceSquared;

   public:
    explicit RKNNResultSet(CountType capacity_, DistanceType maximumSearchDistanceSquared_)
        : indices(nullptr),
          dists(nullptr),
          capacity(capacity_),
          count(0),
          maximumSearchDistanceSquared(maximumSearchDistanceSquared_)
    {
    }

    void init(IndexType* indices_, DistanceType* dists_)
    {
        indices = indices_;
        dists   = dists_;
        count   = 0;
        if (capacity) dists[capacity - 1] = maximumSearchDistanceSquared;
    }

    NANOFLANN_NODISCARD CountType size() const noexcept { return count; }
    NANOFLANN_NODISCARD bool      empty() const noexcept { return count == 0; }
    NANOFLANN_NODISCARD bool      full() const noexcept { return count == capacity; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are
     * sufficient
     */
    bool addPoint(DistanceType dist, IndexType index)
    {
        return detail::addPointToSortedResultSet(dists, indices, count, capacity, dist, index);
    }

    //! Returns the worst distance among found solutions if the search result is
    //! full, or the maximum possible distance, if not full yet.
    NANOFLANN_NODISCARD DistanceType worstDist() const noexcept
    {
        return (count < capacity || !count) ? maximumSearchDistanceSquared : dists[count - 1];
    }

    void sort()
    {
        // already sorted
    }
};

/**
 * A result-set class used when performing a radius based search.
 */
template <typename _DistanceType, typename _IndexType = size_t>
class RadiusResultSet
{
   public:
    using DistanceType = _DistanceType;
    using IndexType    = _IndexType;

   public:
    const DistanceType radius;

    std::vector<ResultItem<IndexType, DistanceType>>& m_indices_dists;

    explicit RadiusResultSet(
        DistanceType radius_, std::vector<ResultItem<IndexType, DistanceType>>& indices_dists)
        : radius(radius_), m_indices_dists(indices_dists)
    {
        init();
    }

    void init() { clear(); }
    void clear() { m_indices_dists.clear(); }

    NANOFLANN_NODISCARD size_t size() const noexcept { return m_indices_dists.size(); }
    NANOFLANN_NODISCARD bool   empty() const noexcept { return m_indices_dists.empty(); }
    NANOFLANN_NODISCARD bool   full() const noexcept { return true; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are
     * sufficient
     */
    bool addPoint(DistanceType dist, IndexType index)
    {
        if (dist < radius) m_indices_dists.emplace_back(index, dist);
        return true;
    }

    NANOFLANN_NODISCARD DistanceType worstDist() const noexcept { return radius; }

    /**
     * Find the worst result (farthest neighbor) without copying or sorting
     * Pre-conditions: size() > 0
     */
    ResultItem<IndexType, DistanceType> worst_item() const
    {
        if (m_indices_dists.empty())
            throw std::runtime_error(
                "Cannot invoke RadiusResultSet::worst_item() on "
                "an empty list of results.");
        auto it =
            std::max_element(m_indices_dists.begin(), m_indices_dists.end(), IndexDist_Sorter());
        return *it;
    }

    void sort() { std::sort(m_indices_dists.begin(), m_indices_dists.end(), IndexDist_Sorter()); }
};

/**
 * A result-set class used when collecting all points contained within an
 * axis-aligned bounding box (see findWithinBox()). Distances are not used;
 * matching point indices are appended to the user-provided vector.
 */
template <typename _IndexType = size_t>
class BoxResultSet
{
   public:
    using IndexType = _IndexType;

    std::vector<IndexType>& m_indices;

    explicit BoxResultSet(std::vector<IndexType>& indices) : m_indices(indices)
    {
        m_indices.clear();
    }

    NANOFLANN_NODISCARD size_t size() const noexcept { return m_indices.size(); }
    NANOFLANN_NODISCARD bool   empty() const noexcept { return m_indices.empty(); }
    NANOFLANN_NODISCARD bool   full() const noexcept { return true; }

    /** Called for each point found inside the query box. The distance argument
     *  is unused (always 0 for a box query). @return always true (keep going). */
    template <typename DistanceType>
    bool addPoint(DistanceType /*dist*/, IndexType index)
    {
        m_indices.push_back(index);
        return true;
    }

    void sort() { std::sort(m_indices.begin(), m_indices.end()); }
};

/** @} */

/** @addtogroup loadsave_grp Load/save auxiliary functions
 * @{ */
template <typename T>
void save_value(std::ostream& stream, const T& value)
{
    stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
void save_value(std::ostream& stream, const std::vector<T>& value)
{
    size_t size = value.size();
    stream.write(reinterpret_cast<const char*>(&size), sizeof(size_t));
    stream.write(reinterpret_cast<const char*>(value.data()), sizeof(T) * size);
}

template <typename T>
void load_value(std::istream& stream, T& value)
{
    stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}

template <typename T>
void load_value(std::istream& stream, std::vector<T>& value)
{
    size_t size;
    stream.read(reinterpret_cast<char*>(&size), sizeof(size_t));
    value.resize(size);
    stream.read(reinterpret_cast<char*>(value.data()), sizeof(T) * size);
}
/** @} */

/** @addtogroup metric_grp Metric (distance) classes
 * @{ */

struct Metric
{
};

/** Manhattan distance functor (generic version, optimized for
 * high-dimensionality data sets). Corresponding distance traits:
 * nanoflann::metric_L1
 *
 * \tparam T Type of the elements (e.g. double, float, uint8_t)
 * \tparam DataSource Source of the data, i.e. where the vectors are stored
 * \tparam _DistanceType Type of distance variables (must be signed)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <class T, class DataSource, typename _DistanceType = T, typename IndexType = size_t>
struct L1_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    L1_Adaptor(const DataSource& _data_source) : data_source(_data_source) {}

    inline DistanceType evalMetric(
        const T* NANOFLANN_RESTRICT a, const IndexType b_idx, size_t size) const
    {
        DistanceType result  = DistanceType();
        const size_t multof4 = (size >> 2) << 2;  // largest multiple of 4
        size_t       d;

        for (d = 0; d < multof4; d += 4)
        {
            const DistanceType diff0 = std::abs(a[d + 0] - data_source.kdtree_get_pt(b_idx, d + 0));
            const DistanceType diff1 = std::abs(a[d + 1] - data_source.kdtree_get_pt(b_idx, d + 1));
            const DistanceType diff2 = std::abs(a[d + 2] - data_source.kdtree_get_pt(b_idx, d + 2));
            const DistanceType diff3 = std::abs(a[d + 3] - data_source.kdtree_get_pt(b_idx, d + 3));
            /* Parentheses break dependency chain: */
            result += (diff0 + diff1) + (diff2 + diff3);
        }
        /* Process last 0-3 components. Unrolled loop with fall-through switch.
         */
        switch (size - multof4)
        {
            case 3:
                result += std::abs(a[d + 2] - data_source.kdtree_get_pt(b_idx, d + 2));
                NANOFLANN_FALLTHROUGH;
            case 2:
                result += std::abs(a[d + 1] - data_source.kdtree_get_pt(b_idx, d + 1));
                NANOFLANN_FALLTHROUGH;
            case 1:
                result += std::abs(a[d + 0] - data_source.kdtree_get_pt(b_idx, d + 0));
                NANOFLANN_FALLTHROUGH;
            case 0:
                break;
        }
        return result;
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t) const
    {
        return std::abs(a - b);
    }
};

/** **Squared** Euclidean distance functor (generic version, optimized for
 * high-dimensionality data sets). Corresponding distance traits:
 * nanoflann::metric_L2
 *
 * \tparam T Type of the elements (e.g. double, float, uint8_t)
 * \tparam DataSource Source of the data, i.e. where the vectors are stored
 * \tparam _DistanceType Type of distance variables (must be signed)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <class T, class DataSource, typename _DistanceType = T, typename IndexType = size_t>
struct L2_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    L2_Adaptor(const DataSource& _data_source) : data_source(_data_source) {}

    inline DistanceType evalMetric(
        const T* NANOFLANN_RESTRICT a, const IndexType b_idx, size_t size) const
    {
        DistanceType result  = DistanceType();
        const size_t multof4 = (size >> 2) << 2;  // largest multiple of 4
        size_t       d;

        for (d = 0; d < multof4; d += 4)
        {
            const DistanceType diff0 = a[d + 0] - data_source.kdtree_get_pt(b_idx, d + 0);
            const DistanceType diff1 = a[d + 1] - data_source.kdtree_get_pt(b_idx, d + 1);
            const DistanceType diff2 = a[d + 2] - data_source.kdtree_get_pt(b_idx, d + 2);
            const DistanceType diff3 = a[d + 3] - data_source.kdtree_get_pt(b_idx, d + 3);
            /* Parentheses break dependency chain: */
            result += (diff0 * diff0 + diff1 * diff1) + (diff2 * diff2 + diff3 * diff3);
        }
        /* Process last 0-3 components. Unrolled loop with fall-through switch.
         */
        DistanceType diff;
        switch (size - multof4)
        {
            case 3:
                diff = a[d + 2] - data_source.kdtree_get_pt(b_idx, d + 2);
                result += diff * diff;
                NANOFLANN_FALLTHROUGH;
            case 2:
                diff = a[d + 1] - data_source.kdtree_get_pt(b_idx, d + 1);
                result += diff * diff;
                NANOFLANN_FALLTHROUGH;
            case 1:
                diff = a[d + 0] - data_source.kdtree_get_pt(b_idx, d + 0);
                result += diff * diff;
                NANOFLANN_FALLTHROUGH;
            case 0:
                break;
        }
        return result;
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t) const
    {
        auto diff = a - b;
        return diff * diff;
    }
};

/** **Squared** Euclidean (L2) distance functor (suitable for low-dimensionality
 * datasets, like 2D or 3D point clouds) Corresponding distance traits:
 * nanoflann::metric_L2_Simple
 *
 * \tparam T Type of the elements (e.g. double, float, uint8_t)
 * \tparam DataSource Source of the data, i.e. where the vectors are stored
 * \tparam _DistanceType Type of distance variables (must be signed)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <class T, class DataSource, typename _DistanceType = T, typename IndexType = size_t>
struct L2_Simple_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    L2_Simple_Adaptor(const DataSource& _data_source) : data_source(_data_source) {}

    inline DistanceType evalMetric(const T* a, const IndexType b_idx, size_t size) const
    {
        DistanceType result = DistanceType();
        for (size_t i = 0; i < size; ++i)
        {
            const DistanceType diff = a[i] - data_source.kdtree_get_pt(b_idx, i);
            result += diff * diff;
        }
        return result;
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t) const
    {
        auto diff = a - b;
        return diff * diff;
    }
};

/** SO2 distance functor
 *  Corresponding distance traits: nanoflann::metric_SO2
 *
 * \tparam T Type of the elements (e.g. double, float, uint8_t)
 * \tparam DataSource Source of the data, i.e. where the vectors are stored
 * \tparam _DistanceType Type of distance variables (must be signed) (e.g.
 * float, double) orientation is constrained to be in [-pi, pi]
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <class T, class DataSource, typename _DistanceType = T, typename IndexType = size_t>
struct SO2_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    SO2_Adaptor(const DataSource& _data_source) : data_source(_data_source) {}

    inline DistanceType evalMetric(const T* a, const IndexType b_idx, size_t size) const
    {
        return accum_dist(a[size - 1], data_source.kdtree_get_pt(b_idx, size - 1), size - 1);
    }

    /** Returns the absolute shortest angular distance between a and b,
     *  assuming both are in [-pi, pi].  The result is in [0, pi], which
     *  satisfies the non-negativity requirement of a kd-tree metric and
     *  gives correct nearest-neighbour pruning.
     */
    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t) const
    {
        DistanceType       diff = static_cast<DistanceType>(b) - static_cast<DistanceType>(a);
        const DistanceType PI   = pi_const<DistanceType>();
        if (diff > PI)
            diff -= 2 * PI;
        else if (diff < -PI)
            diff += 2 * PI;
        return diff < DistanceType(0) ? -diff : diff;  // abs without <cmath> dependency
    }
};

/** SO3 distance functor (Uses L2_Simple)
 *  Corresponding distance traits: nanoflann::metric_SO3
 *
 * \tparam T Type of the elements (e.g. double, float, uint8_t)
 * \tparam DataSource Source of the data, i.e. where the vectors are stored
 * \tparam _DistanceType Type of distance variables (must be signed) (e.g.
 * float, double)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <class T, class DataSource, typename _DistanceType = T, typename IndexType = size_t>
struct SO3_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    L2_Simple_Adaptor<T, DataSource, DistanceType, IndexType> distance_L2_Simple;

    SO3_Adaptor(const DataSource& _data_source) : distance_L2_Simple(_data_source) {}

    inline DistanceType evalMetric(const T* a, const IndexType b_idx, size_t size) const
    {
        return distance_L2_Simple.evalMetric(a, b_idx, size);
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t idx) const
    {
        return distance_L2_Simple.accum_dist(a, b, idx);
    }
};

/** Metaprogramming helper traits class for the L1 (Manhattan) metric */
struct metric_L1 : public Metric
{
    template <class T, class DataSource, typename IndexType = size_t>
    struct traits
    {
        using distance_t = L1_Adaptor<T, DataSource, T, IndexType>;
    };
};
/** Metaprogramming helper traits class for the L2 (Euclidean) **squared**
 * distance metric */
struct metric_L2 : public Metric
{
    template <class T, class DataSource, typename IndexType = size_t>
    struct traits
    {
        using distance_t = L2_Adaptor<T, DataSource, T, IndexType>;
    };
};
/** Metaprogramming helper traits class for the L2_simple (Euclidean)
 * **squared** distance metric */
struct metric_L2_Simple : public Metric
{
    template <class T, class DataSource, typename IndexType = size_t>
    struct traits
    {
        using distance_t = L2_Simple_Adaptor<T, DataSource, T, IndexType>;
    };
};
/** Metaprogramming helper traits class for the SO3_InnerProdQuat metric */
struct metric_SO2 : public Metric
{
    template <class T, class DataSource, typename IndexType = size_t>
    struct traits
    {
        using distance_t = SO2_Adaptor<T, DataSource, T, IndexType>;
    };
};
/** Metaprogramming helper traits class for the SO3_InnerProdQuat metric */
struct metric_SO3 : public Metric
{
    template <class T, class DataSource, typename IndexType = size_t>
    struct traits
    {
        using distance_t = SO3_Adaptor<T, DataSource, T, IndexType>;
    };
};

/** @} */

/** @addtogroup param_grp Parameter structs
 * @{ */

enum class KDTreeSingleIndexAdaptorFlags
{
    None                  = 0,
    SkipInitialBuildIndex = 1
};

inline std::underlying_type<KDTreeSingleIndexAdaptorFlags>::type operator&(
    KDTreeSingleIndexAdaptorFlags lhs, KDTreeSingleIndexAdaptorFlags rhs)
{
    using underlying = typename std::underlying_type<KDTreeSingleIndexAdaptorFlags>::type;
    return static_cast<underlying>(lhs) & static_cast<underlying>(rhs);
}

/** Returns true if \a f has the given \a flag bit set.
 *  Prefer this over the raw operator& in boolean contexts. */
inline bool has_flag(KDTreeSingleIndexAdaptorFlags f, KDTreeSingleIndexAdaptorFlags flag)
{
    return (f & flag) != 0;
}

/**  Parameters (see README.md) */
struct KDTreeSingleIndexAdaptorParams
{
    KDTreeSingleIndexAdaptorParams(
        size_t                        _leaf_max_size  = 10,
        KDTreeSingleIndexAdaptorFlags _flags          = KDTreeSingleIndexAdaptorFlags::None,
        unsigned int                  _n_thread_build = 1)
        : leaf_max_size(_leaf_max_size), flags(_flags), n_thread_build(_n_thread_build)
    {
    }

    size_t                        leaf_max_size;
    KDTreeSingleIndexAdaptorFlags flags;
    unsigned int                  n_thread_build;
};

/** Search options for KDTreeSingleIndexAdaptor::findNeighbors() */
struct SearchParameters
{
    SearchParameters(float eps_ = 0, bool sorted_ = true) : eps(eps_), sorted(sorted_) {}

    float eps;  //!< search for eps-approximate neighbors (default: 0)
    bool  sorted;  //!< only for radius search, require neighbors sorted by
                  //!< distance (default: true)
};
/** @} */

/** @addtogroup memalloc_grp Memory allocation
 * @{ */

/**
 * Pooled storage allocator
 *
 * The following routines allow for the efficient allocation of storage in
 * small chunks from a specified pool.  Rather than allowing each structure
 * to be freed individually, an entire pool of storage is freed at once.
 * This method has two advantages over just using malloc() and free().  First,
 * it is far more efficient for allocating small objects, as there is
 * no overhead for remembering all the information needed to free each
 * object or consolidating fragmented memory.  Second, the decision about
 * how long to keep an object is made at the time of allocation, and there
 * is no need to track down all the objects to free them.
 *
 */
class PooledAllocator
{
    static constexpr size_t WORDSIZE  = 16;  // WORDSIZE must >= 8
    static constexpr size_t BLOCKSIZE = 8192;

    /* We maintain memory alignment to word boundaries by requiring that all
        allocations be in multiples of the machine wordsize.  */
    /* Size of machine word in bytes.  Must be power of 2. */
    /* Minimum number of bytes requested at a time from the system.  Must be
     * multiple of WORDSIZE. */

    using Size = size_t;

    Size  remaining_ = 0;  //!< Number of bytes left in current block of storage
    void* base_      = nullptr;  //!< Pointer to base of current block of storage
    void* loc_       = nullptr;  //!< Current location in block to next allocate

    void internal_init()
    {
        remaining_   = 0;
        base_        = nullptr;
        usedMemory   = 0;
        wastedMemory = 0;
    }

   public:
    Size usedMemory   = 0;
    Size wastedMemory = 0;

    /**
        Default constructor. Initializes a new pool.
     */
    PooledAllocator() { internal_init(); }

    /**
     * Destructor. Frees all the memory allocated in this pool.
     */
    ~PooledAllocator() { free_all(); }

    /** Frees all allocated memory chunks */
    void free_all()
    {
        while (base_ != nullptr)
        {
            // Get pointer to prev block
            void* prev = *(static_cast<void**>(base_));
            ::free(base_);
            base_ = prev;
        }
        internal_init();
    }

    /**
     * Returns a pointer to a piece of new memory of the given size in bytes
     * allocated from the pool.
     */
    void* allocateBytes(const size_t req_size)
    {
        /* Round size up to a multiple of wordsize.  The following expression
            only works for WORDSIZE that is a power of 2, by masking last bits
           of incremented size to zero.
         */
        const Size size = (req_size + (WORDSIZE - 1)) & ~(WORDSIZE - 1);

        /* Check whether a new block must be allocated.  Note that the first
           word of a block is reserved for a pointer to the previous block.
         */
        if (size > remaining_)
        {
            wastedMemory += remaining_;

            /* Allocate new storage. */
            const Size blocksize = size > BLOCKSIZE ? size + WORDSIZE : BLOCKSIZE + WORDSIZE;

            // use the standard C malloc to allocate memory
            void* m = ::malloc(blocksize);
            if (!m)
            {
                throw std::bad_alloc();
            }

            /* Fill first word of new block with pointer to previous block. */
            static_cast<void**>(m)[0] = base_;
            base_                     = m;

            remaining_ = blocksize - WORDSIZE;
            loc_       = static_cast<char*>(m) + WORDSIZE;
        }
        void* rloc = loc_;
        loc_       = static_cast<char*>(loc_) + size;
        remaining_ -= size;

        usedMemory += size;

        return rloc;
    }

    /**
     * Allocates (using this pool) a generic type T.
     *
     * Params:
     *     count = number of instances to allocate.
     * Returns: pointer (of type T*) to memory buffer
     */
    template <typename T>
    T* allocate(const size_t count = 1)
    {
        T* mem = static_cast<T*>(this->allocateBytes(sizeof(T) * count));
        return mem;
    }
};
/** @} */

/** @addtogroup nanoflann_metaprog_grp Auxiliary metaprogramming stuff
 * @{ */

/** Used to declare fixed-size arrays when DIM>0, dynamically-allocated vectors
 * when DIM=-1. Fixed size version for a generic DIM:
 */
template <int32_t DIM, typename T>
struct array_or_vector
{
    using type = std::array<T, DIM>;
};
/** Dynamic size version */
template <typename T>
struct array_or_vector<-1, T>
{
    using type = std::vector<T>;
};

/** @} */

/** kd-tree base-class
 *
 * Contains the member functions common to the classes KDTreeSingleIndexAdaptor
 * and KDTreeSingleIndexDynamicAdaptor_.
 *
 * \tparam Derived The name of the class which inherits this class.
 * \tparam DatasetAdaptor The user-provided adaptor, which must be ensured to
 *         have a lifetime equal or longer than the instance of this class.
 * \tparam Distance The distance metric to use, these are all classes derived
 * from nanoflann::Metric
 * \tparam DIM Dimensionality of data points (e.g. 3 for 3D points)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <
    class Derived, typename Distance, class DatasetAdaptor, int32_t DIM = -1,
    typename index_t = uint32_t>
class KDTreeBaseClass
{
   public:
    /** Frees the previously-built index. Automatically called within
     * buildIndex(). */
    void freeIndex(Derived& obj)
    {
        obj.pool_.free_all();
        obj.root_node_           = nullptr;
        obj.size_at_index_build_ = 0;
    }

    using ElementType  = typename Distance::ElementType;
    using DistanceType = typename Distance::DistanceType;
    using IndexType    = index_t;

    /**
     *  Array of indices to vectors in the dataset_.
     */
    std::vector<IndexType> vAcc_;

    using Offset    = typename decltype(vAcc_)::size_type;
    using Size      = typename decltype(vAcc_)::size_type;
    using Dimension = int32_t;

    /*-------------------------------------------------------------------
     * Internal Data Structures
     *
     * "Node" below can be declared with alignas(N) to improve
     * cache friendliness and SIMD load/store performance.
     *
     * The optimal N depends on the underlying hardware:
     *  + Intel x86-64: 16 for SSE, 32 for AVX/AVX2 and 64 for AVX-512
     *  + NVIDIA Jetson: 16 for ARM + NEON and CUDA float4/
     *  To avoid unnecessary padding, the smallest alignment
     *  compatible with a platform's vector width should be chosen.
     * ------------------------------------------------------------------*/
    struct alignas(NANOFLANN_NODE_ALIGNMENT) Node
    {
        /** Union used because a node can be either a LEAF node or a non-leaf
         * node, so both data fields are never used simultaneously */
        union
        {
            struct leaf
            {
                Offset left, right;  //!< Indices of points in leaf node
            } lr;
            struct nonleaf
            {
                Dimension divfeat;  //!< Dimension used for subdivision.
                /// The values used for subdivision.
                DistanceType divlow, divhigh;
            } sub;
        } node_type;

        /** Child nodes (both=nullptr mean its a leaf node) */
        Node *child1 = nullptr, *child2 = nullptr;
    };

    using NodePtr      = Node*;
    using NodeConstPtr = const Node*;

    struct Interval
    {
        ElementType low, high;
    };

    NodePtr root_node_ = nullptr;

    Size leaf_max_size_ = 0;

    /// Number of thread for concurrent tree build
    Size n_thread_build_ = 1;
    /// Number of current points in the dataset
    Size size_ = 0;
    /// Number of points in the dataset when the index was built
    Size      size_at_index_build_ = 0;
    Dimension dim_                 = 0;  //!< Dimensionality of each data point

    /** Define "BoundingBox" as a fixed-size or variable-size container
     * depending on "DIM" */
    using BoundingBox = typename array_or_vector<DIM, Interval>::type;

    /** Define "distance_vector_t" as a fixed-size or variable-size container
     * depending on "DIM" */
    using distance_vector_t = typename array_or_vector<DIM, DistanceType>::type;

    /** The KD-tree used to find neighbors */
    BoundingBox root_bbox_;

    /**
     * Pooled memory allocator.
     *
     * Using a pooled memory allocator is more efficient
     * than allocating memory directly when there is a large
     * number small of memory allocations.
     */
    PooledAllocator pool_;

    /** Returns number of points in dataset  */
    NANOFLANN_NODISCARD Size size(const Derived& obj) const noexcept { return obj.size_; }

    /** Returns the length of each point in the dataset.
     *  For a fixed-size tree (DIM > 0) this is a compile-time constant; under
     *  C++17 the `if constexpr` lets the compiler drop the runtime read of
     *  `dim_` entirely. The C++11 path keeps the equivalent ternary. */
    NANOFLANN_NODISCARD Size veclen(const Derived& obj) const noexcept
    {
#if defined(__cpp_if_constexpr) && __cpp_if_constexpr >= 201606L
        if constexpr (DIM > 0)
        {
            return DIM;
        }
        else
        {
            return obj.dim_;
        }
#else
        return DIM > 0 ? DIM : obj.dim_;
#endif
    }

    /// Helper accessor to the dataset points:
    ElementType dataset_get(const Derived& obj, IndexType element, Dimension component) const
    {
        return obj.dataset_.kdtree_get_pt(element, component);
    }

    /**
     * Computes the index memory usage
     * Returns: memory used by the index
     */
    NANOFLANN_NODISCARD Size usedMemory(const Derived& obj) const
    {
        return obj.pool_.usedMemory + obj.pool_.wastedMemory +
               obj.dataset_.kdtree_get_point_count() *
                   sizeof(IndexType);  // pool memory and vind array memory
    }

    /**
     * Compute the minimum and maximum element values in the specified dimension
     */
    void computeMinMax(
        const Derived& obj, Offset ind, Size count, Dimension element, ElementType& min_elem,
        ElementType& max_elem) const
    {
        min_elem = dataset_get(obj, vAcc_[ind], element);
        max_elem = min_elem;
        for (Offset i = 1; i < count; ++i)
        {
            ElementType val = dataset_get(obj, vAcc_[ind + i], element);
            if (val < min_elem) min_elem = val;
            if (val > max_elem) max_elem = val;
        }
    }

    /** Returns true if the point at index idx should be visited during search.
     *  The static adaptor always returns true; the dynamic adaptor overrides
     *  this to skip tombstoned (removed) points. */
    NANOFLANN_NODISCARD bool isActive(IndexType /*idx*/) const { return true; }

    /** Computes the bounding box of the points currently in the index.
     *  Uses size_ (set by buildIndex before this is called) so the result is
     *  correct for both the static and dynamic adaptors. */
    void computeBoundingBox(BoundingBox& bbox)
    {
        Derived&        obj  = static_cast<Derived&>(*this);
        const Dimension dims = static_cast<Dimension>(veclen(obj));
        resize(bbox, dims);
        if (obj.dataset_.kdtree_get_bbox(bbox)) return;
        if (!size_)
            throw std::runtime_error(
                "[nanoflann] computeBoundingBox() called but "
                "no data points found.");
        for (Dimension i = 0; i < dims; ++i)
            bbox[i].low = bbox[i].high = dataset_get(obj, vAcc_[0], i);
        for (Offset k = 1; k < size_; ++k)
            for (Dimension i = 0; i < dims; ++i)
            {
                const auto val = dataset_get(obj, vAcc_[k], i);
                if (val < bbox[i].low) bbox[i].low = val;
                if (val > bbox[i].high) bbox[i].high = val;
            }
    }

    /**
     * Performs an exact search in the tree starting from a node.
     * Uses the CRTP-dispatched isActive() hook to skip removed points (no-op
     * in the static adaptor, checks treeIndex_ in the dynamic adaptor).
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     * \return true if the search should be continued, false if the results are
     * sufficient
     */
    template <class RESULTSET>
    bool searchLevel(
        RESULTSET& result_set, const ElementType* vec, const NodePtr node, DistanceType mindist,
        distance_vector_t& dists, const DistanceType epsError) const
    {
        const Derived& obj = static_cast<const Derived&>(*this);
        // If this is a leaf node, then do check and return.
        if (!node->child1)  // (if one node is nullptr, both are)
        {
            // Hoist the point length out of the per-point loop. For a
            // fixed-size tree (DIM > 0) this is a compile-time constant; for a
            // runtime dimension it avoids re-reading obj.dim_ on every point.
            const Size dim = veclen(obj);
            for (Offset i = node->node_type.lr.left; i < node->node_type.lr.right; ++i)
            {
                const IndexType accessor = vAcc_[i];
                if (!obj.isActive(accessor)) continue;
                DistanceType dist = obj.distance_.evalMetric(vec, accessor, dim);
                if (dist < result_set.worstDist())
                {
                    if (!result_set.addPoint(
                            static_cast<typename RESULTSET::DistanceType>(dist),
                            static_cast<typename RESULTSET::IndexType>(accessor)))
                        return false;
                }
            }
            return true;
        }

        /* Which child branch should be taken first? */
        Dimension    idx   = node->node_type.sub.divfeat;
        ElementType  val   = vec[idx];
        DistanceType diff1 = val - node->node_type.sub.divlow;
        DistanceType diff2 = val - node->node_type.sub.divhigh;

        NodePtr      bestChild;
        NodePtr      otherChild;
        DistanceType cut_dist;
        if ((diff1 + diff2) < 0)
        {
            bestChild  = node->child1;
            otherChild = node->child2;
            cut_dist   = obj.distance_.accum_dist(val, node->node_type.sub.divhigh, idx);
        }
        else
        {
            bestChild  = node->child2;
            otherChild = node->child1;
            cut_dist   = obj.distance_.accum_dist(val, node->node_type.sub.divlow, idx);
        }

        /* Call recursively to search next level down. */
        if (!searchLevel(result_set, vec, bestChild, mindist, dists, epsError)) return false;

        DistanceType dst = dists[idx];
        mindist          = mindist + cut_dist - dst;
        dists[idx]       = cut_dist;
        if (mindist * epsError <= result_set.worstDist())
        {
            if (!searchLevel(result_set, vec, otherChild, mindist, dists, epsError)) return false;
        }
        dists[idx] = dst;
        return true;
    }

    /**
     * Create a tree node that subdivides the list of vecs from vind[first]
     * to vind[last].  The routine is called recursively on each sublist.
     *
     * @param left index of the first vector
     * @param right index of the last vector
     * @param bbox bounding box used as input for splitting and output for
     * parent node
     */
    /**
     * Initialize a freshly-allocated node while building the tree: either turn
     * it into a leaf node (computing the leaf bounding-box) or compute the
     * split plane for an interior node. Shared by the sequential and concurrent
     * builders, which differ only in how they recurse.
     *
     * @return true if the node became a leaf (no further recursion needed),
     *         false if it is an interior node and \a idx / \a cutfeat / \a cutval
     *         describe the split plane.
     */
    bool makeNode(
        Derived& obj, NodePtr node, const Offset left, const Offset right, BoundingBox& bbox,
        Offset& idx, Dimension& cutfeat, DistanceType& cutval)
    {
        const Dimension dims = static_cast<Dimension>(veclen(obj));

        /* If too few exemplars remain, then make this a leaf node. */
        if ((right - left) <= static_cast<Offset>(obj.leaf_max_size_))
        {
            node->child1 = node->child2 = nullptr; /* Mark as leaf node. */
            node->node_type.lr.left     = left;
            node->node_type.lr.right    = right;

            // compute bounding-box of leaf points
            for (Dimension i = 0; i < dims; ++i)
            {
                bbox[i].low  = dataset_get(obj, obj.vAcc_[left], i);
                bbox[i].high = dataset_get(obj, obj.vAcc_[left], i);
            }
            for (Offset k = left + 1; k < right; ++k)
            {
                for (Dimension i = 0; i < dims; ++i)
                {
                    const auto val = dataset_get(obj, obj.vAcc_[k], i);
                    if (bbox[i].low > val) bbox[i].low = val;
                    if (bbox[i].high < val) bbox[i].high = val;
                }
            }
            return true;
        }

        /* Determine the index, dimension and value for split plane */
        middleSplit_(obj, left, right - left, idx, cutfeat, cutval, bbox);
        node->node_type.sub.divfeat = cutfeat;
        return false;
    }

    /**
     * After both children of an interior node have been built, record the split
     * planes and expand \a bbox to the union of the children bounding-boxes.
     * Shared by the sequential and concurrent builders.
     */
    void finalizeSplitNode(
        Derived& obj, NodePtr node, const Dimension cutfeat, const BoundingBox& left_bbox,
        const BoundingBox& right_bbox, BoundingBox& bbox)
    {
        node->node_type.sub.divlow  = left_bbox[cutfeat].high;
        node->node_type.sub.divhigh = right_bbox[cutfeat].low;

        const Dimension dims = static_cast<Dimension>(veclen(obj));
        for (Dimension i = 0; i < dims; ++i)
        {
            bbox[i].low  = std::min(left_bbox[i].low, right_bbox[i].low);
            bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
        }
    }

    NodePtr divideTree(Derived& obj, const Offset left, const Offset right, BoundingBox& bbox)
    {
        assert(static_cast<Size>(obj.vAcc_.at(left)) < obj.dataset_.kdtree_get_point_count());

        NodePtr      node = obj.pool_.template allocate<Node>();  // allocate memory
        Offset       idx;
        Dimension    cutfeat;
        DistanceType cutval;
        if (makeNode(obj, node, left, right, bbox, idx, cutfeat, cutval)) return node;

        /* Recurse on left */
        BoundingBox left_bbox(bbox);
        left_bbox[cutfeat].high = cutval;
        node->child1            = this->divideTree(obj, left, left + idx, left_bbox);

        /* Recurse on right */
        BoundingBox right_bbox(bbox);
        right_bbox[cutfeat].low = cutval;
        node->child2            = this->divideTree(obj, left + idx, right, right_bbox);

        finalizeSplitNode(obj, node, cutfeat, left_bbox, right_bbox, bbox);

        return node;
    }

    /**
     * Create a tree node that subdivides the list of vecs from vind[first] to
     * vind[last] concurrently.  The routine is called recursively on each
     * sublist.
     *
     * @param left index of the first vector
     * @param right index of the last vector
     * @param bbox bounding box used as input for splitting and output for
     * parent node
     * @param thread_count count of std::async threads
     * @param mutex mutex for mempool allocation
     */
    NodePtr divideTreeConcurrent(
        Derived& obj, const Offset left, const Offset right, BoundingBox& bbox,
        std::atomic<unsigned int>& thread_count, std::mutex& mutex)
    {
        std::unique_lock<std::mutex> lock(mutex);
        NodePtr                      node = obj.pool_.template allocate<Node>();  // allocate memory
        lock.unlock();

        Offset       idx;
        Dimension    cutfeat;
        DistanceType cutval;
        if (makeNode(obj, node, left, right, bbox, idx, cutfeat, cutval)) return node;

        std::future<NodePtr> right_future;

        /* Recurse on right concurrently, if possible */

        BoundingBox right_bbox(bbox);
        right_bbox[cutfeat].low = cutval;
        if (++thread_count < n_thread_build_)
        {
            /* Concurrent thread for right recursion */

            right_future = std::async(
                std::launch::async, &KDTreeBaseClass::divideTreeConcurrent, this, std::ref(obj),
                left + idx, right, std::ref(right_bbox), std::ref(thread_count), std::ref(mutex));
        }
        else
        {
            --thread_count;
        }

        /* Recurse on left in this thread */

        BoundingBox left_bbox(bbox);
        left_bbox[cutfeat].high = cutval;
        node->child1 =
            this->divideTreeConcurrent(obj, left, left + idx, left_bbox, thread_count, mutex);

        if (right_future.valid())
        {
            /* Block and wait for concurrent right from above */

            node->child2 = right_future.get();
            --thread_count;
        }
        else
        {
            /* Otherwise, recurse on right in this thread */

            node->child2 =
                this->divideTreeConcurrent(obj, left + idx, right, right_bbox, thread_count, mutex);
        }

        finalizeSplitNode(obj, node, cutfeat, left_bbox, right_bbox, bbox);

        return node;
    }

    void middleSplit_(
        const Derived& obj, const Offset ind, const Size count, Offset& index, Dimension& cutfeat,
        DistanceType& cutval, const BoundingBox& bbox)
    {
        const Dimension dims = static_cast<Dimension>(veclen(obj));
        const auto      EPS  = static_cast<DistanceType>(0.00001);

        // Pre-compute max_span once
        ElementType max_span = bbox[0].high - bbox[0].low;
        for (Dimension i = 1; i < dims; ++i)
        {
            ElementType span = bbox[i].high - bbox[i].low;
            if (span > max_span) max_span = span;
        }

        // Two-pass: first find max_span (done above), then scan candidate dims
        // inline — no heap allocation for a candidates vector.
        cutfeat                      = 0;
        ElementType       max_spread = -1;
        ElementType       min_elem = 0, max_elem = 0;
        const ElementType threshold = (1 - EPS) * max_span;

        for (Dimension dim = 0; dim < dims; ++dim)
        {
            if (bbox[dim].high - bbox[dim].low < threshold) continue;

            ElementType local_min = dataset_get(obj, vAcc_[ind], dim);
            ElementType local_max = local_min;

            // Unrolled loop for better performance
            constexpr size_t UNROLL = 4;
            Offset           k      = 1;
            for (; k + UNROLL <= count; k += UNROLL)
            {
                ElementType v0 = dataset_get(obj, vAcc_[ind + k], dim);
                ElementType v1 = dataset_get(obj, vAcc_[ind + k + 1], dim);
                ElementType v2 = dataset_get(obj, vAcc_[ind + k + 2], dim);
                ElementType v3 = dataset_get(obj, vAcc_[ind + k + 3], dim);

                local_min = std::min({local_min, v0, v1, v2, v3});
                local_max = std::max({local_max, v0, v1, v2, v3});
            }

            // Handle remainder
            for (; k < count; ++k)
            {
                ElementType val = dataset_get(obj, vAcc_[ind + k], dim);
                local_min       = std::min(local_min, val);
                local_max       = std::max(local_max, val);
            }

            ElementType spread = local_max - local_min;
            if (spread > max_spread)
            {
                cutfeat    = dim;
                max_spread = spread;
                min_elem   = local_min;
                max_elem   = local_max;
            }
        }

        // Median-of-three for better balance
        DistanceType split_val = (bbox[cutfeat].low + bbox[cutfeat].high) / 2;
        if (split_val < min_elem) split_val = min_elem;
        if (split_val > max_elem) split_val = max_elem;

        cutval = split_val;

        // Optimized partitioning
        Offset lim1, lim2;
        planeSplit(obj, ind, count, cutfeat, cutval, lim1, lim2);

        index = (lim1 > count / 2) ? lim1 : (lim2 < count / 2) ? lim2 : count / 2;
    }

    /**
     *  Subdivide the list of points by a plane perpendicular on the axis
     * corresponding to the 'cutfeat' dimension at 'cutval' position.
     *
     *  On return:
     *  dataset[ind[0..lim1-1]][cutfeat] < cutval
     *  dataset[ind[lim1..lim2-1]][cutfeat] == cutval
     *  dataset[ind[lim2..count]][cutfeat] > cutval
     */
    void planeSplit(
        const Derived& obj, const Offset ind, const Size count, const Dimension cutfeat,
        const DistanceType& cutval, Offset& lim1, Offset& lim2)
    {
        // Dutch National Flag algorithm for three-way partitioning
        Offset left  = 0;
        Offset mid   = 0;
        Offset right = count - 1;

        while (mid <= right)
        {
            ElementType val = dataset_get(obj, vAcc_[ind + mid], cutfeat);

            if (val < cutval)
            {
                std::swap(vAcc_[ind + left], vAcc_[ind + mid]);
                left++;
                mid++;
            }
            else if (val > cutval)
            {
                std::swap(vAcc_[ind + mid], vAcc_[ind + right]);
                right--;
            }
            else
            {
                mid++;
            }
        }

        lim1 = left;
        lim2 = mid;
    }

    DistanceType computeInitialDistances(
        const Derived& obj, const ElementType* vec, distance_vector_t& dists) const
    {
        assert(vec);
        DistanceType dist = DistanceType();

        const Dimension dims = static_cast<Dimension>(veclen(obj));
        for (Dimension i = 0; i < dims; ++i)
        {
            if (vec[i] < obj.root_bbox_[i].low)
            {
                dists[i] = obj.distance_.accum_dist(vec[i], obj.root_bbox_[i].low, i);
                dist += dists[i];
            }
            else if (vec[i] > obj.root_bbox_[i].high)
            {
                dists[i] = obj.distance_.accum_dist(vec[i], obj.root_bbox_[i].high, i);
                dist += dists[i];
            }
        }
        return dist;
    }

    static void save_tree(const Derived& obj, std::ostream& stream, const NodeConstPtr tree)
    {
        save_value(stream, *tree);
        if (tree->child1 != nullptr)
        {
            save_tree(obj, stream, tree->child1);
        }
        if (tree->child2 != nullptr)
        {
            save_tree(obj, stream, tree->child2);
        }
    }

    static void load_tree(Derived& obj, std::istream& stream, NodePtr& tree)
    {
        tree = obj.pool_.template allocate<Node>();
        load_value(stream, *tree);
        if (tree->child1 != nullptr)
        {
            load_tree(obj, stream, tree->child1);
        }
        if (tree->child2 != nullptr)
        {
            load_tree(obj, stream, tree->child2);
        }
    }

    /** Magic number written at the start of every saveIndex() stream.
     *  Spells 'NFLN' in ASCII. */
    static constexpr uint32_t SAVE_MAGIC = 0x4E464C4E;

    /** Stores the index in a binary stream.
     *
     * The set of data points is NOT stored; when reloading, the index object
     * must be constructed with the same dataset. See: examples/saveload_example.cpp
     *
     * \note **Portability limitations** (by design -- fixing them would require
     *   a breaking format change):
     *   - Files are NOT portable across different endianness (e.g. x86 little-endian
     *     vs. big-endian SPARC/PowerPC). No byte-swapping is performed.
     *   - Files are NOT portable across 32-bit vs. 64-bit platforms (sizeof(size_t)
     *     differs).
     *   - Files are NOT portable across different nanoflann versions; loadIndex()
     *     throws if the version in the file does not match the library.
     *   - Files are NOT portable across different template instantiations (e.g.
     *     float vs. double IndexType/ElementType); loadIndex() throws on mismatch.
     *
     * \sa loadIndex
     */
    void saveIndex(const Derived& obj, std::ostream& stream) const
    {
        // 10-byte header: magic | version | sizeof_size_t | sizeof_IndexType
        //                 | sizeof_ElementType | sizeof_DistanceType
        // Use local copies: passing a static constexpr by const-ref ODR-uses it
        // in C++11/14, which requires an out-of-class definition we cannot provide
        // in a header-only library.
        const uint32_t hdr_magic   = SAVE_MAGIC;
        const uint32_t hdr_version = static_cast<uint32_t>(NANOFLANN_VERSION);
        const uint8_t  hdr_sz_st   = static_cast<uint8_t>(sizeof(size_t));
        const uint8_t  hdr_sz_idx  = static_cast<uint8_t>(sizeof(IndexType));
        const uint8_t  hdr_sz_elem = static_cast<uint8_t>(sizeof(ElementType));
        const uint8_t  hdr_sz_dist = static_cast<uint8_t>(sizeof(DistanceType));
        save_value(stream, hdr_magic);
        save_value(stream, hdr_version);
        save_value(stream, hdr_sz_st);
        save_value(stream, hdr_sz_idx);
        save_value(stream, hdr_sz_elem);
        save_value(stream, hdr_sz_dist);

        save_value(stream, obj.size_);
        save_value(stream, obj.dim_);
        save_value(stream, obj.root_bbox_);
        save_value(stream, obj.leaf_max_size_);
        save_value(stream, obj.vAcc_);
        if (obj.root_node_)
        {
            save_tree(obj, stream, obj.root_node_);
        }
    }

    /** Loads an index previously saved with saveIndex() from a binary stream.
     *
     * The index object must be constructed associated to the same dataset that
     * was used when building the saved index. See: examples/saveload_example.cpp
     *
     * \throws std::runtime_error if the stream does not start with the expected
     *   magic number (wrong file or corrupt data), if the nanoflann version in
     *   the file differs from the current library version, if the saved type
     *   sizes (size_t, IndexType, ElementType, DistanceType) do not match the
     *   current template instantiation, or if a read error occurs.
     *
     * \note See saveIndex() for portability limitations.
     *
     * \sa saveIndex
     */
    void loadIndex(Derived& obj, std::istream& stream)
    {
        // Validate header
        uint32_t magic = 0;
        load_value(stream, magic);
        if (stream.fail() || magic != SAVE_MAGIC)
        {
            throw std::runtime_error(
                "nanoflann loadIndex: invalid file (wrong magic number). "
                "The stream was not written by nanoflann saveIndex().");
        }

        uint32_t file_version = 0;
        load_value(stream, file_version);
        if (file_version != static_cast<uint32_t>(NANOFLANN_VERSION))
        {
            char msg[200];
            snprintf(
                msg, sizeof(msg),
                "nanoflann loadIndex: version mismatch "
                "(file=0x%03X, library=0x%03X). Rebuild the index.",
                file_version, static_cast<unsigned>(NANOFLANN_VERSION));
            throw std::runtime_error(msg);
        }

        uint8_t sz_size_t = 0;
        uint8_t sz_idx    = 0;
        uint8_t sz_elem   = 0;
        uint8_t sz_dist   = 0;
        load_value(stream, sz_size_t);
        load_value(stream, sz_idx);
        load_value(stream, sz_elem);
        load_value(stream, sz_dist);
        if (sz_size_t != static_cast<uint8_t>(sizeof(size_t)) ||
            sz_idx != static_cast<uint8_t>(sizeof(IndexType)) ||
            sz_elem != static_cast<uint8_t>(sizeof(ElementType)) ||
            sz_dist != static_cast<uint8_t>(sizeof(DistanceType)))
        {
            throw std::runtime_error(
                "nanoflann loadIndex: type-size mismatch between saved index and "
                "current template instantiation (sizeof size_t / IndexType / "
                "ElementType / DistanceType differ). Rebuild the index.");
        }

        load_value(stream, obj.size_);
        load_value(stream, obj.dim_);
        load_value(stream, obj.root_bbox_);
        load_value(stream, obj.leaf_max_size_);
        load_value(stream, obj.vAcc_);

        if (obj.size_ > 0)
        {
            load_tree(obj, stream, obj.root_node_);
        }

        if (stream.fail())
        {
            throw std::runtime_error(
                "nanoflann loadIndex: unexpected end of stream or read error.");
        }
    }
};

/** @addtogroup kdtrees_grp KD-tree classes and adaptors
 * @{ */

/** kd-tree static index
 *
 * Contains the k-d trees and other information for indexing a set of points
 * for nearest-neighbor matching.
 *
 *  The class "DatasetAdaptor" must provide the following interface (can be
 * non-virtual, inlined methods):
 *
 *  \code
 *   // Must return the number of data points
 *   size_t kdtree_get_point_count() const { ... }
 *
 *
 *   // Must return the dim'th component of the idx'th point in the class:
 *   T kdtree_get_pt(const size_t idx, const size_t dim) const { ... }
 *
 *   // Optional bounding-box computation: return false to default to a standard
 * bbox computation loop.
 *   //   Return true if the BBOX was already computed by the class and returned
 * in "bb" so it can be avoided to redo it again.
 *   //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3
 * for point clouds) template <class BBOX> bool kdtree_get_bbox(BBOX &bb) const
 *   {
 *      bb[0].low = ...; bb[0].high = ...;  // 0th dimension limits
 *      bb[1].low = ...; bb[1].high = ...;  // 1st dimension limits
 *      ...
 *      return true;
 *   }
 *
 *  \endcode
 *
 * \tparam DatasetAdaptor The user-provided adaptor, which must be ensured to
 *         have a lifetime equal or longer than the instance of this class.
 * \tparam Distance The distance metric to use: nanoflann::metric_L1,
 * nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc. \tparam DIM
 * Dimensionality of data points (e.g. 3 for 3D points) \tparam IndexType Will
 * be typically size_t or int
 *
 * \note Threading guarantees:
 *   - Index build: passing `n_thread_build > 1` in the params parallelizes the
 *     build using `std::async` (unless NANOFLANN_NO_THREADS is defined, in which
 *     case requesting more than one thread throws).
 *   - Queries (`findNeighbors`, `knnSearch`, `radiusSearch`, `rknnSearch`) are
 *     `const` and thread-safe for concurrent readers: multiple threads may query
 *     the same index simultaneously, as long as no thread is concurrently
 *     (re)building or modifying it.
 *   - The internal `PooledAllocator` is NOT thread-safe; building an index from
 *     multiple threads, or mixing queries with a concurrent build, is not
 *     supported.
 */
template <typename Distance, class DatasetAdaptor, int32_t DIM = -1, typename index_t = uint32_t>
class KDTreeSingleIndexAdaptor
    : public KDTreeBaseClass<
          KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM, index_t>, Distance,
          DatasetAdaptor, DIM, index_t>
{
   public:
    /** Deleted copy constructor*/
    explicit KDTreeSingleIndexAdaptor(
        const KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM, index_t>&) = delete;

    /** The data source used by this index */
    const DatasetAdaptor& dataset_;

    const KDTreeSingleIndexAdaptorParams indexParams;

    Distance distance_;

    using Base = typename nanoflann::KDTreeBaseClass<
        nanoflann::KDTreeSingleIndexAdaptor<Distance, DatasetAdaptor, DIM, index_t>, Distance,
        DatasetAdaptor, DIM, index_t>;

    using Offset    = typename Base::Offset;
    using Size      = typename Base::Size;
    using Dimension = typename Base::Dimension;

    using ElementType  = typename Base::ElementType;
    using DistanceType = typename Base::DistanceType;
    using IndexType    = typename Base::IndexType;

    using Node    = typename Base::Node;
    using NodePtr = Node*;

    using Interval = typename Base::Interval;

    /** Define "BoundingBox" as a fixed-size or variable-size container
     * depending on "DIM" */
    using BoundingBox = typename Base::BoundingBox;

    /** Define "distance_vector_t" as a fixed-size or variable-size container
     * depending on "DIM" */
    using distance_vector_t = typename Base::distance_vector_t;

    /**
     * KDTree constructor
     *
     * Refer to docs in README.md or online in
     * https://github.com/jlblancoc/nanoflann
     *
     * The KD-Tree point dimension (the length of each point in the dataset, e.g.
     * 3 for 3D points) is determined by means of:
     *  - The \a DIM template parameter if >0 (highest priority)
     *  - Otherwise, the \a dimensionality parameter of this constructor.
     *
     * @param inputData Dataset with the input features. Its lifetime must be
     *  equal or longer than that of the instance of this class.
     * @param params Basically, the maximum leaf node size
     *
     * Note that there is a variable number of optional additional parameters
     * which will be forwarded to the metric class constructor. Refer to example
     * `examples/pointcloud_custom_metric.cpp` for a use case.
     *
     */
    template <class... Args>
    explicit KDTreeSingleIndexAdaptor(
        const Dimension dimensionality, const DatasetAdaptor& inputData,
        const KDTreeSingleIndexAdaptorParams& params, Args&&... args)
        : dataset_(inputData),
          indexParams(params),
          distance_(inputData, std::forward<Args>(args)...)
    {
        init(dimensionality, params);
    }

    explicit KDTreeSingleIndexAdaptor(
        const Dimension dimensionality, const DatasetAdaptor& inputData,
        const KDTreeSingleIndexAdaptorParams& params = {})
        : dataset_(inputData), indexParams(params), distance_(inputData)
    {
        init(dimensionality, params);
    }

   private:
    void init(const Dimension dimensionality, const KDTreeSingleIndexAdaptorParams& params)
    {
        Base::size_                = dataset_.kdtree_get_point_count();
        Base::size_at_index_build_ = Base::size_;
        Base::dim_                 = dimensionality;
        if (DIM > 0) Base::dim_ = DIM;
        Base::leaf_max_size_ = params.leaf_max_size;
        if (params.n_thread_build > 0)
        {
            Base::n_thread_build_ = params.n_thread_build;
        }
        else
        {
            Base::n_thread_build_ = std::max(std::thread::hardware_concurrency(), 1u);
        }

        if (!has_flag(params.flags, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex))
        {
            // Build KD-tree:
            buildIndex();
        }
    }

   public:
    /**
     * Builds the index
     */
    void buildIndex()
    {
        Base::size_                = dataset_.kdtree_get_point_count();
        Base::size_at_index_build_ = Base::size_;
        init_vind();
        this->freeIndex(*this);
        Base::size_at_index_build_ = Base::size_;
        if (Base::size_ == 0) return;
        this->computeBoundingBox(Base::root_bbox_);
        // construct the tree
        if (Base::n_thread_build_ == 1)
        {
            Base::root_node_ = this->divideTree(*this, 0, Base::size_, Base::root_bbox_);
        }
        else
        {
#ifndef NANOFLANN_NO_THREADS
            std::atomic<unsigned int> thread_count(0u);
            std::mutex                mutex;
            Base::root_node_ = this->divideTreeConcurrent(
                *this, 0, Base::size_, Base::root_bbox_, thread_count, mutex);
#else /* NANOFLANN_NO_THREADS */
            throw std::runtime_error("Multithreading is disabled");
#endif /* NANOFLANN_NO_THREADS */
        }
    }

    /** \name Query methods
     * @{ */

    /**
     * Find set of nearest neighbors to vec[0:dim-1]. Their indices are stored
     * inside the result object.
     *
     * Params:
     *     result = the result object in which the indices of the
     * nearest-neighbors are stored vec = the vector for which to search the
     * nearest neighbors
     *
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     * \return  True if the requested neighbors could be found.
     * \sa knnSearch, radiusSearch
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     */
    template <typename RESULTSET>
    bool findNeighbors(
        RESULTSET& result, const ElementType* vec, const SearchParameters& searchParams = {}) const
    {
        assert(vec);
        if (this->size(*this) == 0) return false;
        if (!Base::root_node_)
            throw std::runtime_error(
                "[nanoflann] findNeighbors() called before building the "
                "index.");
        DistanceType epsError = 1 + static_cast<DistanceType>(searchParams.eps);

        // fixed or variable-sized container (depending on DIM)
        distance_vector_t dists;
        // Fill it with zeros.
        auto zero = static_cast<typename RESULTSET::DistanceType>(0);
        assign(dists, this->veclen(*this), zero);
        DistanceType dist = this->computeInitialDistances(*this, vec, dists);
        this->searchLevel(result, vec, Base::root_node_, dist, dists, epsError);

        if (searchParams.sorted) result.sort();

        return result.full();
    }

    /**
     * Find all points contained within the specified bounding box. Their
     * indices are stored inside the result object.
     *
     * Params:
     *     result = the result object in which the indices of the points
     *              within the bounding box are stored
     *     bbox = the bounding box defining the search region
     *
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     * \return  Number of points found within the bounding box.
     * \sa findNeighbors, knnSearch, radiusSearch
     *
     * \note The search is inclusive - points on the boundary are included.
     */
    template <typename RESULTSET>
    NANOFLANN_NODISCARD Size findWithinBox(RESULTSET& result, const BoundingBox& bbox) const
    {
        if (this->size(*this) == 0) return 0;
        if (!Base::root_node_)
            throw std::runtime_error(
                "[nanoflann] findWithinBox() called before building the "
                "index.");

        std::stack<NodePtr> stack;
        stack.push(Base::root_node_);

        while (!stack.empty())
        {
            const NodePtr node = stack.top();
            stack.pop();

            // If this is a leaf node, then do check and return.
            if (!node->child1)  // (if one node is nullptr, both are)
            {
                for (Offset i = node->node_type.lr.left; i < node->node_type.lr.right; ++i)
                {
                    if (contains(bbox, Base::vAcc_[i]))
                    {
                        if (!result.addPoint(0, Base::vAcc_[i]))
                        {
                            // the resultset doesn't want to receive any more
                            // points, we're done searching!
                            return result.size();
                        }
                    }
                }
            }
            else
            {
                const Dimension idx        = node->node_type.sub.divfeat;
                const auto      low_bound  = node->node_type.sub.divlow;
                const auto      high_bound = node->node_type.sub.divhigh;

                if (bbox[idx].low <= low_bound) stack.push(node->child1);
                if (bbox[idx].high >= high_bound) stack.push(node->child2);
            }
        }

        return result.size();
    }

    /**
     * Find the "num_closest" nearest neighbors to the \a query_point[0:dim-1].
     * Their indices and distances are stored in the provided pointers to
     * array/vector.
     *
     * \sa radiusSearch, findNeighbors
     * \return Number `N` of valid points in the result set.
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     *
     * \note Only the first `N` entries in `out_indices` and `out_distances`
     *       will be valid. Return is less than `num_closest` only if the
     *       number of elements in the tree is less than `num_closest`.
     */
    NANOFLANN_NODISCARD Size knnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances) const
    {
        nanoflann::KNNResultSet<DistanceType, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances);
        findNeighbors(resultSet, query_point);
        return resultSet.size();
    }

    /**
     * Find all the neighbors to \a query_point[0:dim-1] within a maximum
     * radius. The output is given as a vector of pairs, of which the first
     * element is a point index and the second the corresponding distance.
     * Previous contents of \a IndicesDists are cleared.
     *
     *  If searchParams.sorted==true, the output list is sorted by ascending
     * distances.
     *
     *  For a better performance, it is advisable to do a .reserve() on the
     * vector if you have any wild guess about the number of expected matches.
     *
     *  \sa knnSearch, findNeighbors, radiusSearchCustomCallback
     * \return The number of points within the given radius (i.e. indices.size()
     * or dists.size() )
     *
     * \note If L2 norms are used, search radius and all returned distances
     *       are actually squared distances.
     */
    NANOFLANN_NODISCARD Size radiusSearch(
        const ElementType* query_point, const DistanceType& radius,
        std::vector<ResultItem<IndexType, DistanceType>>& IndicesDists,
        const SearchParameters&                           searchParams = {}) const
    {
        RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndicesDists);
        const Size nFound = radiusSearchCustomCallback(query_point, resultSet, searchParams);
        return nFound;
    }

    /**
     * Just like radiusSearch() but with a custom callback class for each point
     * found in the radius of the query. See the source of RadiusResultSet<> as
     * a start point for your own classes. \sa radiusSearch
     */
    template <class SEARCH_CALLBACK>
    NANOFLANN_NODISCARD Size radiusSearchCustomCallback(
        const ElementType* query_point, SEARCH_CALLBACK& resultSet,
        const SearchParameters& searchParams = {}) const
    {
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /**
     * Find the N closest neighbors to \a query_point[0:dim-1] that are also
     * within the given maximum radius. Results are stored in the provided
     * output arrays; previous contents are overwritten.
     *
     * \sa radiusSearch, findNeighbors
     * \return Number of valid points written (at most `num_closest`). May be
     *         less if fewer than `num_closest` points lie within the radius.
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     *
     * \note Only the first `N` entries in `out_indices` and `out_distances`
     *       will be valid.
     */
    NANOFLANN_NODISCARD Size rknnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const DistanceType& radius) const
    {
        nanoflann::RKNNResultSet<DistanceType, IndexType> resultSet(num_closest, radius);
        resultSet.init(out_indices, out_distances);
        findNeighbors(resultSet, query_point);
        return resultSet.size();
    }

    /** @} */

   public:
    /** Make sure the auxiliary list \a vind has the same size as the
     * current dataset, and re-generate if size has changed. */
    void init_vind()
    {
        // Create a permutable array of indices to the input vectors.
        Base::size_ = dataset_.kdtree_get_point_count();
        if (Base::vAcc_.size() != Base::size_) Base::vAcc_.resize(Base::size_);
        for (IndexType i = 0; i < static_cast<IndexType>(Base::size_); i++) Base::vAcc_[i] = i;
    }

    bool contains(const BoundingBox& bbox, IndexType idx) const
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
        {
            const auto point = this->dataset_.kdtree_get_pt(idx, i);
            if (point < bbox[i].low || point > bbox[i].high) return false;
        }
        return true;
    }

   public:
    /**  Stores the index in a binary file.
     *   IMPORTANT NOTE: The set of data points is NOT stored in the file, so
     * when loading the index object it must be constructed associated to the
     * same source of data points used while building it. See the example:
     * examples/saveload_example.cpp \sa loadIndex  */
    void saveIndex(std::ostream& stream) const { Base::saveIndex(*this, stream); }

    /**  Loads a previous index from a binary file.
     *   IMPORTANT NOTE: The set of data points is NOT stored in the file, so
     * the index object must be constructed associated to the same source of
     * data points used while building the index. See the example:
     * examples/saveload_example.cpp \sa loadIndex  */
    void loadIndex(std::istream& stream) { Base::loadIndex(*this, stream); }

};  // class KDTree

/** kd-tree dynamic index
 *
 * Contains the k-d trees and other information for indexing a set of points
 * for nearest-neighbor matching.
 *
 * The class "DatasetAdaptor" must provide the following interface (can be
 * non-virtual, inlined methods):
 *
 *  \code
 *   // Must return the number of data points
 *   size_t kdtree_get_point_count() const { ... }
 *
 *   // Must return the dim'th component of the idx'th point in the class:
 *   T kdtree_get_pt(const size_t idx, const size_t dim) const { ... }
 *
 *   // Optional bounding-box computation: return false to default to a standard
 * bbox computation loop.
 *   //   Return true if the BBOX was already computed by the class and returned
 * in "bb" so it can be avoided to redo it again.
 *   //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3
 * for point clouds) template <class BBOX> bool kdtree_get_bbox(BBOX &bb) const
 *   {
 *      bb[0].low = ...; bb[0].high = ...;  // 0th dimension limits
 *      bb[1].low = ...; bb[1].high = ...;  // 1st dimension limits
 *      ...
 *      return true;
 *   }
 *
 *  \endcode
 *
 * \tparam DatasetAdaptor The user-provided adaptor (see comments above).
 * \tparam Distance The distance metric to use: nanoflann::metric_L1,
 * nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
 * \tparam DIM Dimensionality of data points (e.g. 3 for 3D points)
 * \tparam IndexType Type of the arguments with which the data can be
 * accessed (e.g. float, double, int64_t, T*)
 */
template <typename Distance, class DatasetAdaptor, int32_t DIM = -1, typename IndexType = uint32_t>
class KDTreeSingleIndexDynamicAdaptor_
    : public KDTreeBaseClass<
          KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM, IndexType>, Distance,
          DatasetAdaptor, DIM, IndexType>
{
   public:
    /**
     * The dataset used by this index
     */
    const DatasetAdaptor& dataset_;  //!< The source of our data

    KDTreeSingleIndexAdaptorParams index_params_;

    std::vector<int>& treeIndex_;

    Distance distance_;

    using Base = typename nanoflann::KDTreeBaseClass<
        nanoflann::KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM, IndexType>,
        Distance, DatasetAdaptor, DIM, IndexType>;

    using ElementType  = typename Base::ElementType;
    using DistanceType = typename Base::DistanceType;

    using Offset    = typename Base::Offset;
    using Size      = typename Base::Size;
    using Dimension = typename Base::Dimension;

    using Node    = typename Base::Node;
    using NodePtr = Node*;

    using Interval = typename Base::Interval;
    /** Define "BoundingBox" as a fixed-size or variable-size container
     * depending on "DIM" */
    using BoundingBox = typename Base::BoundingBox;

    /** Define "distance_vector_t" as a fixed-size or variable-size container
     * depending on "DIM" */
    using distance_vector_t = typename Base::distance_vector_t;

    /** Returns false for points that have been removed (lazy deletion). */
    NANOFLANN_NODISCARD bool isActive(IndexType idx) const { return treeIndex_[idx] != -1; }

    /**
     * KDTree constructor
     *
     * Refer to docs in README.md or online in
     * https://github.com/jlblancoc/nanoflann
     *
     * The KD-Tree point dimension (the length of each point in the dataset, e.g.
     * 3 for 3D points) is determined by means of:
     *  - The \a DIM template parameter if >0 (highest priority)
     *  - Otherwise, the \a dimensionality parameter of this constructor.
     *
     * @param inputData Dataset with the input features. Its lifetime must be
     *  equal or longer than that of the instance of this class.
     * @param params Basically, the maximum leaf node size
     */
    KDTreeSingleIndexDynamicAdaptor_(
        const Dimension dimensionality, const DatasetAdaptor& inputData,
        std::vector<int>&                     treeIndex,
        const KDTreeSingleIndexAdaptorParams& params = KDTreeSingleIndexAdaptorParams())
        : dataset_(inputData), index_params_(params), treeIndex_(treeIndex), distance_(inputData)
    {
        Base::size_                = 0;
        Base::size_at_index_build_ = 0;
        for (auto& v : Base::root_bbox_) v = {};
        Base::dim_ = dimensionality;
        if (DIM > 0) Base::dim_ = DIM;
        Base::leaf_max_size_ = params.leaf_max_size;
        if (params.n_thread_build > 0)
        {
            Base::n_thread_build_ = params.n_thread_build;
        }
        else
        {
            Base::n_thread_build_ = std::max(std::thread::hardware_concurrency(), 1u);
        }
    }

    /** Explicitly default the copy constructor */
    KDTreeSingleIndexDynamicAdaptor_(const KDTreeSingleIndexDynamicAdaptor_& rhs) = default;

    /** Assignment operator definition */
    KDTreeSingleIndexDynamicAdaptor_& operator=(const KDTreeSingleIndexDynamicAdaptor_& rhs)
    {
        if (this == &rhs) return *this;
        KDTreeSingleIndexDynamicAdaptor_ tmp(rhs);
        std::swap(Base::vAcc_, tmp.Base::vAcc_);
        std::swap(Base::leaf_max_size_, tmp.Base::leaf_max_size_);
        std::swap(index_params_, tmp.index_params_);
        // treeIndex_ is a reference member and cannot be rebound; do not swap.
        std::swap(Base::size_, tmp.Base::size_);
        std::swap(Base::size_at_index_build_, tmp.Base::size_at_index_build_);
        std::swap(Base::root_node_, tmp.Base::root_node_);
        std::swap(Base::root_bbox_, tmp.Base::root_bbox_);
        std::swap(Base::pool_, tmp.Base::pool_);
        return *this;
    }

    /**
     * Builds the index
     */
    void buildIndex()
    {
        Base::size_ = Base::vAcc_.size();
        this->freeIndex(*this);
        Base::size_at_index_build_ = Base::size_;
        if (Base::size_ == 0) return;
        this->computeBoundingBox(Base::root_bbox_);
        // construct the tree
        if (Base::n_thread_build_ == 1)
        {
            Base::root_node_ = this->divideTree(*this, 0, Base::size_, Base::root_bbox_);
        }
        else
        {
#ifndef NANOFLANN_NO_THREADS
            std::atomic<unsigned int> thread_count(0u);
            std::mutex                mutex;
            Base::root_node_ = this->divideTreeConcurrent(
                *this, 0, Base::size_, Base::root_bbox_, thread_count, mutex);
#else /* NANOFLANN_NO_THREADS */
            throw std::runtime_error("Multithreading is disabled");
#endif /* NANOFLANN_NO_THREADS */
        }
    }

    /** \name Query methods
     * @{ */

    /**
     * Find set of nearest neighbors to vec[0:dim-1]. Their indices are stored
     * inside the result object.
     * This is the core search function, all others are wrappers around this
     * one.
     *
     * \param result The result object in which the indices of the
     *               nearest-neighbors are stored.
     * \param vec    The vector of the query point for which to search the
     *               nearest neighbors.
     * \param searchParams Optional parameters for the search.
     *
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     * \return True if the requested neighbors could be found.
     *
     * \sa knnSearch(), radiusSearch(), radiusSearchCustomCallback()
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     */
    template <typename RESULTSET>
    bool findNeighbors(
        RESULTSET& result, const ElementType* vec, const SearchParameters& searchParams = {}) const
    {
        assert(vec);
        if (this->size(*this) == 0) return false;
        if (!Base::root_node_) return false;
        DistanceType epsError = 1 + static_cast<DistanceType>(searchParams.eps);

        // fixed or variable-sized container (depending on DIM)
        distance_vector_t dists;
        // Fill it with zeros.
        assign(dists, this->veclen(*this), static_cast<typename distance_vector_t::value_type>(0));
        DistanceType dist = this->computeInitialDistances(*this, vec, dists);
        this->searchLevel(result, vec, Base::root_node_, dist, dists, epsError);

        if (searchParams.sorted) result.sort();

        return result.full();
    }

    /**
     * Find the "num_closest" nearest neighbors to the \a query_point[0:dim-1].
     * Their indices are stored inside the result object. \sa radiusSearch,
     * findNeighbors
     * \return Number `N` of valid points in
     * the result set.
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     *
     * \note Only the first `N` entries in `out_indices` and `out_distances`
     *       will be valid. Return may be less than `num_closest` only if the
     *       number of elements in the tree is less than `num_closest`.
     */
    NANOFLANN_NODISCARD Size knnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const SearchParameters& searchParams = {}) const
    {
        nanoflann::KNNResultSet<DistanceType, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances);
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /**
     * Find all the neighbors to \a query_point[0:dim-1] within a maximum
     * radius. The output is given as a vector of pairs, of which the first
     * element is a point index and the second the corresponding distance.
     * Previous contents of \a IndicesDists are cleared.
     *
     * If searchParams.sorted==true, the output list is sorted by ascending
     * distances.
     *
     * For a better performance, it is advisable to do a .reserve() on the
     * vector if you have any wild guess about the number of expected matches.
     *
     *  \sa knnSearch, findNeighbors, radiusSearchCustomCallback
     * \return The number of points within the given radius (i.e. indices.size()
     * or dists.size() )
     *
     * \note If L2 norms are used, search radius and all returned distances
     *       are actually squared distances.
     */
    NANOFLANN_NODISCARD Size radiusSearch(
        const ElementType* query_point, const DistanceType& radius,
        std::vector<ResultItem<IndexType, DistanceType>>& IndicesDists,
        const SearchParameters&                           searchParams = {}) const
    {
        RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndicesDists);
        const Size nFound = radiusSearchCustomCallback(query_point, resultSet, searchParams);
        return nFound;
    }

    /**
     * Just like radiusSearch() but with a custom callback class for each point
     * found in the radius of the query. See the source of RadiusResultSet<> as
     * a start point for your own classes. \sa radiusSearch
     */
    template <class SEARCH_CALLBACK>
    NANOFLANN_NODISCARD Size radiusSearchCustomCallback(
        const ElementType* query_point, SEARCH_CALLBACK& resultSet,
        const SearchParameters& searchParams = {}) const
    {
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /** @} */

   public:
   public:
    /**  Stores the index in a binary file.
     *   IMPORTANT NOTE: The set of data points is NOT stored in the file, so
     * when loading the index object it must be constructed associated to the
     * same source of data points used while building it. See the example:
     * examples/saveload_example.cpp \sa loadIndex  */
    void saveIndex(std::ostream& stream) { Base::saveIndex(*this, stream); }

    /**  Loads a previous index from a binary file.
     *   IMPORTANT NOTE: The set of data points is NOT stored in the file, so
     * the index object must be constructed associated to the same source of
     * data points used while building the index. See the example:
     * examples/saveload_example.cpp \sa loadIndex  */
    void loadIndex(std::istream& stream) { Base::loadIndex(*this, stream); }
};

/** kd-tree dynamic index
 *
 * class to create multiple static index and merge their results to behave as
 * single dynamic index as proposed in Logarithmic Approach.
 *
 *  Example of usage:
 *  examples/dynamic_pointcloud_example.cpp
 *
 * \tparam DatasetAdaptor The user-provided adaptor (see comments above).
 * \tparam Distance The distance metric to use: nanoflann::metric_L1,
 * nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc. \tparam DIM
 * Dimensionality of data points (e.g. 3 for 3D points) \tparam IndexType
 * Will be typically size_t or int
 */
template <typename Distance, class DatasetAdaptor, int32_t DIM = -1, typename IndexType = uint32_t>
class KDTreeSingleIndexDynamicAdaptor
{
   public:
    using ElementType  = typename Distance::ElementType;
    using DistanceType = typename Distance::DistanceType;

    using Offset = typename KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM>::Offset;
    using Size   = typename KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM>::Size;
    using Dimension =
        typename KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM>::Dimension;

   protected:
    Size leaf_max_size_;
    Size treeCount_;
    Size pointCount_;

    /**
     * The dataset used by this index
     */
    const DatasetAdaptor& dataset_;  //!< The source of our data

    /** treeIndex[idx] is the index of tree in which point at idx is stored.
     * treeIndex[idx]=-1 means that point has been removed. */
    std::vector<int> treeIndex_;
    /** Maps each currently-removed point index to the sub-tree that still
     * physically holds it (removal is lazy). Used to reactivate the point in
     * place if it is later re-added, instead of inserting a duplicate. */
    std::unordered_map<IndexType, int> removedPoints_;

    KDTreeSingleIndexAdaptorParams index_params_;

    Dimension dim_;  //!< Dimensionality of each data point

    using index_container_t =
        KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM, IndexType>;
    std::vector<index_container_t> index_;

   public:
    /** Get a const ref to the internal list of indices; the number of indices
     * is adapted dynamically as the dataset grows in size. */
    const std::vector<index_container_t>& getAllIndices() const { return index_; }

   private:
    /** finds position of least significant unset bit */
    int First0Bit(Size num)
    {
        int pos = 0;
        while (num & 1)
        {
            num = num >> 1;
            pos++;
        }
        return pos;
    }

    /** Creates multiple empty trees to handle dynamic support */
    void init()
    {
        using my_kd_tree_t =
            KDTreeSingleIndexDynamicAdaptor_<Distance, DatasetAdaptor, DIM, IndexType>;
        std::vector<my_kd_tree_t> index(
            treeCount_, my_kd_tree_t(dim_ /*dim*/, dataset_, treeIndex_, index_params_));
        index_ = index;
    }

   public:
    Distance distance_;

    /**
     * KDTree constructor
     *
     * Refer to docs in README.md or online in
     * https://github.com/jlblancoc/nanoflann
     *
     * The KD-Tree point dimension (the length of each point in the dataset, e.g.
     * 3 for 3D points) is determined by means of:
     *  - The \a DIM template parameter if >0 (highest priority)
     *  - Otherwise, the \a dimensionality parameter of this constructor.
     *
     * @param inputData Dataset with the input features. Its lifetime must be
     *  equal or longer than that of the instance of this class.
     * @param params Basically, the maximum leaf node size
     */
    explicit KDTreeSingleIndexDynamicAdaptor(
        const int dimensionality, const DatasetAdaptor& inputData,
        const KDTreeSingleIndexAdaptorParams& params            = KDTreeSingleIndexAdaptorParams(),
        const size_t                          maximumPointCount = 1000000000U)
        : dataset_(inputData), index_params_(params), distance_(inputData)
    {
        treeCount_  = static_cast<size_t>(std::log2(maximumPointCount)) + 1;
        pointCount_ = 0U;
        dim_        = dimensionality;
        treeIndex_.clear();
        if (DIM > 0) dim_ = DIM;
        leaf_max_size_ = params.leaf_max_size;
        init();
        const size_t num_initial_points = dataset_.kdtree_get_point_count();
        if (num_initial_points > 0)
        {
            addPoints(0, static_cast<IndexType>(num_initial_points - 1));
        }
    }

    /** Deleted copy constructor*/
    explicit KDTreeSingleIndexDynamicAdaptor(
        const KDTreeSingleIndexDynamicAdaptor<Distance, DatasetAdaptor, DIM, IndexType>&) = delete;

    /** Add points to the set, Inserts all points from [start, end] */
    void addPoints(IndexType start, IndexType end)
    {
        int maxIndex = 0;
        for (IndexType idx = start; idx <= end; idx++)
        {
            // If this index was previously removed, its point is still
            // physically present in its sub-tree (removal is lazy and never
            // deletes from vAcc_). Just clear the "removed" mark and restore its
            // tree index. Re-inserting it would create a duplicate entry that
            // grows the trees without bound and yields duplicate search results.
            const auto it = removedPoints_.find(idx);
            if (it != removedPoints_.end())
            {
                treeIndex_[idx] = it->second;
                removedPoints_.erase(it);
                continue;
            }

            const int pos = First0Bit(pointCount_);
            maxIndex      = std::max(pos, maxIndex);
            if (treeIndex_.size() <= static_cast<size_t>(pointCount_))
                treeIndex_.resize(static_cast<size_t>(pointCount_) + 1);
            treeIndex_[pointCount_] = pos;

            for (int i = 0; i < pos; i++)
            {
                for (size_t j = 0; j < index_[i].vAcc_.size(); j++)
                {
                    const IndexType e = index_[i].vAcc_[j];
                    index_[pos].vAcc_.push_back(e);
                    if (treeIndex_[e] != -1)
                        treeIndex_[e] = pos;
                    else
                        removedPoints_[e] = pos;  // keep tombstone's tree index current
                }
                index_[i].vAcc_.clear();
            }
            index_[pos].vAcc_.push_back(idx);
            pointCount_++;
        }

        for (int i = 0; i <= maxIndex; ++i)
        {
            index_[i].freeIndex(index_[i]);
            if (!index_[i].vAcc_.empty()) index_[i].buildIndex();
        }
    }

    /** Remove a point from the set (Lazy Deletion) */
    void removePoint(size_t idx)
    {
        if (idx >= pointCount_) return;
        if (treeIndex_[idx] == -1) return;  // already removed
        // Remember which sub-tree still physically holds this point, so it can
        // be reactivated in place if re-added later (see addPoints).
        removedPoints_[static_cast<IndexType>(idx)] = treeIndex_[idx];
        treeIndex_[idx]                             = -1;
    }

    /**
     * Find set of nearest neighbors to vec[0:dim-1]. Their indices are stored
     * inside the result object.
     *
     * Params:
     *     result = the result object in which the indices of the
     * nearest-neighbors are stored vec = the vector for which to search the
     * nearest neighbors
     *
     * \tparam RESULTSET Should be any ResultSet<DistanceType>
     * \return  True if the requested neighbors could be found.
     * \sa knnSearch, radiusSearch
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     */
    template <typename RESULTSET>
    bool findNeighbors(
        RESULTSET& result, const ElementType* vec, const SearchParameters& searchParams = {}) const
    {
        for (size_t i = 0; i < treeCount_; i++)
        {
            index_[i].findNeighbors(result, &vec[0], searchParams);
        }
        return result.full();
    }
};

/** Parameters for KDTreeSingleIndexIncrementalAdaptor.
 *
 * The two \a alpha_* thresholds drive the weight-balanced (scapegoat-style)
 * partial rebuilds:
 *  - \a alpha_balance : a subtree is rebuilt when the larger of its two child
 *    subtrees holds more than this fraction of the subtree's points. Lower
 *    values keep the tree better balanced (faster queries) at the price of more
 *    frequent rebuilds. Typical range [0.55, 0.85].
 *  - \a alpha_deleted : a subtree is rebuilt (physically dropping tombstoned
 *    points) when the fraction of removed points in it exceeds this value. Lower
 *    values reclaim memory more aggressively. Typical range [0.3, 0.7].
 */
struct KDTreeIncrementalIndexParams
{
    KDTreeIncrementalIndexParams(float alpha_balance_ = 0.75f, float alpha_deleted_ = 0.5f)
        : alpha_balance(alpha_balance_), alpha_deleted(alpha_deleted_)
    {
    }

    float alpha_balance;
    float alpha_deleted;
};

/** kd-tree incremental dynamic index — a single, self-balancing k-d tree.
 *
 * This is an additive alternative to KDTreeSingleIndexDynamicAdaptor (the
 * "logarithmic forest"). Instead of maintaining O(log N) static sub-trees, it
 * keeps a *single* weight-balanced k-d tree that supports cheap incremental
 * point insertion, lazy point removal, and pruned axis-aligned box deletions
 * (removeBox / removeOutsideBox), the latter being the primary map-maintenance
 * primitive used in LiDAR odometry (keep a local cube around the sensor and
 * discard everything outside it as the platform moves).
 *
 * Compared with the forest, the single tree avoids the multiplicative
 * O(log N) query penalty of querying every sub-tree, and keeps deletion garbage
 * bounded (a subtree is rebuilt once its dead fraction crosses
 * `alpha_deleted`), at the price of synchronous O(N) rebuild spikes near the
 * root.
 *
 * Like the other adaptors this is zero-copy: the data points live in the
 * user-provided dataset; the tree only stores point indices. Each tree node
 * holds a single point plus augmentation (subtree size, tombstone count and an
 * axis-aligned bounding box) used to prune the box-region operations.
 *
 * \note Threading: like the rest of nanoflann, const queries are safe for
 *       concurrent readers, but mutating operations (addPoints / removePoint /
 *       removeBox / removeOutsideBox) must not run concurrently with any query.
 *
 * \note A compile-time fixed \a DIM (e.g. 3 for 3D LiDAR) is recommended: the
 *       per-node bounding box is then a stack std::array and no per-node heap
 *       allocation occurs. With DIM=-1 each node's box is a std::vector.
 *
 * \tparam Distance The distance metric (nanoflann::metric_L2_Simple, etc).
 * \tparam DatasetAdaptor The user-provided dataset adaptor.
 * \tparam DIM Dimensionality of the data points (e.g. 3), or -1 for runtime.
 * \tparam IndexType Type used to index data points (e.g. uint32_t).
 */
template <typename Distance, class DatasetAdaptor, int32_t DIM = -1, typename IndexType = uint32_t>
class KDTreeSingleIndexIncrementalAdaptor
    : public KDTreeBaseClass<
          KDTreeSingleIndexIncrementalAdaptor<Distance, DatasetAdaptor, DIM, IndexType>, Distance,
          DatasetAdaptor, DIM, IndexType>
{
   public:
    using Base = typename nanoflann::KDTreeBaseClass<
        KDTreeSingleIndexIncrementalAdaptor<Distance, DatasetAdaptor, DIM, IndexType>, Distance,
        DatasetAdaptor, DIM, IndexType>;

    using ElementType  = typename Base::ElementType;
    using DistanceType = typename Base::DistanceType;

    using Offset    = typename Base::Offset;
    using Size      = typename Base::Size;
    using Dimension = typename Base::Dimension;

    using Interval          = typename Base::Interval;
    using BoundingBox       = typename Base::BoundingBox;
    using distance_vector_t = typename Base::distance_vector_t;

    /** The data source used by this index. */
    const DatasetAdaptor& dataset_;

    Distance distance_;

    /** Augmented tree node: stores a single data point plus the maintenance
     *  metadata. Children pointers are nullptr at the leaves. */
    struct INode
    {
        IndexType   ptIdx         = 0;  //!< index of the stored data point
        Dimension   divfeat       = 0;  //!< splitting axis at this node
        bool        deleted       = false;  //!< this node's point is tombstoned
        bool        treeDeleted   = false;  //!< whole subtree lazily tombstoned
        INode*      child1        = nullptr;  //!< "< split" child (also free-list link)
        INode*      child2        = nullptr;  //!< ">= split" child
        INode*      parent        = nullptr;  //!< parent (nullptr at the root)
        Size        subtree_size  = 0;  //!< number of nodes in this subtree
        Size        invalid_count = 0;  //!< number of tombstoned nodes in subtree
        BoundingBox box;  //!< AABB of all points (live+dead) in this subtree
        //! Cache of this node's own point coordinates, kept in-node to avoid the
        //! dataset_get() indirection on the hot query / insert / box paths. Only
        //! populated for a compile-time fixed DIM (`kCacheCoords`); for DIM=-1 it
        //! stays an empty vector and the code falls back to the dataset.
        typename array_or_vector<DIM, ElementType>::type pcoord;
    };

    /** Whether per-node coordinate caching is active: only for a compile-time
     *  fixed DIM, so the cache is a stack array and adds no per-node heap.
     *  Define NANOFLANN_INCREMENTAL_NO_COORD_CACHE to opt out (e.g. a very large
     *  ElementType) and always read coordinates from the dataset. */
#if defined(NANOFLANN_INCREMENTAL_NO_COORD_CACHE)
    static constexpr bool kCacheCoords = false;
#else
    static constexpr bool kCacheCoords = (DIM > 0);
#endif

   private:
    INode* iroot_    = nullptr;  //!< root of the incremental tree
    INode* freeList_ = nullptr;  //!< recycled nodes (linked via child1)

    Size liveCount_  = 0;  //!< number of live (non-tombstoned) points
    Size totalCount_ = 0;  //!< number of nodes physically in the tree

    float alphaBal_ = 0.75f;
    float alphaDel_ = 0.5f;
    /// Subtrees smaller than this are never rebuilt for *balance* reasons.
    static constexpr Size kMinBalanceRebuild = 4;
    /// addPoints() bulk-builds instead of inserting point-by-point when the
    /// batch is at least this fraction of the current live count (see addPoints).
    static constexpr double kBulkInsertFraction = 0.5;

    /// Highest unbalanced node found during the current insertion (rebuilt once).
    INode* pendingRebuild_ = nullptr;

    /// When false, the index never performs inline (synchronous) balance or
    /// deletion rebuilds: it only appends / lazily tombstones, and balance is
    /// restored externally by full bulk rebuilds. Used by the multi-threaded
    /// wrapper, which offloads the expensive rebuilds to a background thread.
    bool inlineRebuild_ = true;

    /// idx -> node holding that point (nullptr if the point is not present).
    std::vector<INode*> nodeOfPoint_;

    /// Scratch buffer reused across rebuilds (live indices being re-balanced).
    std::vector<IndexType> buildBuf_;

    /// Optional sink for physically-evicted point indices (acquireRemovedPoints).
    bool                   collectRemoved_ = false;
    std::vector<IndexType> removedSink_;

   public:
    /** Constructor.
     * @param dimensionality Runtime dimensionality (ignored if DIM>0).
     * @param inputData Dataset adaptor; its lifetime must outlive this index.
     * @param params Balancing thresholds (see KDTreeIncrementalIndexParams).
     *
     * The tree starts empty regardless of the dataset size; call addPoints()
     * to insert points (their indices must be valid in \a inputData).
     */
    explicit KDTreeSingleIndexIncrementalAdaptor(
        const Dimension dimensionality, const DatasetAdaptor& inputData,
        const KDTreeIncrementalIndexParams& params = {})
        : dataset_(inputData), distance_(inputData)
    {
        Base::dim_ = dimensionality;
        if (DIM > 0) Base::dim_ = DIM;
        alphaBal_ = params.alpha_balance;
        alphaDel_ = params.alpha_deleted;
        resize(Base::root_bbox_, static_cast<Dimension>(this->veclen(*this)));
    }

    /** Deleted copy constructor (owns raw node memory). */
    KDTreeSingleIndexIncrementalAdaptor(const KDTreeSingleIndexIncrementalAdaptor&) = delete;
    KDTreeSingleIndexIncrementalAdaptor& operator=(const KDTreeSingleIndexIncrementalAdaptor&) =
        delete;

    ~KDTreeSingleIndexIncrementalAdaptor() { destroyNodeObjects(); }

    /** \name Modifiers
     * @{ */

    /** Insert a single point (by its index in the dataset). */
    void addPoint(IndexType idx)
    {
        ensureNodeMap(idx);
        insertOne(idx);
        syncRootBox();
    }

    /** Insert all points with indices in the inclusive range [start, end].
     *
     *  When the batch is large relative to the current tree (an empty tree, or
     *  a batch comparable to the live count — e.g. a full LiDAR scan rebuilding
     *  a heavily-trimmed map) it is cheaper to flatten the live points and
     *  bulk-build one balanced tree than to descend-insert each point and
     *  trigger near-root scapegoat rebuilds. Small batches relative to a large
     *  map take the incremental per-point path. */
    void addPoints(IndexType start, IndexType end)
    {
        if (end < start) return;
        ensureNodeMap(end);
        const Size batch = static_cast<Size>(end - start) + 1;
        if (!iroot_ ||
            static_cast<double>(batch) >= kBulkInsertFraction * static_cast<double>(liveCount_))
        {
            buildBuf_.clear();
            if (iroot_) collectLiveAndFree(iroot_, buildBuf_);  // keep existing live points
            for (IndexType idx = start; idx <= end; ++idx) buildBuf_.push_back(idx);
            iroot_      = buildBalanced(buildBuf_, 0, buildBuf_.size(), 0, nullptr);
            liveCount_  = buildBuf_.size();
            totalCount_ = liveCount_;
        }
        else
        {
            for (IndexType idx = start; idx <= end; ++idx) insertOne(idx);
        }
        syncRootBox();
    }

    /** Lazily remove the point with the given index (no-op if absent/removed). */
    void removePoint(IndexType idx)
    {
        if (idx >= nodeOfPoint_.size()) return;
        INode* n = nodeOfPoint_[idx];
        if (!n || n->deleted) return;
        // A point can also be logically dead via a treeDeleted ancestor (a
        // lazily-killed box region). Detect that and treat it as already gone.
        for (INode* p = n->parent; p; p = p->parent)
            if (p->treeDeleted) return;

        n->deleted = true;
        for (INode* p = n; p; p = p->parent) ++p->invalid_count;
        --liveCount_;
        maybeRebuildForDeletion();
        syncRootBox();
    }

    /** Remove every live point lying inside the axis-aligned box \a box. */
    void removeBox(const BoundingBox& box)
    {
        if (iroot_) removeBoxRec(iroot_, box);
        maybeRebuildForDeletion();
        syncRootBox();
    }

    /** Remove every live point lying *outside* the axis-aligned box \a keep.
     *  This is the LiDAR sliding-window map-trimming primitive. */
    void removeOutsideBox(const BoundingBox& keep)
    {
        if (iroot_) removeOutsideBoxRec(iroot_, keep);
        maybeRebuildForDeletion();
        syncRootBox();
    }

    /** Enable/disable recording of physically-evicted point indices, returned
     *  by acquireRemovedPoints(). Off by default (cost-free when unused). */
    void setCollectRemovedPoints(bool enable)
    {
        collectRemoved_ = enable;
        if (!enable) std::vector<IndexType>().swap(removedSink_);
    }

    /** Move out the list of point indices physically dropped (during rebuilds)
     *  since the last call. Requires setCollectRemovedPoints(true). */
    std::vector<IndexType> acquireRemovedPoints()
    {
        std::vector<IndexType> out;
        out.swap(removedSink_);
        return out;
    }

    /** Enable/disable inline (synchronous) rebalancing. When disabled the index
     *  only appends and lazily tombstones; balance must be restored externally
     *  via buildFromIndices(). Used by the multi-threaded wrapper. */
    void setInlineRebuild(bool enable) { inlineRebuild_ = enable; }

    /** Append the live point indices (DFS, skipping tombstones) into \a out.
     *  Non-destructive; used to snapshot the tree for a background rebuild. */
    void snapshotLiveIndices(std::vector<IndexType>& out) const { snapshotRec(iroot_, out); }

    /** Append EVERY physically-stored point index (live and tombstoned) into
     *  \a out. Used by the multi-threaded wrapper to detect which dataset slots
     *  become free after a background rebuild swap. */
    void collectPhysicalIndices(std::vector<IndexType>& out) const { collectAllRec(iroot_, out); }

    /** True if some tree node currently references the given point index (i.e.
     *  the dataset slot is in use and must not be recycled). */
    NANOFLANN_NODISCARD bool referencesIndex(IndexType idx) const
    {
        return idx < nodeOfPoint_.size() && nodeOfPoint_[idx] != nullptr;
    }

    /** Discard the current tree and bulk-build a fresh, balanced tree over the
     *  given point indices. O(M log M). Reuses recycled nodes via the pool. */
    void buildFromIndices(const std::vector<IndexType>& idxs)
    {
        // Validate (and grow the point->node map) before touching the current
        // tree, so a rejected index list leaves the index untouched.
        IndexType maxIdx = 0;
        for (IndexType v : idxs) maxIdx = std::max(maxIdx, v);
        if (!idxs.empty()) ensureNodeMap(maxIdx);

        if (iroot_)
        {
            buildBuf_.clear();
            collectLiveAndFree(iroot_, buildBuf_);  // recycle existing nodes
            iroot_ = nullptr;
        }
        buildBuf_.assign(idxs.begin(), idxs.end());
        iroot_      = buildBalanced(buildBuf_, 0, buildBuf_.size(), 0, nullptr);
        liveCount_  = buildBuf_.size();
        totalCount_ = liveCount_;
        syncRootBox();
    }

    /** @} */

    /** \name Capacity / observers
     * @{ */

    /** Number of live (non-removed) points currently in the index. */
    NANOFLANN_NODISCARD Size size() const noexcept { return liveCount_; }
    NANOFLANN_NODISCARD bool empty() const noexcept { return liveCount_ == 0; }

    /** Number of nodes physically stored (live + not-yet-reclaimed tombstones). */
    NANOFLANN_NODISCARD Size physicalSize() const noexcept { return totalCount_; }

    /** Approximate bytes used by the node pool and the index->node map. */
    NANOFLANN_NODISCARD Size usedMemory() const
    {
        return Base::pool_.usedMemory + Base::pool_.wastedMemory +
               nodeOfPoint_.capacity() * sizeof(INode*);
    }

    /** Axis-aligned bounding box of all points currently in the index (live and
     *  not-yet-reclaimed tombstones — a conservative superset of the live set).
     *  O(1): returns the cached root box. Meaningless if empty() (all zeros). */
    NANOFLANN_NODISCARD BoundingBox boundingBox() const { return Base::root_bbox_; }

    /** Pre-size the internal index->node map (and the rebuild scratch buffer) to
     *  avoid reallocations while the point count grows toward \a n. */
    void reserve(Size n)
    {
        nodeOfPoint_.reserve(n);
        buildBuf_.reserve(n);
    }

    /** @} */

    /** \name Query methods
     * @{ */

    /** Core search: find neighbors of \a vec, storing them in \a result. */
    template <typename RESULTSET>
    bool findNeighbors(
        RESULTSET& result, const ElementType* vec, const SearchParameters& searchParams = {}) const
    {
        assert(vec);
        if (!iroot_ || liveCount_ == 0) return false;
        const DistanceType epsError = 1 + static_cast<DistanceType>(searchParams.eps);

        distance_vector_t dists;
        assign(dists, this->veclen(*this), static_cast<typename distance_vector_t::value_type>(0));
        const DistanceType dist = this->computeInitialDistances(*this, vec, dists);
        searchLevelInc(result, vec, iroot_, dist, dists, epsError, this->veclen(*this));
        if (searchParams.sorted) result.sort();
        return result.full();
    }

    /** Find the \a num_closest nearest neighbors to \a query_point. */
    NANOFLANN_NODISCARD Size knnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const SearchParameters& searchParams = {}) const
    {
        nanoflann::KNNResultSet<DistanceType, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances);
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /** Radius search around \a query_point. */
    NANOFLANN_NODISCARD Size radiusSearch(
        const ElementType* query_point, const DistanceType& radius,
        std::vector<ResultItem<IndexType, DistanceType>>& IndicesDists,
        const SearchParameters&                           searchParams = {}) const
    {
        RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndicesDists);
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /** Custom-callback radius search. */
    template <class SEARCH_CALLBACK>
    NANOFLANN_NODISCARD Size radiusSearchCustomCallback(
        const ElementType* query_point, SEARCH_CALLBACK& resultSet,
        const SearchParameters& searchParams = {}) const
    {
        findNeighbors(resultSet, query_point, searchParams);
        return resultSet.size();
    }

    /** Radius-limited KNN around \a query_point. */
    NANOFLANN_NODISCARD Size rknnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const DistanceType& radius) const
    {
        nanoflann::RKNNResultSet<DistanceType, IndexType> resultSet(num_closest, radius);
        resultSet.init(out_indices, out_distances);
        findNeighbors(resultSet, query_point);
        return resultSet.size();
    }

    /** Find all live points contained within the box \a bbox. */
    template <typename RESULTSET>
    NANOFLANN_NODISCARD Size findWithinBox(RESULTSET& result, const BoundingBox& bbox) const
    {
        if (iroot_) findWithinBoxRec(result, iroot_, bbox);
        return result.size();
    }

    /** @} */

    /** \name Persistence (index topology only; the dataset is NOT stored)
     * @{ */

    /** Magic number written at the start of every saveIndex() stream of this
     *  incremental index. Distinct from the static KDTreeSingleIndexAdaptor's
     *  SAVE_MAGIC so that loading the wrong kind of file fails fast with a
     *  clear error instead of silently misinterpreting the bytes.
     *  Spells 'NFLI' (nanoflann incremental) in ASCII. */
    static constexpr uint32_t INCREMENTAL_SAVE_MAGIC = 0x4E464C49;

    /** Serializes the tree *topology* (split axis, tombstone flags, and the
     *  live/dead tree shape) to a binary stream. Point coordinates are NOT
     *  stored: as with the static index's saveIndex(), the object must be
     *  reattached to a dataset with the very same content before loadIndex()
     *  is called on it.
     *
     *  Every other per-node field (bounding box, subtree size, tombstone
     *  counts, coordinate cache, parent pointer) is recomputed on load instead
     *  of stored, since they are all structural invariants derivable from the
     *  fields above; this keeps the format independent of DIM being
     *  compile-time fixed or runtime, unlike a raw struct byte-dump.
     *
     * \note Portability limitations as in the static index's saveIndex(): not
     *   portable across endianness, 32- vs 64-bit size_t, nanoflann versions,
     *   or IndexType/ElementType/DistanceType instantiations (checked, throws
     *   on mismatch).
     *
     * \sa loadIndex
     */
    void saveIndex(std::ostream& stream) const
    {
        const uint32_t hdr_magic   = INCREMENTAL_SAVE_MAGIC;
        const uint32_t hdr_version = static_cast<uint32_t>(NANOFLANN_VERSION);
        const uint8_t  hdr_sz_st   = static_cast<uint8_t>(sizeof(size_t));
        const uint8_t  hdr_sz_idx  = static_cast<uint8_t>(sizeof(IndexType));
        const uint8_t  hdr_sz_elem = static_cast<uint8_t>(sizeof(ElementType));
        const uint8_t  hdr_sz_dist = static_cast<uint8_t>(sizeof(DistanceType));
        save_value(stream, hdr_magic);
        save_value(stream, hdr_version);
        save_value(stream, hdr_sz_st);
        save_value(stream, hdr_sz_idx);
        save_value(stream, hdr_sz_elem);
        save_value(stream, hdr_sz_dist);

        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        save_value(stream, dims);

        const uint8_t hasRoot = iroot_ ? 1 : 0;
        save_value(stream, hasRoot);
        if (iroot_) saveNode(stream, iroot_);
    }

    /** Loads a tree topology previously saved with saveIndex().
     *
     * \note Must be called on a freshly constructed index (mirrors the static
     *   index's loadIndex() precondition): loading into an already-populated
     *   tree is not supported and would leak its nodes. The object must
     *   already be attached (via the constructor) to a dataset holding the
     *   very same points that were indexed when saveIndex() was called.
     *
     * \throws std::runtime_error on a magic-number/version/type-size/
     *   dimensionality mismatch, or a stream read error.
     *
     * \sa saveIndex
     */
    void loadIndex(std::istream& stream)
    {
        uint32_t magic = 0;
        load_value(stream, magic);
        if (stream.fail() || magic != INCREMENTAL_SAVE_MAGIC)
        {
            throw std::runtime_error(
                "KDTreeSingleIndexIncrementalAdaptor::loadIndex: invalid file (wrong magic "
                "number). The stream was not written by this class' saveIndex(), or was written "
                "by the static KDTreeSingleIndexAdaptor instead.");
        }

        uint32_t file_version = 0;
        load_value(stream, file_version);
        if (file_version != static_cast<uint32_t>(NANOFLANN_VERSION))
        {
            char msg[200];
            snprintf(
                msg, sizeof(msg),
                "KDTreeSingleIndexIncrementalAdaptor::loadIndex: version mismatch "
                "(file=0x%03X, library=0x%03X). Rebuild the index.",
                file_version, static_cast<unsigned>(NANOFLANN_VERSION));
            throw std::runtime_error(msg);
        }

        uint8_t sz_size_t = 0;
        uint8_t sz_idx    = 0;
        uint8_t sz_elem   = 0;
        uint8_t sz_dist   = 0;
        load_value(stream, sz_size_t);
        load_value(stream, sz_idx);
        load_value(stream, sz_elem);
        load_value(stream, sz_dist);
        if (sz_size_t != static_cast<uint8_t>(sizeof(size_t)) ||
            sz_idx != static_cast<uint8_t>(sizeof(IndexType)) ||
            sz_elem != static_cast<uint8_t>(sizeof(ElementType)) ||
            sz_dist != static_cast<uint8_t>(sizeof(DistanceType)))
        {
            throw std::runtime_error(
                "KDTreeSingleIndexIncrementalAdaptor::loadIndex: type-size mismatch between "
                "saved index and current template instantiation (sizeof size_t / IndexType / "
                "ElementType / DistanceType differ). Rebuild the index.");
        }

        Dimension dims = 0;
        load_value(stream, dims);
        if (dims != static_cast<Dimension>(this->veclen(*this)))
        {
            throw std::runtime_error(
                "KDTreeSingleIndexIncrementalAdaptor::loadIndex: dimensionality mismatch "
                "between the saved index and this object's dataset.");
        }

        uint8_t hasRoot = 0;
        load_value(stream, hasRoot);
        iroot_ = hasRoot ? loadNode(stream, nullptr) : nullptr;

        if (stream.fail())
        {
            throw std::runtime_error(
                "KDTreeSingleIndexIncrementalAdaptor::loadIndex: unexpected end of stream or "
                "read error.");
        }

        totalCount_ = iroot_ ? iroot_->subtree_size : 0;
        liveCount_  = iroot_ ? iroot_->subtree_size - iroot_->invalid_count : 0;
        syncRootBox();
    }

    /** @} */

   private:
    // --------------------------------------------------------------------
    //  Node allocation (bump-allocate from the pool, recycle via free-list)
    // --------------------------------------------------------------------
    INode* allocNode()
    {
        if (freeList_)
        {
            INode* n  = freeList_;
            freeList_ = n->child1;
            return n;  // already constructed; box storage reused
        }
        INode* n = Base::pool_.template allocate<INode>();
        // Placement-new so that, for DIM=-1, the std::vector box is constructed.
        ::new (static_cast<void*>(n)) INode();
        resize(n->box, static_cast<Dimension>(this->veclen(*this)));
        if (kCacheCoords) resize(n->pcoord, static_cast<Dimension>(this->veclen(*this)));
        return n;
    }

    /** Fill the node's cached coordinates from the dataset (fixed DIM only). */
    void cacheCoords(INode* n)
    {
        if (!kCacheCoords) return;
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i) n->pcoord[i] = pt(n->ptIdx, i);
    }

    /** Coordinate of a node's own point along axis \a d (cached for fixed DIM). */
    ElementType nodeCoord(const INode* n, Dimension d) const
    {
        return kCacheCoords ? n->pcoord[d] : pt(n->ptIdx, d);
    }

    /** Whether a node's own point lies inside box \a b (uses the coord cache). */
    bool nodeInBox(const INode* n, const BoundingBox& b) const
    {
        if (!kCacheCoords) return pointInBox(n->ptIdx, b);
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
            if (n->pcoord[i] < b[i].low || n->pcoord[i] > b[i].high) return false;
        return true;
    }

    void recycleNode(INode* n)
    {
        n->child1 = freeList_;
        freeList_ = n;
    }

    /** Destroy all constructed INode objects (only needed when the box type is
     *  not trivially destructible, i.e. DIM=-1 where it owns a std::vector). */
    void destroyNodeObjects()
    {
        if (std::is_trivially_destructible<INode>::value) return;
        destroySubtree(iroot_);
        iroot_ = nullptr;
        while (freeList_)
        {
            INode* n  = freeList_;
            freeList_ = n->child1;
            n->~INode();
        }
    }

    void destroySubtree(INode* n)
    {
        if (!n) return;
        destroySubtree(n->child1);
        destroySubtree(n->child2);
        n->~INode();
    }

    // --------------------------------------------------------------------
    //  Helpers
    // --------------------------------------------------------------------
    ElementType pt(IndexType idx, Dimension d) const { return dataset_.kdtree_get_pt(idx, d); }

    void ensureNodeMap(IndexType idx)
    {
        // The point->node map is grown to hold `idx`, so a bogus index is not a
        // wrong result but an out-of-memory: the maximum IndexType value alone
        // asks for a 2^32-entry (32 GB) map on the default uint32_t. That value
        // is what `static_cast<IndexType>(n - 1)` produces when a caller forgets
        // to special-case an empty (n == 0) dataset, and it would additionally
        // make the [start,end] loops of addPoints() wrap around forever, so
        // reject it here, where every index-taking entry point passes through.
        if (idx == (std::numeric_limits<IndexType>::max)())
        {
            throw std::invalid_argument(
                "[nanoflann] KDTreeSingleIndexIncrementalAdaptor: point index equal to the "
                "maximum IndexType value; this is almost certainly an underflowed 'size - 1' "
                "on an empty dataset.");
        }
        if (idx >= nodeOfPoint_.size()) nodeOfPoint_.resize(static_cast<size_t>(idx) + 1, nullptr);
    }

    void syncRootBox()
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        if (iroot_)
            for (Dimension i = 0; i < dims; ++i) Base::root_bbox_[i] = iroot_->box[i];
        else
            for (Dimension i = 0; i < dims; ++i) Base::root_bbox_[i] = Interval{0, 0};
    }

    void initBoxToPoint(INode* n)
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
        {
            const ElementType v = pt(n->ptIdx, i);
            n->box[i].low = n->box[i].high = v;
        }
    }

    void expandBoxToPoint(INode* n, IndexType idx)
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
        {
            const ElementType v = pt(idx, i);
            if (v < n->box[i].low) n->box[i].low = v;
            if (v > n->box[i].high) n->box[i].high = v;
        }
    }

    void unionBox(INode* n, const INode* c)
    {
        if (!c) return;
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
        {
            if (c->box[i].low < n->box[i].low) n->box[i].low = c->box[i].low;
            if (c->box[i].high > n->box[i].high) n->box[i].high = c->box[i].high;
        }
    }

    bool pointInBox(IndexType idx, const BoundingBox& b) const
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
        {
            const ElementType v = pt(idx, i);
            if (v < b[i].low || v > b[i].high) return false;
        }
        return true;
    }

    bool boxFullyInside(const BoundingBox& inner, const BoundingBox& outer) const
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
            if (inner[i].low < outer[i].low || inner[i].high > outer[i].high) return false;
        return true;
    }

    bool boxDisjoint(const BoundingBox& a, const BoundingBox& b) const
    {
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        for (Dimension i = 0; i < dims; ++i)
            if (a[i].high < b[i].low || a[i].low > b[i].high) return true;
        return false;
    }

    // --------------------------------------------------------------------
    //  Insertion
    // --------------------------------------------------------------------
    INode* makeLeaf(IndexType idx, Dimension depth, INode* parent)
    {
        INode*          n    = allocNode();
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));
        n->ptIdx             = idx;
        n->divfeat           = static_cast<Dimension>(depth % dims);
        n->deleted           = false;
        n->treeDeleted       = false;
        n->child1 = n->child2 = nullptr;
        n->parent             = parent;
        n->subtree_size       = 1;
        n->invalid_count      = 0;
        initBoxToPoint(n);
        cacheCoords(n);
        nodeOfPoint_[idx] = n;
        return n;
    }

    /** Insert one point and, in the same pass, rebuild the highest unbalanced
     *  node on the insertion path (BB[alpha] scapegoat rebalancing). */
    void insertOne(IndexType idx)
    {
        pendingRebuild_ = nullptr;
        iroot_          = insertRec(iroot_, idx, 0, nullptr);
        ++liveCount_;
        ++totalCount_;
        if (pendingRebuild_ && inlineRebuild_) rebuildAt(pendingRebuild_);
    }

    INode* insertRec(INode* node, IndexType idx, Dimension depth, INode* parent)
    {
        if (!node) return makeLeaf(idx, depth, parent);
        if (node->treeDeleted) pushDownDelete(node);

        ++node->subtree_size;
        expandBoxToPoint(node, idx);

        const Dimension axis = node->divfeat;
        if (pt(idx, axis) < nodeCoord(node, axis))
            node->child1 = insertRec(node->child1, idx, static_cast<Dimension>(depth + 1), node);
        else
            node->child2 = insertRec(node->child2, idx, static_cast<Dimension>(depth + 1), node);

        // On the unwind, remember the *highest* unbalanced node seen on the
        // path (ancestors are visited after descendants, so the last write
        // wins). insertOne() rebuilds it once, avoiding a second descent.
        if (isBalanceScapegoat(node)) pendingRebuild_ = node;
        return node;
    }

    /** Make the lazy whole-subtree tombstone one level explicit so the subtree
     *  is consistent before we descend into it for an insertion. */
    void pushDownDelete(INode* node)
    {
        node->deleted = true;
        if (node->child1)
        {
            node->child1->treeDeleted   = true;
            node->child1->invalid_count = node->child1->subtree_size;
        }
        if (node->child2)
        {
            node->child2->treeDeleted   = true;
            node->child2->invalid_count = node->child2->subtree_size;
        }
        node->treeDeleted = false;  // invalid_count already == subtree_size
    }

    Size maxChildSize(const INode* node) const
    {
        const Size l = node->child1 ? node->child1->subtree_size : 0;
        const Size r = node->child2 ? node->child2->subtree_size : 0;
        return l > r ? l : r;
    }

    bool isBalanceScapegoat(const INode* node) const
    {
        if (node->subtree_size < kMinBalanceRebuild) return false;
        return static_cast<float>(maxChildSize(node)) >
               alphaBal_ * static_cast<float>(node->subtree_size);
    }

    // --------------------------------------------------------------------
    //  Deletion (lazy) + box-region deletion
    // --------------------------------------------------------------------
    /** Kill an entire subtree in O(1): mark it as wholly tombstoned. */
    void killSubtree(INode* node)
    {
        node->treeDeleted   = true;
        node->invalid_count = node->subtree_size;
    }

    /** Remove points outside \a keep. Returns the number newly tombstoned. */
    Size removeOutsideBoxRec(INode* node, const BoundingBox& keep)
    {
        if (!node) return 0;
        if (node->invalid_count == node->subtree_size) return 0;  // already all dead
        if (boxFullyInside(node->box, keep)) return 0;  // keep entire subtree
        if (boxDisjoint(node->box, keep))
        {
            const Size newly = node->subtree_size - node->invalid_count;
            killSubtree(node);
            liveCount_ -= newly;
            return newly;
        }
        Size newly = 0;
        if (!node->deleted && !nodeInBox(node, keep))
        {
            node->deleted = true;
            ++newly;
            --liveCount_;
        }
        newly += removeOutsideBoxRec(node->child1, keep);
        newly += removeOutsideBoxRec(node->child2, keep);
        node->invalid_count += newly;
        return newly;
    }

    /** Remove points inside \a box. Returns the number newly tombstoned. */
    Size removeBoxRec(INode* node, const BoundingBox& box)
    {
        if (!node) return 0;
        if (node->invalid_count == node->subtree_size) return 0;
        if (boxDisjoint(node->box, box)) return 0;  // nothing inside
        if (boxFullyInside(node->box, box))
        {
            const Size newly = node->subtree_size - node->invalid_count;
            killSubtree(node);
            liveCount_ -= newly;
            return newly;
        }
        Size newly = 0;
        if (!node->deleted && nodeInBox(node, box))
        {
            node->deleted = true;
            ++newly;
            --liveCount_;
        }
        newly += removeBoxRec(node->child1, box);
        newly += removeBoxRec(node->child2, box);
        node->invalid_count += newly;
        return newly;
    }

    bool isDeletionScapegoat(const INode* node) const
    {
        if (node->subtree_size == 0) return false;
        return static_cast<float>(node->invalid_count) >
               alphaDel_ * static_cast<float>(node->subtree_size);
    }

    INode* findDeletionScapegoat(INode* node) const
    {
        if (!node) return nullptr;
        if (isDeletionScapegoat(node)) return node;  // highest wins
        if (INode* l = findDeletionScapegoat(node->child1)) return l;
        return findDeletionScapegoat(node->child2);
    }

    void maybeRebuildForDeletion()
    {
        if (!iroot_ || !inlineRebuild_) return;
        if (INode* sg = findDeletionScapegoat(iroot_)) rebuildAt(sg);
    }

    // --------------------------------------------------------------------
    //  Partial rebuild (scapegoat): flatten live points, rebuild balanced
    // --------------------------------------------------------------------
    void rebuildAt(INode* node)
    {
        INode*  par  = node->parent;
        INode** link = par ? (par->child1 == node ? &par->child1 : &par->child2) : &iroot_;

        const Size oldSize    = node->subtree_size;
        const Size oldInvalid = node->invalid_count;

        buildBuf_.clear();
        collectLiveAndFree(node, buildBuf_);

        INode* nb = buildBalanced(buildBuf_, 0, buildBuf_.size(), 0, par);
        *link     = nb;

        const Size newSize = nb ? nb->subtree_size : 0;  // == number of live pts
        // Propagate the change in (size, invalid) up to the ancestors.
        for (INode* p = par; p; p = p->parent)
        {
            p->subtree_size  = p->subtree_size - oldSize + newSize;
            p->invalid_count = p->invalid_count - oldInvalid;
        }
        totalCount_ = totalCount_ - oldSize + newSize;
    }

    /** Collect the live point indices under \a node (DFS) and recycle every
     *  node to the free-list. Tombstoned points are dropped (and optionally
     *  recorded for acquireRemovedPoints()). */
    void collectLiveAndFree(INode* node, std::vector<IndexType>& out)
    {
        if (!node) return;
        if (node->treeDeleted)
        {
            freeDeadSubtree(node);
            return;
        }
        if (!node->deleted)
            out.push_back(node->ptIdx);
        else
            dropDeadPoint(node->ptIdx);
        collectLiveAndFree(node->child1, out);
        collectLiveAndFree(node->child2, out);
        recycleNode(node);
    }

    void freeDeadSubtree(INode* node)
    {
        if (!node) return;
        dropDeadPoint(node->ptIdx);
        freeDeadSubtree(node->child1);
        freeDeadSubtree(node->child2);
        recycleNode(node);
    }

    void dropDeadPoint(IndexType idx)
    {
        if (idx < nodeOfPoint_.size()) nodeOfPoint_[idx] = nullptr;
        if (collectRemoved_) removedSink_.push_back(idx);
    }

    /** DFS collecting live point indices (skips tombstoned points/subtrees). */
    void snapshotRec(const INode* node, std::vector<IndexType>& out) const
    {
        if (!node) return;
        if (node->invalid_count == node->subtree_size) return;  // whole subtree dead
        if (!node->deleted) out.push_back(node->ptIdx);
        snapshotRec(node->child1, out);
        snapshotRec(node->child2, out);
    }

    /** DFS collecting every physically-stored index (live and tombstoned). */
    void collectAllRec(const INode* node, std::vector<IndexType>& out) const
    {
        if (!node) return;
        out.push_back(node->ptIdx);
        collectAllRec(node->child1, out);
        collectAllRec(node->child2, out);
    }

    // --------------------------------------------------------------------
    //  Persistence (see saveIndex()/loadIndex())
    // --------------------------------------------------------------------
    /** Writes one node and its subtree. Only the fields that cannot be
     *  derived from the rest are stored; see loadNode(). */
    void saveNode(std::ostream& stream, const INode* n) const
    {
        save_value(stream, n->ptIdx);
        save_value(stream, n->divfeat);
        save_value(stream, n->deleted);
        save_value(stream, n->treeDeleted);

        const uint8_t hasChild1 = n->child1 ? 1 : 0;
        save_value(stream, hasChild1);
        if (n->child1) saveNode(stream, n->child1);

        const uint8_t hasChild2 = n->child2 ? 1 : 0;
        save_value(stream, hasChild2);
        if (n->child2) saveNode(stream, n->child2);
    }

    /** Loads one node and its subtree. Reconstructs every field not written
     *  by saveNode() (box, subtree_size, invalid_count, coordinate cache,
     *  parent link, and the idx->node map entry) from invariants that hold
     *  regardless of how the tree was originally built:
     *   - subtree_size is always 1 + the children's (0 if absent).
     *   - invalid_count equals subtree_size for a treeDeleted node (see
     *     killSubtree()), otherwise it is this node's own deleted flag plus
     *     the children's invalid_count.
     *   - box is this node's point, unioned with the children's boxes.
     */
    INode* loadNode(std::istream& stream, INode* parent)
    {
        INode* n = allocNode();
        load_value(stream, n->ptIdx);
        load_value(stream, n->divfeat);
        load_value(stream, n->deleted);
        load_value(stream, n->treeDeleted);
        n->parent = parent;

        uint8_t hasChild1 = 0;
        load_value(stream, hasChild1);
        n->child1 = hasChild1 ? loadNode(stream, n) : nullptr;

        uint8_t hasChild2 = 0;
        load_value(stream, hasChild2);
        n->child2 = hasChild2 ? loadNode(stream, n) : nullptr;

        n->subtree_size = 1 + (n->child1 ? n->child1->subtree_size : 0) +
                          (n->child2 ? n->child2->subtree_size : 0);
        n->invalid_count = n->treeDeleted ? n->subtree_size
                                          : static_cast<Size>(n->deleted ? 1 : 0) +
                                                (n->child1 ? n->child1->invalid_count : 0) +
                                                (n->child2 ? n->child2->invalid_count : 0);

        initBoxToPoint(n);
        unionBox(n, n->child1);
        unionBox(n, n->child2);
        cacheCoords(n);

        ensureNodeMap(n->ptIdx);
        nodeOfPoint_[n->ptIdx] = n;

        return n;
    }

    /** Build a balanced subtree over buf[lo,hi) (median split on widest axis). */
    INode* buildBalanced(
        std::vector<IndexType>& buf, size_t lo, size_t hi, Dimension depth, INode* parent)
    {
        if (lo >= hi) return nullptr;
        const Dimension dims = static_cast<Dimension>(this->veclen(*this));

        // Widest-spread axis over buf[lo,hi).
        Dimension   axis     = static_cast<Dimension>(depth % dims);
        ElementType bestSpan = -1;
        for (Dimension d = 0; d < dims; ++d)
        {
            ElementType mn = pt(buf[lo], d), mx = mn;
            for (size_t k = lo + 1; k < hi; ++k)
            {
                const ElementType v = pt(buf[k], d);
                if (v < mn) mn = v;
                if (v > mx) mx = v;
            }
            const ElementType span = mx - mn;
            if (span > bestSpan)
            {
                bestSpan = span;
                axis     = d;
            }
        }

        const size_t mid = lo + (hi - lo) / 2;
        std::nth_element(
            buf.begin() + lo, buf.begin() + mid, buf.begin() + hi,
            [this, axis](IndexType a, IndexType b) { return pt(a, axis) < pt(b, axis); });

        INode* node       = allocNode();
        node->ptIdx       = buf[mid];
        node->divfeat     = axis;
        node->deleted     = false;
        node->treeDeleted = false;
        node->parent      = parent;
        cacheCoords(node);
        nodeOfPoint_[buf[mid]] = node;

        node->child1 = buildBalanced(buf, lo, mid, static_cast<Dimension>(depth + 1), node);
        node->child2 = buildBalanced(buf, mid + 1, hi, static_cast<Dimension>(depth + 1), node);

        node->subtree_size  = hi - lo;
        node->invalid_count = 0;
        initBoxToPoint(node);
        unionBox(node, node->child1);
        unionBox(node, node->child2);
        return node;
    }

    // --------------------------------------------------------------------
    //  Search
    // --------------------------------------------------------------------
    template <class RESULTSET>
    void searchLevelInc(
        RESULTSET& rs, const ElementType* vec, const INode* node, DistanceType mindist,
        distance_vector_t& dists, const DistanceType epsError, const Size dim) const
    {
        if (!node) return;
        if (node->invalid_count == node->subtree_size) return;  // whole subtree dead

        if (!node->deleted)
        {
#if defined(NANOFLANN_INCREMENTAL_INNODE_DISTANCE)
            // Opt-in: compute the node distance from the in-node coordinate cache
            // as a sum of per-axis accum_dist contributions. This avoids the
            // dataset_get() indirection and is ~12% faster on KNN, but is only
            // valid for *additive* (axis-decomposable) metrics — L1, L2,
            // L2_Simple. Do NOT enable it for SO2/SO3.
            DistanceType d = DistanceType();
            if (kCacheCoords)
                for (Size i = 0; i < dim; ++i)
                    d += distance_.accum_dist(
                        vec[i], node->pcoord[static_cast<Dimension>(i)], static_cast<Dimension>(i));
            else
                d = distance_.evalMetric(vec, node->ptIdx, dim);
#else
            const DistanceType d = distance_.evalMetric(vec, node->ptIdx, dim);
#endif
            if (d < rs.worstDist())
                rs.addPoint(
                    static_cast<typename RESULTSET::DistanceType>(d),
                    static_cast<typename RESULTSET::IndexType>(node->ptIdx));
        }

        const Dimension    axis     = node->divfeat;
        const ElementType  splitval = nodeCoord(node, axis);
        const ElementType  val      = vec[axis];
        const DistanceType cut      = distance_.accum_dist(val, splitval, axis);

        const INode* nearChild;
        const INode* farChild;
        if (val < splitval)
        {
            nearChild = node->child1;
            farChild  = node->child2;
        }
        else
        {
            nearChild = node->child2;
            farChild  = node->child1;
        }

        searchLevelInc(rs, vec, nearChild, mindist, dists, epsError, dim);

        const DistanceType dst    = dists[axis];
        const DistanceType newmin = mindist + cut - dst;
        dists[axis]               = cut;
        if (newmin * epsError <= rs.worstDist())
            searchLevelInc(rs, vec, farChild, newmin, dists, epsError, dim);
        dists[axis] = dst;
    }

    template <typename RESULTSET>
    void findWithinBoxRec(RESULTSET& result, const INode* node, const BoundingBox& bbox) const
    {
        if (!node) return;
        if (node->invalid_count == node->subtree_size) return;
        if (boxDisjoint(node->box, bbox)) return;
        if (!node->deleted && nodeInBox(node, bbox)) result.addPoint(0, node->ptIdx);
        findWithinBoxRec(result, node->child1, bbox);
        findWithinBoxRec(result, node->child2, bbox);
    }
};

#ifndef NANOFLANN_NO_THREADS
/** Multi-threaded variant of KDTreeSingleIndexIncrementalAdaptor that hides the
 *  O(N) near-root rebuild spike: the large balancing rebuild is performed on a
 *  background thread while the foreground tree keeps serving inserts, deletions
 *  and queries.
 *
 *  Model (single foreground thread + one background rebuild thread):
 *   - The live ("active") tree never rebalances inline; it only appends and
 *     lazily tombstones, so every foreground call returns quickly.
 *   - When the active tree has grown / accumulated tombstones past a threshold,
 *     a snapshot of its live point indices is taken and a background thread
 *     bulk-builds a fresh, balanced tree from it. Foreground operations meanwhile
 *     keep mutating the active tree and are appended to a small op-log.
 *     That background thread is **one persistent worker** started on the first
 *     rebuild and reused for every later one, not one thread per rebuild: a
 *     long-running incremental map triggers rebuilds continuously, and a thread
 *     per rebuild would make the host process churn through thousands of OS
 *     threads, multiplying every per-thread cost it carries (malloc arenas,
 *     sanitizer bookkeeping) for no benefit.
 *   - At the next foreground call after the build finishes, the op-log is
 *     replayed onto the fresh tree and it atomically replaces the active tree.
 *
 *  This keeps the foreground tail latency bounded (snapshot + replay) instead of
 *  paying the full O(N) rebuild inline. It matches ikd-Tree's async-rebuild idea
 *  but with a thread-isolated build (no per-node locks), a bounded `std::deque`-
 *  style op-log (no fixed 10⁶ queue), and no PCL/pthread dependency.
 *
 *  \warning Same threading contract as the synchronous index for the *foreground*
 *  thread (const queries are safe for concurrent readers; no concurrent writer).
 *  Additionally, because the background thread reads point coordinates from the
 *  dataset adaptor, the **dataset must keep stable element storage while a
 *  rebuild is in flight** (e.g. `reserve()` the backing vector, or use a
 *  std::deque) so that appends on the foreground thread do not reallocate it.
 *  Disabled entirely under NANOFLANN_NO_THREADS.
 */
template <typename Distance, class DatasetAdaptor, int32_t DIM = -1, typename IndexType = uint32_t>
class KDTreeSingleIndexIncrementalAdaptorMT
{
   public:
    using Inner = KDTreeSingleIndexIncrementalAdaptor<Distance, DatasetAdaptor, DIM, IndexType>;
    using ElementType  = typename Inner::ElementType;
    using DistanceType = typename Inner::DistanceType;
    using Size         = typename Inner::Size;
    using Dimension    = typename Inner::Dimension;
    using BoundingBox  = typename Inner::BoundingBox;

    /** Constructor.
     *  @param rebuild_growth Trigger a background rebuild once the physical node
     *         count exceeds this factor of the live count at the last rebuild
     *         (captures both appends and tombstone accumulation).
     *  @param min_rebuild_size Never trigger below this many physical nodes. */
    explicit KDTreeSingleIndexIncrementalAdaptorMT(
        const Dimension dimensionality, const DatasetAdaptor& inputData,
        const KDTreeIncrementalIndexParams& params = {}, double rebuild_growth = 1.3,
        Size min_rebuild_size = 10000)
        : dataset_(inputData),
          dim_(dimensionality),
          params_(params),
          rebuildGrowth_(rebuild_growth),
          minRebuildSize_(min_rebuild_size)
    {
        active_.reset(new Inner(dimensionality, inputData, params));
        active_->setInlineRebuild(false);
    }

    KDTreeSingleIndexIncrementalAdaptorMT(const KDTreeSingleIndexIncrementalAdaptorMT&) = delete;
    KDTreeSingleIndexIncrementalAdaptorMT& operator=(const KDTreeSingleIndexIncrementalAdaptorMT&) =
        delete;

    ~KDTreeSingleIndexIncrementalAdaptorMT()
    {
        // Lets any in-flight build finish before teardown: the worker reads the
        // caller's dataset, which is typically freed right after this returns.
        stopWorker();
    }

    /** \name Modifiers @{ */
    void addPoints(IndexType start, IndexType end)
    {
        integrateIfReady();
        active_->addPoints(start, end);
        if (building_) log_.push_back({OpKind::Add, start, end, {}});
        maybeTriggerRebuild();
    }
    void addPoint(IndexType idx) { addPoints(idx, idx); }

    void removePoint(IndexType idx)
    {
        integrateIfReady();
        active_->removePoint(idx);
        if (building_) log_.push_back({OpKind::Remove, idx, idx, {}});
        maybeTriggerRebuild();
    }
    void removeBox(const BoundingBox& box)
    {
        integrateIfReady();
        active_->removeBox(box);
        if (building_) log_.push_back({OpKind::RemoveBox, 0, 0, box});
        maybeTriggerRebuild();
    }
    void removeOutsideBox(const BoundingBox& keep)
    {
        integrateIfReady();
        active_->removeOutsideBox(keep);
        if (building_) log_.push_back({OpKind::RemoveOutsideBox, 0, 0, keep});
        maybeTriggerRebuild();
    }
    /** @} */

    /** \name Query methods (forwarded to the active tree) @{ */
    template <typename RESULTSET>
    bool findNeighbors(
        RESULTSET& result, const ElementType* vec, const SearchParameters& sp = {}) const
    {
        return active_->findNeighbors(result, vec, sp);
    }
    Size knnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const SearchParameters& sp = {}) const
    {
        return active_->knnSearch(query_point, num_closest, out_indices, out_distances, sp);
    }
    Size radiusSearch(
        const ElementType* query_point, const DistanceType& radius,
        std::vector<ResultItem<IndexType, DistanceType>>& IndicesDists,
        const SearchParameters&                           sp = {}) const
    {
        return active_->radiusSearch(query_point, radius, IndicesDists, sp);
    }
    Size rknnSearch(
        const ElementType* query_point, const Size num_closest, IndexType* out_indices,
        DistanceType* out_distances, const DistanceType& radius) const
    {
        return active_->rknnSearch(query_point, num_closest, out_indices, out_distances, radius);
    }
    template <typename RESULTSET>
    Size findWithinBox(RESULTSET& result, const BoundingBox& bbox) const
    {
        return active_->findWithinBox(result, bbox);
    }
    /** @} */

    /** \name Observers @{ */
    Size size() const noexcept { return active_->size(); }
    bool empty() const noexcept { return active_->empty(); }
    Size physicalSize() const noexcept { return active_->physicalSize(); }
    bool isRebuilding() const noexcept { return building_; }

    /** Live AABB of the active tree (see the synchronous index). */
    NANOFLANN_NODISCARD BoundingBox boundingBox() const { return active_->boundingBox(); }

    /** Append the live point indices of the active tree into \a out. */
    void snapshotLiveIndices(std::vector<IndexType>& out) const
    {
        active_->snapshotLiveIndices(out);
    }

    /** Pre-size the active tree's internal map (see the synchronous index). */
    void reserve(Size n) { active_->reserve(n); }

    /** Block until any in-flight background rebuild has been integrated. */
    void sync()
    {
        if (building_)
        {
            std::unique_lock<std::mutex> lk(workerMtx_);
            workerCvDone_.wait(lk, [this] { return resultReady_; });
        }
        integrateIfReady();
    }

    /** Access the underlying active tree (e.g. for further query types). */
    const Inner& activeIndex() const { return *active_; }
    /** @} */

    /** \name Persistence (see the synchronous index's saveIndex()/loadIndex())
     * @{ */

    /** Blocks until any in-flight background rebuild is integrated (see
     *  sync()), then serializes the active tree's topology. */
    void saveIndex(std::ostream& stream)
    {
        sync();
        active_->saveIndex(stream);
    }

    /** Blocks until any in-flight background rebuild is integrated, then loads
     *  a previously saved topology into the active tree (see the synchronous
     *  index's loadIndex() precondition: the active tree must be freshly
     *  constructed / empty). */
    void loadIndex(std::istream& stream)
    {
        sync();
        active_->loadIndex(stream);
        lastBuildLive_ = active_->size();
    }
    /** @} */

    /** Set a callback invoked **on the background worker thread** on each freshly
     *  built tree, right after it is balanced and before it is handed back for
     *  integration. Lets the caller recompute per-point auxiliary data (e.g.
     *  covariances) off the foreground thread. The callback runs concurrently
     *  with foreground queries on the *old* tree, so it must only touch the
     *  passed-in fresh index and its own/snapshot data — never foreground-shared
     *  state without external synchronization. Pass {} to clear. */
    void setRebuildCallback(std::function<void(Inner&)> cb) { rebuildCallback_ = std::move(cb); }

    /** \name Dataset-storage reclamation @{ */

    /** Enable recording of dataset slots that become free when a background
     *  rebuild drops tombstoned points (so the caller can recycle them and keep
     *  the dataset bounded). Off by default (cost-free when unused). */
    void setCollectRemovedPoints(bool enable)
    {
        collectRemoved_ = enable;
        if (!enable) std::vector<IndexType>().swap(removedSink_);
    }

    /** Move out the point indices whose dataset slots became free since the last
     *  call (no live OR tombstoned tree node references them — safe to recycle).
     *  Requires setCollectRemovedPoints(true). */
    std::vector<IndexType> acquireRemovedPoints()
    {
        std::vector<IndexType> out;
        out.swap(removedSink_);
        return out;
    }
    /** @} */

   private:
    enum class OpKind
    {
        Add,
        Remove,
        RemoveBox,
        RemoveOutsideBox
    };
    struct LoggedOp
    {
        OpKind      kind;
        IndexType   a, b;
        BoundingBox box;
    };

    void maybeTriggerRebuild()
    {
        if (building_) return;
        const Size phys = active_->physicalSize();
        if (phys < minRebuildSize_) return;
        const Size base = lastBuildLive_ ? lastBuildLive_ : Size(1);
        if (static_cast<double>(phys) < rebuildGrowth_ * static_cast<double>(base)) return;

        // Snapshot the live indices on the foreground thread, then hand them to
        // the background worker, which bulk-builds a fresh balanced tree.
        auto snapshot = std::make_shared<std::vector<IndexType>>();
        active_->snapshotLiveIndices(*snapshot);

        // Started on the first rebuild only, so an index that never rebuilds
        // costs no thread at all:
        if (!workerThread_.joinable())
        {
            workerThread_ = std::thread(&KDTreeSingleIndexIncrementalAdaptorMT::workerLoop, this);
        }

        {
            std::lock_guard<std::mutex> lk(workerMtx_);
            pendingJob_ = std::move(snapshot);
            // Copied per job, so the worker never reads rebuildCallback_ while
            // setRebuildCallback() writes it:
            pendingCallback_ = rebuildCallback_;
            builtTree_.reset();
            buildError_  = nullptr;
            resultReady_ = false;
        }
        workerCvJob_.notify_one();

        building_ = true;
        log_.clear();
    }

    /** Body of the single persistent rebuild worker: wait for a job, bulk-build
     *  a balanced tree from it, publish the result, repeat. */
    void workerLoop()
    {
        for (;;)
        {
            std::shared_ptr<std::vector<IndexType>> job;
            std::function<void(Inner&)>             cb;
            {
                std::unique_lock<std::mutex> lk(workerMtx_);
                workerCvJob_.wait(lk, [this] { return workerStop_ || pendingJob_ != nullptr; });
                // A job already handed over is always finished before stopping:
                // the destructor relies on that to keep the caller's dataset
                // alive for as long as the build reads it.
                if (workerStop_ && !pendingJob_) return;
                job = std::move(pendingJob_);
                cb  = std::move(pendingCallback_);
            }

            // dim_, dataset_ and params_ are set at construction and never
            // mutated, so the worker reads them without synchronization.
            std::unique_ptr<Inner> t;
            std::exception_ptr     err;
            try
            {
                t.reset(new Inner(dim_, dataset_, params_));
                t->setInlineRebuild(false);
                t->buildFromIndices(*job);
                if (cb) cb(*t);  // background post-rebuild hook (e.g. recompute covariances)
            }
            catch (...)
            {
                // Handed to the foreground thread instead of terminating the
                // process; see integrateIfReady().
                err = std::current_exception();
                t.reset();
            }

            {
                std::lock_guard<std::mutex> lk(workerMtx_);
                builtTree_   = std::move(t);
                buildError_  = err;
                resultReady_ = true;
            }
            workerCvDone_.notify_all();
        }
    }

    /** Stops the worker once its current build (if any) is done, and joins it. */
    void stopWorker()
    {
        if (!workerThread_.joinable()) return;
        {
            std::lock_guard<std::mutex> lk(workerMtx_);
            workerStop_ = true;
        }
        workerCvJob_.notify_all();
        workerThread_.join();
    }

    void integrateIfReady()
    {
        if (!building_) return;

        std::unique_ptr<Inner> fresh;
        std::exception_ptr     err;
        {
            std::lock_guard<std::mutex> lk(workerMtx_);
            if (!resultReady_) return;  // the rebuild is still running
            resultReady_ = false;
            fresh        = std::move(builtTree_);
            err          = buildError_;
            buildError_  = nullptr;
        }

        // However this attempt ends (integrated, failed build, or a failed
        // replay below), it is over: return to the "not rebuilding" state so a
        // later rebuild can be triggered again. Latching `building_` would
        // silently disable rebuilding for good, and with it the reclaiming of
        // tombstoned nodes and of the dataset slots reported by
        // acquireRemovedPoints() (i.e. unbounded growth), and it would make a
        // later sync() wait forever for a result nobody is producing.
        struct EndOfRebuild
        {
            KDTreeSingleIndexIncrementalAdaptorMT& self;
            ~EndOfRebuild()
            {
                self.log_.clear();
                self.building_ = false;
            }
        } endOfRebuild{*this};

        // The build failed (e.g. std::bad_alloc). The active tree was never
        // touched by it, so it is still correct: just drop the attempt.
        if (err) std::rethrow_exception(err);

        fresh->setInlineRebuild(false);
        // Replay the operations buffered while the background build was running.
        for (const auto& op : log_)
        {
            switch (op.kind)
            {
                case OpKind::Add:
                    fresh->addPoints(op.a, op.b);
                    break;
                case OpKind::Remove:
                    fresh->removePoint(op.a);
                    break;
                case OpKind::RemoveBox:
                    fresh->removeBox(op.box);
                    break;
                case OpKind::RemoveOutsideBox:
                    fresh->removeOutsideBox(op.box);
                    break;
            }
        }
        // Dataset slots referenced by the OLD tree but not by the fresh one are
        // now free for the caller to recycle (no node references them anymore).
        if (collectRemoved_)
        {
            std::vector<IndexType> oldPhysical;
            active_->collectPhysicalIndices(oldPhysical);
            for (IndexType idx : oldPhysical)
                if (!fresh->referencesIndex(idx)) removedSink_.push_back(idx);
        }
        active_        = std::move(fresh);
        lastBuildLive_ = active_->size();
    }

    const DatasetAdaptor&        dataset_;
    Dimension                    dim_;
    KDTreeIncrementalIndexParams params_;
    double                       rebuildGrowth_;
    Size                         minRebuildSize_;

    std::unique_ptr<Inner> active_;
    /// Foreground-only: a rebuild has been handed to the worker and not
    /// integrated back yet.
    bool                  building_      = false;
    Size                  lastBuildLive_ = 0;
    std::vector<LoggedOp> log_;

    /** @name Background rebuild worker
     *  One persistent thread, created on the first rebuild (see
     *  maybeTriggerRebuild()) and joined by the destructor. Everything below is
     *  guarded by workerMtx_, except workerThread_ itself, which is only
     *  touched by the foreground thread.
     *  @{ */
    std::thread                             workerThread_;
    std::mutex                              workerMtx_;
    std::condition_variable                 workerCvJob_;  // foreground -> worker
    std::condition_variable                 workerCvDone_;  // worker -> foreground
    std::shared_ptr<std::vector<IndexType>> pendingJob_;  // live indices to build from
    std::function<void(Inner&)>             pendingCallback_;
    std::unique_ptr<Inner>                  builtTree_;  // result of the last build
    std::exception_ptr                      buildError_;  // ...or how it failed
    bool                                    resultReady_ = false;
    bool                                    workerStop_  = false;
    /** @} */

    bool                        collectRemoved_ = false;
    std::vector<IndexType>      removedSink_;
    std::function<void(Inner&)> rebuildCallback_;
};
#endif  // NANOFLANN_NO_THREADS

/** An L2-metric KD-tree adaptor for working with data directly stored in an
 * Eigen Matrix, without duplicating the data storage. You can select whether a
 * row or column in the matrix represents a point in the state space.
 *
 * Example of usage:
 * \code
 * Eigen::Matrix<num_t,Eigen::Dynamic,Eigen::Dynamic>  mat;
 *
 * // Fill out "mat"...
 * using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
 *   Eigen::Matrix<num_t,Dynamic,Dynamic>>;
 *
 * const int max_leaf = 10;
 * my_kd_tree_t mat_index(mat, max_leaf);
 * mat_index.index->...
 * \endcode
 *
 *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality
 * for the points in the data set, allowing more compiler optimizations.
 * \tparam Distance The distance metric to use: nanoflann::metric_L1,
 * nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
 * \tparam row_major If set to true the rows of the matrix are used as the
 *         points, if set to false  the columns of the matrix are used as the
 *         points.
 */
template <
    class MatrixType, int32_t DIM = -1, class Distance = nanoflann::metric_L2,
    bool row_major = true>
struct KDTreeEigenMatrixAdaptor
{
    using self_t    = KDTreeEigenMatrixAdaptor<MatrixType, DIM, Distance, row_major>;
    using num_t     = typename MatrixType::Scalar;
    using IndexType = typename MatrixType::Index;
    using metric_t  = typename Distance::template traits<num_t, self_t, IndexType>::distance_t;

    using index_t = KDTreeSingleIndexAdaptor<
        metric_t, self_t, row_major ? MatrixType::ColsAtCompileTime : MatrixType::RowsAtCompileTime,
        IndexType>;

    index_t* index_;  //! The kd-tree index for the user to call its methods as
                      //! usual with any other FLANN index.

    using Offset    = typename index_t::Offset;
    using Size      = typename index_t::Size;
    using Dimension = typename index_t::Dimension;

    /// Constructor: takes a const ref to the matrix object with the data points
    explicit KDTreeEigenMatrixAdaptor(
        const Dimension dimensionality, const std::reference_wrapper<const MatrixType>& mat,
        const int leaf_max_size = 10, const unsigned int n_thread_build = 1)
        : m_data_matrix(mat)
    {
        const auto dims = row_major ? mat.get().cols() : mat.get().rows();
        if (static_cast<Dimension>(dims) != dimensionality)
            throw std::runtime_error(
                "Error: 'dimensionality' must match column count in data "
                "matrix");
        if (DIM > 0 && static_cast<int32_t>(dims) != DIM)
            throw std::runtime_error(
                "Data set dimensionality does not match the 'DIM' template "
                "argument");
        index_ = new index_t(
            static_cast<Dimension>(dims), *this /* adaptor */,
            nanoflann::KDTreeSingleIndexAdaptorParams(
                leaf_max_size, nanoflann::KDTreeSingleIndexAdaptorFlags::None, n_thread_build));
    }

   public:
    /** Deleted copy constructor */
    KDTreeEigenMatrixAdaptor(const self_t&) = delete;
    self_t& operator=(const self_t&)        = delete;

    /** Move operations are deleted: the owned index_ stores a reference back to
     * this adaptor object (passed as the dataset adaptor at construction), so
     * moving would leave that reference dangling. Deleting them also prevents
     * a double-free of the raw index_ pointer. */
    KDTreeEigenMatrixAdaptor(self_t&&) = delete;
    self_t& operator=(self_t&&)        = delete;

    ~KDTreeEigenMatrixAdaptor() { delete index_; }

    const std::reference_wrapper<const MatrixType> m_data_matrix;

    /** Query for the \a num_closest closest points to a given point (entered as
     * query_point[0:dim-1]). Note that this is a short-cut method for
     * index->findNeighbors(). The user can also call index->... methods as
     * desired.
     *
     * \note If L2 norms are used, all returned distances are actually squared
     *       distances.
     */
    void query(
        const num_t* query_point, const Size num_closest, IndexType* out_indices,
        num_t* out_distances) const
    {
        nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances);
        index_->findNeighbors(resultSet, query_point);
    }

    /** @name Interface expected by KDTreeSingleIndexAdaptor
     * @{ */

    inline const self_t& derived() const noexcept { return *this; }
    inline self_t&       derived() noexcept { return *this; }

    // Must return the number of data points
    inline Size kdtree_get_point_count() const
    {
        if (row_major)
            return m_data_matrix.get().rows();
        else
            return m_data_matrix.get().cols();
    }

    // Returns the dim'th component of the idx'th point in the class:
    inline num_t kdtree_get_pt(const IndexType idx, size_t dim) const
    {
        if (row_major)
            return m_data_matrix.get().coeff(idx, IndexType(dim));
        else
            return m_data_matrix.get().coeff(IndexType(dim), idx);
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    inline bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }

    /** @} */

};  // end of KDTreeEigenMatrixAdaptor
/** @} */

/** @} */  // end of grouping
}  // namespace nanoflann

#undef NANOFLANN_RESTRICT
