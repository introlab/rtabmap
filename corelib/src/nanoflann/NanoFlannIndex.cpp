/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "nanoflann/NanoFlannIndex.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

#include <algorithm>

#include "nanoflann/nanoflann.h"

#include <sstream>

namespace rtabmap {

namespace {

// Points indexed by the tree. nanoflann is zero-copy: it only stores indexes in
// this container, which holds a pointer to the first coordinate of each point
// rather than a copy of it, the way rtflann's NNIndex::points_ does. The
// features they point into are kept alive by "blocks" below. The container is
// append-only so that the indexes handed out by addPoints() stay valid (removed
// points leave a hole behind).
struct PointCloud
{
	std::vector<const float*> pts;
	std::vector<cv::Mat> blocks; // owners of the rows pts points into
	int dim = 0;

	inline size_t kdtree_get_point_count() const {return pts.size();}
	inline float kdtree_get_pt(const size_t idx, const size_t d) const {return pts[idx][d];}
	template <class BBOX> bool kdtree_get_bbox(BBOX & /* bb */) const {return false;}
};

}

// Type-erases the metric and the compile-time dimension of the tree, so that
// nanoflann's templates stay in this file.
class NanoFlannIndexImpl
{
public:
	virtual ~NanoFlannIndexImpl() {}

	// index every point currently in cloud
	virtual void buildIndex() = 0;
	virtual bool isIncremental() const = 0;

	// [start, end] indexes of points already appended to cloud
	virtual void addPoints(size_t start, size_t end) = 0;
	virtual void removePoint(size_t index) = 0;
	virtual size_t size() const = 0;
	virtual size_t usedMemory() const = 0;
	virtual size_t knnSearch(const float * query, size_t knn, unsigned int * indices, float * dists) const = 0;
	virtual size_t radiusSearch(
			const float * query,
			float radiusSqr,
			std::vector<nanoflann::ResultItem<unsigned int, float> > & matches,
			const nanoflann::SearchParameters & params) const = 0;
	virtual void saveIndex(std::ostream & stream) const = 0;
	// throws std::runtime_error if the stream doesn't match this instantiation
	virtual void loadIndex(std::istream & stream) = 0;

	PointCloud cloud;
};

namespace {

template<class Metric, int32_t DIM>
class NanoFlannTree : public NanoFlannIndexImpl
{
public:
	// cloud is a base class member, so it is already constructed here. The
	// incremental tree always starts empty, whatever the dataset holds.
	NanoFlannTree(int dim, float alphaDeleted) :
		tree_(dim, cloud, nanoflann::KDTreeIncrementalIndexParams(0.75f, alphaDeleted)) {}

	virtual void buildIndex() override
	{
		const size_t count = cloud.kdtree_get_point_count();
		if(count)
		{
			tree_.addPoints(0, (unsigned int)(count-1));
		}
	}
	virtual bool isIncremental() const override {return true;}
	virtual void addPoints(size_t start, size_t end) override {tree_.addPoints((unsigned int)start, (unsigned int)end);}
	virtual void removePoint(size_t index) override {tree_.removePoint((unsigned int)index);}
	virtual size_t size() const override {return tree_.size();}
	virtual size_t usedMemory() const override {return tree_.usedMemory();}
	virtual size_t knnSearch(const float * query, size_t knn, unsigned int * indices, float * dists) const override
	{
		return tree_.knnSearch(query, knn, indices, dists);
	}
	virtual size_t radiusSearch(
			const float * query,
			float radiusSqr,
			std::vector<nanoflann::ResultItem<unsigned int, float> > & matches,
			const nanoflann::SearchParameters & params) const override
	{
		return tree_.radiusSearch(query, radiusSqr, matches, params);
	}
	virtual void saveIndex(std::ostream & stream) const override {tree_.saveIndex(stream);}
	virtual void loadIndex(std::istream & stream) override {tree_.loadIndex(stream);}

private:
	nanoflann::KDTreeSingleIndexIncrementalAdaptor<Metric, PointCloud, DIM, unsigned int> tree_;
};

template<class Metric, int32_t DIM>
class NanoFlannStaticTree : public NanoFlannIndexImpl
{
public:
	// The static tree indexes the dataset as it is when it is built, and cloud
	// is still empty here: the initial build is skipped, buildIndex() or
	// loadIndex() is called once the points are in.
	NanoFlannStaticTree(int dim, size_t leafMaxSize) :
		tree_(dim, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(
			leafMaxSize, nanoflann::KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex)) {}

	virtual void buildIndex() override {tree_.buildIndex();}
	virtual bool isIncremental() const override {return false;}
	virtual void addPoints(size_t, size_t) override {UFATAL("Not supported by the static nanoflann index.");}
	virtual void removePoint(size_t) override {UFATAL("Not supported by the static nanoflann index.");}
	// no removed points to exclude
	virtual size_t size() const override {return cloud.kdtree_get_point_count();}
	virtual size_t usedMemory() const override {return tree_.usedMemory(tree_);}
	virtual size_t knnSearch(const float * query, size_t knn, unsigned int * indices, float * dists) const override
	{
		return tree_.knnSearch(query, knn, indices, dists);
	}
	virtual size_t radiusSearch(
			const float * query,
			float radiusSqr,
			std::vector<nanoflann::ResultItem<unsigned int, float> > & matches,
			const nanoflann::SearchParameters & params) const override
	{
		return tree_.radiusSearch(query, radiusSqr, matches, params);
	}
	virtual void saveIndex(std::ostream & stream) const override {tree_.saveIndex(stream);}
	virtual void loadIndex(std::istream & stream) override {tree_.loadIndex(stream);}

private:
	nanoflann::KDTreeSingleIndexAdaptor<Metric, PointCloud, DIM, unsigned int> tree_;
};

// L2_Simple is the metric recommended by nanoflann for 2D and 3D point clouds,
// L2 (with its partial distance early exit) for the higher dimensions of the
// descriptors. A compile-time dimension additionally keeps the per-node
// bounding boxes on the stack, so the two point cloud cases are instantiated
// with theirs.
template<template<class, int32_t> class Tree, class ... Args>
NanoFlannIndexImpl * createTree(int dim, bool useDistanceL1, Args ... args)
{
	if(useDistanceL1)
	{
		return new Tree<nanoflann::L1_Adaptor<float, PointCloud>, -1>(dim, args...);
	}
	if(dim == 2)
	{
		return new Tree<nanoflann::L2_Simple_Adaptor<float, PointCloud>, 2>(dim, args...);
	}
	if(dim == 3)
	{
		return new Tree<nanoflann::L2_Simple_Adaptor<float, PointCloud>, 3>(dim, args...);
	}
	return new Tree<nanoflann::L2_Adaptor<float, PointCloud>, -1>(dim, args...);
}

NanoFlannIndexImpl * createImpl(int dim, bool useDistanceL1, bool incremental, float removedRatio, int leafMaxSize)
{
	if(incremental)
	{
		// nanoflann's alpha_deleted: the fraction of removed points above which
		// a subtree is rebuilt, dropping them.
		return createTree<NanoFlannTree>(dim, useDistanceL1, removedRatio);
	}
	UASSERT(leafMaxSize > 0);
	return createTree<NanoFlannStaticTree>(dim, useDistanceL1, (size_t)leafMaxSize);
}

}

NanoFlannIndex::NanoFlannIndex() :
	index_(0),
	featuresDim_(0),
	useDistanceL1_(false),
	removedRatio_(0.5f)
{
}

NanoFlannIndex::~NanoFlannIndex()
{
	this->release();
}

void NanoFlannIndex::release()
{
	delete index_;
	index_ = 0;
	featuresDim_ = 0;
}

void NanoFlannIndex::buildIndex(
		const cv::Mat & features,
		bool useDistanceL1,
		bool incremental,
		float removedRatio,
		int leafMaxSize)
{
	this->release();

	UASSERT_MSG(features.type() == CV_32FC1, "Only 32F features are supported by the nanoflann index.");
	UASSERT(features.cols > 0);

	featuresDim_ = features.cols;
	useDistanceL1_ = useDistanceL1;
	removedRatio_ = removedRatio;
	index_ = createImpl(featuresDim_, useDistanceL1, incremental, removedRatio, leafMaxSize);
	index_->cloud.dim = featuresDim_;

	this->appendPoints(features);
	index_->buildIndex();
}

size_t NanoFlannIndex::indexedFeatures() const
{
	return index_?index_->size():0;
}

// return Bytes
size_t NanoFlannIndex::memoryUsed() const
{
	if(!index_)
	{
		return 0;
	}
	// Like the rtflann backend, the features themselves are not counted: they
	// are owned by the caller, only referenced here.
	return sizeof(NanoFlannIndex) +
			index_->cloud.pts.capacity() * sizeof(const float*) +
			index_->cloud.blocks.capacity() * sizeof(cv::Mat) +
			index_->usedMemory();
}

// Reference the points at the end of the storage, without indexing them, and
// return the index of the first one added.
size_t NanoFlannIndex::appendPoints(const cv::Mat & features)
{
	PointCloud & cloud = index_->cloud;
	const size_t start = cloud.pts.size();
	if(cloud.pts.capacity() < start + (size_t)features.rows)
	{
		// Grow geometrically: reserving exactly what is needed would make every
		// single point insertion reallocate and copy the whole storage.
		cloud.pts.reserve(std::max(start + (size_t)features.rows, cloud.pts.capacity()*2));
	}
	// Keeping the header alive is what keeps the rows valid, cv::Mat data being
	// reference counted. One header covers the whole batch.
	cloud.blocks.push_back(features);
	for(int i=0; i<features.rows; ++i)
	{
		cloud.pts.push_back(features.ptr<float>(i));
	}
	return start;
}

// Swap the tree that is built once for the one that accepts points, keeping
// the points already indexed and the indexes they were given.
void NanoFlannIndex::makeIncremental()
{
	UDEBUG("Rebuilding the nanoflann index as an incremental one (%d points)",
		(int)index_->cloud.pts.size());

	const cv::Mat points = this->indexedPoints();
	delete index_;
	index_ = createImpl(featuresDim_, useDistanceL1_, true, removedRatio_, 10);
	index_->cloud.dim = featuresDim_;
	if(!points.empty())
	{
		this->appendPoints(points);
		index_->buildIndex();
	}
}

std::vector<unsigned int> NanoFlannIndex::addPoints(const cv::Mat & features)
{
	if(!index_)
	{
		UERROR("Nanoflann index not yet created!");
		return std::vector<unsigned int>();
	}
	if(!index_->isIncremental())
	{
		// Built as the tree that cannot be added to, but points are added after
		// all: rebuild it as the one that can.
		this->makeIncremental();
	}
	UASSERT(features.type() == CV_32FC1);
	UASSERT(features.cols == featuresDim_);

	std::vector<unsigned int> indexes;
	if(features.rows == 0)
	{
		return indexes;
	}

	const size_t start = this->appendPoints(features);
	index_->addPoints(start, start + (size_t)features.rows - 1);

	indexes.resize(features.rows);
	for(size_t i=0; i<indexes.size(); ++i)
	{
		indexes[i] = (unsigned int)(start + i);
	}
	return indexes;
}

std::vector<unsigned char> NanoFlannIndex::serializeIndex() const
{
	if(!index_)
	{
		return std::vector<unsigned char>();
	}
	if(index_->size() != index_->cloud.kdtree_get_point_count())
	{
		// The tree indexes holes in the point storage, which the features
		// matrix given back to loadIndex() cannot reproduce.
		UWARN("Points have been removed from the nanoflann index (%d indexed of %d points), "
			  "it cannot be serialized before being rebuilt.",
			(int)index_->size(), (int)index_->cloud.kdtree_get_point_count());
		return std::vector<unsigned char>();
	}

	std::ostringstream stream(std::ios_base::out | std::ios_base::binary);
	index_->saveIndex(stream);
	const std::string data = stream.str();
	return std::vector<unsigned char>(data.begin(), data.end());
}

bool NanoFlannIndex::loadIndex(
		const cv::Mat & features,
		bool useDistanceL1,
		bool incremental,
		const unsigned char * indexData,
		size_t indexDataSize,
		float removedRatio,
		int leafMaxSize,
		std::string * errorMsg)
{
	this->release();

	UASSERT_MSG(features.type() == CV_32FC1, "Only 32F features are supported by the nanoflann index.");
	UASSERT(features.cols > 0);

	if(indexData == 0 || indexDataSize == 0)
	{
		if(errorMsg)
		{
			*errorMsg = "Trying to load an empty nanoflann index.";
		}
		return false;
	}

	featuresDim_ = features.cols;
	useDistanceL1_ = useDistanceL1;
	removedRatio_ = removedRatio;
	index_ = createImpl(featuresDim_, useDistanceL1, incremental, removedRatio, leafMaxSize);
	index_->cloud.dim = featuresDim_;
	this->appendPoints(features);

	// nanoflann checks its own magic number, version and type sizes, and
	// throws when the stream wasn't written by the same instantiation.
	try
	{
		std::istringstream stream(
			std::string((const char *)indexData, indexDataSize),
			std::ios_base::in | std::ios_base::binary);
		index_->loadIndex(stream);
	}
	catch(const std::exception & e)
	{
		if(errorMsg)
		{
			*errorMsg = uFormat("Nanoflann index cannot be loaded: %s", e.what());
		}
		this->release();
		return false;
	}

	if(index_->size() != (size_t)features.rows)
	{
		if(errorMsg)
		{
			*errorMsg = uFormat("Serialized nanoflann index has %d points, but %d features were given.",
				(int)index_->size(), features.rows);
		}
		this->release();
		return false;
	}
	return true;
}

cv::Mat NanoFlannIndex::indexedPoints() const
{
	if(!index_ || index_->cloud.pts.empty())
	{
		return cv::Mat();
	}
	// The points are referenced row by row, so a continuous matrix of them has
	// to be materialized. Only used to serialize the index.
	cv::Mat points((int)index_->cloud.pts.size(), featuresDim_, CV_32FC1);
	for(int i=0; i<points.rows; ++i)
	{
		memcpy(points.ptr<float>(i), index_->cloud.pts[i], featuresDim_*sizeof(float));
	}
	return points;
}

void NanoFlannIndex::removePoint(unsigned int index)
{
	if(!index_)
	{
		UERROR("Nanoflann index not yet created!");
		return;
	}
	if(!index_->isIncremental())
	{
		// Same as addPoints(): a tree built without the intention of changing
		// it can still be changed.
		this->makeIncremental();
	}
	// The point stays in cloud so that the indexes of the other points don't
	// move, only the tree drops it.
	index_->removePoint(index);
}

void NanoFlannIndex::knnSearch(
		const cv::Mat & query,
		cv::Mat & indices,
		cv::Mat & dists,
		int knn) const
{
	if(!index_)
	{
		UERROR("Nanoflann index not yet created!");
		return;
	}
	UASSERT(query.type() == CV_32FC1 && query.cols == featuresDim_);
	UASSERT(knn > 0);

	indices = cv::Mat(query.rows, knn, CV_32SC1, cv::Scalar(-1));
	dists = cv::Mat(query.rows, knn, CV_32FC1, cv::Scalar(-1.0f));

	std::vector<unsigned int> resultIndices(knn);
	std::vector<float> resultDists(knn);
	for(int i=0; i<query.rows; ++i)
	{
		size_t found = index_->knnSearch(query.ptr<float>(i), knn, resultIndices.data(), resultDists.data());
		for(size_t j=0; j<found; ++j)
		{
			indices.at<int>(i, j) = (int)resultIndices[j];
			dists.at<float>(i, j) = resultDists[j];
		}
	}
}

void NanoFlannIndex::radiusSearch(
		const cv::Mat & query,
		std::vector<std::vector<size_t> > & indices,
		std::vector<std::vector<float> > & dists,
		float radius,
		int maxNeighbors,
		float eps,
		bool sorted) const
{
	if(!index_)
	{
		UERROR("Nanoflann index not yet created!");
		return;
	}
	UASSERT(query.type() == CV_32FC1 && query.cols == featuresDim_);

	indices.resize(query.rows);
	dists.resize(query.rows);

	// nanoflann compares squared distances, and sorting is required to know
	// which neighbors are the closest ones when maxNeighbors is set.
	const float radiusSqr = radius * radius;
	nanoflann::SearchParameters params(eps, sorted || maxNeighbors>0);

	std::vector<nanoflann::ResultItem<unsigned int, float> > matches;
	for(int i=0; i<query.rows; ++i)
	{
		size_t found = index_->radiusSearch(query.ptr<float>(i), radiusSqr, matches, params);
		if(maxNeighbors > 0 && found > (size_t)maxNeighbors)
		{
			found = (size_t)maxNeighbors;
		}

		indices[i].resize(found);
		dists[i].resize(found);
		for(size_t j=0; j<found; ++j)
		{
			indices[i][j] = (size_t)matches[j].first;
			dists[i][j] = matches[j].second;
		}
	}
}

} /* namespace rtabmap */
