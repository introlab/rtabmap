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

#ifndef NEARESTNEIGHBOR_H_
#define NEARESTNEIGHBOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <map>
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

class VisualWord;

class RTABMAP_EXP NearestNeighbor
{
public:

public:
	virtual ~NearestNeighbor() {}

	virtual void setData(const cv::Mat & data) = 0;

	virtual void search(
			const cv::Mat & queries,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn = 1,
			int emax = 64) = 0;

	virtual void search(
			const cv::Mat & data,
			const cv::Mat & queries,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn = 1,
			int emax = 64) const = 0;

	virtual bool isDist64F() const = 0;
	virtual bool isDistSquared() const = 0;

	virtual void parseParameters(const ParametersMap & parameters) {}

protected:
	NearestNeighbor() {}
};



/////////////////////////
// KdTreeNN
/////////////////////////
class RTABMAP_EXP KdTreeNN : public NearestNeighbor
{
public:
	KdTreeNN(const ParametersMap & parameters = ParametersMap());
	virtual ~KdTreeNN();

	virtual void setData(const cv::Mat & data);

	virtual void search(const cv::Mat & queries,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn = 1,
			int emax = 64);

	virtual void search(const cv::Mat & data,
			const cv::Mat & queries,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn = 1,
			int emax = 64) const;

	virtual bool isDist64F() const {return true;}
	virtual bool isDistSquared() const {return false;}
	virtual void parseParameters(const ParametersMap & parameters);
private:
	CvFeatureTree * _tree;
	CvMat _dataMat;
};






/////////////////////////
// FlannKdTreeNN
/////////////////////////
class RTABMAP_EXP FlannKdTreeNN : public NearestNeighbor
{
public:
	enum Strategy{kLinear, kKDTree, kMeans, kComposite, kAutoTuned, kUndefined};

public:
	FlannKdTreeNN(const ParametersMap & parameters = ParametersMap());
	FlannKdTreeNN(Strategy s, const ParametersMap & parameters = ParametersMap());
	virtual ~FlannKdTreeNN();

	void setStrategy(Strategy s) {if(_strategy!=kUndefined) _strategy = s;}

	virtual void setData(const cv::Mat & data);

	virtual void search(const cv::Mat & queries,
			cv::Mat & indices,
			cv::Mat & dists,
			int knn = 1,
			int emax = 64);

	virtual void search(const cv::Mat & data,
				const cv::Mat & queries,
				cv::Mat & indices,
				cv::Mat & dists,
				int knn = 1,
				int emax = 64) const;

	virtual bool isDist64F() const {return false;}
	virtual bool isDistSquared() const {return true;}
	virtual void parseParameters(const ParametersMap & parameters);

private:
	cv::flann::Index * createIndex(const cv::Mat & data, Strategy s) const;

private:
	cv::flann::Index * _treeFlannIndex;
	Strategy _strategy;
};

}

#endif /* NEARESTNEIGHBOR_H_ */
