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

#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QVTKWidget.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include "rtabmap/core/Transform.h"
#include <QtCore/QMap>
#include <QtCore/QSet>
#include <QtCore/qnamespace.h>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/mouse_event.h>
#include <pcl/PCLPointCloud2.h>

namespace pcl {
	namespace visualization {
		class PCLVisualizer;
	}
}

class QMenu;

namespace rtabmap {

class RTABMAPGUI_EXP CloudViewer : public QVTKWidget
{
	Q_OBJECT

public:
	CloudViewer(QWidget * parent = 0);
	virtual ~CloudViewer();

	bool updateCloudPose(
		const std::string & id,
		const Transform & pose); //including mesh

	bool updateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity());

	bool updateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity());

	bool addOrUpdateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity(),
		const QColor & color = Qt::gray);

	bool addOrUpdateCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity(),
			const QColor & color = Qt::gray);

	bool addCloud(
			const std::string & id,
			const pcl::PCLPointCloud2Ptr & binaryCloud,
			const Transform & pose,
			bool rgb,
			const QColor & color = Qt::gray);

	bool addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity(),
		const QColor & color = Qt::gray);

	bool addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity(),
		const QColor & color = Qt::gray);

	bool addCloudMesh(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
		const std::string & id,
		const pcl::PolygonMesh::Ptr & mesh,
		const Transform & pose = Transform::getIdentity());

	bool addOccupancyGridMap(
			const cv::Mat & map8U,
			float resolution, // cell size
			float xMin,
			float yMin,
			float opacity);
	void removeOccupancyGridMap();

	void updateCameraPosition(
		const Transform & pose);

	void addOrUpdateGraph(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & graph,
			const QColor & color = Qt::gray);
	void removeGraph(const std::string & id);
	void removeAllGraphs();

	void setTrajectoryShown(bool shown);
	void setTrajectorySize(int value);
	void clearTrajectory();

	void removeAllClouds(); //including meshes
	bool removeCloud(const std::string & id); //including mesh

	bool getPose(const std::string & id, Transform & pose); //including meshes
	bool getCloudVisibility(const std::string & id);

	const QMap<std::string, Transform> & getAddedClouds() {return _addedClouds;} //including meshes

	void setCameraTargetLocked(bool enabled = true);
	void setCameraTargetFollow(bool enabled = true);
	void setCameraFree();
	void setCameraLockZ(bool enabled = true);
	void setGridShown(bool shown);
	void setWorkingDirectory(const QString & path) {_workingDirectory = path;}

public slots:
	void render();
	void setBackgroundColor(const QColor & color);
	void setCloudVisibility(const std::string & id, bool isVisible);
	void setCloudOpacity(const std::string & id, double opacity = 1.0);
	void setCloudPointSize(const std::string & id, int size);
	virtual void clear() {removeAllClouds(); clearTrajectory();}

protected:
	virtual void keyReleaseEvent(QKeyEvent * event);
	virtual void keyPressEvent(QKeyEvent * event);
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void handleAction(QAction * event);
	QMenu * menu() {return _menu;}

private:
	void createMenu();
	void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);
	void addGrid();
	void removeGrid();

private:
    pcl::visualization::PCLVisualizer * _visualizer;
    QAction * _aLockCamera;
    QAction * _aFollowCamera;
    QAction * _aResetCamera;
    QAction * _aLockViewZ;
    QAction * _aShowTrajectory;
    QAction * _aSetTrajectorySize;
    QAction * _aClearTrajectory;
    QAction * _aShowGrid;
    QAction * _aSetBackgroundColor;
    QMenu * _menu;
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr > _graphes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trajectory;
    unsigned int _maxTrajectorySize;
    QMap<std::string, Transform> _addedClouds; // include cloud, scan, meshes
    Transform _lastPose;
    std::list<std::string> _gridLines;
    QSet<Qt::Key> _keysPressed;
    QString _workingDirectory;
};

} /* namespace rtabmap */
#endif /* CLOUDVIEWER_H_ */
