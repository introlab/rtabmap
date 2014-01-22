/*
 * CloudViewer.h
 *
 *  Created on: 2013-10-13
 *      Author: Mathieu
 */

#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QVTKWidget.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include "rtabmap/core/Transform.h"
#include <QtCore/QMap>

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
		const Transform & pose = Transform::getIdentity());

	bool addOrUpdateCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity());

	bool addCloud(
			const std::string & id,
			const pcl::PCLPointCloud2Ptr & binaryCloud,
			const Transform & pose,
			bool rgb);

	bool addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity());

	bool addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
		const std::string & id,
		const pcl::PolygonMesh::Ptr & mesh,
		const Transform & pose = Transform::getIdentity());

	void updateCameraPosition(
		const Transform & pose);

	void setTrajectoryShown(bool shown);
	void setTrajectorySize(int value);

	void removeAllClouds(); //including meshes
	bool removeCloud(const std::string & id); //including mesh

	bool getPose(const std::string & id, Transform & pose); //including meshes
	bool getCloudVisibility(const std::string & id);

	const QMap<std::string, Transform> & getAddedClouds() {return _addedClouds;} //including meshes

public slots:
	void render();
	void setBackgroundColor(const QColor & color);
	void setCloudVisibility(const std::string & id, bool isVisible);
	void setCloudOpacity(const std::string & id, double opacity = 1.0);
	void setCloudPointSize(const std::string & id, int size);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void handleAction(QAction * event);
	QMenu * menu() {return _menu;}

private:
	void createMenu();

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
    QMenu * _menu;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trajectory;
    unsigned int _maxTrajectorySize;
    QMap<std::string, Transform> _addedClouds; // include meshes
    Transform _lastPose;
    std::list<std::string> _gridLines;
};

} /* namespace rtabmap */
#endif /* CLOUDVIEWER_H_ */
