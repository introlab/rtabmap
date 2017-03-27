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

#ifndef CLOUDVIEWER_H_
#define CLOUDVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/core/Transform.h"
#include "rtabmap/core/StereoCameraModel.h"

#include <QVTKWidget.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>

#include <QtCore/QMap>
#include <QtCore/QSet>
#include <QtCore/qnamespace.h>
#include <QtCore/QSettings>

#include <opencv2/opencv.hpp>
#include <set>

#include <pcl/visualization/mouse_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/PCLPointCloud2.h>

namespace pcl {
	namespace visualization {
		class PCLVisualizer;
	}
}

class QMenu;
class vtkProp;

namespace rtabmap {

class OctoMap;

class RTABMAPGUI_EXP CloudViewer : public QVTKWidget
{
	Q_OBJECT

public:
	CloudViewer(QWidget * parent = 0);
	virtual ~CloudViewer();

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");

	bool updateCloudPose(
		const std::string & id,
		const Transform & pose); //including mesh

	bool addCloud(
			const std::string & id,
			const pcl::PCLPointCloud2Ptr & binaryCloud,
			const Transform & pose,
			bool rgb,
			bool haveNormals,
			const QColor & color = QColor());

	bool addCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity(),
			const QColor & color = QColor());

	bool addCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity(),
			const QColor & color = QColor());

	bool addCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity(),
			const QColor & color = QColor());

	bool addCloud(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
			const Transform & pose = Transform::getIdentity(),
			const QColor & color = QColor());

	bool addCloudMesh(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
			const std::vector<pcl::Vertices> & polygons,
			const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
			const std::vector<pcl::Vertices> & polygons,
			const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
			const std::vector<pcl::Vertices> & polygons,
			const Transform & pose = Transform::getIdentity());

	bool addCloudMesh(
			const std::string & id,
			const pcl::PolygonMesh::Ptr & mesh,
			const Transform & pose = Transform::getIdentity());

	// Only one texture per mesh is supported!
	bool addCloudTextureMesh(
			const std::string & id,
			const pcl::TextureMesh::Ptr & textureMesh,
			const cv::Mat & texture,
			const Transform & pose = Transform::getIdentity());

	bool addOctomap(const OctoMap * octomap, unsigned int treeDepth = 0);
	void removeOctomap();

	// Only one texture per mesh is supported!
	bool addTextureMesh (
		   const pcl::TextureMesh &mesh,
		   const cv::Mat & texture, 
		   const std::string &id = "texture",
		   int viewport = 0);
	bool addOccupancyGridMap(
			const cv::Mat & map8U,
			float resolution, // cell size
			float xMin,
			float yMin,
			float opacity);
	void removeOccupancyGridMap();

	void updateCameraTargetPosition(
		const Transform & pose);

	void updateCameraFrustum(
			const Transform & pose,
			const StereoCameraModel & model);

	void updateCameraFrustum(
			const Transform & pose,
			const CameraModel & model);

	void updateCameraFrustums(
			const Transform & pose,
			const std::vector<CameraModel> & models);

	void addOrUpdateCoordinate(
			const std::string & id,
			const Transform & transform,
			double scale);
	bool updateCoordinatePose(
			const std::string & id,
			const Transform & transform);
	void removeCoordinate(const std::string & id);
	void removeAllCoordinates();

	void addOrUpdateLine(
				const std::string & id,
				const Transform & from,
				const Transform & to,
				const QColor & color,
				bool arrow = false);
	void removeLine(const std::string & id);
	void removeAllLines();

	void addOrUpdateFrustum(
			const std::string & id,
			const Transform & transform,
			const Transform & localTransform,
			double scale,
			const QColor & color = QColor());
	bool updateFrustumPose(
			const std::string & id,
			const Transform & pose);
	void removeFrustum(const std::string & id);
	void removeAllFrustums(bool exceptCameraReference = false);
	const QMap<std::string, Transform> & getAddedFrustums() const {return _frustums;}

	void addOrUpdateGraph(
			const std::string & id,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr & graph,
			const QColor & color = Qt::gray);
	void removeGraph(const std::string & id);
	void removeAllGraphs();

	void addOrUpdateText(
			const std::string & id,
			const std::string & text,
			const Transform & position,
			double scale,
			const QColor & color);
	void removeText(const std::string & id);
	void removeAllTexts();

	bool isTrajectoryShown() const;
	unsigned int getTrajectorySize() const;
	void setTrajectoryShown(bool shown);
	void setTrajectorySize(unsigned int value);
	void clearTrajectory();
	bool isFrustumShown() const;
	float getFrustumScale() const;
	QColor getFrustumColor() const;
	void setFrustumShown(bool shown);
	void setFrustumScale(float value);
	void setFrustumColor(QColor value);
	void resetCamera();

	void removeAllClouds(); //including meshes
	bool removeCloud(const std::string & id); //including mesh

	bool getPose(const std::string & id, Transform & pose); //including meshes
	bool getCloudVisibility(const std::string & id);

	const QMap<std::string, Transform> & getAddedClouds() const {return _addedClouds;} //including meshes
	const QColor & getDefaultBackgroundColor() const;
	const QColor & getBackgroundColor() const;
	Transform getTargetPose() const;

	void setBackfaceCulling(bool enabled, bool frontfaceCulling);
	void setRenderingRate(double rate);
	void setLighting(bool on);
	void setShading(bool on);
	void setEdgeVisibility(bool visible);
	double getRenderingRate() const;

	void getCameraPosition(
			float & x, float & y, float & z,
			float & focalX, float & focalY, float & focalZ,
			float & upX, float & upY, float & upZ) const;
	bool isCameraTargetLocked() const;
	bool isCameraTargetFollow() const;
	bool isCameraFree() const;
	bool isCameraLockZ() const;
	bool isGridShown() const;
	unsigned int getGridCellCount() const;
	float getGridCellSize() const;

	void setCameraPosition(
			float x, float y, float z,
			float focalX, float focalY, float focalZ,
			float upX, float upY, float upZ);
	void setCameraTargetLocked(bool enabled = true);
	void setCameraTargetFollow(bool enabled = true);
	void setCameraFree();
	void setCameraLockZ(bool enabled = true);
	void setGridShown(bool shown);
	void setGridCellCount(unsigned int count);
	void setGridCellSize(float size);

public slots:
	void setDefaultBackgroundColor(const QColor & color);
	void setBackgroundColor(const QColor & color);
	void setCloudVisibility(const std::string & id, bool isVisible);
	void setCloudOpacity(const std::string & id, double opacity = 1.0);
	void setCloudPointSize(const std::string & id, int size);
	virtual void clear();

signals:
	void configChanged();

protected:
	virtual void keyReleaseEvent(QKeyEvent * event);
	virtual void keyPressEvent(QKeyEvent * event);
	virtual void mousePressEvent(QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent * event);
	virtual void wheelEvent(QWheelEvent * event);
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void handleAction(QAction * event);
	QMenu * menu() {return _menu;}

private:
	void createMenu();
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
    QAction * _aShowFrustum;
    QAction * _aSetFrustumScale;
    QAction * _aSetFrustumColor;
    QAction * _aShowGrid;
    QAction * _aSetGridCellCount;
    QAction * _aSetGridCellSize;
    QAction * _aSetBackgroundColor;
    QAction * _aSetRenderingRate;
    QAction * _aSetLighting;
    QAction * _aSetFlatShading;
    QAction * _aSetEdgeVisibility;
    QAction * _aBackfaceCulling;
    QMenu * _menu;
    std::set<std::string> _graphes;
    std::set<std::string> _coordinates;
    std::set<std::string> _texts;
    std::set<std::string> _lines;
    QMap<std::string, Transform> _frustums;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trajectory;
    unsigned int _maxTrajectorySize;
    float _frustumScale;
    QColor _frustumColor;
    unsigned int _gridCellCount;
    float _gridCellSize;
    cv::Vec3d _lastCameraOrientation;
    cv::Vec3d _lastCameraPose;
    QMap<std::string, Transform> _addedClouds; // include cloud, scan, meshes
    Transform _lastPose;
    std::list<std::string> _gridLines;
    QSet<Qt::Key> _keysPressed;
    QColor _defaultBgColor;
    QColor _currentBgColor;
    bool _frontfaceCulling;
    double _renderingRate;
    vtkProp * _octomapActor;
};

} /* namespace rtabmap */
#endif /* CLOUDVIEWER_H_ */
