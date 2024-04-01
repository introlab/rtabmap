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

#ifndef RTABMAP_CORE_EXPORTCLOUDSDIALOG_H_
#define RTABMAP_CORE_EXPORTCLOUDSDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QDialog>
#include <QMap>
#include <QtCore/QSettings>

#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_base.h>

class Ui_ExportCloudsDialog;
class QAbstractButton;

namespace rtabmap {
class ProgressDialog;
class GainCompensator;
class DBDriver;

class RTABMAP_GUI_EXPORT ExportCloudsDialog : public QDialog
{
	Q_OBJECT

public:
	ExportCloudsDialog(QWidget *parent = 0);

	virtual ~ExportCloudsDialog();

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");

	void setDBDriver(const DBDriver * dbDriver) {_dbDriver = dbDriver;}
	void forceAssembling(bool enabled);
	void setProgressDialogToMax();
	void setSaveButton();
	void setOkButton();

	void exportClouds(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, int> & mapIds,
			const QMap<int, Signature> & cachedSignatures,
			const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
			const std::map<int, LaserScan> & cachedScans,
			const QString & workingDirectory,
			const ParametersMap & parameters);

	void viewClouds(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, int> & mapIds,
			const QMap<int, Signature> & cachedSignatures,
			const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
			const std::map<int, LaserScan> & cachedScans,
			const QString & workingDirectory,
			const ParametersMap & parameters);

	bool getExportedClouds(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, int> & mapIds,
			const QMap<int, Signature> & cachedSignatures,
			const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
			const std::map<int, LaserScan> & cachedScans,
			const QString & workingDirectory,
			const ParametersMap & parameters,
			std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & clouds,
			std::map<int, pcl::PolygonMesh::Ptr> & meshes,
			std::map<int, pcl::TextureMesh::Ptr> & textureMeshes,
			std::vector<std::map<int, pcl::PointXY> > & textureVertexToPixels);

	int getTextureSize() const;
	int getMaxTextures() const;
	bool isGainCompensation() const;
	double getGainBeta() const;
	bool isGainRGB() const;
	bool isBlending() const;
	int getBlendingDecimation() const;
	int getTextureBrightnessConstrastRatioLow() const;
	int getTextureBrightnessConstrastRatioHigh() const;
	bool isExposeFusion() const;

	static bool removeDirRecursively(const QString & dirName);

Q_SIGNALS:
	void configChanged();

public Q_SLOTS:
	void restoreDefaults();

private Q_SLOTS:
	void loadSettings();
	void saveSettings();
	void updateReconstructionFlavor();
	void selectDistortionModel();
	void selectCamProjMask();
	void updateMLSGrpVisibility();
	void cancel();

private:
	std::map<int, Transform> filterNodes(const std::map<int, Transform> & poses);
	std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, pcl::IndicesPtr> > getClouds(
			const std::map<int, Transform> & poses,
			const QMap<int, Signature> & cachedSignatures,
			const std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr> > & cachedClouds,
			const std::map<int, LaserScan> & cachedScans,
			const ParametersMap & parameters,
			bool & has2dScans,
			bool & scansHaveRGB) const;
	void saveClouds(const QString & workingDirectory, const std::map<int, Transform> & poses, const std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> & clouds, bool binaryMode = true, const std::vector<std::map<int, pcl::PointXY> > & pointToPixels = std::vector<std::map<int, pcl::PointXY> >());
	void saveMeshes(const QString & workingDirectory, const std::map<int, Transform> & poses, const std::map<int, pcl::PolygonMesh::Ptr> & meshes, bool binaryMode = true);
	void saveTextureMeshes(const QString & workingDirectory, const std::map<int, Transform> & poses, std::map<int, pcl::TextureMesh::Ptr> & textureMeshes, const QMap<int, Signature> & cachedSignatures, const std::vector<std::map<int, pcl::PointXY> > & textureVertexToPixels);

private:
	Ui_ExportCloudsDialog * _ui;
	ProgressDialog * _progressDialog;
	QString _workingDirectory;
	bool _canceled;
	GainCompensator * _compensator;
	const DBDriver * _dbDriver;
	bool _scansHaveRGB;

    bool saveOBJFile(const QString &path, pcl::TextureMesh::Ptr &mesh) const;
    bool saveOBJFile(const QString &path, pcl::PolygonMesh &mesh) const;

};

}

#endif /* EXPORTCLOUDSDIALOG_H_ */
