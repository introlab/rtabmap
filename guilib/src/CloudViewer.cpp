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

#include "rtabmap/gui/CloudViewer.h"

#include <rtabmap/core/Version.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <QMenu>
#include <QAction>
#include <QtGui/QContextMenuEvent>
#include <QInputDialog>
#include <QtGui/QWheelEvent>
#include <QtGui/QKeyEvent>
#include <QColorDialog>
#include <QtGui/QVector3D>
#include <QMainWindow>
#include <set>

#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>
#include <vtkTextureUnitManager.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkPNMReader.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPointPicker.h>
#include <opencv/vtkImageMatSource.h>

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif

namespace rtabmap {

class MyInteractorStyle: public pcl::visualization::PCLVisualizerInteractorStyle
{
public:
	virtual void Rotate()
	{
		if (this->CurrentRenderer == NULL)
		{
			return;
		}

		vtkRenderWindowInteractor *rwi = this->Interactor;

		int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
		int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

		int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

		double delta_elevation = -20.0 / size[1];
		double delta_azimuth = -20.0 / size[0];

		double rxf = dx * delta_azimuth * this->MotionFactor;
		double ryf = dy * delta_elevation * this->MotionFactor;

		vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
		camera->Azimuth(rxf);
		camera->Elevation(ryf);
		camera->OrthogonalizeViewUp();

		if (this->AutoAdjustCameraClippingRange)
		{
			this->CurrentRenderer->ResetCameraClippingRange();
		}

		if (rwi->GetLightFollowCamera())
		{
			this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
		}

		//rwi->Render();
	}

protected:
	virtual void OnLeftButtonDown()
	{
		// http://www.vtk.org/Wiki/VTK/Examples/Cxx/Interaction/DoubleClick
		// http://www.vtk.org/Wiki/VTK/Examples/Cxx/Interaction/PointPicker

		this->NumberOfClicks++;
		int pickPosition[2];
		this->GetInteractor()->GetEventPosition(pickPosition);

		int xdist = pickPosition[0] - this->PreviousPosition[0];
		int ydist = pickPosition[1] - this->PreviousPosition[1];

		this->PreviousPosition[0] = pickPosition[0];
		this->PreviousPosition[1] = pickPosition[1];

		int moveDistance = (int)sqrt((double)(xdist*xdist + ydist*ydist));

		// Reset numClicks - If mouse moved further than resetPixelDistance
		if(moveDistance > this->ResetPixelDistance)
		{
			this->NumberOfClicks = 1;
		}

		if(this->NumberOfClicks == 2)
		{
			this->NumberOfClicks = 0;

			this->Interactor->GetPicker()->Pick(pickPosition[0], pickPosition[1],
					0,  // always zero.
					this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);
			UINFO("Double clicked! Picked value: %f %f %f", picked[0], picked[1], picked[2]);

			vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
			double position[3];
			double focal[3];
			camera->GetPosition(position[0], position[1], position[2]);
			camera->GetFocalPoint(focal[0], focal[1], focal[2]);
			//camera->SetPosition (position[0] + (picked[0]-focal[0]), position[1] + (picked[1]-focal[1]), position[2] + (picked[2]-focal[2]));
			camera->SetFocalPoint (picked[0], picked[1], picked[2]);
			camera->OrthogonalizeViewUp();

			if (this->AutoAdjustCameraClippingRange)
			{
				this->CurrentRenderer->ResetCameraClippingRange();
			}

			if (this->Interactor->GetLightFollowCamera())
			{
				this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
			}
		}

		// Forward events
		PCLVisualizerInteractorStyle::OnLeftButtonDown();
	}

private:
	unsigned int NumberOfClicks;
	int PreviousPosition[2];
	int ResetPixelDistance;
};


CloudViewer::CloudViewer(QWidget *parent) :
		QVTKWidget(parent),
		_aLockCamera(0),
		_aFollowCamera(0),
		_aResetCamera(0),
		_aLockViewZ(0),
		_aShowTrajectory(0),
		_aSetTrajectorySize(0),
		_aClearTrajectory(0),
		_aShowFrustum(0),
		_aSetFrustumScale(0),
		_aSetFrustumColor(0),
		_aShowGrid(0),
		_aSetGridCellCount(0),
		_aSetGridCellSize(0),
		_aShowNormals(0),
		_aSetNormalsStep(0),
		_aSetNormalsScale(0),
		_aSetBackgroundColor(0),
		_aSetRenderingRate(0),
		_aSetLighting(0),
		_aSetFlatShading(0),
		_aSetEdgeVisibility(0),
		_aBackfaceCulling(0),
		_menu(0),
		_trajectory(new pcl::PointCloud<pcl::PointXYZ>),
		_maxTrajectorySize(100),
		_frustumScale(0.5f),
		_frustumColor(Qt::gray),
		_gridCellCount(50),
		_gridCellSize(1),
		_normalsStep(1),
		_normalsScale(0.2),
		_lastCameraOrientation(0,0,0),
		_lastCameraPose(0,0,0),
		_defaultBgColor(Qt::black),
		_currentBgColor(Qt::black),
		_frontfaceCulling(false),
		_renderingRate(5.0),
		_octomapActor(0)
{
	UDEBUG("");
	this->setMinimumSize(200, 200);

	int argc = 0;
	_visualizer = new pcl::visualization::PCLVisualizer(
		argc, 
		0, 
		"PCLVisualizer", 
		vtkSmartPointer<MyInteractorStyle>(new MyInteractorStyle()),
		false);

	_visualizer->setShowFPS(false);
	
	this->SetRenderWindow(_visualizer->getRenderWindow());

	// Replaced by the second line, to avoid a crash in Mac OS X on close, as well as
	// the "Invalid drawable" warning when the view is not visible.
	//_visualizer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
	this->GetInteractor()->SetInteractorStyle (_visualizer->getInteractorStyle());
	// setup a simple point picker
	vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
	UDEBUG("pick tolerance=%f", pp->GetTolerance());
	pp->SetTolerance (pp->GetTolerance()/2.0);
	this->GetInteractor()->SetPicker (pp);

	setRenderingRate(_renderingRate);

	_visualizer->setCameraPosition(
				-1, 0, 0,
				0, 0, 0,
				0, 0, 1);
#ifndef _WIN32
	// Crash on startup on Windows (vtk issue)
	this->addOrUpdateCoordinate("reference", Transform::getIdentity(), 0.2);
#endif

	//setup menu/actions
	createMenu();

	setMouseTracking(false);
}

CloudViewer::~CloudViewer()
{
	UDEBUG("");
	this->clear();
	delete _visualizer;
	UDEBUG("");
}

void CloudViewer::clear()
{
	this->removeAllClouds();
	this->removeAllGraphs();
	this->removeAllCoordinates();
	this->removeAllLines();
	this->removeAllFrustums();
	this->removeAllTexts();
	this->removeOccupancyGridMap();
	this->removeOctomap();

	this->addOrUpdateCoordinate("reference", Transform::getIdentity(), 0.2);
	_lastPose.setNull();
	if(_aLockCamera->isChecked() || _aFollowCamera->isChecked())
	{
		resetCamera();
	}
	this->clearTrajectory();
}

void CloudViewer::createMenu()
{
	_aLockCamera = new QAction("Lock target", this);
	_aLockCamera->setCheckable(true);
	_aLockCamera->setChecked(false);
	_aFollowCamera = new QAction("Follow", this);
	_aFollowCamera->setCheckable(true);
	_aFollowCamera->setChecked(true);
	QAction * freeCamera = new QAction("Free", this);
	freeCamera->setCheckable(true);
	freeCamera->setChecked(false);
	_aLockViewZ = new QAction("Lock view Z", this);
	_aLockViewZ->setCheckable(true);
	_aLockViewZ->setChecked(true);
	_aResetCamera = new QAction("Reset position", this);
	_aShowTrajectory= new QAction("Show trajectory", this);
	_aShowTrajectory->setCheckable(true);
	_aShowTrajectory->setChecked(true);
	_aSetTrajectorySize = new QAction("Set trajectory size...", this);
	_aClearTrajectory = new QAction("Clear trajectory", this);
	_aShowFrustum= new QAction("Show frustum", this);
	_aShowFrustum->setCheckable(true);
	_aShowFrustum->setChecked(false);
	_aSetFrustumScale = new QAction("Set frustum scale...", this);
	_aSetFrustumColor = new QAction("Set frustum color...", this);
	_aShowGrid = new QAction("Show grid", this);
	_aShowGrid->setCheckable(true);
	_aSetGridCellCount = new QAction("Set cell count...", this);
	_aSetGridCellSize = new QAction("Set cell size...", this);
	_aShowNormals = new QAction("Show normals", this);
	_aShowNormals->setCheckable(true);
	_aSetNormalsStep = new QAction("Set normals step...", this);
	_aSetNormalsScale = new QAction("Set normals scale...", this);
	_aSetBackgroundColor = new QAction("Set background color...", this);	
	_aSetRenderingRate = new QAction("Set rendering rate...", this);
	_aSetLighting = new QAction("Lighting", this);
	_aSetLighting->setCheckable(true);
	_aSetLighting->setChecked(false);
	_aSetFlatShading = new QAction("Flat Shading", this);
	_aSetFlatShading->setCheckable(true);
	_aSetFlatShading->setChecked(false);
	_aSetEdgeVisibility = new QAction("Show edges", this);
	_aSetEdgeVisibility->setCheckable(true);
	_aSetEdgeVisibility->setChecked(false);
	_aBackfaceCulling = new QAction("Backface culling", this);
	_aBackfaceCulling->setCheckable(true);
	_aBackfaceCulling->setChecked(true);

	QMenu * cameraMenu = new QMenu("Camera", this);
	cameraMenu->addAction(_aLockCamera);
	cameraMenu->addAction(_aFollowCamera);
	cameraMenu->addAction(freeCamera);
	cameraMenu->addSeparator();
	cameraMenu->addAction(_aLockViewZ);
	cameraMenu->addAction(_aResetCamera);
	QActionGroup * group = new QActionGroup(this);
	group->addAction(_aLockCamera);
	group->addAction(_aFollowCamera);
	group->addAction(freeCamera);

	QMenu * trajectoryMenu = new QMenu("Trajectory", this);
	trajectoryMenu->addAction(_aShowTrajectory);
	trajectoryMenu->addAction(_aSetTrajectorySize);
	trajectoryMenu->addAction(_aClearTrajectory);

	QMenu * frustumMenu = new QMenu("Frustum", this);
	frustumMenu->addAction(_aShowFrustum);
	frustumMenu->addAction(_aSetFrustumScale);
	frustumMenu->addAction(_aSetFrustumColor);

	QMenu * gridMenu = new QMenu("Grid", this);
	gridMenu->addAction(_aShowGrid);
	gridMenu->addAction(_aSetGridCellCount);
	gridMenu->addAction(_aSetGridCellSize);

	QMenu * normalsMenu = new QMenu("Normals", this);
	normalsMenu->addAction(_aShowNormals);
	normalsMenu->addAction(_aSetNormalsStep);
	normalsMenu->addAction(_aSetNormalsScale);

	//menus
	_menu = new QMenu(this);
	_menu->addMenu(cameraMenu);
	_menu->addMenu(trajectoryMenu);
	_menu->addMenu(frustumMenu);
	_menu->addMenu(gridMenu);
	_menu->addMenu(normalsMenu);
	_menu->addAction(_aSetBackgroundColor);
	_menu->addAction(_aSetRenderingRate);
	_menu->addAction(_aSetLighting);
	_menu->addAction(_aSetFlatShading);
	_menu->addAction(_aSetEdgeVisibility);
	_menu->addAction(_aBackfaceCulling);
}

void CloudViewer::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	float poseX, poseY, poseZ, focalX, focalY, focalZ, upX, upY, upZ;
	this->getCameraPosition(poseX, poseY, poseZ, focalX, focalY, focalZ, upX, upY, upZ);
	QVector3D pose(poseX, poseY, poseZ);
	QVector3D focal(focalX, focalY, focalZ);
	if(!this->isCameraFree())
	{
		// make camera position relative to target
		Transform T = this->getTargetPose();
		if(this->isCameraTargetLocked())
		{
			T = Transform(T.x(), T.y(), T.z(), 0,0,0);
		}
		Transform F(focalX, focalY, focalZ, 0,0,0);
		Transform P(poseX, poseY, poseZ, 0,0,0);
		Transform newFocal = T.inverse() * F;
		Transform newPose = newFocal * F.inverse() * P;
		pose = QVector3D(newPose.x(), newPose.y(), newPose.z());
		focal = QVector3D(newFocal.x(), newFocal.y(), newFocal.z());
	}
	settings.setValue("camera_pose", pose);
	settings.setValue("camera_focal", focal);
	settings.setValue("camera_up", QVector3D(upX, upY, upZ));

	settings.setValue("grid", this->isGridShown());
	settings.setValue("grid_cell_count", this->getGridCellCount());
	settings.setValue("grid_cell_size", (double)this->getGridCellSize());

	settings.setValue("normals", this->isNormalsShown());
	settings.setValue("normals_step", this->getNormalsStep());
	settings.setValue("normals_scale", (double)this->getNormalsScale());

	settings.setValue("trajectory_shown", this->isTrajectoryShown());
	settings.setValue("trajectory_size", this->getTrajectorySize());

	settings.setValue("frustum_shown", this->isFrustumShown());
	settings.setValue("frustum_scale", this->getFrustumScale());
	settings.setValue("frustum_color", this->getFrustumColor());

	settings.setValue("camera_target_locked", this->isCameraTargetLocked());
	settings.setValue("camera_target_follow", this->isCameraTargetFollow());
	settings.setValue("camera_free", this->isCameraFree());
	settings.setValue("camera_lockZ", this->isCameraLockZ());

	settings.setValue("bg_color", this->getDefaultBackgroundColor());
	settings.setValue("rendering_rate", this->getRenderingRate());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void CloudViewer::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}

	float poseX, poseY, poseZ, focalX, focalY, focalZ, upX, upY, upZ;
	this->getCameraPosition(poseX, poseY, poseZ, focalX, focalY, focalZ, upX, upY, upZ);
	QVector3D pose(poseX, poseY, poseZ), focal(focalX, focalY, focalZ), up(upX, upY, upZ);
	pose = settings.value("camera_pose", pose).value<QVector3D>();
	focal = settings.value("camera_focal", focal).value<QVector3D>();
	up = settings.value("camera_up", up).value<QVector3D>();
	this->setCameraPosition(pose.x(),pose.y(),pose.z(), focal.x(),focal.y(),focal.z(), up.x(),up.y(),up.z());

	this->setGridShown(settings.value("grid", this->isGridShown()).toBool());
	this->setGridCellCount(settings.value("grid_cell_count", this->getGridCellCount()).toUInt());
	this->setGridCellSize(settings.value("grid_cell_size", this->getGridCellSize()).toFloat());

	this->setNormalsShown(settings.value("normals", this->isNormalsShown()).toBool());
	this->setNormalsStep(settings.value("normals_step", this->getNormalsStep()).toInt());
	this->setNormalsScale(settings.value("normals_scale", this->getNormalsScale()).toFloat());

	this->setTrajectoryShown(settings.value("trajectory_shown", this->isTrajectoryShown()).toBool());
	this->setTrajectorySize(settings.value("trajectory_size", this->getTrajectorySize()).toUInt());

	this->setFrustumShown(settings.value("frustum_shown", this->isFrustumShown()).toBool());
	this->setFrustumScale(settings.value("frustum_scale", this->getFrustumScale()).toDouble());
	this->setFrustumColor(settings.value("frustum_color", this->getFrustumColor()).value<QColor>());

	this->setCameraTargetLocked(settings.value("camera_target_locked", this->isCameraTargetLocked()).toBool());
	this->setCameraTargetFollow(settings.value("camera_target_follow", this->isCameraTargetFollow()).toBool());
	if(settings.value("camera_free", this->isCameraFree()).toBool())
	{
		this->setCameraFree();
	}
	this->setCameraLockZ(settings.value("camera_lockZ", this->isCameraLockZ()).toBool());

	this->setDefaultBackgroundColor(settings.value("bg_color", this->getDefaultBackgroundColor()).value<QColor>());
	
	this->setRenderingRate(settings.value("rendering_rate", this->getRenderingRate()).toDouble());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}

	this->update();
}

bool CloudViewer::updateCloudPose(
		const std::string & id,
		const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		//UDEBUG("Updating pose %s to %s", id.c_str(), pose.prettyPrint().c_str());
		bool samePose = _addedClouds.find(id).value() == pose;
		Eigen::Affine3f posef = pose.toEigen3f();
		if(samePose ||
		   _visualizer->updatePointCloudPose(id, posef))
		{
			_addedClouds.find(id).value() = pose;
			if(!samePose)
			{
				std::string idNormals = id+"-normals";
				if(_addedClouds.find(idNormals)!=_addedClouds.end())
				{
					_visualizer->updatePointCloudPose(idNormals, posef);
					_addedClouds.find(idNormals).value() = pose;
				}
			}
			return true;
		}
	}
	return false;
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PCLPointCloud2Ptr & binaryCloud,
		const Transform & pose,
		bool rgb,
		bool haveNormals,
		const QColor & color)
{
	int previousColorIndex = -1;
	if(_addedClouds.contains(id))
	{
		previousColorIndex = _visualizer->getColorHandlerIndex(id);
		this->removeCloud(id);
	}

	Eigen::Vector4f origin(pose.x(), pose.y(), pose.z(), 0.0f);
	Eigen::Quaternionf orientation = Eigen::Quaternionf(pose.toEigen3f().linear());

	if(haveNormals && _aShowNormals->isChecked())
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromPCLPointCloud2 (*binaryCloud, *cloud_xyz);
		std::string idNormals = id + "-normals";
		if(_visualizer->addPointCloudNormals<pcl::PointNormal>(cloud_xyz, _normalsStep, _normalsScale, idNormals, 0))
		{
			_visualizer->updatePointCloudPose(idNormals, pose.toEigen3f());
			_addedClouds.insert(idNormals, pose);
		}
	}

	// add random color channel
	 pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr colorHandler;
	 colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (binaryCloud));
	 if(_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id))
	 {
		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (binaryCloud, c.red(), c.green(), c.blue()));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);

		// x,y,z
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "x"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "y"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "z"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);

		if(rgb)
		{
			//rgb
			colorHandler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(binaryCloud));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
		}
		else if(previousColorIndex == 5)
		{
			previousColorIndex = -1;
		}

		if(haveNormals)
		{
			//normals
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_x"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_y"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_z"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id);
		}
		else if(previousColorIndex > 5)
		{
			previousColorIndex = -1;
		}

		if(previousColorIndex>=0)
		{
			_visualizer->updateColorHandlerIndex(id, previousColorIndex);
		}
		else if(rgb)
		{
			_visualizer->updateColorHandlerIndex(id, 5);
		}
		else if(color.isValid())
		{
			_visualizer->updateColorHandlerIndex(id, 1);
		}

		_addedClouds.insert(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, true, true, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, true, false, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, true, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, false, color);
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
	const std::vector<pcl::Vertices> & polygons,
	const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		this->removeCloud(id);
	}

	UDEBUG("Adding %s with %d points and %d polygons", id.c_str(), (int)cloud->size(), (int)polygons.size());
	if(_visualizer->addPolygonMesh<pcl::PointXYZ>(cloud, polygons, id))
	{
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		_addedClouds.insert(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
	const std::vector<pcl::Vertices> & polygons,
	const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		this->removeCloud(id);
	}

	UDEBUG("Adding %s with %d points and %d polygons", id.c_str(), (int)cloud->size(), (int)polygons.size());
	if(_visualizer->addPolygonMesh<pcl::PointXYZRGB>(cloud, polygons, id))
	{
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		_addedClouds.insert(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
	const std::vector<pcl::Vertices> & polygons,
	const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		this->removeCloud(id);
	}

	UDEBUG("Adding %s with %d points and %d polygons", id.c_str(), (int)cloud->size(), (int)polygons.size());
	if(_visualizer->addPolygonMesh<pcl::PointXYZRGBNormal>(cloud, polygons, id))
	{
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		_addedClouds.insert(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PolygonMesh::Ptr & mesh,
	const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		this->removeCloud(id);
	}

	UDEBUG("Adding %s with %d polygons", id.c_str(), (int)mesh->polygons.size());
	if(_visualizer->addPolygonMesh(*mesh, id))
	{
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		_addedClouds.insert(id, pose);
		return true;
	}

	return false;
}

bool CloudViewer::addCloudTextureMesh(
	const std::string & id,
	const pcl::TextureMesh::Ptr & textureMesh,
	const cv::Mat & texture,
	const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		this->removeCloud(id);
	}

	UDEBUG("Adding %s", id.c_str());
	if(this->addTextureMesh(*textureMesh, texture, id))
	{
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		if(!textureMesh->cloud.is_dense)
		{
			_visualizer->getCloudActorMap()->find(id)->second.actor->GetTexture()->SetInterpolate(1);
			_visualizer->getCloudActorMap()->find(id)->second.actor->GetTexture()->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_REPLACE);
		}
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		_addedClouds.insert(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addOctomap(const OctoMap * octomap, unsigned int treeDepth)
{
	UDEBUG("");
#ifdef RTABMAP_OCTOMAP
	UASSERT(octomap!=0);

	pcl::IndicesPtr obstacles(new std::vector<int>);

	if(treeDepth == 0 || treeDepth > octomap->octree()->getTreeDepth())
	{
		if(treeDepth>0)
		{
			UWARN("Tree depth requested (%d) is deeper than the "
				  "actual maximum tree depth of %d. Using maximum depth.",
				  (int)treeDepth, (int)octomap->octree()->getTreeDepth());
		}
		treeDepth = octomap->octree()->getTreeDepth();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap->createCloud(treeDepth, obstacles.get());
	if(obstacles->size())
	{
		//get the renderer of the visualizer object
		vtkRenderer *renderer = _visualizer->getRenderWindow()->GetRenderers()->GetFirstRenderer();

		if(_octomapActor)
		{
			renderer->RemoveActor(_octomapActor);
			_octomapActor = 0;
		}

		//vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		//colors->SetName("colors");
		//colors->SetNumberOfComponents(3);
		vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
		colors->SetName("colors");
		colors->SetNumberOfValues(obstacles->size());

		vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
		lut->SetNumberOfTableValues(obstacles->size());
		lut->Build();

		// Create points
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		double s = octomap->octree()->getNodeSize(treeDepth) / 2.0;
		for (unsigned int i = 0; i < obstacles->size(); i++)
		{
			points->InsertNextPoint(
					cloud->at(obstacles->at(i)).x,
					cloud->at(obstacles->at(i)).y,
					cloud->at(obstacles->at(i)).z);
			colors->InsertValue(i,i);

			lut->SetTableValue(i,
					double(cloud->at(obstacles->at(i)).r) / 255.0,
					double(cloud->at(obstacles->at(i)).g) / 255.0,
					double(cloud->at(obstacles->at(i)).b) / 255.0);
		}

		// Combine into a polydata
		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);
		polydata->GetPointData()->SetScalars(colors);

		// Create anything you want here, we will use a cube for the demo.
		vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
		cubeSource->SetBounds(-s, s, -s, s, -s, s);

		vtkSmartPointer<vtkGlyph3DMapper> mapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
		mapper->SetSourceConnection(cubeSource->GetOutputPort());
#if VTK_MAJOR_VERSION <= 5
		mapper->SetInputConnection(polydata->GetProducerPort());
#else
		mapper->SetInputData(polydata);
#endif
		mapper->SetScalarRange(0, obstacles->size() - 1);
		mapper->SetLookupTable(lut);
		mapper->ScalingOff();
		mapper->Update();

		vtkSmartPointer<vtkActor> octomapActor = vtkSmartPointer<vtkActor>::New();
		octomapActor->SetMapper(mapper);

		octomapActor->GetProperty()->SetRepresentationToSurface();
		octomapActor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		octomapActor->GetProperty()->SetLighting(_aSetLighting->isChecked());

		renderer->AddActor(octomapActor);
		_octomapActor = octomapActor.GetPointer();

		return true;
	}
#endif
	return false;
}

void CloudViewer::removeOctomap()
{
	UDEBUG("");
#ifdef RTABMAP_OCTOMAP
	if(_octomapActor)
	{
		vtkRenderer *renderer = _visualizer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
		renderer->RemoveActor(_octomapActor);
		_octomapActor = 0;
	}
#endif
}

bool CloudViewer::addTextureMesh (
	   const pcl::TextureMesh &mesh,
	   const cv::Mat & image,
	   const std::string &id,
	   int viewport)
{
	// Copied from PCL 1.8, modified to ignore vertex color and accept only one material (loaded from memory instead of file)

  pcl::visualization::CloudActorMap::iterator am_it = _visualizer->getCloudActorMap()->find (id);
  if (am_it != _visualizer->getCloudActorMap()->end ())
  {
    PCL_ERROR ("[PCLVisualizer::addTextureMesh] A shape with id <%s> already exists!"
               " Please choose a different id and retry.\n",
               id.c_str ());
    return (false);
  }
  // no texture materials --> exit
  if (mesh.tex_materials.size () == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No textures found!\n");
    return (false);
  }
  else if (mesh.tex_materials.size() > 1)
  {
	  PCL_ERROR("[PCLVisualizer::addTextureMesh] only one material per mesh is supported!\n");
	  return (false);
  }
  // polygons are mapped to texture materials
  if (mesh.tex_materials.size () != mesh.tex_polygons.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Materials number %lu differs from polygons number %lu!\n",
              mesh.tex_materials.size (), mesh.tex_polygons.size ());
    return (false);
  }
  // each texture material should have its coordinates set
  if (mesh.tex_materials.size () != mesh.tex_coordinates.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Coordinates number %lu differs from materials number %lu!\n",
              mesh.tex_coordinates.size (), mesh.tex_materials.size ());
    return (false);
  }
  // total number of vertices
  std::size_t nb_vertices = 0;
  for (std::size_t i = 0; i < mesh.tex_polygons.size (); ++i)
    nb_vertices+= mesh.tex_polygons[i].size ();
  // no vertices --> exit
  if (nb_vertices == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No vertices found!\n");
    return (false);
  }
  // total number of coordinates
  std::size_t nb_coordinates = 0;
  for (std::size_t i = 0; i < mesh.tex_coordinates.size (); ++i)
    nb_coordinates+= mesh.tex_coordinates[i].size ();
  // no texture coordinates --> exit
  if (nb_coordinates == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No textures coordinates found!\n");
    return (false);
  }

  // Create points from mesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  bool has_color = false;
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2 (mesh.cloud, *cloud);
    // no points --> exit
    if (cloud->points.size () == 0)
    {
      PCL_ERROR("[PCLVisualizer::addTextureMesh] Cloud is empty!\n");
      return (false);
    }
    pcl::visualization::PCLVisualizer::convertToVtkMatrix (cloud->sensor_origin_, cloud->sensor_orientation_, transformation);
    poly_points->SetNumberOfPoints (cloud->points.size ());
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
      const pcl::PointXYZ &p = cloud->points[i];
      poly_points->InsertPoint (i, p.x, p.y, p.z);
    }

  //create polys from polyMesh.tex_polygons
  vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New ();
  for (std::size_t i = 0; i < mesh.tex_polygons.size (); i++)
  {
    for (std::size_t j = 0; j < mesh.tex_polygons[i].size (); j++)
    {
      std::size_t n_points = mesh.tex_polygons[i][j].vertices.size ();
      polys->InsertNextCell (int (n_points));
      for (std::size_t k = 0; k < n_points; k++)
        polys->InsertCellPoint (mesh.tex_polygons[i][j].vertices[k]);
    }
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPolys (polys);
  polydata->SetPoints (poly_points);
  if (has_color)
    polydata->GetPointData()->SetScalars(colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
#if VTK_MAJOR_VERSION < 6
    mapper->SetInput (polydata);
#else
    mapper->SetInputData (polydata);
#endif

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  vtkTextureUnitManager* tex_manager = vtkOpenGLRenderWindow::SafeDownCast (_visualizer->getRenderWindow())->GetTextureUnitManager ();
  if (!tex_manager)
    return (false);
 
    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New ();
    // fill vtkTexture from pcl::TexMaterial structure
	vtkSmartPointer<vtkImageMatSource> cvImageToVtk = vtkSmartPointer<vtkImageMatSource>::New();
	cvImageToVtk->SetImage(image);
	cvImageToVtk->Update();
	texture->SetInputConnection(cvImageToVtk->GetOutputPort());

    // set texture coordinates
    vtkSmartPointer<vtkFloatArray> coordinates = vtkSmartPointer<vtkFloatArray>::New ();
    coordinates->SetNumberOfComponents (2);
    coordinates->SetNumberOfTuples (mesh.tex_coordinates[0].size ());
    for (std::size_t tc = 0; tc < mesh.tex_coordinates[0].size (); ++tc)
    {
      const Eigen::Vector2f &uv = mesh.tex_coordinates[0][tc];
      coordinates->SetTuple2 (tc, (double)uv[0], (double)uv[1]);
    }
    coordinates->SetName ("TCoords");
    polydata->GetPointData ()->SetTCoords(coordinates);
    // apply texture
    actor->SetTexture (texture);

  // set mapper
  actor->SetMapper (mapper);


  //_visualizer->addActorToRenderer (actor, viewport);
  // Add it to all renderers
	_visualizer->getRendererCollection()->InitTraversal ();
	vtkRenderer* renderer = NULL;
	int i = 0;
	while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
	{
		// Should we add the actor to all renderers?
		if (viewport == 0)
		{
			renderer->AddActor (actor);
		}
		else if (viewport == i)               // add the actor only to the specified viewport
		{
			renderer->AddActor (actor);
		}
		++i;
	}

  // Save the pointer/ID pair to the global actor map
  (*_visualizer->getCloudActorMap())[id].actor = actor;

  // Save the viewpoint transformation matrix to the global actor map
  (*_visualizer->getCloudActorMap())[id].viewpoint_transformation_ = transformation;

  _visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
  _visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
  _visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
  _visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
  _visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
  return true;
}

bool CloudViewer::addOccupancyGridMap(
		const cv::Mat & map8U,
		float resolution, // cell size
		float xMin,
		float yMin,
		float opacity)
{
	UASSERT(map8U.channels() == 1 && map8U.type() == CV_8U);

	float xSize = float(map8U.cols) * resolution;
	float ySize = float(map8U.rows) * resolution;

	UDEBUG("resolution=%f, xSize=%f, ySize=%f, xMin=%f, yMin=%f", resolution, xSize, ySize, xMin, yMin);
	if(_visualizer->getCloudActorMap()->find("map") != _visualizer->getCloudActorMap()->end())
	{
		_visualizer->removePointCloud("map");
	}

	if(xSize > 0.0f && ySize > 0.0f)
	{
		pcl::TextureMeshPtr mesh(new pcl::TextureMesh());
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.push_back(pcl::PointXYZ(xMin,       yMin,       0));
		cloud.push_back(pcl::PointXYZ(xSize+xMin, yMin,       0));
		cloud.push_back(pcl::PointXYZ(xSize+xMin, ySize+yMin, 0));
		cloud.push_back(pcl::PointXYZ(xMin,       ySize+yMin, 0));
		pcl::toPCLPointCloud2(cloud, mesh->cloud);

		std::vector<pcl::Vertices> polygons(1);
		polygons[0].vertices.push_back(0);
		polygons[0].vertices.push_back(1);
		polygons[0].vertices.push_back(2);
		polygons[0].vertices.push_back(3);
		polygons[0].vertices.push_back(0);
		mesh->tex_polygons.push_back(polygons);

		// default texture materials parameters
		pcl::TexMaterial material;
		material.tex_file = "";
		mesh->tex_materials.push_back(material);

#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
		std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > coordinates;
#else
		std::vector<Eigen::Vector2f> coordinates;
#endif
		coordinates.push_back(Eigen::Vector2f(0,1));
		coordinates.push_back(Eigen::Vector2f(1,1));
		coordinates.push_back(Eigen::Vector2f(1,0));
		coordinates.push_back(Eigen::Vector2f(0,0));
		mesh->tex_coordinates.push_back(coordinates);

		this->addTextureMesh(*mesh, map8U, "map");
		setCloudOpacity("map", opacity);
	}
	return true;
}

void CloudViewer::removeOccupancyGridMap()
{
	if(_visualizer->getCloudActorMap()->find("map") != _visualizer->getCloudActorMap()->end())
	{
		_visualizer->removePointCloud("map");
	}
}

void CloudViewer::addOrUpdateCoordinate(
			const std::string & id,
			const Transform & transform,
			double scale)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeCoordinate(id);

	if(!transform.isNull())
	{
		_coordinates.insert(id);
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		_visualizer->addCoordinateSystem(scale, transform.toEigen3f(), id);
#else
		// Well, on older versions, just update the main coordinate
		_visualizer->addCoordinateSystem(scale, transform.toEigen3f(), 0);
#endif
	}
}

bool CloudViewer::updateCoordinatePose(
		const std::string & id,
		const Transform & pose)
{
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	if(_coordinates.find(id) != _coordinates.end() && !pose.isNull())
	{
		UDEBUG("Updating pose %s to %s", id.c_str(), pose.prettyPrint().c_str());
		return _visualizer->updateCoordinateSystemPose(id, pose.toEigen3f());
	}
#else
	UERROR("CloudViewer::updateCoordinatePose() is not available on PCL < 1.7.2");
#endif
	return false;
}

void CloudViewer::removeCoordinate(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_coordinates.find(id) != _coordinates.end())
	{
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		_visualizer->removeCoordinateSystem(id);
#else
		// Well, on older versions, just update the main coordinate
		_visualizer->removeCoordinateSystem(0);
#endif
		_coordinates.erase(id);
	}
}

void CloudViewer::removeAllCoordinates()
{
	std::set<std::string> coordinates = _coordinates;
	for(std::set<std::string>::iterator iter = coordinates.begin(); iter!=coordinates.end(); ++iter)
	{
		this->removeCoordinate(*iter);
	}
	UASSERT(_coordinates.empty());
}

void CloudViewer::addOrUpdateLine(
			const std::string & id,
			const Transform & from,
			const Transform & to,
			const QColor & color,
			bool arrow)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeLine(id);

	if(!from.isNull() && !to.isNull())
	{
		_lines.insert(id);

		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}

		pcl::PointXYZ pt1(from.x(), from.y(), from.z());
		pcl::PointXYZ pt2(to.x(), to.y(), to.z());

		if(arrow)
		{
			_visualizer->addArrow(pt2, pt1, c.redF(), c.greenF(), c.blueF(), false, id);
		}
		else
		{
			_visualizer->addLine(pt2, pt1, c.redF(), c.greenF(), c.blueF(), id);
		}
	}
}

void CloudViewer::removeLine(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_lines.find(id) != _lines.end())
	{
		_visualizer->removeShape(id);
		_lines.erase(id);
	}
}

void CloudViewer::removeAllLines()
{
	std::set<std::string> arrows = _lines;
	for(std::set<std::string>::iterator iter = arrows.begin(); iter!=arrows.end(); ++iter)
	{
		this->removeLine(*iter);
	}
	UASSERT(_lines.empty());
}

static const float frustum_vertices[] = {
    0.0f,  0.0f, 0.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, -1.0f, 1.0f,
	1.0f, -1.0f, -1.0f,
	1.0f, 1.0f, -1.0f};

static const int frustum_indices[] = {
    1, 2, 3, 4, 1, 0, 2, 0, 3, 0, 4};

void CloudViewer::addOrUpdateFrustum(
			const std::string & id,
			const Transform & transform,
			const Transform & localTransform,
			double scale,
			const QColor & color)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	this->removeFrustum(id);
#endif

	if(!transform.isNull())
	{
		if(_frustums.find(id)==_frustums.end())
		{
			_frustums.insert(id, Transform());

			int frustumSize = sizeof(frustum_vertices)/sizeof(float);
			UASSERT(frustumSize>0 && frustumSize % 3 == 0);
			frustumSize/=3;
			pcl::PointCloud<pcl::PointXYZ> frustumPoints;
			frustumPoints.resize(frustumSize);
			float scaleX = 0.5f * scale;
			float scaleY = 0.4f * scale; //4x3 arbitrary ratio
			float scaleZ = 0.3f * scale;
			QColor c = Qt::gray;
			if(color.isValid())
			{
				c = color;
			}
			Transform opticalRotInv(0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0);

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
			Eigen::Affine3f t = (transform*localTransform*opticalRotInv).toEigen3f();
#else
			Eigen::Affine3f t = (localTransform*opticalRotInv).toEigen3f();
#endif
			for(int i=0; i<frustumSize; ++i)
			{
				frustumPoints[i].x = frustum_vertices[i*3]*scaleX;
				frustumPoints[i].y = frustum_vertices[i*3+1]*scaleY;
				frustumPoints[i].z = frustum_vertices[i*3+2]*scaleZ;
				frustumPoints[i] = pcl::transformPoint(frustumPoints[i], t);
			}

			pcl::PolygonMesh mesh;
			pcl::Vertices vertices;
			vertices.vertices.resize(sizeof(frustum_indices)/sizeof(int));
			for(unsigned int i=0; i<vertices.vertices.size(); ++i)
			{
				vertices.vertices[i] = frustum_indices[i];
			}
			pcl::toPCLPointCloud2(frustumPoints, mesh.cloud);
			mesh.polygons.push_back(vertices);
			_visualizer->addPolylineFromPolygonMesh(mesh, id);
			_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c.redF(), c.greenF(), c.blueF(), id);
		}
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		if(!this->updateFrustumPose(id, transform))
		{
			UERROR("Failed updating pose of frustum %s!?", id.c_str());
		}
#endif
	}
	else
	{
		removeFrustum(id);
	}
}

bool CloudViewer::updateFrustumPose(
		const std::string & id,
		const Transform & pose)
{
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	QMap<std::string, Transform>::iterator iter=_frustums.find(id);
	if(iter != _frustums.end() && !pose.isNull())
	{
		if(iter.value() == pose)
		{
			// same pose, just return
			return true;
		}

		pcl::visualization::ShapeActorMap::iterator am_it = _visualizer->getShapeActorMap()->find (id);

		vtkActor* actor;

		if (am_it == _visualizer->getShapeActorMap()->end ())
			return (false);
		else
			actor = vtkActor::SafeDownCast (am_it->second);

		if (!actor)
			return (false);

		vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();

		pcl::visualization::PCLVisualizer::convertToVtkMatrix (pose.toEigen3f().matrix (), matrix);

		actor->SetUserMatrix (matrix);
		actor->Modified ();

		iter.value() = pose;

		return true;
	}
#else
	UERROR("updateFrustumPose() cannot be used with PCL<1.7.2. Use addOrUpdateFrustum() instead.");
#endif
	return false;
}

void CloudViewer::removeFrustum(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_frustums.find(id) != _frustums.end())
	{
		_visualizer->removeShape(id);
		_frustums.remove(id);
	}
}

void CloudViewer::removeAllFrustums(bool exceptCameraReference)
{
	QMap<std::string, Transform> frustums = _frustums;
	for(QMap<std::string, Transform>::iterator iter = frustums.begin(); iter!=frustums.end(); ++iter)
	{
		if(!exceptCameraReference || !uStrContains(iter.key(), "reference_frustum"))
		{
			this->removeFrustum(iter.key());
		}
	}
	UASSERT(exceptCameraReference || _frustums.empty());
}

void CloudViewer::addOrUpdateGraph(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & graph,
		const QColor & color)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeGraph(id);

	if(graph->size())
	{
		_graphes.insert(id);

		pcl::PolygonMesh mesh;
		pcl::Vertices vertices;
		vertices.vertices.resize(graph->size());
		for(unsigned int i=0; i<vertices.vertices.size(); ++i)
		{
			vertices.vertices[i] = i;
		}
		pcl::toPCLPointCloud2(*graph, mesh.cloud);
		mesh.polygons.push_back(vertices);
		_visualizer->addPolylineFromPolygonMesh(mesh, id);
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), id);

		this->addCloud(id+"_nodes", graph, Transform::getIdentity(), color);
		this->setCloudPointSize(id+"_nodes", 5);
	}
}

void CloudViewer::removeGraph(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_graphes.find(id) != _graphes.end())
	{
		_visualizer->removeShape(id);
		_graphes.erase(id);
		removeCloud(id+"_nodes");
	}
}

void CloudViewer::removeAllGraphs()
{
	std::set<std::string> graphes = _graphes;
	for(std::set<std::string>::iterator iter = graphes.begin(); iter!=graphes.end(); ++iter)
	{
		this->removeGraph(*iter);
	}
	UASSERT(_graphes.empty());
}

void CloudViewer::addOrUpdateText(
			const std::string & id,
			const std::string & text,
			const Transform & position,
			double scale,
			const QColor & color)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeCoordinate(id);

	if(!position.isNull())
	{
		_texts.insert(id);
		_visualizer->addText3D(
				text,
				pcl::PointXYZ(position.x(), position.y(), position.z()),
				scale,
				color.redF(),
				color.greenF(),
				color.blueF(),
				id);
	}
}

void CloudViewer::removeText(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_texts.find(id) != _texts.end())
	{
		_visualizer->removeText3D(id);
		_texts.erase(id);
	}
}

void CloudViewer::removeAllTexts()
{
	std::set<std::string> texts = _texts;
	for(std::set<std::string>::iterator iter = texts.begin(); iter!=texts.end(); ++iter)
	{
		this->removeText(*iter);
	}
	UASSERT(_texts.empty());
}

bool CloudViewer::isTrajectoryShown() const
{
	return _aShowTrajectory->isChecked();
}

unsigned int CloudViewer::getTrajectorySize() const
{
	return _maxTrajectorySize;
}

void CloudViewer::setTrajectoryShown(bool shown)
{
	_aShowTrajectory->setChecked(shown);
}

void CloudViewer::setTrajectorySize(unsigned int value)
{
	_maxTrajectorySize = value;
}

void CloudViewer::clearTrajectory()
{
	_trajectory->clear();
	_visualizer->removeShape("trajectory");
	this->update();
}

bool CloudViewer::isFrustumShown() const
{
	return _aShowFrustum->isChecked();
}

float CloudViewer::getFrustumScale() const
{
	return _frustumScale;
}

QColor CloudViewer::getFrustumColor() const
{
	return _frustumColor;
}

void CloudViewer::setFrustumShown(bool shown)
{
	if(!shown)
	{
		QMap<std::string, Transform> frustumsCopy = _frustums;
		for(QMap<std::string, Transform>::iterator iter=frustumsCopy.begin(); iter!=frustumsCopy.end(); ++iter)
		{
			if(uStrContains(iter.key(), "reference_frustum"))
			{
				this->removeFrustum(iter.key());
			}
		}
		std::set<std::string> linesCopy = _lines;
		for(std::set<std::string>::iterator iter=linesCopy.begin(); iter!=linesCopy.end(); ++iter)
		{
			if(uStrContains(*iter, "reference_frustum_line"))
			{
				this->removeLine(*iter);
			}
		}
		this->update();
	}
	_aShowFrustum->setChecked(shown);
}

void CloudViewer::setFrustumScale(float value)
{
	_frustumScale = value;
}

void CloudViewer::setFrustumColor(QColor value)
{
	if(!value.isValid())
	{
		value = Qt::gray;
	}
	for(QMap<std::string, Transform>::iterator iter=_frustums.begin(); iter!=_frustums.end(); ++iter)
	{
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, value.redF(), value.greenF(), value.blueF(), iter.key());
	}
	this->update();
	_frustumColor = value;
}

void CloudViewer::resetCamera()
{
	_lastCameraOrientation= _lastCameraPose = cv::Vec3f(0,0,0);
	if((_aFollowCamera->isChecked() || _aLockCamera->isChecked()) && !_lastPose.isNull())
	{
		// reset relative to last current pose
		cv::Point3f pt = util3d::transformPoint(cv::Point3f(_lastPose.x(), _lastPose.y(), _lastPose.z()), ( _lastPose.rotation()*Transform(-1, 0, 0)).translation());
		if(_aLockViewZ->isChecked())
		{
			_visualizer->setCameraPosition(
					pt.x, pt.y, pt.z,
					_lastPose.x(), _lastPose.y(), _lastPose.z(),
					0, 0, 1);
		}
		else
		{
			_visualizer->setCameraPosition(
					pt.x, pt.y, pt.z,
					_lastPose.x(), _lastPose.y(), _lastPose.z(),
					_lastPose.r31(), _lastPose.r32(), _lastPose.r33());
		}
	}
	else
	{
		_visualizer->setCameraPosition(
				-1, 0, 0,
				0, 0, 0,
				0, 0, 1);
	}
	this->update();
}

void CloudViewer::removeAllClouds()
{
	_addedClouds.clear();
	_visualizer->removeAllPointClouds();
}


bool CloudViewer::removeCloud(const std::string & id)
{
	bool success = _visualizer->removePointCloud(id);
	_visualizer->removePointCloud(id+"-normals");
	_addedClouds.remove(id); // remove after visualizer
	_addedClouds.remove(id+"-normals");
	return success;
}

bool CloudViewer::getPose(const std::string & id, Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		pose = _addedClouds.value(id);
		return true;
	}
	return false;
}

Transform CloudViewer::getTargetPose() const
{
	if(_lastPose.isNull())
	{
		return Transform::getIdentity();
	}
	return _lastPose;
}

void CloudViewer::setBackfaceCulling(bool enabled, bool frontfaceCulling)
{
	_aBackfaceCulling->setChecked(enabled);
	_frontfaceCulling = frontfaceCulling;

	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		iter->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
	}
	this->update();
}

void CloudViewer::setRenderingRate(double rate)
{
	_renderingRate = rate;
	_visualizer->getInteractorStyle()->GetInteractor()->SetDesiredUpdateRate(_renderingRate);
}

void CloudViewer::setLighting(bool on)
{
	_aSetLighting->setChecked(on);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
	}
	this->update();
}

void CloudViewer::setShading(bool on)
{
	_aSetFlatShading->setChecked(on);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG); // VTK_FLAT - VTK_GOURAUD - VTK_PHONG
	}
	this->update();
}

void CloudViewer::setEdgeVisibility(bool visible)
{
	_aSetEdgeVisibility->setChecked(visible);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
	}
	this->update();
}

void CloudViewer::getCameraPosition(
		float & x, float & y, float & z,
		float & focalX, float & focalY, float & focalZ,
		float & upX, float & upY, float & upZ) const
{
	std::vector<pcl::visualization::Camera> cameras;
	_visualizer->getCameras(cameras);
	if(cameras.size())
	{
		x = cameras.begin()->pos[0];
		y = cameras.begin()->pos[1];
		z = cameras.begin()->pos[2];
		focalX = cameras.begin()->focal[0];
		focalY = cameras.begin()->focal[1];
		focalZ = cameras.begin()->focal[2];
		upX = cameras.begin()->view[0];
		upY = cameras.begin()->view[1];
		upZ = cameras.begin()->view[2];
	}
	else
	{
		UERROR("No camera set!?");
	}
}

void CloudViewer::setCameraPosition(
		float x, float y, float z,
		float focalX, float focalY, float focalZ,
		float upX, float upY, float upZ)
{
	_lastCameraOrientation= _lastCameraPose= cv::Vec3f(0,0,0);
	_visualizer->setCameraPosition(x,y,z, focalX,focalY,focalX, upX,upY,upZ);
}

void CloudViewer::updateCameraTargetPosition(const Transform & pose)
{
	if(!pose.isNull())
	{
		Eigen::Affine3f m = pose.toEigen3f();
		Eigen::Vector3f pos = m.translation();

		Eigen::Vector3f lastPos(0,0,0);
		if(_trajectory->size())
		{
			lastPos[0]=_trajectory->back().x;
			lastPos[1]=_trajectory->back().y;
			lastPos[2]=_trajectory->back().z;
		}

		_trajectory->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
		if(_maxTrajectorySize>0)
		{
			while(_trajectory->size() > _maxTrajectorySize)
			{
				_trajectory->erase(_trajectory->begin());
			}
		}
		if(_aShowTrajectory->isChecked())
		{
			_visualizer->removeShape("trajectory");
			pcl::PolygonMesh mesh;
			pcl::Vertices vertices;
			vertices.vertices.resize(_trajectory->size());
			for(unsigned int i=0; i<vertices.vertices.size(); ++i)
			{
				vertices.vertices[i] = i;
			}
			pcl::toPCLPointCloud2(*_trajectory, mesh.cloud);
			mesh.polygons.push_back(vertices);
			_visualizer->addPolylineFromPolygonMesh(mesh, "trajectory");
		}

		if(pose != _lastPose || _lastPose.isNull())
		{
			if(_lastPose.isNull())
			{
				_lastPose.setIdentity();
			}

			std::vector<pcl::visualization::Camera> cameras;
			_visualizer->getCameras(cameras);

			if(_aLockCamera->isChecked())
			{
				//update camera position
				Eigen::Vector3f diff = pos - Eigen::Vector3f(_lastPose.x(), _lastPose.y(), _lastPose.z());
				cameras.front().pos[0] += diff[0];
				cameras.front().pos[1] += diff[1];
				cameras.front().pos[2] += diff[2];
				cameras.front().focal[0] += diff[0];
				cameras.front().focal[1] += diff[1];
				cameras.front().focal[2] += diff[2];
			}
			else if(_aFollowCamera->isChecked())
			{
				Eigen::Vector3f vPosToFocal = Eigen::Vector3f(cameras.front().focal[0] - cameras.front().pos[0],
															  cameras.front().focal[1] - cameras.front().pos[1],
															  cameras.front().focal[2] - cameras.front().pos[2]).normalized();
				Eigen::Vector3f zAxis(cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
				Eigen::Vector3f yAxis = zAxis.cross(vPosToFocal);
				Eigen::Vector3f xAxis = yAxis.cross(zAxis);
				Transform PR(xAxis[0], xAxis[1], xAxis[2],0,
							yAxis[0], yAxis[1], yAxis[2],0,
							zAxis[0], zAxis[1], zAxis[2],0);

				Transform P(PR[0], PR[1], PR[2], cameras.front().pos[0],
							PR[4], PR[5], PR[6], cameras.front().pos[1],
							PR[8], PR[9], PR[10], cameras.front().pos[2]);
				Transform F(PR[0], PR[1], PR[2], cameras.front().focal[0],
							PR[4], PR[5], PR[6], cameras.front().focal[1],
							PR[8], PR[9], PR[10], cameras.front().focal[2]);
				Transform N = pose;
				Transform O = _lastPose;
				Transform O2N = O.inverse()*N;
				Transform F2O = F.inverse()*O;
				Transform T = F2O * O2N * F2O.inverse();
				Transform Fp = F * T;
				Transform P2F = P.inverse()*F;
				Transform Pp = P * P2F * T * P2F.inverse();

				cameras.front().pos[0] = Pp.x();
				cameras.front().pos[1] = Pp.y();
				cameras.front().pos[2] = Pp.z();
				cameras.front().focal[0] = Fp.x();
				cameras.front().focal[1] = Fp.y();
				cameras.front().focal[2] = Fp.z();
				//FIXME: the view up is not set properly...
				cameras.front().view[0] = _aLockViewZ->isChecked()?0:Fp[8];
				cameras.front().view[1] = _aLockViewZ->isChecked()?0:Fp[9];
				cameras.front().view[2] = _aLockViewZ->isChecked()?1:Fp[10];
			}

#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
			if(_coordinates.find("reference") != _coordinates.end())
			{
				this->updateCoordinatePose("reference", pose);
			}
			else
#endif
			{
				this->addOrUpdateCoordinate("reference", pose, 0.2);
			}

			vtkRenderer* renderer = _visualizer->getRendererCollection()->GetFirstRenderer();
			vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
			cam->SetPosition (cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2]);
			cam->SetFocalPoint (cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2]);
			cam->SetViewUp (cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
			renderer->ResetCameraClippingRange();
		}
	}

	_lastPose = pose;
}

void CloudViewer::updateCameraFrustum(const Transform & pose, const StereoCameraModel & model)
{
	std::vector<CameraModel> models;
	models.push_back(model.left());
	updateCameraFrustums(pose, models);
}

void CloudViewer::updateCameraFrustum(const Transform & pose, const CameraModel & model)
{
	std::vector<CameraModel> models;
	models.push_back(model);
	updateCameraFrustums(pose, models);
}

void CloudViewer::updateCameraFrustums(const Transform & pose, const std::vector<CameraModel> & models)
{
	if(!pose.isNull())
	{
		if(_aShowFrustum->isChecked())
		{
			Transform baseToCamera;
			for(unsigned int i=0; i<models.size(); ++i)
			{
				baseToCamera = Transform::getIdentity();
				if(!models[i].localTransform().isNull() && !models[i].localTransform().isIdentity())
				{
					baseToCamera = models[i].localTransform();
				}
				std::string id = uFormat("reference_frustum_%d", i);
				this->removeFrustum(id);
				this->addOrUpdateFrustum(id, pose, baseToCamera, _frustumScale, _frustumColor);
				if(!baseToCamera.isIdentity())
				{
					this->addOrUpdateLine(uFormat("reference_frustum_line_%d", i), pose, pose * baseToCamera, _frustumColor);
				}
			}
		}
	}
}

const QColor & CloudViewer::getDefaultBackgroundColor() const
{
	return _defaultBgColor;
}

void CloudViewer::setDefaultBackgroundColor(const QColor & color)
{
	if(_currentBgColor == _defaultBgColor)
	{
		setBackgroundColor(color);
	}
	_defaultBgColor = color;
}

const QColor & CloudViewer::getBackgroundColor() const
{
	return _currentBgColor;
}

void CloudViewer::setBackgroundColor(const QColor & color)
{
	_currentBgColor = color;
	_visualizer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
}

void CloudViewer::setCloudVisibility(const std::string & id, bool isVisible)
{
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	pcl::visualization::CloudActorMap::iterator iter = cloudActorMap->find(id);
	if(iter != cloudActorMap->end())
	{
		iter->second.actor->SetVisibility(isVisible?1:0);

		iter = cloudActorMap->find(id+"-normals");
		if(iter != cloudActorMap->end())
		{
			iter->second.actor->SetVisibility(isVisible&&_aShowNormals->isChecked()?1:0);
		}
	}
	else
	{
		UERROR("Cannot find actor named \"%s\".", id.c_str());
	}
}

bool CloudViewer::getCloudVisibility(const std::string & id)
{
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	pcl::visualization::CloudActorMap::iterator iter = cloudActorMap->find(id);
	if(iter != cloudActorMap->end())
	{
		return iter->second.actor->GetVisibility() != 0;
	}
	else
	{
		UERROR("Cannot find actor named \"%s\".", id.c_str());
	}
	return false;
}

void CloudViewer::setCloudOpacity(const std::string & id, double opacity)
{
	double lastOpacity;
	_visualizer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, lastOpacity, id);
	if(lastOpacity != opacity)
	{
		_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
	}
}

void CloudViewer::setCloudPointSize(const std::string & id, int size)
{
	double lastSize;
	_visualizer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, lastSize, id);
	if((int)lastSize != size)
	{
		_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (double)size, id);
	}
}

void CloudViewer::setCameraTargetLocked(bool enabled)
{
	_aLockCamera->setChecked(enabled);
}

void CloudViewer::setCameraTargetFollow(bool enabled)
{
	_aFollowCamera->setChecked(enabled);
}

void CloudViewer::setCameraFree()
{
	_aLockCamera->setChecked(false);
	_aFollowCamera->setChecked(false);
}

void CloudViewer::setCameraLockZ(bool enabled)
{
	_lastCameraOrientation= _lastCameraPose = cv::Vec3f(0,0,0);
	_aLockViewZ->setChecked(enabled);
}
bool CloudViewer::isCameraTargetLocked() const
{
	return _aLockCamera->isChecked();
}
bool CloudViewer::isCameraTargetFollow() const
{
	return _aFollowCamera->isChecked();
}
bool CloudViewer::isCameraFree() const
{
	return !_aFollowCamera->isChecked() && !_aLockCamera->isChecked();
}
bool CloudViewer::isCameraLockZ() const
{
	return _aLockViewZ->isChecked();
}
double CloudViewer::getRenderingRate() const
{
	return _renderingRate;
}

void CloudViewer::setGridShown(bool shown)
{
	_aShowGrid->setChecked(shown);
	if(shown)
	{
		this->addGrid();
	}
	else
	{
		this->removeGrid();
	}
}
bool CloudViewer::isGridShown() const
{
	return _aShowGrid->isChecked();
}
unsigned int CloudViewer::getGridCellCount() const
{
	return _gridCellCount;
}
float CloudViewer::getGridCellSize() const
{
	return _gridCellSize;
}
void CloudViewer::setGridCellCount(unsigned int count)
{
	if(count > 0)
	{
		_gridCellCount = count;
		if(_aShowGrid->isChecked())
		{
			this->removeGrid();
			this->addGrid();
		}
	}
	else
	{
		UERROR("Cannot set grid cell count < 1, count=%d", count);
	}
}
void CloudViewer::setGridCellSize(float size)
{
	if(size > 0)
	{
		_gridCellSize = size;
		if(_aShowGrid->isChecked())
		{
			this->removeGrid();
			this->addGrid();
		}
	}
	else
	{
		UERROR("Cannot set grid cell size <= 0, value=%f", size);
	}
}
void CloudViewer::addGrid()
{
	if(_gridLines.empty())
	{
		float cellSize = _gridCellSize;
		int cellCount = _gridCellCount;
		double r=0.5;
		double g=0.5;
		double b=0.5;
		int id = 0;
		float min = -float(cellCount/2) * cellSize;
		float max = float(cellCount/2) * cellSize;
		std::string name;
		for(float i=min; i<=max; i += cellSize)
		{
			//over x
			name = uFormat("line%d", ++id);
			_visualizer->addLine(pcl::PointXYZ(i, min, 0.0f), pcl::PointXYZ(i, max, 0.0f), r, g, b, name);
			_gridLines.push_back(name);
			//over y or z
			name = uFormat("line%d", ++id);
			_visualizer->addLine(
					pcl::PointXYZ(min, i, 0),
					pcl::PointXYZ(max, i, 0),
					r, g, b, name);
			_gridLines.push_back(name);
		}
	}
}

void CloudViewer::removeGrid()
{
	for(std::list<std::string>::iterator iter = _gridLines.begin(); iter!=_gridLines.end(); ++iter)
	{
		_visualizer->removeShape(*iter);
	}
	_gridLines.clear();
}

void CloudViewer::setNormalsShown(bool shown)
{
	_aShowNormals->setChecked(shown);
	QList<std::string> ids = _addedClouds.keys();
	for(QList<std::string>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
	{
		std::string idNormals = *iter + "-normals";
		if(_addedClouds.find(idNormals) != _addedClouds.end())
		{
			this->setCloudVisibility(idNormals, this->getCloudVisibility(*iter) && shown);
		}
	}
}
bool CloudViewer::isNormalsShown() const
{
	return _aShowNormals->isChecked();
}
int CloudViewer::getNormalsStep() const
{
	return _normalsStep;
}
float CloudViewer::getNormalsScale() const
{
	return _normalsScale;
}
void CloudViewer::setNormalsStep(int step)
{
	if(step > 0)
	{
		_normalsStep = step;
	}
	else
	{
		UERROR("Cannot set normals step <= 0, step=%d", step);
	}
}
void CloudViewer::setNormalsScale(float scale)
{
	if(scale > 0)
	{
		_normalsScale= scale;
	}
	else
	{
		UERROR("Cannot set normals scale <= 0, value=%f", scale);
	}
}

Eigen::Vector3f rotatePointAroundAxe(
		const Eigen::Vector3f & point,
		const Eigen::Vector3f & axis,
		float angle)
{
	Eigen::Vector3f direction = point;
	Eigen::Vector3f zAxis = axis;
	float dotProdZ = zAxis.dot(direction);
	Eigen::Vector3f ptOnZaxis = zAxis * dotProdZ;
	direction -= ptOnZaxis;
	Eigen::Vector3f xAxis = direction.normalized();
	Eigen::Vector3f yAxis = zAxis.cross(xAxis);

	Eigen::Matrix3f newFrame;
	newFrame << xAxis[0], yAxis[0], zAxis[0],
				  xAxis[1], yAxis[1], zAxis[1],
				  xAxis[2], yAxis[2], zAxis[2];

	// transform to axe frame
	// transpose=inverse for orthogonal matrices
	Eigen::Vector3f newDirection = newFrame.transpose() * direction;

	// rotate about z
	float cosTheta = cos(angle);
	float sinTheta = sin(angle);
	float magnitude = newDirection.norm();
	newDirection[0] = ( magnitude * cosTheta );
	newDirection[1] = ( magnitude * sinTheta );

	// transform back to global frame
	direction = newFrame * newDirection;

	return direction + ptOnZaxis;
}

void CloudViewer::keyReleaseEvent(QKeyEvent * event) {
	if(event->key() == Qt::Key_Up ||
		event->key() == Qt::Key_Down ||
		event->key() == Qt::Key_Left ||
		event->key() == Qt::Key_Right)
	{
		_keysPressed -= (Qt::Key)event->key();
	}
	else
	{
		QVTKWidget::keyPressEvent(event);
	}
}

void CloudViewer::keyPressEvent(QKeyEvent * event)
{
	if(event->key() == Qt::Key_Up ||
		event->key() == Qt::Key_Down ||
		event->key() == Qt::Key_Left ||
		event->key() == Qt::Key_Right)
	{
		_keysPressed += (Qt::Key)event->key();

		std::vector<pcl::visualization::Camera> cameras;
		_visualizer->getCameras(cameras);

		//update camera position
		Eigen::Vector3f pos(cameras.front().pos[0], cameras.front().pos[1], _aLockViewZ->isChecked()?0:cameras.front().pos[2]);
		Eigen::Vector3f focal(cameras.front().focal[0], cameras.front().focal[1], _aLockViewZ->isChecked()?0:cameras.front().focal[2]);
		Eigen::Vector3f viewUp(cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
		Eigen::Vector3f cummulatedDir(0,0,0);
		Eigen::Vector3f cummulatedFocalDir(0,0,0);
		float step = 0.2f;
		float stepRot = 0.02f; // radian
		if(_keysPressed.contains(Qt::Key_Up))
		{
			Eigen::Vector3f dir;
			if(event->modifiers() & Qt::ShiftModifier)
			{
				dir = viewUp * step;// up
			}
			else
			{
				dir = (focal-pos).normalized() * step; // forward
			}
			cummulatedDir += dir;
		}
		if(_keysPressed.contains(Qt::Key_Down))
		{
			Eigen::Vector3f dir;
			if(event->modifiers() & Qt::ShiftModifier)
			{
				dir = viewUp * -step;// down
			}
			else
			{
				dir = (focal-pos).normalized() * -step; // backward
			}
			cummulatedDir += dir;
		}
		if(_keysPressed.contains(Qt::Key_Right))
		{
			if(event->modifiers() & Qt::ShiftModifier)
			{
				// rotate right
				Eigen::Vector3f point = (focal-pos);
				Eigen::Vector3f newPoint = rotatePointAroundAxe(point, viewUp, -stepRot);
				Eigen::Vector3f diff = newPoint - point;
				cummulatedFocalDir += diff;
			}
			else
			{
				Eigen::Vector3f dir = ((focal-pos).cross(viewUp)).normalized() * step; // strafing right
				cummulatedDir += dir;
			}
		}
		if(_keysPressed.contains(Qt::Key_Left))
		{
			if(event->modifiers() & Qt::ShiftModifier)
			{
				// rotate left
				Eigen::Vector3f point = (focal-pos);
				Eigen::Vector3f newPoint = rotatePointAroundAxe(point, viewUp, stepRot);
				Eigen::Vector3f diff = newPoint - point;
				cummulatedFocalDir += diff;
			}
			else
			{
				Eigen::Vector3f dir = ((focal-pos).cross(viewUp)).normalized() * -step; // strafing left
				cummulatedDir += dir;
			}
		}

		cameras.front().pos[0] += cummulatedDir[0];
		cameras.front().pos[1] += cummulatedDir[1];
		cameras.front().pos[2] += cummulatedDir[2];
		cameras.front().focal[0] += cummulatedDir[0] + cummulatedFocalDir[0];
		cameras.front().focal[1] += cummulatedDir[1] + cummulatedFocalDir[1];
		cameras.front().focal[2] += cummulatedDir[2] + cummulatedFocalDir[2];
		_visualizer->setCameraPosition(
			cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
			cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
			cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

		update();

		emit configChanged();
	}
	else
	{
		QVTKWidget::keyPressEvent(event);
	}
}

void CloudViewer::mousePressEvent(QMouseEvent * event)
{
	if(event->button() == Qt::RightButton)
	{
		event->accept();
	}
	else
	{
		QVTKWidget::mousePressEvent(event);
	}
}

void CloudViewer::mouseMoveEvent(QMouseEvent * event)
{
	QVTKWidget::mouseMoveEvent(event);

	// camera view up z locked?
	if(_aLockViewZ->isChecked())
	{
		std::vector<pcl::visualization::Camera> cameras;
		_visualizer->getCameras(cameras);

		cv::Vec3d newCameraOrientation = cv::Vec3d(0,0,1).cross(cv::Vec3d(cameras.front().pos)-cv::Vec3d(cameras.front().focal));

		if(	_lastCameraOrientation!=cv::Vec3d(0,0,0) &&
			_lastCameraPose!=cv::Vec3d(0,0,0) &&
			(uSign(_lastCameraOrientation[0]) != uSign(newCameraOrientation[0]) &&
			 uSign(_lastCameraOrientation[1]) != uSign(newCameraOrientation[1])))
		{
			cameras.front().pos[0] = _lastCameraPose[0];
			cameras.front().pos[1] = _lastCameraPose[1];
			cameras.front().pos[2] = _lastCameraPose[2];
		}
		else if(newCameraOrientation != cv::Vec3d(0,0,0))
		{
			_lastCameraOrientation = newCameraOrientation;
			_lastCameraPose = cv::Vec3d(cameras.front().pos);
		}
		cameras.front().view[0] = 0;
		cameras.front().view[1] = 0;
		cameras.front().view[2] = 1;

		_visualizer->setCameraPosition(
			cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
			cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
			cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

	}
	this->update();

	emit configChanged();
}

void CloudViewer::wheelEvent(QWheelEvent * event)
{
	QVTKWidget::wheelEvent(event);
	if(_aLockViewZ->isChecked())
	{
		std::vector<pcl::visualization::Camera> cameras;
		_visualizer->getCameras(cameras);
		_lastCameraPose = cv::Vec3d(cameras.front().pos);
	}
	emit configChanged();
}

void CloudViewer::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * a = _menu->exec(event->globalPos());
	if(a)
	{
		handleAction(a);
		emit configChanged();
	}
}

void CloudViewer::handleAction(QAction * a)
{
	if(a == _aSetTrajectorySize)
	{
		bool ok;
		int value = QInputDialog::getInt(this, tr("Set trajectory size"), tr("Size (0=infinite)"), _maxTrajectorySize, 0, 10000, 10, &ok);
		if(ok)
		{
			_maxTrajectorySize = value;
		}
	}
	else if(a == _aClearTrajectory)
	{
		this->clearTrajectory();
	}
	else if(a == _aShowFrustum)
	{
		this->setFrustumShown(a->isChecked());
	}
	else if(a == _aSetFrustumScale)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set frustum scale"), tr("Scale"), _frustumScale, 0.0, 999.0, 1, &ok);
		if(ok)
		{
			this->setFrustumScale(value);
		}
	}
	else if(a == _aSetFrustumColor)
	{
		QColor value = QColorDialog::getColor(_frustumColor, this);
		if(value.isValid())
		{
			this->setFrustumColor(value);
		}
	}
	else if(a == _aResetCamera)
	{
		this->resetCamera();
	}
	else if(a == _aShowGrid)
	{
		if(_aShowGrid->isChecked())
		{
			this->addGrid();
		}
		else
		{
			this->removeGrid();
		}

		this->update();
	}
	else if(a == _aSetGridCellCount)
	{
		bool ok;
		int value = QInputDialog::getInt(this, tr("Set grid cell count"), tr("Count"), _gridCellCount, 1, 10000, 10, &ok);
		if(ok)
		{
			this->setGridCellCount(value);
		}
	}
	else if(a == _aSetGridCellSize)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set grid cell size"), tr("Size (m)"), _gridCellSize, 0.01, 10, 2, &ok);
		if(ok)
		{
			this->setGridCellSize(value);
		}
	}
	else if(a == _aShowNormals)
	{
		this->setNormalsShown(_aShowNormals->isChecked());
		this->update();
	}
	else if(a == _aSetNormalsStep)
	{
		bool ok;
		int value = QInputDialog::getInt(this, tr("Set normals step"), tr("Step"), _normalsStep, 1, 10000, 1, &ok);
		if(ok)
		{
			this->setNormalsStep(value);
		}
	}
	else if(a == _aSetNormalsScale)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set normals scale"), tr("Scale (m)"), _normalsScale, 0.01, 10, 2, &ok);
		if(ok)
		{
			this->setNormalsScale(value);
		}
	}
	else if(a == _aSetBackgroundColor)
	{
		QColor color = this->getDefaultBackgroundColor();
		color = QColorDialog::getColor(color, this);
		if(color.isValid())
		{
			this->setDefaultBackgroundColor(color);
			this->update();
		}
	}
	else if(a == _aSetRenderingRate)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Rendering rate"), tr("Rate (hz)"), _renderingRate, 0, 60, 0, &ok);
		if(ok)
		{
			this->setRenderingRate(value);
		}
	}
	else if(a == _aLockViewZ)
	{
		if(_aLockViewZ->isChecked())
		{
			this->update();
		}
	}
	else if(a == _aSetLighting)
	{
		this->setLighting(_aSetLighting->isChecked());
	}
	else if(a == _aSetFlatShading)
		{
			this->setShading(_aSetFlatShading->isChecked());
		}
	else if(a == _aSetEdgeVisibility)
	{
		this->setEdgeVisibility(_aSetEdgeVisibility->isChecked());
	}
	else if(a == _aBackfaceCulling)
	{
		this->setBackfaceCulling(_aBackfaceCulling->isChecked(), _frontfaceCulling);
	}
}

} /* namespace rtabmap */
