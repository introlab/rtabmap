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
#include "rtabmap/gui/CloudViewerCellPicker.h"

#include <rtabmap/core/Version.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util2d.h>
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
#include <vtkSmartVolumeMapper.h>
#include <vtkVolumeProperty.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkImageData.h>
#include <vtkLookupTable.h>
#include <vtkTextureUnitManager.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkPNMReader.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPointPicker.h>
#include <vtkTextActor.h>
#include <vtkTexture.h>
#include <vtkOBBTree.h>
#include <vtkObjectFactory.h>
#include <vtkQuad.h>
#include <opencv/vtkImageMatSource.h>

#if VTK_MAJOR_VERSION >= 7
#include <vtkEDLShading.h>
#include <vtkRenderStepsPass.h>
#include <vtkOpenGLRenderer.h>
#endif


#if VTK_MAJOR_VERSION >= 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif

namespace rtabmap {

CloudViewer::CloudViewer(QWidget *parent, CloudViewerInteractorStyle * style) :
		PCLQVTKWidget(parent),
		_aLockCamera(0),
		_aFollowCamera(0),
		_aResetCamera(0),
		_aLockViewZ(0),
		_aCameraOrtho(0),
		_aShowTrajectory(0),
		_aSetTrajectorySize(0),
		_aClearTrajectory(0),
		_aShowCameraAxis(0),
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
		_aSetEDLShading(0),
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
		_buildLocator(false),
		_lastCameraOrientation(0,0,0),
		_lastCameraPose(0,0,0),
		_defaultBgColor(Qt::black),
		_currentBgColor(Qt::black),
		_frontfaceCulling(false),
		_renderingRate(5.0),
		_octomapActor(0),
		_intensityAbsMax(100.0f),
		_coordinateFrameScale(1.0)
{
	UDEBUG("");
	this->setMinimumSize(200, 200);
	
	int argc = 0;
	UASSERT(style!=0);
	style->setCloudViewer(this);
	style->AutoAdjustCameraClippingRangeOff();
#if VTK_MAJOR_VERSION > 8
	auto renderer1 = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow1 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow1->AddRenderer(renderer1);
	_visualizer = new pcl::visualization::PCLVisualizer(
		argc, 
		0, 
		renderer1,
		renderWindow1,
		"PCLVisualizer", 
		style,
		false);
#else
		_visualizer = new pcl::visualization::PCLVisualizer(
		argc, 
		0, 
		"PCLVisualizer", 
		style,
		false);
#endif

	_visualizer->setShowFPS(false);
	
	int viewport;
	// Layer 0: unavailable layer, used as "all" by PCLVisualizer
	_visualizer->createViewPort (0,0,1.0, 1.0, viewport); // Layer 1: all clouds here
	_visualizer->createViewPort (0,0,1.0, 1.0, viewport); // Layer 2: all 3d objects here
	_visualizer->createViewPort (0,0,1.0, 1.0, viewport); // Layer 3: text overlay
	_visualizer->getRendererCollection()->InitTraversal ();
	vtkRenderer* renderer = NULL;
	int i =0;
	while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
	{
		renderer->SetLayer(i);
		if(i==1)
		{
#if VTK_MAJOR_VERSION >= 7
			renderer->PreserveColorBufferOff();
#endif
			renderer->PreserveDepthBufferOff();
			_visualizer->getInteractorStyle()->SetDefaultRenderer(renderer);
		}
		else if(i==2)
		{
#if VTK_MAJOR_VERSION >= 7
			renderer->PreserveColorBufferOn();
#endif
			renderer->PreserveDepthBufferOn();
		}
		++i;
	}
	_visualizer->getRenderWindow()->SetNumberOfLayers(4);

#if VTK_MAJOR_VERSION > 8
	this->setRenderWindow(_visualizer->getRenderWindow());
#else
	this->SetRenderWindow(_visualizer->getRenderWindow());
#endif

	// Replaced by the second line, to avoid a crash in Mac OS X on close, as well as
	// the "Invalid drawable" warning when the view is not visible.
	//_visualizer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#if VTK_MAJOR_VERSION > 8
	this->interactor()->SetInteractorStyle (_visualizer->getInteractorStyle());
#else
	this->GetInteractor()->SetInteractorStyle (_visualizer->getInteractorStyle());
#endif
	// setup a simple point picker
	vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
	UDEBUG("pick tolerance=%f", pp->GetTolerance());
	pp->SetTolerance (pp->GetTolerance()/2.0);
#if VTK_MAJOR_VERSION > 8
	this->interactor()->SetPicker (pp);
#else
	this->GetInteractor()->SetPicker (pp);
#endif

	setRenderingRate(_renderingRate);

	this->setCameraPosition(
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

	if(_aShowCameraAxis->isChecked())
	{
		this->addOrUpdateCoordinate("reference", Transform::getIdentity(), 0.2);
	}
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
	_aCameraOrtho = new QAction("Ortho mode", this);
	_aCameraOrtho->setCheckable(true);
	_aCameraOrtho->setChecked(false);
	_aResetCamera = new QAction("Reset position", this);
	_aShowTrajectory= new QAction("Show trajectory", this);
	_aShowTrajectory->setCheckable(true);
	_aShowTrajectory->setChecked(true);
	_aSetTrajectorySize = new QAction("Set trajectory size...", this);
	_aClearTrajectory = new QAction("Clear trajectory", this);
	_aShowCameraAxis= new QAction("Show base frame", this);
	_aShowCameraAxis->setCheckable(true);
	_aShowCameraAxis->setChecked(true);
	_aSetFrameScale= new QAction("Set frame scale...", this);
	_aShowCameraAxis->setChecked(true);
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
	_aSetIntensityRedColormap = new QAction("Red/Yellow Colormap", this);
	_aSetIntensityRedColormap->setCheckable(true);
	_aSetIntensityRedColormap->setChecked(true);
	_aSetIntensityRainbowColormap = new QAction("Rainbow Colormap", this);
	_aSetIntensityRainbowColormap->setCheckable(true);
	_aSetIntensityRainbowColormap->setChecked(false);
	_aSetIntensityMaximum = new QAction("Set maximum absolute intensity...", this);
	_aSetBackgroundColor = new QAction("Set background color...", this);	
	_aSetRenderingRate = new QAction("Set rendering rate...", this);
	_aSetEDLShading = new QAction("Eye-Dome Lighting Shading", this);
	_aSetEDLShading->setCheckable(true);
	_aSetEDLShading->setChecked(false);
#if VTK_MAJOR_VERSION < 7
	_aSetEDLShading->setEnabled(false);
#endif
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
	_aPolygonPicking = new QAction("Polygon picking", this);
	_aPolygonPicking->setCheckable(true);
	_aPolygonPicking->setChecked(false);

	QMenu * cameraMenu = new QMenu("Camera", this);
	cameraMenu->addAction(_aLockCamera);
	cameraMenu->addAction(_aFollowCamera);
	cameraMenu->addAction(freeCamera);
	cameraMenu->addSeparator();
	cameraMenu->addAction(_aLockViewZ);
	cameraMenu->addAction(_aCameraOrtho);
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

	QMenu * scanMenu = new QMenu("Scan color", this);
	scanMenu->addAction(_aSetIntensityRedColormap);
	scanMenu->addAction(_aSetIntensityRainbowColormap);
	scanMenu->addAction(_aSetIntensityMaximum);

	//menus
	_menu = new QMenu(this);
	_menu->addMenu(cameraMenu);
	_menu->addMenu(trajectoryMenu);
	_menu->addAction(_aShowCameraAxis);
	_menu->addAction(_aSetFrameScale);
	_menu->addMenu(frustumMenu);
	_menu->addMenu(gridMenu);
	_menu->addMenu(normalsMenu);
	_menu->addMenu(scanMenu);
	_menu->addAction(_aSetBackgroundColor);
	_menu->addAction(_aSetRenderingRate);
	_menu->addAction(_aSetEDLShading);
	_menu->addAction(_aSetLighting);
	_menu->addAction(_aSetFlatShading);
	_menu->addAction(_aSetEdgeVisibility);
	_menu->addAction(_aBackfaceCulling);
	_menu->addAction(_aPolygonPicking);
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

	settings.setValue("intensity_red_colormap", this->isIntensityRedColormap());
	settings.setValue("intensity_rainbow_colormap", this->isIntensityRainbowColormap());
	settings.setValue("intensity_max", (double)this->getIntensityMax());

	settings.setValue("trajectory_shown", this->isTrajectoryShown());
	settings.setValue("trajectory_size", this->getTrajectorySize());

	settings.setValue("camera_axis_shown", this->isCameraAxisShown());
	settings.setValue("coordinate_frame_scale", this->getCoordinateFrameScale());

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
	_lastCameraOrientation= _lastCameraPose= cv::Vec3f(0,0,0);
	this->setCameraPosition(pose.x(),pose.y(),pose.z(), focal.x(),focal.y(),focal.z(), up.x(),up.y(),up.z());

	this->setGridShown(settings.value("grid", this->isGridShown()).toBool());
	this->setGridCellCount(settings.value("grid_cell_count", this->getGridCellCount()).toUInt());
	this->setGridCellSize(settings.value("grid_cell_size", this->getGridCellSize()).toFloat());

	this->setNormalsShown(settings.value("normals", this->isNormalsShown()).toBool());
	this->setNormalsStep(settings.value("normals_step", this->getNormalsStep()).toInt());
	this->setNormalsScale(settings.value("normals_scale", this->getNormalsScale()).toFloat());

	this->setIntensityRedColormap(settings.value("intensity_red_colormap", this->isIntensityRedColormap()).toBool());
	this->setIntensityRainbowColormap(settings.value("intensity_rainbow_colormap", this->isIntensityRainbowColormap()).toBool());
	this->setIntensityMax(settings.value("intensity_max", this->getIntensityMax()).toFloat());

	this->setTrajectoryShown(settings.value("trajectory_shown", this->isTrajectoryShown()).toBool());
	this->setTrajectorySize(settings.value("trajectory_size", this->getTrajectorySize()).toUInt());

	this->setCameraAxisShown(settings.value("camera_axis_shown", this->isCameraAxisShown()).toBool());
	this->setCoordinateFrameScale(settings.value("coordinate_frame_scale", this->getCoordinateFrameScale()).toDouble());

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

	this->refreshView();
}

void CloudViewer::refreshView()
{
#if VTK_MAJOR_VERSION > 8
	this->renderWindow()->Render();
#else
	this->update();
#endif
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
		if(!samePose)
		{
			// PointCloud / Mesh
			bool updated = _visualizer->updatePointCloudPose(id, posef);

#if VTK_MAJOR_VERSION >= 7
			if(!updated)
			{
				// TextureMesh, cannot use updateShapePose because it searches for vtkLODActor, not a vtkActor
				pcl::visualization::ShapeActorMap::iterator am_it = _visualizer->getShapeActorMap()->find (id);
				vtkActor* actor;
				if (am_it != _visualizer->getShapeActorMap()->end ())
				{
					actor = vtkActor::SafeDownCast (am_it->second);
					if (actor)
					{
						vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();
						pcl::visualization::PCLVisualizer::convertToVtkMatrix (pose.toEigen3f().matrix (), matrix);
						actor->SetUserMatrix (matrix);
						actor->Modified ();
						updated = true;
					}
				}
			}
#endif

			if(updated)
			{
				_addedClouds.find(id).value() = pose;
				std::string idNormals = id+"-normals";
				if(_addedClouds.find(idNormals)!=_addedClouds.end())
				{
					_visualizer->updatePointCloudPose(idNormals, posef);
					_addedClouds.find(idNormals).value() = pose;
				}
				return true;
			}
		}
	}
	return false;
}

class PointCloudColorHandlerIntensityField : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
{
	typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef PointCloud::ConstPtr PointCloudConstPtr;

public:
	typedef boost::shared_ptr<PointCloudColorHandlerIntensityField > Ptr;
	typedef boost::shared_ptr<const PointCloudColorHandlerIntensityField > ConstPtr;

	/** \brief Constructor. */
	PointCloudColorHandlerIntensityField (const PointCloudConstPtr &cloud, float maxAbsIntensity = 0.0f, int colorMap = 0) :
		pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler (cloud),
		maxAbsIntensity_(maxAbsIntensity),
		colormap_(colorMap)
		{
		field_idx_  = pcl::getFieldIndex (*cloud, "intensity");
		if (field_idx_ != -1)
			capable_ = true;
		else
			capable_ = false;
		}

	/** \brief Empty destructor */
	virtual ~PointCloudColorHandlerIntensityField () {}

	/** \brief Obtain the actual color for the input dataset as vtk scalars.
	 * \param[out] scalars the output scalars containing the color for the dataset
	 * \return true if the operation was successful (the handler is capable and
	 * the input cloud was given as a valid pointer), false otherwise
	 */
#if PCL_VERSION_COMPARE(>, 1, 11, 1)
	virtual vtkSmartPointer<vtkDataArray> getColor () const {
		vtkSmartPointer<vtkDataArray> scalars;
		if (!capable_ || !cloud_)
			return scalars;
#else
	virtual bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const {
		if (!capable_ || !cloud_)
			return (false);
#endif
		if (!scalars)
			scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
		scalars->SetNumberOfComponents (3);

		vtkIdType nr_points = cloud_->width * cloud_->height;
		// Allocate enough memory to hold all colors
		float * intensities = new float[nr_points];
		float intensity;
		size_t point_offset = cloud_->fields[field_idx_].offset;
		size_t j = 0;

		// If XYZ present, check if the points are invalid
		int x_idx = pcl::getFieldIndex (*cloud_, "x");
		if (x_idx != -1)
		{
			float x_data, y_data, z_data;
			size_t x_point_offset = cloud_->fields[x_idx].offset;

			// Color every point
			for (vtkIdType cp = 0; cp < nr_points; ++cp,
			point_offset += cloud_->point_step,
			x_point_offset += cloud_->point_step)
			{
				// Copy the value at the specified field
				memcpy (&intensity, &cloud_->data[point_offset], sizeof (float));

				memcpy (&x_data, &cloud_->data[x_point_offset], sizeof (float));
				memcpy (&y_data, &cloud_->data[x_point_offset + sizeof (float)], sizeof (float));
				memcpy (&z_data, &cloud_->data[x_point_offset + 2 * sizeof (float)], sizeof (float));

				if (!std::isfinite (x_data) || !std::isfinite (y_data) || !std::isfinite (z_data))
					continue;

				intensities[j++] = intensity;
			}
		}
		// No XYZ data checks
		else
		{
			// Color every point
			for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
			{
				// Copy the value at the specified field
				memcpy (&intensity, &cloud_->data[point_offset], sizeof (float));

				intensities[j++] = intensity;
			}
		}
		if (j != 0)
		{
			// Allocate enough memory to hold all colors
			unsigned char* colors = new unsigned char[j * 3];
			float min, max;
			if(maxAbsIntensity_>0.0f)
			{
				max = maxAbsIntensity_;
			}
			else
			{
				uMinMax(intensities, j, min, max);
			}
			for(size_t k=0; k<j; ++k)
			{
				colors[k*3+0] = colors[k*3+1] = colors[k*3+2] = max>0?(unsigned char)(std::min(intensities[k]/max*255.0f, 255.0f)):255;
				if(colormap_ == 1)
				{
					colors[k*3+0] = 255;
					colors[k*3+2] = 0;
				}
				else if(colormap_ == 2)
				{
					float r,g,b;
					util2d::HSVtoRGB(&r, &g, &b, colors[k*3+0]*299.0f/255.0f, 1.0f, 1.0f);
					colors[k*3+0] = r*255.0f;
					colors[k*3+1] = g*255.0f;
					colors[k*3+2] = b*255.0f;
				}
			}
			reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (j);
			reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, j*3, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
		}
		else
			reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (0);
		//delete [] colors;
		delete [] intensities;
#if PCL_VERSION_COMPARE(>, 1, 11, 1)
		return scalars;
#else
		return (true);
#endif
	}

protected:
	/** \brief Get the name of the class. */
	virtual std::string
	getName () const { return ("PointCloudColorHandlerIntensityField"); }

	/** \brief Get the name of the field used. */
	virtual std::string
	getFieldName () const { return ("intensity"); }

private:
	float maxAbsIntensity_;
	int colormap_; // 0=grayscale, 1=redYellow, 2=RainbowHSV
};

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PCLPointCloud2Ptr & binaryCloud,
		const Transform & pose,
		bool rgb,
		bool hasNormals,
		bool hasIntensity,
		const QColor & color,
		int viewport)
{
	int previousColorIndex = -1;
	if(_addedClouds.contains(id))
	{
		previousColorIndex = _visualizer->getColorHandlerIndex(id);
		this->removeCloud(id);
	}

	Eigen::Vector4f origin(pose.x(), pose.y(), pose.z(), 0.0f);
	Eigen::Quaternionf orientation = Eigen::Quaternionf(pose.toEigen3f().linear());

	if(hasNormals && _aShowNormals->isChecked())
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromPCLPointCloud2 (*binaryCloud, *cloud_xyz);
		std::string idNormals = id + "-normals";
		if(_visualizer->addPointCloudNormals<pcl::PointNormal>(cloud_xyz, _normalsStep, _normalsScale, idNormals, viewport))
		{
			_visualizer->updatePointCloudPose(idNormals, pose.toEigen3f());
			_addedClouds.insert(idNormals, pose);
		}
	}

	// add random color channel
	 pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr colorHandler;
	 colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (binaryCloud));
	 if(_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport))
	 {
		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (binaryCloud, c.red(), c.green(), c.blue()));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);

		// x,y,z
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "x"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "y"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
		colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "z"));
		_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);

		if(rgb)
		{
			//rgb
			colorHandler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(binaryCloud));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
		}
		else if(hasIntensity)
		{
			//intensity
			colorHandler.reset(new PointCloudColorHandlerIntensityField(binaryCloud, _intensityAbsMax, _aSetIntensityRedColormap->isChecked()?1:_aSetIntensityRainbowColormap->isChecked()?2:0));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
		}
		else if(previousColorIndex == 5)
		{
			previousColorIndex = -1;
		}

		if(hasNormals)
		{
			//normals
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_x"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_y"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (binaryCloud, "normal_z"));
			_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id, viewport);
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
		else if(hasNormals)
		{
			_visualizer->updateColorHandlerIndex(id, hasIntensity?8:7);
		}
		else if(hasIntensity)
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
	return addCloud(id, binaryCloud, pose, true, true, false, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, true, false, false, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, true, true, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, false, true, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, true, false, color);
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose,
		const QColor & color)
{
	pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *binaryCloud);
	return addCloud(id, binaryCloud, pose, false, false, false, color);
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
	if(_visualizer->addPolygonMesh<pcl::PointXYZ>(cloud, polygons, id, 1))
	{
#if VTK_MAJOR_VERSION >= 7
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.1);
#else
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.5);
#endif
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		if(_buildLocator)
		{
			vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
			tree->SetDataSet(_visualizer->getCloudActorMap()->find(id)->second.actor->GetMapper()->GetInput());
			tree->BuildLocator();
			_locators.insert(std::make_pair(id, tree));
		}
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
	if(_visualizer->addPolygonMesh<pcl::PointXYZRGB>(cloud, polygons, id, 1))
	{
#if VTK_MAJOR_VERSION >= 7
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.1);
#else
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.5);
#endif
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		if(_buildLocator)
		{
			vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
			tree->SetDataSet(_visualizer->getCloudActorMap()->find(id)->second.actor->GetMapper()->GetInput());
			tree->BuildLocator();
			_locators.insert(std::make_pair(id, tree));
		}
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
	if(_visualizer->addPolygonMesh<pcl::PointXYZRGBNormal>(cloud, polygons, id, 1))
	{
#if VTK_MAJOR_VERSION >= 7
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.1);
#else
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.5);
#endif
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		if(_buildLocator)
		{
			vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
			tree->SetDataSet(_visualizer->getCloudActorMap()->find(id)->second.actor->GetMapper()->GetInput());
			tree->BuildLocator();
			_locators.insert(std::make_pair(id, tree));
		}
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
	if(_visualizer->addPolygonMesh(*mesh, id, 1))
	{
#if VTK_MAJOR_VERSION >= 7
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.1);
#else
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetAmbient(0.5);
#endif
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
		_visualizer->getCloudActorMap()->find(id)->second.actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		_visualizer->updatePointCloudPose(id, pose.toEigen3f());
		if(_buildLocator)
		{
			vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
			tree->SetDataSet(_visualizer->getCloudActorMap()->find(id)->second.actor->GetMapper()->GetInput());
			tree->BuildLocator();
			_locators.insert(std::make_pair(id, tree));
		}
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
	if(this->addTextureMesh(*textureMesh, texture, id, 1))
	{
#if VTK_MAJOR_VERSION >= 7
		vtkActor* actor = vtkActor::SafeDownCast (_visualizer->getShapeActorMap()->find(id)->second);
#else
		vtkActor* actor = vtkActor::SafeDownCast (_visualizer->getCloudActorMap()->find(id)->second.actor);
#endif
		UASSERT(actor);
		if(!textureMesh->cloud.is_dense)
		{
			actor->GetTexture()->SetInterpolate(1);
			actor->GetTexture()->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_REPLACE);
		}
		if(_buildLocator)
		{
			vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
			tree->SetDataSet(actor->GetMapper()->GetInput());
			tree->BuildLocator();
			_locators.insert(std::make_pair(id, tree));
		}
		_addedClouds.insert(id, Transform::getIdentity());
		this->updateCloudPose(id, pose);
		return true;
	}
	return false;
}

bool CloudViewer::addOctomap(const OctoMap * octomap, unsigned int treeDepth, bool volumeRepresentation)
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

	removeOctomap();

	if(!volumeRepresentation)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap->createCloud(treeDepth, obstacles.get(), 0, 0, false);
		if(obstacles->size())
		{
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
			points->SetNumberOfPoints(obstacles->size());
			double s = octomap->octree()->getNodeSize(treeDepth) / 2.0;
			for (unsigned int i = 0; i < obstacles->size(); i++)
			{
				points->InsertPoint(i,
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

			_visualizer->getRendererCollection()->InitTraversal ();
			vtkRenderer* renderer = NULL;
			renderer = _visualizer->getRendererCollection()->GetNextItem ();
			renderer = _visualizer->getRendererCollection()->GetNextItem ();
			UASSERT(renderer);
			renderer->AddActor(octomapActor);

			_octomapActor = octomapActor.GetPointer();
			return true;
		}
	}
	else
	{
		if(octomap->octree()->size())
		{
			// Create an image data
			vtkSmartPointer<vtkImageData> imageData =
					vtkSmartPointer<vtkImageData>::New();

			double sizeX, sizeY, sizeZ;
			double minX, minY, minZ;
			double maxX, maxY, maxZ;
			octomap->getGridMin(minX, minY, minZ);
			octomap->getGridMax(maxX, maxY, maxZ);
			sizeX = maxX-minX;
			sizeY = maxY-minY;
			sizeZ = maxZ-minZ;
			double cellSize = octomap->octree()->getNodeSize(treeDepth);

			UTimer t;
			// Specify the size of the image data
			imageData->SetExtent(0, int(sizeX/cellSize+0.5), 0, int(sizeY/cellSize+0.5), 0, int(sizeZ/cellSize+0.5)); // 3D image
#if VTK_MAJOR_VERSION <= 5
			imageData->SetNumberOfScalarComponents(4);
			imageData->SetScalarTypeToUnsignedChar();
#else
			imageData->AllocateScalars(VTK_UNSIGNED_CHAR,4);
#endif

			int dims[3];
			imageData->GetDimensions(dims);

			memset(imageData->GetScalarPointer(), 0, imageData->GetScalarSize()*imageData->GetNumberOfScalarComponents()*dims[0]*dims[1]*dims[2]);

			for (RtabmapColorOcTree::iterator it = octomap->octree()->begin(treeDepth); it != octomap->octree()->end(); ++it)
			{
				if(octomap->octree()->isNodeOccupied(*it))
				{
					octomap::point3d pt = octomap->octree()->keyToCoord(it.getKey());
					int x = (pt.x()-minX) / cellSize;
					int y = (pt.y()-minY) / cellSize;
					int z = (pt.z()-minZ) / cellSize;
					if(x>=0 && x<dims[0] && y>=0 && y<dims[1] && z>=0 && z<dims[2])
					{
						unsigned char* pixel = static_cast<unsigned char*>(imageData->GetScalarPointer(x,y,z));
						if(octomap->octree()->getTreeDepth() == it.getDepth() && it->isColorSet())
						{
							pixel[0] = it->getColor().r;
							pixel[1] = it->getColor().g;
							pixel[2] = it->getColor().b;
						}
						else
						{
							// Gradiant color on z axis
							float H = (maxZ - pt.z())*299.0f/(maxZ-minZ);
							float r,g,b;
							util2d::HSVtoRGB(&r, &g, &b, H, 1, 1);
							pixel[0] = r*255.0f;
							pixel[1] = g*255.0f;
							pixel[2] = b*255.0f;
						}
						pixel[3] = 255;
					}
				}
			}
			vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper =
					vtkSmartPointer<vtkSmartVolumeMapper>::New();
			volumeMapper->SetBlendModeToComposite(); // composite first
#if VTK_MAJOR_VERSION <= 5
			volumeMapper->SetInputConnection(imageData->GetProducerPort());
#else
			volumeMapper->SetInputData(imageData);
#endif
			vtkSmartPointer<vtkVolumeProperty> volumeProperty =
					vtkSmartPointer<vtkVolumeProperty>::New();
			volumeProperty->ShadeOff();
			volumeProperty->IndependentComponentsOff();

			vtkSmartPointer<vtkPiecewiseFunction> compositeOpacity =
					vtkSmartPointer<vtkPiecewiseFunction>::New();
			compositeOpacity->AddPoint(0.0,0.0);
			compositeOpacity->AddPoint(255.0,1.0);
			volumeProperty->SetScalarOpacity(0, compositeOpacity); // composite first.

			vtkSmartPointer<vtkVolume> volume =
					vtkSmartPointer<vtkVolume>::New();
			volume->SetMapper(volumeMapper);
			volume->SetProperty(volumeProperty);
			volume->SetScale(cellSize);
			volume->SetPosition(minX, minY, minZ);

			_visualizer->getRendererCollection()->InitTraversal ();
			vtkRenderer* renderer = NULL;
			renderer = _visualizer->getRendererCollection()->GetNextItem ();
			renderer = _visualizer->getRendererCollection()->GetNextItem ();
			UASSERT(renderer);
			renderer->AddViewProp(volume);

			// 3D texture mode. For coverage.
#if !defined(VTK_LEGACY_REMOVE) && !defined(VTK_OPENGL2) && VTK_MAJOR_VERSION < 9
			volumeMapper->SetRequestedRenderModeToRayCastAndTexture();
#endif // VTK_LEGACY_REMOVE

			// Software mode, for coverage. It also makes sure we will get the same
			// regression image on all platforms.
			volumeMapper->SetRequestedRenderModeToRayCast();
			_octomapActor = volume.GetPointer();

			return true;
		}
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
		_visualizer->getRendererCollection()->InitTraversal ();
		vtkRenderer* renderer = NULL;
		renderer = _visualizer->getRendererCollection()->GetNextItem ();
		renderer = _visualizer->getRendererCollection()->GetNextItem ();
		UASSERT(renderer);
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

#if VTK_MAJOR_VERSION >= 7
  pcl::visualization::ShapeActorMap::iterator am_it = _visualizer->getShapeActorMap()->find (id);
  if (am_it != _visualizer->getShapeActorMap()->end ())
#else
  pcl::visualization::CloudActorMap::iterator am_it = _visualizer->getCloudActorMap()->find (id);
  if (am_it != _visualizer->getCloudActorMap()->end ())
#endif
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

#if VTK_MAJOR_VERSION >= 7
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New ();
#else
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
#endif
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
#if VTK_MAJOR_VERSION >= 7
  (*_visualizer->getShapeActorMap())[id] = actor;
#else
  (*_visualizer->getCloudActorMap())[id].actor = actor;
   // Save the viewpoint transformation matrix to the global actor map
  (*_visualizer->getCloudActorMap())[id].viewpoint_transformation_ = transformation;
#endif

#if VTK_MAJOR_VERSION >= 7
  actor->GetProperty()->SetAmbient(0.1);
#else
  actor->GetProperty()->SetAmbient(0.5);
#endif
  actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
  actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG);
  actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
  actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
  actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
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
#if VTK_MAJOR_VERSION >= 7
	if(_visualizer->getShapeActorMap()->find("map") != _visualizer->getShapeActorMap()->end())
	{
		_visualizer->removeShape("map");
	}
#else
	if(_visualizer->getCloudActorMap()->find("map") != _visualizer->getCloudActorMap()->end())
	{
		_visualizer->removePointCloud("map");
	}
#endif

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

		this->addTextureMesh(*mesh, map8U, "map", 1);
		setCloudOpacity("map", opacity);
	}
	return true;
}

void CloudViewer::removeOccupancyGridMap()
{
#if VTK_MAJOR_VERSION >= 7
	if(_visualizer->getShapeActorMap()->find("map") != _visualizer->getShapeActorMap()->end())
	{
		_visualizer->removeShape("map");
	}
#else
	if(_visualizer->getCloudActorMap()->find("map") != _visualizer->getCloudActorMap()->end())
	{
		_visualizer->removePointCloud("map");
	}
#endif
}

void CloudViewer::addOrUpdateCoordinate(
			const std::string & id,
			const Transform & transform,
			double scale,
			bool foreground)
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
		_visualizer->addCoordinateSystem(scale*_coordinateFrameScale, transform.toEigen3f(), id, foreground?3:2);
#else
		// Well, on older versions, just update the main coordinate
		_visualizer->addCoordinateSystem(scale*_coordinateFrameScale, transform.toEigen3f(), 0);
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

void CloudViewer::removeAllCoordinates(const std::string & prefix)
{
	std::set<std::string> coordinates = _coordinates;
	for(std::set<std::string>::iterator iter = coordinates.begin(); iter!=coordinates.end(); ++iter)
	{
		if(prefix.empty() || iter->find(prefix) != std::string::npos)
		{
			this->removeCoordinate(*iter);
		}
	}
	UASSERT(!prefix.empty() || _coordinates.empty());
}

void CloudViewer::addOrUpdateLine(
			const std::string & id,
			const Transform & from,
			const Transform & to,
			const QColor & color,
			bool arrow,
			bool foreground)
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
			_visualizer->addArrow(pt2, pt1, c.redF(), c.greenF(), c.blueF(), false, id, foreground?3:2);
		}
		else
		{
			_visualizer->addLine(pt2, pt1, c.redF(), c.greenF(), c.blueF(), id, foreground?3:2);
		}
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, c.alphaF(), id);
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

void CloudViewer::addOrUpdateSphere(
			const std::string & id,
			const Transform & pose,
			float radius,
			const QColor & color,
			bool foreground)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeSphere(id);

	if(!pose.isNull())
	{
		_spheres.insert(id);

		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}

		pcl::PointXYZ center(pose.x(), pose.y(), pose.z());
		_visualizer->addSphere(center, radius, c.redF(), c.greenF(), c.blueF(), id, foreground?3:2);
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, c.alphaF(), id);
	}
}

void CloudViewer::removeSphere(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_spheres.find(id) != _spheres.end())
	{
		_visualizer->removeShape(id);
		_spheres.erase(id);
	}
}

void CloudViewer::removeAllSpheres()
{
	std::set<std::string> spheres = _spheres;
	for(std::set<std::string>::iterator iter = spheres.begin(); iter!=spheres.end(); ++iter)
	{
		this->removeSphere(*iter);
	}
	UASSERT(_spheres.empty());
}

void CloudViewer::addOrUpdateCube(
			const std::string & id,
			const Transform & pose,
			float width,
			float height,
			float depth,
			const QColor & color,
			bool wireframe,
			bool foreground)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeCube(id);

	if(!pose.isNull())
	{
		_cubes.insert(id);

		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}
		_visualizer->addCube(Eigen::Vector3f(pose.x(), pose.y(), pose.z()), pose.getQuaternionf(), width, height, depth, id, foreground?3:2);
		if(wireframe)
		{
			_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
		}
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c.redF(), c.greenF(), c.blueF(), id);
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, c.alphaF(), id);
	}
}

void CloudViewer::removeCube(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_cubes.find(id) != _cubes.end())
	{
		_visualizer->removeShape(id);
		_cubes.erase(id);
	}
}

void CloudViewer::removeAllCubes()
{
	std::set<std::string> cubes = _cubes;
	for(std::set<std::string>::iterator iter = cubes.begin(); iter!=cubes.end(); ++iter)
	{
		this->removeCube(*iter);
	}
	UASSERT(_cubes.empty());
}

void CloudViewer::addOrUpdateQuad(
		const std::string & id,
		const Transform & pose,
		float width,
		float height,
		const QColor & color,
		bool foreground)
{
	addOrUpdateQuad(id, pose, width/2.0f, width/2.0f, height/2.0f, height/2.0f, color, foreground);
}

void CloudViewer::addOrUpdateQuad(
		const std::string & id,
		const Transform & pose,
		float widthLeft,
		float widthRight,
		float heightBottom,
		float heightTop,
		const QColor & color,
		bool foreground)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeQuad(id);

	if(!pose.isNull())
	{
		_quads.insert(id);

		QColor c = Qt::gray;
		if(color.isValid())
		{
			c = color;
		}

		// Create four points (must be in counter clockwise order)
		double p0[3] = {0.0, -widthLeft, heightTop};
		double p1[3] = {0.0, -widthLeft, -heightBottom};
		double p2[3] = {0.0, widthRight, -heightBottom};
		double p3[3] = {0.0, widthRight, heightTop};

		// Add the points to a vtkPoints object
		vtkSmartPointer<vtkPoints> points =
				vtkSmartPointer<vtkPoints>::New();
		points->InsertNextPoint(p0);
		points->InsertNextPoint(p1);
		points->InsertNextPoint(p2);
		points->InsertNextPoint(p3);

		// Create a quad on the four points
		vtkSmartPointer<vtkQuad> quad =
				vtkSmartPointer<vtkQuad>::New();
		quad->GetPointIds()->SetId(0,0);
		quad->GetPointIds()->SetId(1,1);
		quad->GetPointIds()->SetId(2,2);
		quad->GetPointIds()->SetId(3,3);

		// Create a cell array to store the quad in
		vtkSmartPointer<vtkCellArray> quads =
				vtkSmartPointer<vtkCellArray>::New();
		quads->InsertNextCell(quad);

		// Create a polydata to store everything in
		vtkSmartPointer<vtkPolyData> polydata =
				vtkSmartPointer<vtkPolyData>::New();

		// Add the points and quads to the dataset
		polydata->SetPoints(points);
		polydata->SetPolys(quads);

		// Setup actor and mapper
		vtkSmartPointer<vtkPolyDataMapper> mapper =
				vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
		mapper->SetInput(polydata);
#else
		mapper->SetInputData(polydata);
#endif

		vtkSmartPointer<vtkLODActor> actor =
				vtkSmartPointer<vtkLODActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(c.redF(), c.greenF(), c.blueF());

		//_visualizer->addActorToRenderer (actor, viewport);
		// Add it to all renderers
		_visualizer->getRendererCollection()->InitTraversal ();
		vtkRenderer* renderer = NULL;
		int i = 0;
		while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
		{
			if ((foreground?3:2) == i)               // add the actor only to the specified viewport
			{
				renderer->AddActor (actor);
			}
			++i;
		}

		// Save the pointer/ID pair to the global actor map
		(*_visualizer->getCloudActorMap())[id].actor = actor;

		// Save the viewpoint transformation matrix to the global actor map
		vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
		pcl::visualization::PCLVisualizer::convertToVtkMatrix (pose.toEigen3f().matrix (), transformation);
		(*_visualizer->getCloudActorMap())[id].viewpoint_transformation_ = transformation;
		(*_visualizer->getCloudActorMap())[id].actor->SetUserMatrix (transformation);
		(*_visualizer->getCloudActorMap())[id].actor->Modified ();

		(*_visualizer->getCloudActorMap())[id].actor->GetProperty()->SetLighting(false);
		_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, c.alphaF(), id);
	}
}

void CloudViewer::removeQuad(const std::string & id)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	if(_quads.find(id) != _quads.end())
	{
		_visualizer->removeShape(id);
		_quads.erase(id);
	}
}

void CloudViewer::removeAllQuads()
{
	std::set<std::string> quads = _quads;
	for(std::set<std::string>::iterator iter = quads.begin(); iter!=quads.end(); ++iter)
	{
		this->removeQuad(*iter);
	}
	UASSERT(_quads.empty());
}

static const float frustum_vertices[] = {
    0.0f,  0.0f, 0.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, -1.0f, 1.0f,
	-1.0f, -1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f};

static const int frustum_indices[] = {
    1, 2, 3, 4, 1, 0, 2, 0, 3, 0, 4};

void CloudViewer::addOrUpdateFrustum(
			const std::string & id,
			const Transform & pose,
			const Transform & localTransform,
			double scale,
			const QColor & color,
			float fovX,
			float fovY)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
	this->removeFrustum(id);
#endif

	if(!pose.isNull())
	{
		if(_frustums.find(id)==_frustums.end())
		{
			_frustums.insert(id, Transform());

			int frustumSize = sizeof(frustum_vertices)/sizeof(float);
			UASSERT(frustumSize>0 && frustumSize % 3 == 0);
			frustumSize/=3;
			pcl::PointCloud<pcl::PointXYZ> frustumPoints;
			frustumPoints.resize(frustumSize);
			float scaleX = tan((fovX>0?fovX:1.1)/2.0f) * scale;
			float scaleY = tan((fovY>0?fovY:0.85)/2.0f) * scale;
			float scaleZ = scale;
			QColor c = Qt::gray;
			if(color.isValid())
			{
				c = color;
			}
			Transform opticalRotInv(0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0);

#if PCL_VERSION_COMPARE(<, 1, 7, 2)
			Eigen::Affine3f t = (pose*localTransform).toEigen3f();
#else
			Eigen::Affine3f t = (localTransform).toEigen3f();
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
			_visualizer->addPolylineFromPolygonMesh(mesh, id, 2);
			_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c.redF(), c.greenF(), c.blueF(), id);
			_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, c.alphaF(), id);
		}
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		if(!this->updateFrustumPose(id, pose))
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
		_visualizer->addPolylineFromPolygonMesh(mesh, id, 2);
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.redF(), color.greenF(), color.blueF(), id);
		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, color.alphaF(), id);

		pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
		pcl::toPCLPointCloud2(*graph, *binaryCloud);
		this->addCloud(id+"_nodes", binaryCloud, Transform::getIdentity(), false, false, false, color, 2);
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
			const QColor & color,
			bool foreground)
{
	if(id.empty())
	{
		UERROR("id should not be empty!");
		return;
	}

	removeText(id);

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
				id,
				foreground?3:2);
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
	this->refreshView();
}

bool CloudViewer::isCameraAxisShown() const
{
	return _aShowCameraAxis->isChecked();
}

void CloudViewer::setCameraAxisShown(bool shown)
{
	if(!shown)
	{
		this->removeCoordinate("reference");
	}
	else
	{
		this->addOrUpdateCoordinate("reference", Transform::getIdentity(), 0.2);
	}
	this->refreshView();
	_aShowCameraAxis->setChecked(shown);
}

double CloudViewer::getCoordinateFrameScale() const
{
	return _coordinateFrameScale;
}

void CloudViewer::setCoordinateFrameScale(double scale)
{
	_coordinateFrameScale = std::max(0.1, scale);
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
		this->refreshView();
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
	this->refreshView();
	_frustumColor = value;
}

void CloudViewer::resetCamera()
{
	_lastCameraOrientation= _lastCameraPose = cv::Vec3f(0,0,0);
	if((_aFollowCamera->isChecked() || _aLockCamera->isChecked()) && !_lastPose.isNull())
	{
		// reset relative to last current pose
		cv::Point3f pt = util3d::transformPoint(cv::Point3f(_lastPose.x(), _lastPose.y(), _lastPose.z()), ( _lastPose.rotation()*Transform(-1, 0, 0)).translation());
		if(_aCameraOrtho->isChecked())
		{
			this->setCameraPosition(
					_lastPose.x(), _lastPose.y(), _lastPose.z()+5,
					_lastPose.x(), _lastPose.y(), _lastPose.z(),
					1, 0, 0);
		}
		else if(_aLockViewZ->isChecked())
		{
			this->setCameraPosition(
					pt.x, pt.y, pt.z,
					_lastPose.x(), _lastPose.y(), _lastPose.z(),
					0, 0, 1);
		}
		else
		{
			this->setCameraPosition(
					pt.x, pt.y, pt.z,
					_lastPose.x(), _lastPose.y(), _lastPose.z(),
					_lastPose.r31(), _lastPose.r32(), _lastPose.r33());
		}
	}
	else if(_aCameraOrtho->isChecked())
	{
		this->setCameraPosition(
				0, 0, 5,
				0, 0, 0,
				1, 0, 0);
	}
	else
	{
		this->setCameraPosition(
				-1, 0, 0,
				0, 0, 0,
				0, 0, 1);
	}
}

void CloudViewer::removeAllClouds()
{
	QMap<std::string, Transform> addedClouds = _addedClouds;
	QList<std::string> ids = _addedClouds.keys();
	for(QList<std::string>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
	{
		removeCloud(*iter);
	}
	UASSERT(_addedClouds.empty());
	UASSERT(_locators.empty());
}


bool CloudViewer::removeCloud(const std::string & id)
{
	bool success = _visualizer->removePointCloud(id);
#if VTK_MAJOR_VERSION >= 7
	if(!success)
	{
		success = _visualizer->removeShape(id);
	}
#endif
	_visualizer->removePointCloud(id+"-normals");
	_addedClouds.remove(id); // remove after visualizer
	_addedClouds.remove(id+"-normals");
	_locators.erase(id);
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

std::string CloudViewer::getIdByActor(vtkProp * actor) const
{
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		if(iter->second.actor.GetPointer() == actor)
		{
			return iter->first;
		}
	}

#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	// getShapeActorMap() not available in version < 1.7.2
	pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
	for(pcl::visualization::ShapeActorMap::iterator iter=shapeActorMap->begin(); iter!=shapeActorMap->end(); ++iter)
	{
		if(iter->second.GetPointer() == actor)
		{
			std::string id = iter->first;
			while(id.size() && id.at(id.size()-1) == '*')
			{
				id.erase(id.size()-1);
			}

			return id;
		}
	}
#endif
	return std::string();
}

QColor CloudViewer::getColor(const std::string & id)
{
	QColor color;
	pcl::visualization::CloudActorMap::iterator iter = _visualizer->getCloudActorMap()->find(id);
	if(iter != _visualizer->getCloudActorMap()->end())
	{
		double r,g,b,a;
		iter->second.actor->GetProperty()->GetColor(r,g,b);
		a = iter->second.actor->GetProperty()->GetOpacity();
		color.setRgbF(r, g, b, a);
	}
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	// getShapeActorMap() not available in version < 1.7.2
	else
	{
		std::string idLayer1 = id+"*";
		std::string idLayer2 = id+"**";
		pcl::visualization::ShapeActorMap::iterator iter = _visualizer->getShapeActorMap()->find(id);
		if(iter == _visualizer->getShapeActorMap()->end())
		{
			iter = _visualizer->getShapeActorMap()->find(idLayer1);
			if(iter == _visualizer->getShapeActorMap()->end())
			{
				iter = _visualizer->getShapeActorMap()->find(idLayer2);
			}
		}
		if(iter != _visualizer->getShapeActorMap()->end())
		{
			vtkActor * actor = vtkActor::SafeDownCast(iter->second);
			if(actor)
			{
				double r,g,b,a;
				actor->GetProperty()->GetColor(r,g,b);
				a = actor->GetProperty()->GetOpacity();
				color.setRgbF(r, g, b, a);
			}
		}
	}
#endif
	return color;
}

void CloudViewer::setColor(const std::string & id, const QColor & color)
{
	pcl::visualization::CloudActorMap::iterator iter = _visualizer->getCloudActorMap()->find(id);
	if(iter != _visualizer->getCloudActorMap()->end())
	{
		iter->second.actor->GetProperty()->SetColor(color.redF(),color.greenF(),color.blueF());
		iter->second.actor->GetProperty()->SetOpacity(color.alphaF());
	}
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	// getShapeActorMap() not available in version < 1.7.2
	else
	{
		std::string idLayer1 = id+"*";
		std::string idLayer2 = id+"**";
		pcl::visualization::ShapeActorMap::iterator iter = _visualizer->getShapeActorMap()->find(id);
		if(iter == _visualizer->getShapeActorMap()->end())
		{
			iter = _visualizer->getShapeActorMap()->find(idLayer1);
			if(iter == _visualizer->getShapeActorMap()->end())
			{
				iter = _visualizer->getShapeActorMap()->find(idLayer2);
			}
		}
		if(iter != _visualizer->getShapeActorMap()->end())
		{
			vtkActor * actor = vtkActor::SafeDownCast(iter->second);
			if(actor)
			{
				actor->GetProperty()->SetColor(color.redF(),color.greenF(),color.blueF());
				actor->GetProperty()->SetOpacity(color.alphaF());
			}
		}
	}
#endif
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
#if VTK_MAJOR_VERSION >= 7
	pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
	for(pcl::visualization::ShapeActorMap::iterator iter=shapeActorMap->begin(); iter!=shapeActorMap->end(); ++iter)
	{
		vtkActor* actor = vtkActor::SafeDownCast (iter->second);
		if(actor)
		{
			actor->GetProperty()->SetBackfaceCulling(_aBackfaceCulling->isChecked());
			actor->GetProperty()->SetFrontfaceCulling(_frontfaceCulling);
		}
	}
#endif
	this->refreshView();
}

void CloudViewer::setPolygonPicking(bool enabled)
{
	_aPolygonPicking->setChecked(enabled);

	if(!_aPolygonPicking->isChecked())
	{
		vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
		pp->SetTolerance (pp->GetTolerance());
		#if VTK_MAJOR_VERSION > 8
		this->interactor()->SetPicker (pp);
#else
		this->GetInteractor()->SetPicker (pp);
#endif
		setMouseTracking(false);
	}
	else
	{
		vtkSmartPointer<CloudViewerCellPicker> pp = vtkSmartPointer<CloudViewerCellPicker>::New ();
		pp->SetTolerance (pp->GetTolerance());
#if VTK_MAJOR_VERSION > 8
		this->interactor()->SetPicker (pp);
#else
		this->GetInteractor()->SetPicker (pp);
#endif
		setMouseTracking(true);
	}
}



void CloudViewer::setRenderingRate(double rate)
{
	_renderingRate = rate;
	_visualizer->getInteractorStyle()->GetInteractor()->SetDesiredUpdateRate(_renderingRate);
}

void CloudViewer::setEDLShading(bool on)
{
#if VTK_MAJOR_VERSION >= 7
	_aSetEDLShading->setChecked(on);
	_visualizer->getRendererCollection()->InitTraversal ();
	vtkRenderer* renderer = NULL;
	renderer = _visualizer->getRendererCollection()->GetNextItem ();
	renderer = _visualizer->getRendererCollection()->GetNextItem (); // Get Layer 1
	UASSERT(renderer);

	vtkOpenGLRenderer* glrenderer = vtkOpenGLRenderer::SafeDownCast(renderer);
	UASSERT(glrenderer);
	if(on)
	{
		// EDL shader
		vtkSmartPointer<vtkRenderStepsPass> basicPasses = vtkSmartPointer<vtkRenderStepsPass>::New ();
		vtkSmartPointer<vtkEDLShading> edl = vtkSmartPointer<vtkEDLShading>::New ();
		edl->SetDelegatePass(basicPasses);
		glrenderer->SetPass(edl);
	}
	else if(glrenderer->GetPass())
	{
		glrenderer->GetPass()->ReleaseGraphicsResources(NULL);
		glrenderer->SetPass(NULL);
	}

	this->refreshView();
#else
	if(on)
	{
		UERROR("RTAB-Map must be built with VTK>=7 to enable EDL shading!");
	}
#endif
}

void CloudViewer::setLighting(bool on)
{
	_aSetLighting->setChecked(on);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
	}
#if VTK_MAJOR_VERSION >= 7
	pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
	for(pcl::visualization::ShapeActorMap::iterator iter=shapeActorMap->begin(); iter!=shapeActorMap->end(); ++iter)
	{
		vtkActor* actor = vtkActor::SafeDownCast (iter->second);
		if(actor && _addedClouds.contains(iter->first))
		{
			actor->GetProperty()->SetLighting(_aSetLighting->isChecked());
		}
	}
#endif
	this->refreshView();
}

void CloudViewer::setShading(bool on)
{
	_aSetFlatShading->setChecked(on);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG); // VTK_FLAT - VTK_GOURAUD - VTK_PHONG
	}
#if VTK_MAJOR_VERSION >= 7
	pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
	for(pcl::visualization::ShapeActorMap::iterator iter=shapeActorMap->begin(); iter!=shapeActorMap->end(); ++iter)
	{
		vtkActor* actor = vtkActor::SafeDownCast (iter->second);
		if(actor && _addedClouds.contains(iter->first))
		{
			actor->GetProperty()->SetInterpolation(_aSetFlatShading->isChecked()?VTK_FLAT:VTK_PHONG); // VTK_FLAT - VTK_GOURAUD - VTK_PHONG
		}
	}
#endif
	this->refreshView();
}

void CloudViewer::setEdgeVisibility(bool visible)
{
	_aSetEdgeVisibility->setChecked(visible);
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	for(pcl::visualization::CloudActorMap::iterator iter=cloudActorMap->begin(); iter!=cloudActorMap->end(); ++iter)
	{
		iter->second.actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
	}
#if VTK_MAJOR_VERSION >= 7
	pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
	for(pcl::visualization::ShapeActorMap::iterator iter=shapeActorMap->begin(); iter!=shapeActorMap->end(); ++iter)
	{
		vtkActor* actor = vtkActor::SafeDownCast (iter->second);
		if(actor && _addedClouds.contains(iter->first))
		{
			actor->GetProperty()->SetEdgeVisibility(_aSetEdgeVisibility->isChecked());
		}
	}
#endif
	this->refreshView();
}

void CloudViewer::setInteractorLayer(int layer)
{
	_visualizer->getRendererCollection()->InitTraversal ();
	vtkRenderer* renderer = NULL;
	int i =0;
	while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
	{
		if(i==layer)
		{
			_visualizer->getInteractorStyle()->SetDefaultRenderer(renderer);
			_visualizer->getInteractorStyle()->SetCurrentRenderer(renderer);
			return;
		}
		++i;
	}
	UWARN("Could not set layer %d to interactor (layers=%d).", layer, _visualizer->getRendererCollection()->GetNumberOfItems());
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
	vtkRenderer* renderer = NULL;
	double boundingBox[6] = {1, -1, 1, -1, 1, -1};

	// compute global bounding box
	_visualizer->getRendererCollection()->InitTraversal ();
	while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
	{
		vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
		cam->SetPosition (x, y, z);
		cam->SetFocalPoint (focalX, focalY, focalZ);
		cam->SetViewUp (upX, upY, upZ);

		double BB[6];
		renderer->ComputeVisiblePropBounds(BB);
		for (int i = 0; i < 6; i++) {
			if (i % 2 == 0) {
				// Even Index is Min
				if (BB[i] < boundingBox[i]) {
					boundingBox[i] = BB[i];
				}
			} else {
				// Odd Index is Max
				if (BB[i] > boundingBox[i]) {
					boundingBox[i] = BB[i];
				}
			}
		}
	}

	_visualizer->getRendererCollection()->InitTraversal ();
	while ((renderer = _visualizer->getRendererCollection()->GetNextItem ()) != NULL)
	{
		renderer->ResetCameraClippingRange(boundingBox);
	}

	_visualizer->getRenderWindow()->Render ();
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
			_visualizer->addPolylineFromPolygonMesh(mesh, "trajectory", 2);
		}

		if(pose != _lastPose || _lastPose.isNull())
		{
			if(_lastPose.isNull())
			{
				_lastPose.setIdentity();
			}

			std::vector<pcl::visualization::Camera> cameras;
			_visualizer->getCameras(cameras);

			if(_aLockCamera->isChecked() || _aCameraOrtho->isChecked())
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

				PR.normalizeRotation();

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

			if(_aShowCameraAxis->isChecked())
			{
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
			}

			this->setCameraPosition(
				cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
				cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
				cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
		}
	}

	_lastPose = pose;
}

void CloudViewer::updateCameraFrustum(const Transform & pose, const StereoCameraModel & model)
{
	std::vector<CameraModel> models;
	models.push_back(model.left());
	CameraModel right = model.right();
	if(!model.left().localTransform().isNull())
	{
		right.setLocalTransform(model.left().localTransform() * Transform(model.baseline(), 0, 0, 0, 0, 0));
	}
	models.push_back(right);
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
				this->addOrUpdateFrustum(id, pose, baseToCamera, _frustumScale, _frustumColor, models[i].fovX(), models[i].fovY());
				if(!baseToCamera.isIdentity())
				{
					this->addOrUpdateLine(uFormat("reference_frustum_line_%d", i), pose, pose * baseToCamera, _frustumColor);
				}
			}
		}
	}
}
void CloudViewer::updateCameraFrustums(const Transform & pose, const std::vector<StereoCameraModel> & stereoModels)
{
	std::vector<CameraModel> models;
	for(size_t i=0; i<stereoModels.size(); ++i)
	{
		models.push_back(stereoModels[i].left());
		CameraModel right = stereoModels[i].right();
		if(!stereoModels[i].left().localTransform().isNull())
		{
			right.setLocalTransform(stereoModels[i].left().localTransform() * Transform(stereoModels[i].baseline(), 0, 0, 0, 0, 0));
		}
		models.push_back(right);
		updateCameraFrustums(pose, models);
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
#if VTK_MAJOR_VERSION >= 7
		pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
		pcl::visualization::ShapeActorMap::iterator iter = shapeActorMap->find(id);
		if(iter != shapeActorMap->end())
		{
			vtkActor* actor = vtkActor::SafeDownCast (iter->second);
			if(actor)
			{
				actor->SetVisibility(isVisible?1:0);
				return;
			}
		}
#endif
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
#if VTK_MAJOR_VERSION >= 7
		pcl::visualization::ShapeActorMapPtr shapeActorMap = _visualizer->getShapeActorMap();
		pcl::visualization::ShapeActorMap::iterator iter = shapeActorMap->find(id);
		if(iter != shapeActorMap->end())
		{
			vtkActor* actor = vtkActor::SafeDownCast (iter->second);
			if(actor)
			{
				return actor->GetVisibility() != 0;
			}
		}
#endif
		UERROR("Cannot find actor named \"%s\".", id.c_str());
	}
	return false;
}

void CloudViewer::setCloudColorIndex(const std::string & id, int index)
{
	if(index>0)
	{
		_visualizer->updateColorHandlerIndex(id, index-1);
	}
}

void CloudViewer::setCloudOpacity(const std::string & id, double opacity)
{
	double lastOpacity;
	if(_visualizer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, lastOpacity, id))
	{
		if(lastOpacity != opacity)
		{
			_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
		}
	}
#if VTK_MAJOR_VERSION >= 7
	else
	{
		pcl::visualization::ShapeActorMap::iterator am_it = _visualizer->getShapeActorMap()->find (id);
		if (am_it != _visualizer->getShapeActorMap()->end ())
		{
			vtkActor* actor = vtkActor::SafeDownCast (am_it->second);
			if(actor)
			{
				actor->GetProperty ()->SetOpacity (opacity);
				actor->Modified ();
			}
		}
	}
#endif
}

void CloudViewer::setCloudPointSize(const std::string & id, int size)
{
	double lastSize;
	if(_visualizer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, lastSize, id))
	{
		if((int)lastSize != size)
		{
			_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, (double)size, id);
		}
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
void CloudViewer::setCameraOrtho(bool enabled)
{
	_lastCameraOrientation= _lastCameraPose = cv::Vec3f(0,0,0);
#if VTK_MAJOR_VERSION > 8
	CloudViewerInteractorStyle * interactor = CloudViewerInteractorStyle::SafeDownCast(this->interactor()->GetInteractorStyle());
#else
	CloudViewerInteractorStyle * interactor = CloudViewerInteractorStyle::SafeDownCast(this->GetInteractor()->GetInteractorStyle());
#endif
	if(interactor)
	{
		interactor->setOrthoMode(enabled);
		this->refreshView();
	}
	_aCameraOrtho->setChecked(enabled);
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
bool CloudViewer::isCameraOrtho() const
{
	return _aCameraOrtho->isChecked();
}
bool CloudViewer::isBackfaceCulling() const
{
	return _aBackfaceCulling->isChecked();
}
bool CloudViewer::isFrontfaceCulling() const
{
	return _frontfaceCulling;
}
bool CloudViewer::isPolygonPicking() const
{
	return _aPolygonPicking->isChecked();
}
bool CloudViewer::isEDLShadingOn() const
{
	return _aSetEDLShading->isChecked();
}
bool CloudViewer::isLightingOn() const
{
	return _aSetLighting->isChecked();
}
bool CloudViewer::isShadingOn() const
{
	return _aSetFlatShading->isChecked();
}
bool CloudViewer::isEdgeVisible() const
{
	return _aSetEdgeVisibility->isChecked();
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
			_visualizer->addLine(
					pcl::PointXYZ(i, min, 0.0f),
					pcl::PointXYZ(i, max, 0.0f),
					r, g, b, name, 2);
			_gridLines.push_back(name);
			//over y or z
			name = uFormat("line%d", ++id);
			_visualizer->addLine(
					pcl::PointXYZ(min, i, 0),
					pcl::PointXYZ(max, i, 0),
					r, g, b, name, 2);
			_gridLines.push_back(name);
		}
		// this will update clipping planes
		std::vector<pcl::visualization::Camera> cameras;
		_visualizer->getCameras(cameras);
		this->setCameraPosition(
				cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
				cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
				cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
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

bool CloudViewer::isIntensityRedColormap() const
{
	return _aSetIntensityRedColormap->isChecked();
}
bool CloudViewer::isIntensityRainbowColormap() const
{
	return _aSetIntensityRainbowColormap->isChecked();
}
float CloudViewer::getIntensityMax() const
{
	return _intensityAbsMax;
}

void CloudViewer::setIntensityRedColormap(bool on)
{
	_aSetIntensityRedColormap->setChecked(on);
	if(on)
	{
		_aSetIntensityRainbowColormap->setChecked(false);
	}
}
void CloudViewer::setIntensityRainbowColormap(bool on)
{
	_aSetIntensityRainbowColormap->setChecked(on);
	if(on)
	{
		_aSetIntensityRedColormap->setChecked(false);
	}
}
void CloudViewer::setIntensityMax(float value)
{
	if(value >= 0.0f)
	{
		_intensityAbsMax = value;
	}
	else
	{
		UERROR("Cannot set normals scale < 0, value=%f", value);
	}
}

void CloudViewer::buildPickingLocator(bool enable)
{
	_buildLocator = enable;
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
		PCLQVTKWidget::keyPressEvent(event);
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
		this->setCameraPosition(
			cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
			cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
			cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

		Q_EMIT configChanged();
	}
	else
	{
		PCLQVTKWidget::keyPressEvent(event);
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
		PCLQVTKWidget::mousePressEvent(event);
	}
}

void CloudViewer::mouseMoveEvent(QMouseEvent * event)
{
	PCLQVTKWidget::mouseMoveEvent(event);

	std::vector<pcl::visualization::Camera> cameras;
	_visualizer->getCameras(cameras);

	// camera view up z locked?
	if(_aLockViewZ->isChecked() && !_aCameraOrtho->isChecked())
	{
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
		else
		{
			if(cameras.front().view[2] == 0)
			{
				cameras.front().pos[0] -= 0.00001*cameras.front().view[0];
				cameras.front().pos[1] -= 0.00001*cameras.front().view[1];
			}
			else
			{
				cameras.front().pos[0] -= 0.00001;
			}
		}
		cameras.front().view[0] = 0;
		cameras.front().view[1] = 0;
		cameras.front().view[2] = 1;
	}

	this->setCameraPosition(
			cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
			cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
			cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

	Q_EMIT configChanged();
}

void CloudViewer::wheelEvent(QWheelEvent * event)
{
	PCLQVTKWidget::wheelEvent(event);

	std::vector<pcl::visualization::Camera> cameras;
	_visualizer->getCameras(cameras);

	if(_aLockViewZ->isChecked() && !_aCameraOrtho->isChecked())
	{
		_lastCameraPose = cv::Vec3d(cameras.front().pos);
	}

	this->setCameraPosition(
		cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
		cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
		cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

	Q_EMIT configChanged();
}

void CloudViewer::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * a = _menu->exec(event->globalPos());
	if(a)
	{
		handleAction(a);
		Q_EMIT configChanged();
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
	else if(a == _aShowCameraAxis)
	{
		this->setCameraAxisShown(a->isChecked());
	}
	else if(a == _aSetFrameScale)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set frame scale"), tr("Scale"), _coordinateFrameScale, 0.1, 999.0, 1, &ok);
		if(ok)
		{
			this->setCoordinateFrameScale(value);
		}
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

		this->refreshView();
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
		double value = QInputDialog::getDouble(this, tr("Set grid cell size"), tr("Size (m)"), _gridCellSize, 0.01, 1000, 2, &ok);
		if(ok)
		{
			this->setGridCellSize(value);
		}
	}
	else if(a == _aShowNormals)
	{
		this->setNormalsShown(_aShowNormals->isChecked());
		this->refreshView();
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
	else if(a == _aSetIntensityMaximum)
	{
		bool ok;
		double value = QInputDialog::getDouble(this, tr("Set maximum absolute intensity"), tr("Intensity (0=auto)"), _intensityAbsMax, 0.0, 99999, 2, &ok);
		if(ok)
		{
			this->setIntensityMax(value);
		}
	}
	else if(a == _aSetIntensityRedColormap)
	{
		this->setIntensityRedColormap(_aSetIntensityRedColormap->isChecked());
	}
	else if(a == _aSetIntensityRainbowColormap)
	{
		this->setIntensityRainbowColormap(_aSetIntensityRainbowColormap->isChecked());
	}
	else if(a == _aSetBackgroundColor)
	{
		QColor color = this->getDefaultBackgroundColor();
		color = QColorDialog::getColor(color, this);
		if(color.isValid())
		{
			this->setDefaultBackgroundColor(color);
			this->refreshView();
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
			this->refreshView();
		}
	}
	else if(a == _aCameraOrtho)
	{
		this->setCameraOrtho(_aCameraOrtho->isChecked());
	}
	else if(a == _aSetEDLShading)
	{
		this->setEDLShading(_aSetEDLShading->isChecked());
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
	else if(a == _aPolygonPicking)
	{
		this->setPolygonPicking(_aPolygonPicking->isChecked());
	}
}

} /* namespace rtabmap */
