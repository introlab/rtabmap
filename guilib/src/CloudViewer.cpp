/*
 * CloudViewer.cpp
 *
 *  Created on: 2013-10-13
 *      Author: Mathieu
 */

#include "rtabmap/gui/CloudViewer.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/frustum_culling.h>
#include <QtGui/QMenu>
#include <QtGui/QAction>
#include <QtGui/QContextMenuEvent>
#include <QtGui/QInputDialog>
#include <QtGui/QWheelEvent>
#include <QtGui/QKeyEvent>
#include <QtGui/QColorDialog>
#include <set>

#include <vtkRenderWindow.h>

namespace rtabmap {

void CloudViewer::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	if (event.getButton () == pcl::visualization::MouseEvent::LeftButton ||
	  event.getButton () == pcl::visualization::MouseEvent::MiddleButton)
	{
		this->render(); // this will apply frustum
	}
}

CloudViewer::CloudViewer(QWidget *parent) :
		QVTKWidget(parent),
#if defined(WIN32) || defined(__APPLE__)
		_visualizer(new pcl::visualization::PCLVisualizer("PCLVisualizer")),
#else
		_visualizer(new pcl::visualization::PCLVisualizer("PCLVisualizer", false)),
#endif
		_aLockCamera(0),
		_aFollowCamera(0),
		_aResetCamera(0),
		_aLockViewZ(0),
		_aShowTrajectory(0),
		_aSetTrajectorySize(0),
		_aClearTrajectory(0),
		_aShowGrid(0),
		_aSetBackgroundColor(0),
		_menu(0),
		_trajectory(new pcl::PointCloud<pcl::PointXYZ>),
		_maxTrajectorySize(100),
		_lastPose(Transform::getIdentity())
{
	this->setMinimumSize(200, 200);

	this->SetRenderWindow(_visualizer->getRenderWindow());

#if !defined(WIN32) && !defined(__APPLE__)
	_visualizer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif

	_visualizer->registerMouseCallback (&CloudViewer::mouseEventOccurred, *this, (void*)_visualizer);
	_visualizer->setCameraPosition(
				-1, 0, 0,
				0, 0, 0,
				0, 0, 1);

	//setup menu/actions
	createMenu();
}

CloudViewer::~CloudViewer()
{
	UDEBUG("");
	this->removeAllClouds();
	delete _visualizer;
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
	_aShowGrid = new QAction("Show grid", this);
	_aShowGrid->setCheckable(true);
	_aSetBackgroundColor = new QAction("Set background color...", this);

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

	//menus
	_menu = new QMenu(this);
	_menu->addMenu(cameraMenu);
	_menu->addMenu(trajectoryMenu);
	_menu->addAction(_aShowGrid);
	_menu->addAction(_aSetBackgroundColor);
}

bool CloudViewer::updateCloudPose(
		const std::string & id,
		const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		UDEBUG("Updating pose %s to %s", id.c_str(), pose.prettyPrint().c_str());
		if(_addedClouds.find(id).value() == pose ||
		   _visualizer->updatePointCloudPose(id, util3d::transformToEigen3f(pose)))
		{
			_addedClouds.find(id).value() = pose;
			return true;
		}
	}
	return false;
}

bool CloudViewer::updateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		UDEBUG("Updating %s with %d points", id.c_str(), (int)cloud->size());
		int index = _visualizer->getColorHandlerIndex(id);
		this->removeCloud(id);
		if(this->addCloud(id, cloud, pose))
		{
			_visualizer->updateColorHandlerIndex(id, index);
			return true;
		}
	}
	return false;
}

bool CloudViewer::updateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose)
{
	if(_addedClouds.contains(id))
	{
		UDEBUG("Updating %s with %d points", id.c_str(), (int)cloud->size());
		int index = _visualizer->getColorHandlerIndex(id);
		this->removeCloud(id);
		if(this->addCloud(id, cloud, pose))
		{
			_visualizer->updateColorHandlerIndex(id, index);
			return true;
		}
	}
	return false;
}

bool CloudViewer::addOrUpdateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose)
{
	if(!updateCloud(id, cloud, pose))
	{
		return addCloud(id, cloud, pose);
	}
	return true;
}

bool CloudViewer::addOrUpdateCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose)
{
	if(!updateCloud(id, cloud, pose))
	{
		return addCloud(id, cloud, pose);
	}
	return true;
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PCLPointCloud2Ptr & binaryCloud,
		const Transform & pose,
		bool rgb)
{
	if(!_addedClouds.contains(id))
	{
		Eigen::Vector4f origin(pose.x(), pose.y(), pose.z(), 0.0f);
		Eigen::Quaternionf orientation = Eigen::Quaternionf(util3d::transformToEigen3f(pose).rotation());

		// add random color channel
		 pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr colorHandler;
		 colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (binaryCloud));
		 if(_visualizer->addPointCloud (binaryCloud, colorHandler, origin, orientation, id))
		{
			 // white
			colorHandler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (binaryCloud, 255, 255, 255));
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

				_visualizer->updateColorHandlerIndex(id, 5);
			}

			_addedClouds.insert(id, pose);
			return true;
		}
	}
	return false;
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & pose)
{
	if(!_addedClouds.contains(id))
	{
		UDEBUG("Adding %s with %d points", id.c_str(), (int)cloud->size());

		pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
		pcl::toPCLPointCloud2(*cloud, *binaryCloud);
		return addCloud(id, binaryCloud, pose, true);
	}
	return false;
}

bool CloudViewer::addCloud(
		const std::string & id,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & pose)
{
	if(!_addedClouds.contains(id))
	{
		UDEBUG("Adding %s with %d points", id.c_str(), (int)cloud->size());

		pcl::PCLPointCloud2Ptr binaryCloud(new pcl::PCLPointCloud2);
		pcl::toPCLPointCloud2(*cloud, *binaryCloud);
		return addCloud(id, binaryCloud, pose, false);
	}
	return false;
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
	const std::vector<pcl::Vertices> & polygons,
	const Transform & pose)
{
	if(!_addedClouds.contains(id))
	{
		UDEBUG("Adding %s with %d points and %d polygons", id.c_str(), (int)cloud->size(), (int)polygons.size());
		if(_visualizer->addPolygonMesh<pcl::PointXYZRGB>(cloud, polygons, id))
		{
			_visualizer->updatePointCloudPose(id, util3d::transformToEigen3f(pose));
			_addedClouds.insert(id, pose);
			return true;
		}
	}
	return false;
}

bool CloudViewer::addCloudMesh(
	const std::string & id,
	const pcl::PolygonMesh::Ptr & mesh,
	const Transform & pose)
{
	if(!_addedClouds.contains(id))
	{
		UDEBUG("Adding %s with %d polygons", id.c_str(), (int)mesh->polygons.size());
		if(_visualizer->addPolygonMesh(*mesh, id))
		{
			_visualizer->updatePointCloudPose(id, util3d::transformToEigen3f(pose));
			_addedClouds.insert(id, pose);
			return true;
		}
	}
	return false;
}

void CloudViewer::setTrajectoryShown(bool shown)
{
	_aShowTrajectory->setChecked(shown);
}

void CloudViewer::setTrajectorySize(int value)
{
	_maxTrajectorySize = value;
}

void CloudViewer::removeAllClouds()
{
	_addedClouds.clear();
	_visualizer->removeAllPointClouds();
}


bool CloudViewer::removeCloud(const std::string & id)
{
	_addedClouds.remove(id);
	return _visualizer->removePointCloud(id);
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

void CloudViewer::updateCameraPosition(const Transform & pose)
{
	if(!pose.isNull())
	{
		Eigen::Affine3f m = util3d::transformToEigen3f(pose);
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

		if(pose != _lastPose)
		{
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
				cameras.front().view[0] = Fp[8];
				cameras.front().view[1] = Fp[9];
				cameras.front().view[2] = Fp[10];
			}

			_visualizer->setCameraPosition(
					cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
					cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
					cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);
		}
		_visualizer->removeCoordinateSystem();
		_visualizer->addCoordinateSystem(0.2, m);
	}

	_lastPose = pose;
}

void CloudViewer::render()
{
	// camera view up z locked?
	if(_aLockViewZ->isChecked())
	{
		std::vector<pcl::visualization::Camera> cameras;
		_visualizer->getCameras(cameras);

		cameras.front().view[0] = 0;
		cameras.front().view[1] = 0;
		cameras.front().view[2] = 1;

		_visualizer->setCameraPosition(
			cameras.front().pos[0], cameras.front().pos[1], cameras.front().pos[2],
			cameras.front().focal[0], cameras.front().focal[1], cameras.front().focal[2],
			cameras.front().view[0], cameras.front().view[1], cameras.front().view[2]);

	}

	this->GetRenderWindow()->Render();
}

void CloudViewer::setBackgroundColor(const QColor & color)
{
	_visualizer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
}

void CloudViewer::setCloudVisibility(const std::string & id, bool isVisible)
{
	pcl::visualization::CloudActorMapPtr cloudActorMap = _visualizer->getCloudActorMap();
	pcl::visualization::CloudActorMap::iterator iter = cloudActorMap->find(id);
	if(iter != cloudActorMap->end())
	{
		iter->second.actor->SetVisibility(isVisible?1:0);
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

void CloudViewer::setCameraTargetLocked()
{
	_aLockCamera->setChecked(true);
}

void CloudViewer::setCameraTargetFollow()
{
	_aFollowCamera->setChecked(true);
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

		render();
	}
	else
	{
		QVTKWidget::keyPressEvent(event);
	}
}

void CloudViewer::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * a = _menu->exec(event->globalPos());
	if(a)
	{
		handleAction(a);
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
		_trajectory->clear();
		_visualizer->removeShape("trajectory");
		this->render();
	}
	else if(a == _aResetCamera)
	{
		_visualizer->setCameraPosition(
				-1, 0, 0,
				0, 0, 0,
				0, 0, 1);
		this->render();
	}
	else if(a == _aShowGrid)
	{
		if(_aShowGrid->isChecked())
		{
			float cellSize = 1.0f;
			int cellCount = 50;
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
				//over y
				name = uFormat("line%d", ++id);
				_visualizer->addLine(pcl::PointXYZ(min, i, 0.0f), pcl::PointXYZ(max, i, 0.0f), r, g, b, name);
				_gridLines.push_back(name);
			}
		}
		else
		{
			for(std::list<std::string>::iterator iter = _gridLines.begin(); iter!=_gridLines.end(); ++iter)
			{
				_visualizer->removeShape(*iter);
			}
			_gridLines.clear();
		}

		this->render();
	}
	else if(a == _aSetBackgroundColor)
	{
		QColor color = Qt::black;
		color = QColorDialog::getColor(color, this);
		this->setBackgroundColor(color);
	}
}

} /* namespace rtabmap */
