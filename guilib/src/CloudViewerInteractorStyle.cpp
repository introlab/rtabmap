/*
 * CloudViewerInteractorStyl.cpp
 *
 *  Created on: Aug 21, 2018
 *      Author: mathieu
 */

#include "rtabmap/gui/CloudViewerInteractorStyle.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/gui/CloudViewerCellPicker.h"

#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UMath.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkObjectFactory.h>
#include <vtkOBBTree.h>
#include <vtkCamera.h>

namespace rtabmap {

// Standard VTK macro for *New ()
vtkStandardNewMacro (CloudViewerInteractorStyle);

CloudViewerInteractorStyle::CloudViewerInteractorStyle() :
	pcl::visualization::PCLVisualizerInteractorStyle(),
	viewer_(0),
	NumberOfClicks(0),
	ResetPixelDistance(0),
	pointsHolder_(new pcl::PointCloud<pcl::PointXYZRGB>),
	orthoMode_(false)
{
	PreviousPosition[0] = PreviousPosition[1] = 0;
	PreviousMeasure[0] = PreviousMeasure[1] =  PreviousMeasure[2] = 0.0f;

	this->MotionFactor = 5;
}

void CloudViewerInteractorStyle::Rotate()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
	int dy = orthoMode_?0:rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

	int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

	double delta_elevation = -20.0 / size[1];
	double delta_azimuth = -20.0 / size[0];

	double rxf = dx * delta_azimuth * this->MotionFactor;
	double ryf = dy * delta_elevation * this->MotionFactor;

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	UASSERT(camera);
	if(!orthoMode_)
	{
		camera->Azimuth(rxf);
		camera->Elevation(ryf);
		camera->OrthogonalizeViewUp();
	}
	else
	{
		camera->Roll(-rxf);
	}

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

void CloudViewerInteractorStyle::setOrthoMode(bool enabled)
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkCamera *camera = CurrentRenderer->GetActiveCamera ();
	UASSERT(camera);
	camera->SetParallelProjection (enabled);
	if(enabled)
	{
		double x,y,z;
		camera->GetFocalPoint(x, y, z);
		camera->SetPosition(x, y, z+(camera->GetDistance()<=5?5:camera->GetDistance()));
		camera->SetViewUp(1, 0, 0);
	}
	CurrentRenderer->SetActiveCamera (camera);
	orthoMode_ = enabled;
}

void CloudViewerInteractorStyle::OnMouseMove()
{
	if(this->CurrentRenderer &&
		this->CurrentRenderer->GetLayer() == 1 &&
		this->GetInteractor()->GetShiftKey() && this->GetInteractor()->GetControlKey() &&
		viewer_ &&
		viewer_->getLocators().size())
	{
		CloudViewerCellPicker * cellPicker = dynamic_cast<CloudViewerCellPicker*>(this->Interactor->GetPicker());
		if(cellPicker)
		{
			int pickPosition[2];
			this->GetInteractor()->GetEventPosition(pickPosition);
			this->Interactor->GetPicker()->Pick(pickPosition[0], pickPosition[1],
					0,  // always zero.
					this->CurrentRenderer);
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

			UDEBUG("Control move! Picked value: %f %f %f", picked[0], picked[1], picked[2]);

			float textSize = 0.05;

			viewer_->removeCloud("interactor_points_alt");
			pointsHolder_->resize(2);
			pcl::PointXYZRGB pt(255,0,0);
			pt.x = picked[0];
			pt.y = picked[1];
			pt.z = picked[2];
			pointsHolder_->at(0) = pt;

			viewer_->removeLine("interactor_ray_alt");
			viewer_->removeText("interactor_ray_text_alt");

			// Intersect the locator with the line
			double length = 5.0;
			double pickedNormal[3];
			cellPicker->GetPickNormal(pickedNormal);
			double lineP0[3] = {picked[0], picked[1], picked[2]};
			double lineP1[3] = {picked[0]+pickedNormal[0]*length, picked[1]+pickedNormal[1]*length, picked[2]+pickedNormal[2]*length};
			vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();

			viewer_->getLocators().begin()->second->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);

			// Display list of intersections
			double intersection[3];
			double previous[3] = {picked[0], picked[1], picked[2]};
			for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
			{
				intersectPoints->GetPoint(i, intersection);

				Eigen::Vector3f v(intersection[0]-previous[0], intersection[1]-previous[1], intersection[2]-previous[2]);
				float n = v.norm();
				if(n  > 0.01f)
				{
					v/=n;
					v *= n/2.0f;
					pt.r = 125;
					pt.g = 125;
					pt.b = 125;
					pt.x = intersection[0];
					pt.y = intersection[1];
					pt.z = intersection[2];
					pointsHolder_->at(1) = pt;
					viewer_->addOrUpdateText("interactor_ray_text_alt", uFormat("%.2f m", n),
							Transform(previous[0]+v[0], previous[1]+v[1],previous[2]+v[2], 0, 0, 0),
							textSize,
							Qt::gray);
					viewer_->addOrUpdateLine("interactor_ray_alt",
							Transform(previous[0], previous[1], previous[2], 0, 0, 0),
							Transform(intersection[0], intersection[1], intersection[2], 0, 0, 0),
							Qt::gray);

					previous[0] = intersection[0];
					previous[1] = intersection[1];
					previous[2] = intersection[2];
					break;
				}
			}
			viewer_->addCloud("interactor_points_alt", pointsHolder_);
			viewer_->setCloudPointSize("interactor_points_alt", 15);
			viewer_->setCloudOpacity("interactor_points_alt", 0.5);
		}
	}
	// Forward events
	PCLVisualizerInteractorStyle::OnMouseMove();
}

void CloudViewerInteractorStyle::OnLeftButtonDown()
{
	// http://www.vtk.org/Wiki/VTK/Examples/Cxx/Interaction/DoubleClick
	// http://www.vtk.org/Wiki/VTK/Examples/Cxx/Interaction/PointPicker
	if(this->CurrentRenderer && this->CurrentRenderer->GetLayer() == 1)
	{
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

		if(this->NumberOfClicks >= 2)
		{
			this->NumberOfClicks = 0;
			this->Interactor->GetPicker()->Pick(pickPosition[0], pickPosition[1],
					0,  // always zero.
					this->CurrentRenderer);
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);
			UDEBUG("Double clicked! Picked value: %f %f %f", picked[0], picked[1], picked[2]);
			if(this->GetInteractor()->GetControlKey()==0)
			{
				vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
				UASSERT(camera);
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
			else if(viewer_)
			{
				viewer_->removeText("interactor_pose");
				viewer_->removeLine("interactor_line");
				viewer_->removeCloud("interactor_points");
				viewer_->removeLine("interactor_ray");
				viewer_->removeText("interactor_ray_text");
				viewer_->removeCloud("interactor_points_alt");
				viewer_->removeLine("interactor_ray_alt");
				viewer_->removeText("interactor_ray_text_alt");
				PreviousMeasure[0] = 0.0f;
				PreviousMeasure[1] = 0.0f;
				PreviousMeasure[2] = 0.0f;
			}
		}
		else if(this->GetInteractor()->GetControlKey() && viewer_)
		{
			this->Interactor->GetPicker()->Pick(pickPosition[0], pickPosition[1],
					0,  // always zero.
					this->CurrentRenderer);
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);

			UDEBUG("Shift clicked! Picked value: %f %f %f", picked[0], picked[1], picked[2]);

			float textSize = 0.05;

			viewer_->removeCloud("interactor_points");
			pointsHolder_->clear();
			pcl::PointXYZRGB pt(255,0,0);
			pt.x = picked[0];
			pt.y = picked[1];
			pt.z = picked[2];
			pointsHolder_->push_back(pt);

			viewer_->removeLine("interactor_ray");
			viewer_->removeText("interactor_ray_text");

			if(	PreviousMeasure[0] != 0.0f && PreviousMeasure[1] != 0.0f && PreviousMeasure[2] != 0.0f &&
				viewer_->getAddedLines().find("interactor_line") == viewer_->getAddedLines().end())
			{
				viewer_->addOrUpdateLine("interactor_line",
						Transform(PreviousMeasure[0], PreviousMeasure[1], PreviousMeasure[2], 0, 0, 0),
						Transform(picked[0], picked[1], picked[2], 0, 0, 0),
						Qt::red);
				pt.x = PreviousMeasure[0];
				pt.y = PreviousMeasure[1];
				pt.z = PreviousMeasure[2];
				pointsHolder_->push_back(pt);

				Eigen::Vector3f v(picked[0]-PreviousMeasure[0], picked[1]-PreviousMeasure[1], picked[2]-PreviousMeasure[2]);
				float n = v.norm();
				v/=n;
				v *= n/2.0f;
				viewer_->addOrUpdateText("interactor_pose", uFormat("%.2f m", n),
						Transform(PreviousMeasure[0]+v[0], PreviousMeasure[1]+v[1],PreviousMeasure[2]+v[2], 0, 0, 0),
						textSize,
						Qt::red);
			}
			else
			{
				viewer_->removeText("interactor_pose");
				viewer_->removeLine("interactor_line");
			}
			PreviousMeasure[0] = picked[0];
			PreviousMeasure[1] = picked[1];
			PreviousMeasure[2] = picked[2];

			viewer_->addCloud("interactor_points", pointsHolder_);
			viewer_->setCloudPointSize("interactor_points", 15);
			viewer_->setCloudOpacity("interactor_points", 0.5);
		}
	}

	// Forward events
	PCLVisualizerInteractorStyle::OnLeftButtonDown();
}

} /* namespace rtabmap */
