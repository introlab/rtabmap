/*
 * CloudViewerInteractorStyl.h
 *
 *  Created on: Aug 21, 2018
 *      Author: mathieu
 */

#ifndef GUILIB_SRC_CLOUDVIEWERINTERACTORSTYLE_H_
#define GUILIB_SRC_CLOUDVIEWERINTERACTORSTYLE_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <pcl/visualization/mouse_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/point_types.h>

namespace rtabmap {

class CloudViewer;

class RTABMAP_GUI_EXPORT CloudViewerInteractorStyle: public pcl::visualization::PCLVisualizerInteractorStyle
{
public:
    static CloudViewerInteractorStyle *New ();
    vtkTypeMacro(CloudViewerInteractorStyle, pcl::visualization::PCLVisualizerInteractorStyle);

public:
	CloudViewerInteractorStyle();
	virtual void Rotate();
	void setOrthoMode(bool enabled);
protected:
	virtual void OnMouseMove();
	virtual void OnLeftButtonDown();

protected:
	friend class CloudViewer;
	void setCloudViewer(CloudViewer * cloudViewer) {viewer_ = cloudViewer;}
	CloudViewer * viewer_;

private:
	unsigned int NumberOfClicks;
	int PreviousPosition[2];
	int ResetPixelDistance;
	float PreviousMeasure[3];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsHolder_;
	bool orthoMode_;
};

} /* namespace rtabmap */

#endif /* GUILIB_SRC_CLOUDVIEWERINTERACTORSTYLE_H_ */
