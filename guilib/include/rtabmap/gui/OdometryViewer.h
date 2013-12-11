/*
 * OdometryViewer.h
 *
 *  Created on: 2013-10-15
 *      Author: Mathieu
 */

#ifndef ODOMETRYVIEWER_H_
#define ODOMETRYVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/core/Image.h"
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UMutex.h"

namespace rtabmap {

class RTABMAPGUI_EXP OdometryViewer : public CloudViewer, public UEventsHandler
{
	Q_OBJECT

public:
	OdometryViewer(int maxClouds = 10, int decimation = 2, float voxelSize = 0.0f, QWidget * parent = 0);
	virtual ~OdometryViewer() {}

protected:
	void handleAction(QAction * a);
	virtual void handleEvent(UEvent * event);

private slots:
	void processData();

private:
	UMutex dataMutex_;
	std::list<rtabmap::Image> buffer_;
	UTimer timer_;
	int maxClouds_;
	float voxelSize_;
	int decimation_;
	int id_;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_;
	QAction * _aSetVoxelSize;
	QAction * _aSetDecimation;
	QAction * _aSetCloudHistorySize;
	QAction * _aPause;
};

} /* namespace rtabmap */
#endif /* ODOMETRYVIEWER_H_ */
