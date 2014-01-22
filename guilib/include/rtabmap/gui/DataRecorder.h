/*
 * DataRecorder.h
 *
 *  Created on: 2013-10-30
 *      Author: Mathieu
 */

#ifndef DATARECORDER_H_
#define DATARECORDER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <rtabmap/utilite/UEventsHandler.h>
#include <QtGui/QWidget>
#include <rtabmap/core/Image.h>
#include <rtabmap/utilite/UTimer.h>

namespace rtabmap {

class Memory;
class ImageView;

class RTABMAPGUI_EXP DataRecorder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	DataRecorder(QWidget * parent = 0);
	bool init(const QString & path, bool recordInRAM = true);

	void close();

	virtual ~DataRecorder();

public slots:
	void addData(const rtabmap::Image & image);
	void showImage(const rtabmap::Image & image);
protected:
	void handleEvent(UEvent * event);

private:
	Memory * memory_;
	ImageView* imageView_;
	UTimer timer_;
	int dataQueue_;
};

} /* namespace rtabmap */
#endif /* DATARECORDER_H_ */
