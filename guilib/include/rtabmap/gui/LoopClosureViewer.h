/*
 * LoopClosureViewer.h
 *
 *  Created on: 2013-10-21
 *      Author: Mathieu
 */

#ifndef LOOPCLOSUREVIEWER_H_
#define LOOPCLOSUREVIEWER_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/opencv.hpp>
#include <QtGui/QWidget>

class Ui_loopClosureViewer;

namespace rtabmap {

class Signature;

class RTABMAPGUI_EXP LoopClosureViewer : public QWidget {

	Q_OBJECT

public:
	LoopClosureViewer(QWidget * parent);
	virtual ~LoopClosureViewer();

	// take ownership
	void setData(Signature * sA, Signature * sB); // sB contains loop transform as pose() from sA

	const Signature * sA() const {return sA_;}
	const Signature * sB() const {return sB_;}

public slots:
	void setDecimation(int decimation) {decimation_ = decimation;}
	void setMaxDepth(int maxDepth) {maxDepth_ = maxDepth;}
	void setSamples(int samples) {samples_ = samples;}
	void updateView(const Transform & AtoB = Transform());

protected:
	virtual void showEvent(QShowEvent * event);

private:
	Ui_loopClosureViewer * ui_;

	Signature * sA_;
	Signature * sB_;
	Transform transform_;

	int decimation_;
	float maxDepth_;
	int samples_;
};

} /* namespace rtabmap */
#endif /* LOOPCLOSUREVIEWER_H_ */
