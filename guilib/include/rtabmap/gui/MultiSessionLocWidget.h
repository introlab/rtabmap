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

#ifndef RTABMAP_MULTISESSIONLOCWIDGET_H_
#define RTABMAP_MULTISESSIONLOCWIDGET_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Signature.h"
#include <QWidget>

class QPushButton;
class QProgressBar;

namespace rtabmap {

class MultiSessionLocSubView;
class ImageView;

class RTABMAPGUI_EXP MultiSessionLocWidget : public QWidget
{
	Q_OBJECT

public:
	MultiSessionLocWidget(
			const QMap<int, Signature> * cache,
			const std::map<int, int> * mapIds,
			QWidget * parent = 0);
	virtual ~MultiSessionLocWidget();

	ImageView * getImageView() {return imageView_;}

public Q_SLOTS:
	void updateView(
			const Signature & lastSignature,
			const Statistics & stats);
	void clear();

private:
	const QMap<int, Signature> * cache_;
	const std::map<int, int> * mapIds_;
	std::map<int, std::pair<MultiSessionLocSubView*, int> > subViews_;
	ImageView * imageView_;
	QProgressBar * totalLocProgressBar_;
	QPushButton * resetbutton_;
	int totalFrames_;
	int totalLoops_;
};

} /* namespace rtabmap */
#endif /* RTABMAP_MULTISESSIONLOCWIDGET_H_ */
