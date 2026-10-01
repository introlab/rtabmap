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

#ifndef RTABMAP_GUILIB_SRC_GUIUTIL_H_
#define RTABMAP_GUILIB_SRC_GUIUTIL_H_

#include <QWidget>
#include <QApplication>
#include <QEventLoop>
#include <QElapsedTimer>
#include <QtGui/QWindow>

namespace rtabmap {

// Show a widget and block until it is actually painted on screen. show() only
// schedules window mapping + painting
inline void showAndWaitExposed(QWidget * widget)
{
	widget->show();
	if(widget->windowHandle())
	{
		QElapsedTimer timer;
		timer.start();
		while(!widget->windowHandle()->isExposed() && timer.elapsed() < 2000)
		{
			QApplication::processEvents(QEventLoop::ExcludeUserInputEvents | QEventLoop::WaitForMoreEvents, 50);
		}
	}
	QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
	widget->repaint(); // synchronous, unlike update()
}

} // namespace rtabmap

#endif /* RTABMAP_GUILIB_SRC_GUIUTIL_H_ */
