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


#ifndef RTABMAP_TEXTURINGSTATE_H_
#define RTABMAP_TEXTURINGSTATE_H_

#include "rtabmap/gui/ProgressDialog.h"
#include "rtabmap/core/ProgressState.h"
#include <QApplication>

namespace rtabmap {

class TexturingState : public QObject, public ProgressState
{
	Q_OBJECT

public:
	TexturingState(ProgressDialog * dialog, bool incrementOnMsgReceived): dialog_(dialog)
	{
		_increment = incrementOnMsgReceived;
		connect(dialog_, SIGNAL(canceled()), this, SLOT(cancel()));
	}
	virtual ~TexturingState() {}
	virtual bool callback(const std::string & msg) const
	{
		if(!msg.empty())
		{
			dialog_->appendText(msg.c_str());
			if(_increment)
			{
				dialog_->incrementStep();
			}
		}
		QApplication::processEvents();
		if(!isCanceled())
		{
			return ProgressState::callback(msg);
		}
		return false;
	}

public Q_SLOTS:
	void cancel()
	{
		setCanceled(true);
	}

private:
	ProgressDialog * dialog_;
	bool _increment;
};

}

#endif /* GUILIB_SRC_TEXTURINGSTATE_H_ */
