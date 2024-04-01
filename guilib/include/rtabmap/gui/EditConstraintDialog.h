/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef RTABMAP_EDITCONSTRAINTDIALOG_H_
#define RTABMAP_EDITCONSTRAINTDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QDialog>
#include <rtabmap/core/Transform.h>

class Ui_EditConstraintDialog;

namespace rtabmap {

class RTABMAP_GUI_EXPORT EditConstraintDialog : public QDialog
{
	Q_OBJECT

public:
	EditConstraintDialog(const Transform & constraint, double linearSigma = 1, double angularSigma = 1, QWidget * parent = 0);

	virtual ~EditConstraintDialog();
	Transform getTransform() const;
	double getLinearVariance() const;
	double getAngularVariance() const;

private Q_SLOTS:
	void switchUnits();

private:
	Ui_EditConstraintDialog * _ui;
};

}

#endif /* EDITCONSTRAINTDIALOG_H_ */
