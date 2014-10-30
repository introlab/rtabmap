/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "PostProcessingDialog.h"
#include "ui_postProcessingDialog.h"

#include <QtGui/QPushButton>

namespace rtabmap {

PostProcessingDialog::PostProcessingDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_PostProcessingDialog();
	_ui->setupUi(this);

	connect(_ui->detectMoreLoopClosures, SIGNAL(clicked(bool)), this, SLOT(updateButtonBox()));
	connect(_ui->refineNeighborLinks, SIGNAL(stateChanged(int)), this, SLOT(updateButtonBox()));
	connect(_ui->refineLoopClosureLinks, SIGNAL(stateChanged(int)), this, SLOT(updateButtonBox()));
}

PostProcessingDialog::~PostProcessingDialog()
{
	delete _ui;
}

void PostProcessingDialog::updateButtonBox()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(
			isDetectMoreLoopClosures() || isRefineNeighborLinks() || isRefineLoopClosureLinks());
}

bool PostProcessingDialog::isDetectMoreLoopClosures() const
{
	return _ui->detectMoreLoopClosures->isChecked();
}

double PostProcessingDialog::clusterRadius() const
{
	return _ui->clusterRadius->value();
}

double PostProcessingDialog::clusterAngle() const
{
	return _ui->clusterAngle->value();
}

int PostProcessingDialog::iterations() const
{
	return _ui->iterations->value();
}

bool PostProcessingDialog::isReextractFeatures() const
{
	return _ui->reextractFeatures->isChecked();
}

bool PostProcessingDialog::isRefineNeighborLinks() const
{
	return _ui->refineNeighborLinks->isChecked();
}

bool PostProcessingDialog::isRefineLoopClosureLinks() const
{
	return _ui->refineLoopClosureLinks->isChecked();
}

}
