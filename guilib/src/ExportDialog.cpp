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

#include "ExportDialog.h"
#include "ui_exportDialog.h"

#include <QtGui/QFileDialog>

namespace rtabmap {

ExportDialog::ExportDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportDialog();
	_ui->setupUi(this);

	connect(_ui->toolButton_path, SIGNAL(clicked()), this, SLOT(getPath()));

	_ui->lineEdit_path->setText(QDir::homePath()+QDir::separator()+"output.db");
}

ExportDialog::~ExportDialog()
{
	delete _ui;
}

void ExportDialog::getPath()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Output database path..."), _ui->lineEdit_path->text(), tr("RTAB-Map database (*.db)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_path->setText(path);
	}
}

QString ExportDialog::outputPath() const
{
	return _ui->lineEdit_path->text();
}

int ExportDialog::framesIgnored() const
{
	return _ui->spinBox_ignored->value();
}

bool ExportDialog::isRgbExported() const
{
	return _ui->checkBox_rgb->isChecked();
}

bool ExportDialog::isDepthExported() const
{
	return _ui->checkBox_depth->isChecked();
}

bool ExportDialog::isDepth2dExported() const
{
	return _ui->checkBox_depth2d->isChecked();
}

bool ExportDialog::isOdomExported() const
{
	return _ui->checkBox_odom->isChecked();
}

}
