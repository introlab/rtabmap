/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
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
