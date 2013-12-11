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

#include "AboutDialog.h"
#include "rtabmap/core/Rtabmap.h"
#include "ui_aboutDialog.h"
#include <opencv2/core/version.hpp>
#include <pcl/pcl_config.h>

namespace rtabmap {

AboutDialog::AboutDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_aboutDialog();
	_ui->setupUi(this);
	QString version = Rtabmap::getVersion().c_str();
#if DEMO_BUILD
	version.append(" [DEMO]");
#endif
	_ui->label_version->setText(version);
	_ui->label_opencv_version->setText(CV_VERSION);
	_ui->label_pcl_version->setText(PCL_VERSION_PRETTY);
}

AboutDialog::~AboutDialog()
{
	delete _ui;
}

}
