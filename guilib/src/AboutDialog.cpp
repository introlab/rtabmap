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

#include "AboutDialog.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/Optimizer.h"
#include "ui_aboutDialog.h"
#include <opencv2/core/version.hpp>
#include <pcl/pcl_config.h>

namespace rtabmap {

AboutDialog::AboutDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_aboutDialog();
	_ui->setupUi(this);
	QString version = Parameters::getVersion().c_str();
#if DEMO_BUILD
	version.append(" [DEMO]");
#endif
	QString cv_version = CV_VERSION;
#ifdef RTABMAP_NONFREE
	cv_version.append(" [With nonfree]");
#else
	cv_version.append(" [Without nonfree]");
#endif
	_ui->label_version->setText(version);
	_ui->label_opencv_version->setText(cv_version);
	_ui->label_pcl_version->setText(PCL_VERSION_PRETTY);
#ifdef RTABMAP_OCTOMAP
	_ui->label_octomap->setText("Yes");
#else
	_ui->label_octomap->setText("No");
#endif
	_ui->label_freenect->setText(CameraFreenect::available()?"Yes":"No");
	_ui->label_openni2->setText(CameraOpenNI2::available()?"Yes":"No");
	_ui->label_freenect2->setText(CameraFreenect2::available()?"Yes":"No");
	_ui->label_dc1394->setText(CameraStereoDC1394::available()?"Yes":"No");
	_ui->label_flycapture2->setText(CameraStereoFlyCapture2::available()?"Yes":"No");
	_ui->label_zed->setText(CameraStereoZed::available()?"Yes":"No");

	_ui->label_g2o->setText(Optimizer::isAvailable(Optimizer::kTypeG2O)?"Yes":"No");
	_ui->label_gtsam->setText(Optimizer::isAvailable(Optimizer::kTypeGTSAM)?"Yes":"No");
	_ui->label_cvsba->setText(Optimizer::isAvailable(Optimizer::kTypeCVSBA)?"Yes":"No");

}

AboutDialog::~AboutDialog()
{
	delete _ui;
}

}
