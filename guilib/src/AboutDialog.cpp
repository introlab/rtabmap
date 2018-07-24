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

#include "rtabmap/gui/AboutDialog.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/Optimizer.h"
#include "ui_aboutDialog.h"
#include <opencv2/core/version.hpp>
#include <pcl/pcl_config.h>
#include <vtkVersion.h>

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
	_ui->label_opencv_license->setText("Not Commercial");
#else
	cv_version.append(" [Without nonfree]");
	_ui->label_opencv_license->setText("BSD");
#endif
	_ui->label_version->setText(version);
	_ui->label_opencv_version->setText(cv_version);
	_ui->label_pcl_version->setText(PCL_VERSION_PRETTY);
	_ui->label_vtk_version->setText(vtkVersion::GetVTKVersion());
	_ui->label_qt_version->setText(qVersion());
#ifdef RTABMAP_OCTOMAP
	_ui->label_octomap->setText("Yes");
	_ui->label_octomap_license->setEnabled(true);
#else
	_ui->label_octomap->setText("No");
	_ui->label_octomap_license->setEnabled(false);
#endif
#ifdef RTABMAP_CPUTSDF
	_ui->label_cputsdf->setText("Yes");
	_ui->label_cputsdf_license->setEnabled(true);
#else
	_ui->label_cputsdf->setText("No");
	_ui->label_cputsdf_license->setEnabled(false);
#endif
#ifdef RTABMAP_OPENCHISEL
	_ui->label_openchisel->setText("Yes");
#else
	_ui->label_openchisel->setText("No");
#endif

	_ui->label_freenect->setText(CameraFreenect::available()?"Yes":"No");
	_ui->label_freenect_license->setEnabled(CameraFreenect::available());
	_ui->label_openni2->setText(CameraOpenNI2::available()?"Yes":"No");
	_ui->label_openni2_license->setEnabled(CameraOpenNI2::available());
	_ui->label_freenect2->setText(CameraFreenect2::available()?"Yes":"No");
	_ui->label_freenect2_license->setEnabled(CameraFreenect2::available());
	_ui->label_realsense->setText(CameraRealSense::available() ? "Yes" : "No");
	_ui->label_realsense_license->setEnabled(CameraRealSense::available());
	_ui->label_realsense2->setText(CameraRealSense2::available() ? "Yes" : "No");
	_ui->label_realsense2_license->setEnabled(CameraRealSense2::available());
	_ui->label_dc1394->setText(CameraStereoDC1394::available()?"Yes":"No");
	_ui->label_dc1394_license->setEnabled(CameraStereoDC1394::available());
	_ui->label_flycapture2->setText(CameraStereoFlyCapture2::available()?"Yes":"No");
	_ui->label_zed->setText(CameraStereoZed::available()?"Yes":"No");

	_ui->label_toro->setText(Optimizer::isAvailable(Optimizer::kTypeTORO)?"Yes":"No");
	_ui->label_toro_license->setEnabled(Optimizer::isAvailable(Optimizer::kTypeTORO)?true:false);
	_ui->label_g2o->setText(Optimizer::isAvailable(Optimizer::kTypeG2O)?"Yes":"No");
	_ui->label_g2o_license->setEnabled(Optimizer::isAvailable(Optimizer::kTypeG2O)?true:false);
	_ui->label_gtsam->setText(Optimizer::isAvailable(Optimizer::kTypeGTSAM)?"Yes":"No");
	_ui->label_gtsam_license->setEnabled(Optimizer::isAvailable(Optimizer::kTypeGTSAM)?true:false);
	_ui->label_cvsba->setText(Optimizer::isAvailable(Optimizer::kTypeCVSBA)?"Yes":"No");
	_ui->label_cvsba_license->setEnabled(Optimizer::isAvailable(Optimizer::kTypeCVSBA)?true:false);

#ifdef RTABMAP_POINTMATCHER
	_ui->label_libpointmatcher->setText("Yes");
	_ui->label_libpointmatcher_license->setEnabled(true);
#else
	_ui->label_libpointmatcher->setText("No");
	_ui->label_libpointmatcher_license->setEnabled(false);
#endif

#ifdef RTABMAP_FOVIS
	_ui->label_fovis->setText("Yes");
	_ui->label_fovis_license->setEnabled(true);
#else
	_ui->label_fovis->setText("No");
	_ui->label_fovis_license->setEnabled(false);
#endif
#ifdef RTABMAP_VISO2
	_ui->label_viso2->setText("Yes");
	_ui->label_viso2_license->setEnabled(true);
#else
	_ui->label_viso2->setText("No");
	_ui->label_viso2_license->setEnabled(false);
#endif
#ifdef RTABMAP_DVO
	_ui->label_dvo->setText("Yes");
	_ui->label_dvo_license->setEnabled(true);
#else
	_ui->label_dvo->setText("No");
	_ui->label_dvo_license->setEnabled(false);
#endif
#ifdef RTABMAP_ORB_SLAM2
	_ui->label_orbslam2->setText("Yes");
	_ui->label_orbslam2_license->setEnabled(true);
#else
	_ui->label_orbslam2->setText("No");
	_ui->label_orbslam2_license->setEnabled(false);
#endif

#ifdef RTABMAP_OKVIS
	_ui->label_okvis->setText("Yes");
	_ui->label_okvis_license->setEnabled(true);
#else
	_ui->label_okvis->setText("No");
	_ui->label_okvis_license->setEnabled(false);
#endif

#ifdef RTABMAP_LOAM
	_ui->label_loam->setText("Yes");
	_ui->label_loam_license->setEnabled(true);
#else
	_ui->label_loam->setText("No");
	_ui->label_loam_license->setEnabled(false);
#endif

#ifdef RTABMAP_MSCKF_VIO
	_ui->label_msckf->setText("Yes");
	_ui->label_msckf_license->setEnabled(true);
#else
	_ui->label_msckf->setText("No");
	_ui->label_msckf_license->setEnabled(false);
#endif

}

AboutDialog::~AboutDialog()
{
	delete _ui;
}

}
