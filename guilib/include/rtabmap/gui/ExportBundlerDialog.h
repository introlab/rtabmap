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

#ifndef RTABMAP_CORE_EXPORTBUNDLERDIALOG_H_
#define RTABMAP_CORE_EXPORTBUNDLERDIALOG_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <QDialog>
#include <QSettings>

class Ui_ExportBundlerDialog;

namespace rtabmap {

class RTABMAP_GUI_EXPORT ExportBundlerDialog : public QDialog
{
	Q_OBJECT

public:
	ExportBundlerDialog(QWidget * parent = 0);

	virtual ~ExportBundlerDialog();

	void saveSettings(QSettings & settings, const QString & group = "") const;
	void loadSettings(QSettings & settings, const QString & group = "");

	void setWorkingDirectory(const QString & path);

	void exportBundler(
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const QMap<int, Signature> & signatures,
			const ParametersMap & parameters);

Q_SIGNALS:
	void configChanged();

private Q_SLOTS:
	void getPath();
	void restoreDefaults();
	void updateVisibility();

private:
	Ui_ExportBundlerDialog * _ui;
};

}

#endif /* EXPORTBUNDLERDIALOG_H_ */
