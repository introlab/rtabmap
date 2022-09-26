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

//
// Original version from Find-Object: https://github.com/introlab/find-object
//

#ifndef RTABMAP_PARAMETERSTOOLBOX_H_
#define RTABMAP_PARAMETERSTOOLBOX_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <QWidget>
#include <QSet>

class QVBoxLayout;
class QStackedWidget;
class QComboBox;

namespace rtabmap {

class RTABMAP_GUI_EXPORT ParametersToolBox: public QWidget
{
	Q_OBJECT

public:
	ParametersToolBox(QWidget *parent = 0);
	virtual ~ParametersToolBox();

	void setupUi(const ParametersMap & parameters);
	QWidget * getParameterWidget(const QString & key);
	void updateParameter(const std::string & key, const std::string & value);
	const ParametersMap & getParameters() const {return parameters_;}

private:
	void addParameter(QVBoxLayout * layout, const std::string & key, const std::string & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const QString & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const int & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const double & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const bool & value);
	void addParameter(QVBoxLayout * layout, const QString & name, QWidget * widget);

Q_SIGNALS:
	void parametersChanged(const QStringList & name);

private Q_SLOTS:
	void changeParameter();
	void changeParameter(const QString & value);
	void changeParameter(const int & value);
	void resetCurrentPage();
	void resetAllPages();

private:
	QStringList resetPage(int index);
	void updateParametersVisibility();

private:
	QComboBox * comboBox_;
	QStackedWidget * stackedWidget_;
	ParametersMap parameters_;
};

} // namespace find_object

#endif /* PARAMETERSTOOLBOX_H_ */
