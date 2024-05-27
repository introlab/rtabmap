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

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Optimizer.h>

#include "rtabmap/gui/ParametersToolBox.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QStackedWidget>
#include <QScrollArea>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <stdio.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/opencv_modules.hpp>

namespace rtabmap {

ParametersToolBox::ParametersToolBox(QWidget *parent) :
	QWidget(parent),
	comboBox_(new QComboBox(this)),
	stackedWidget_(new QStackedWidget(this))
{
	QVBoxLayout * layout = new QVBoxLayout(this);
	this->setLayout(layout);

	layout->addWidget(comboBox_);
	layout->addWidget(stackedWidget_, 1);
	QPushButton * resetButton = new QPushButton(this);
	resetButton->setText(tr("Restore Defaults"));
	layout->addWidget(resetButton);
	connect(resetButton, SIGNAL(clicked()), this, SLOT(resetCurrentPage()));
}

ParametersToolBox::~ParametersToolBox()
{
}

QWidget * ParametersToolBox::getParameterWidget(const QString & key)
{
	return this->findChild<QWidget*>(key);
}

QStringList ParametersToolBox::resetPage(int index)
{
	QStringList paramChanged;
	const QObjectList & children = stackedWidget_->widget(index)->children().first()->children().first()->children();
	for(int j=0; j<children.size();++j)
	{
		QString key = children.at(j)->objectName();
		// ignore working memory
		QString group = key.split("/").first();
		if(parameters_.find(key.toStdString())!=parameters_.end())
		{
			UASSERT_MSG(parameters_.find(key.toStdString()) != parameters_.end(), uFormat("key=%s", key.toStdString().c_str()).c_str());
			std::string value = Parameters::getDefaultParameters().at(key.toStdString());
			parameters_.at(key.toStdString()) = value;

			if(qobject_cast<QComboBox*>(children.at(j)))
			{
				if(((QComboBox*)children.at(j))->currentIndex() != QString::fromStdString(value).split(':').first().toInt())
				{
					((QComboBox*)children.at(j))->setCurrentIndex(QString::fromStdString(value).split(':').first().toInt());
					paramChanged.append(key);
				}
			}
			else if(qobject_cast<QSpinBox*>(children.at(j)))
			{
				if(((QSpinBox*)children.at(j))->value() != uStr2Int(value))
				{
					((QSpinBox*)children.at(j))->setValue(uStr2Int(value));
					paramChanged.append(key);
				}
			}
			else if(qobject_cast<QDoubleSpinBox*>(children.at(j)))
			{
				if(((QDoubleSpinBox*)children.at(j))->value() != uStr2Double(value))
				{
					((QDoubleSpinBox*)children.at(j))->setValue(uStr2Double(value));
					paramChanged.append(key);
				}
			}
			else if(qobject_cast<QCheckBox*>(children.at(j)))
			{
				if(((QCheckBox*)children.at(j))->isChecked() != uStr2Bool(value))
				{
					((QCheckBox*)children.at(j))->setChecked(uStr2Bool(value));
					paramChanged.append(key);
				}
			}
			else if(qobject_cast<QLineEdit*>(children.at(j)))
			{
				if(((QLineEdit*)children.at(j))->text().compare(QString::fromStdString(value)) != 0)
				{
					((QLineEdit*)children.at(j))->setText(QString::fromStdString(value));
					paramChanged.append(key);
				}
			}
		}
	}
	return paramChanged;
}

void ParametersToolBox::resetCurrentPage()
{
	this->blockSignals(true);
	QStringList paramChanged = this->resetPage(stackedWidget_->currentIndex());
	this->blockSignals(false);
	Q_EMIT parametersChanged(paramChanged);
}

void ParametersToolBox::resetAllPages()
{
	QStringList paramChanged;
	this->blockSignals(true);
	for(int i=0; i< stackedWidget_->count(); ++i)
	{
		paramChanged.append(this->resetPage(i));
	}
	this->blockSignals(false);
	Q_EMIT parametersChanged(paramChanged);
}

void ParametersToolBox::updateParametersVisibility()
{
	//show/hide not used parameters
	/*QComboBox * descriptorBox = this->findChild<QComboBox*>(Parameters::kFeature2D_2Descriptor());
	QComboBox * detectorBox = this->findChild<QComboBox*>(Parameters::kFeature2D_1Detector());
	if(descriptorBox && detectorBox)
	{
		QString group = Parameters::kFeature2D_2Descriptor().split('/').first();
		QWidget * panel = 0;
		for(int i=0; i<this->count(); ++i)
		{
			if(this->widget(i)->objectName().compare(group) == 0)
			{
				panel = this->widget(i);
				break;
			}
		}
		if(panel)
		{
			const QObjectList & objects = panel->children();
			QString descriptorName = descriptorBox->currentText();
			QString detectorName = detectorBox->currentText();

			for(int i=0; i<objects.size(); ++i)
			{
				if(!objects[i]->objectName().isEmpty())
				{
					if(objects[i]->objectName().contains(descriptorName) || objects[i]->objectName().contains(detectorName))
					{
						((QWidget*)objects[i])->setVisible(true);
					}
					else if(objects[i]->objectName().contains("Fast") && detectorName == QString("ORB"))
					{
						((QWidget*)objects[i])->setVisible(true);	// ORB uses some FAST parameters
					}
					else if(!objects[i]->objectName().split('/').at(1).at(0).isDigit())
					{
						((QWidget*)objects[i])->setVisible(false);
					}
				}
			}
		}
	}*/
}

void ParametersToolBox::setupUi(const ParametersMap & parameters)
{
	parameters_ = parameters;
	QWidget * currentItem = 0;
	QStringList groups;
	for(ParametersMap::const_iterator iter=parameters.begin();
			iter!=parameters.end();
			++iter)
	{
		QStringList splitted = QString::fromStdString(iter->first).split('/');
		QString group = splitted.first();

		QString name = splitted.last();
		if(currentItem == 0 || currentItem->objectName().compare(group) != 0)
		{
			groups.push_back(group);
			QScrollArea * area = new QScrollArea(this);
			stackedWidget_->addWidget(area);
			currentItem = new QWidget();
			currentItem->setObjectName(group);
			QVBoxLayout * layout = new QVBoxLayout(currentItem);
			layout->setSizeConstraint(QLayout::SetMinimumSize);
			layout->setContentsMargins(0,0,0,0);
			layout->setSpacing(0);
			area->setWidget(currentItem);

			addParameter(layout, iter->first, iter->second);
		}
		else
		{
			addParameter((QVBoxLayout*)currentItem->layout(), iter->first, iter->second);
		}
	}
	comboBox_->addItems(groups);
	connect(comboBox_, SIGNAL(currentIndexChanged(int)), stackedWidget_, SLOT(setCurrentIndex(int)));

	updateParametersVisibility();
}

void ParametersToolBox::updateParameter(const std::string & key, const std::string & value)
{
	QString group = QString::fromStdString(key).split("/").first();
	if(parameters_.find(key) != parameters_.end())
	{
		parameters_.at(key) = value;
		QWidget * widget = this->findChild<QWidget*>(key.c_str());
		QString type = QString::fromStdString(Parameters::getType(key));
		if(type.compare("string") == 0)
		{
			QString valueQt = QString::fromStdString(value);
			if(valueQt.contains(';'))
			{
				// It's a list, just change the index
				QStringList splitted = valueQt.split(':');
				((QComboBox*)widget)->setCurrentIndex(splitted.first().toInt());
			}
			else
			{
				((QLineEdit*)widget)->setText(valueQt);
			}
		}
		else if(type.compare("int") == 0)
		{
			((QSpinBox*)widget)->setValue(uStr2Int(value));
		}
		else if(type.compare("uint") == 0)
		{
			((QSpinBox*)widget)->setValue(uStr2Int(value));
		}
		else if(type.compare("double") == 0)
		{
			((QDoubleSpinBox*)widget)->setValue(uStr2Double(value));
		}
		else if(type.compare("float") == 0)
		{
			((QDoubleSpinBox*)widget)->setValue(uStr2Float(value));
		}
		else if(type.compare("bool") == 0)
		{
			((QCheckBox*)widget)->setChecked(uStr2Bool(value));
		}
	}
}

void ParametersToolBox::addParameter(
		QVBoxLayout * layout,
		const std::string & key,
		const std::string & value)
{
	std::string type = Parameters::getType(key);
	if(type.compare("string") == 0)
	{
		addParameter(layout, key.c_str(), QString::fromStdString(value));
	}
	else if(type.compare("int") == 0 ||
	        type.compare("uint") == 0 ||
	        type.compare("unsigned int") == 0)
	{
		addParameter(layout, key.c_str(), uStr2Int(value));
	}
	else if(type.compare("double") == 0 ||
	        type.compare("float") == 0)
	{
		addParameter(layout, key.c_str(), uStr2Double(value));
	}
	else if(type.compare("bool") == 0)
	{
		addParameter(layout, key.c_str(), uStr2Bool(value));
	}
	else
	{
	    UWARN("Not implemented type \"%s\" for parameter \"%s\". Parameter is not added to toolbox.", type.c_str(), key.c_str());
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const QString & value)
{
	if(value.contains(';'))
	{
		QComboBox * widget = new QComboBox(this);
		widget->setObjectName(key);
		QStringList splitted = value.split(':');
		widget->addItems(splitted.last().split(';'));

		widget->setCurrentIndex(splitted.first().toInt());
		connect(widget, SIGNAL(currentIndexChanged(int)), this, SLOT(changeParameter(int)));
		addParameter(layout, key, widget);
	}
	else
	{
		QLineEdit * widget = new QLineEdit(value, this);
		widget->setObjectName(key);
		connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
		addParameter(layout, key, widget);
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const double & value)
{
	QDoubleSpinBox * widget = new QDoubleSpinBox(this);
	int decimals = 0;
	int decimalValue = 0;

	QString str = Parameters::getDefaultParameters().at(key.toStdString()).c_str();
	if(!str.isEmpty())
	{
		str.replace(',', '.');
		QStringList items = str.split('.');
		if(items.size() == 2)
		{
			decimals = items.back().length();
			decimalValue = items.back().toInt();
		}
	}

	double def = uStr2Double(Parameters::getDefaultParameters().at(key.toStdString()));
	if(def<0.001 || (decimals >= 4 && decimalValue>0))
	{
		widget->setDecimals(5);
		widget->setSingleStep(0.0001);
	}
	else if(def<0.01 || (decimals >= 3 && decimalValue>0))
	{
		widget->setDecimals(4);
		widget->setSingleStep(0.001);
	}
	else if(def<0.1 || (decimals >= 2 && decimalValue>0))
	{
		widget->setDecimals(3);
		widget->setSingleStep(0.01);
	}
	else if(def<1.0 || (decimals >= 1 && decimalValue>0))
	{
		widget->setDecimals(2);
		widget->setSingleStep(0.1);
	}
	else
	{
		widget->setDecimals(1);
	}


	if(def>0.0)
	{
		widget->setMaximum(def*1000000.0);
	}
	else if(def==0.0)
	{
		widget->setMaximum(1000000.0);
	}
	else if(def<0.0)
	{
		widget->setMinimum(def*1000000.0);
		widget->setMaximum(0.0);
	}

	// set minimum for selected parameters
	if(key.compare(Parameters::kGridMinGroundHeight().c_str()) == 0 ||
		key.compare(Parameters::kGridMaxGroundHeight().c_str()) == 0 ||
		key.compare(Parameters::kGridMaxObstacleHeight().c_str()) == 0)
	{
		widget->setMinimum(-1000000.0);
	}

	widget->setValue(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const int & value)
{
	QSpinBox * widget = new QSpinBox(this);
	int def = uStr2Int(Parameters::getDefaultParameters().at(key.toStdString()));

	if(def>0)
	{
		widget->setMaximum(def*1000000);
	}
	else if(def == 0)
	{
		widget->setMaximum(1000000);
	}
	else if(def<0)
	{
		widget->setMinimum(def*1000000);
		widget->setMaximum(0);
	}
	widget->setValue(value);
	widget->setObjectName(key);

	if(key.compare(Parameters::kVisFeatureType().c_str()) == 0)
	{
#ifndef RTABMAP_NONFREE
		if(value <= 1)
		{
			UWARN("SURF/SIFT not available, setting feature default to FAST/BRIEF.");
			widget->setValue(4);
		}
#endif
	}
	if(key.compare(Parameters::kOptimizerStrategy().c_str()) == 0)
	{
		if(value == 0 && !Optimizer::isAvailable(Optimizer::kTypeTORO))
		{
			if(Optimizer::isAvailable(Optimizer::kTypeGTSAM))
			{
				UWARN("TORO is not available, setting optimization default to GTSAM.");
				widget->setValue(2);
			}
			else if(Optimizer::isAvailable(Optimizer::kTypeG2O))
			{
				UWARN("TORO is not available, setting optimization default to g2o.");
				widget->setValue(1);
			}
		}
		if(value == 1 && !Optimizer::isAvailable(Optimizer::kTypeG2O))
		{
			if(Optimizer::isAvailable(Optimizer::kTypeGTSAM))
			{
				UWARN("g2o is not available, setting optimization default to GTSAM.");
				widget->setValue(2);
			}
			else if(Optimizer::isAvailable(Optimizer::kTypeTORO))
			{
				UWARN("g2o is not available, setting optimization default to TORO.");
				widget->setValue(0);
			}
		}
		if(value == 2 && !Optimizer::isAvailable(Optimizer::kTypeGTSAM))
		{
			if(Optimizer::isAvailable(Optimizer::kTypeG2O))
			{
				UWARN("GTSAM is not available, setting optimization default to g2o.");
				widget->setValue(2);
			}
			else if(Optimizer::isAvailable(Optimizer::kTypeTORO))
			{
				UWARN("GTSAM is not available, setting optimization default to TORO.");
				widget->setValue(1);
			}
		}
		if(!Optimizer::isAvailable(Optimizer::kTypeG2O) &&
			!Optimizer::isAvailable(Optimizer::kTypeGTSAM) &&
			!Optimizer::isAvailable(Optimizer::kTypeTORO))
		{
			widget->setEnabled(false);
		}
	}

	connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const bool & value)
{
	QCheckBox * widget = new QCheckBox(this);
	widget->setChecked(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(stateChanged(int)), this, SLOT(changeParameter(int)));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout, const QString & key, QWidget * widget)
{
	QHBoxLayout * hLayout = new QHBoxLayout();
	layout->insertLayout(layout->count()-1, hLayout);
	QString tmp = key.split('/').last();
	QLabel * label = new QLabel(tmp, this);
	label->setObjectName(key+"/label");
	label->setToolTip(QString("<FONT>%1 [default=%2]</FONT>")
			.arg(Parameters::getDescription(key.toStdString()).c_str())
			.arg(uValue(Parameters::getDefaultParameters(), key.toStdString(), std::string("?")).c_str()));
	label->setTextInteractionFlags(Qt::TextSelectableByMouse);
	hLayout->addWidget(label);
	hLayout->addWidget(widget);
}

void ParametersToolBox::changeParameter(const QString & value)
{
	if(sender())
	{
		parameters_.at(sender()->objectName().toStdString()) = value.toStdString();
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}
void ParametersToolBox::changeParameter()
{
	if(sender())
	{
		QDoubleSpinBox * doubleSpinBox = qobject_cast<QDoubleSpinBox*>(sender());
		QSpinBox * spinBox = qobject_cast<QSpinBox*>(sender());
		QLineEdit * lineEdit = qobject_cast<QLineEdit*>(sender());
		if(doubleSpinBox)
		{
			parameters_.at(sender()->objectName().toStdString()) = uNumber2Str(doubleSpinBox->value());
		}
		else if(spinBox)
		{
			parameters_.at(sender()->objectName().toStdString()) = uNumber2Str(spinBox->value());
		}
		else if(lineEdit)
		{
			parameters_.at(sender()->objectName().toStdString()) =  lineEdit->text().toStdString();
		}
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}

void ParametersToolBox::changeParameter(const int & value)
{
	if(sender())
	{
		QStringList paramChanged;
		QComboBox * comboBox = qobject_cast<QComboBox*>(sender());
		QCheckBox * checkBox = qobject_cast<QCheckBox*>(sender());
		if(comboBox)
		{
			QStringList items;
			for(int i=0; i<comboBox->count(); ++i)
			{
				items.append(comboBox->itemText(i));
			}
			QString merged = QString::number(value) + QString(":") + items.join(";");
			parameters_.at(sender()->objectName().toStdString()) = merged.toStdString();

			this->updateParametersVisibility();
		}
		else if(checkBox)
		{
			parameters_.at(sender()->objectName().toStdString()) = uBool2Str(value==Qt::Checked?true:false);
		}

		paramChanged.append(sender()->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}

} // namespace find_object
