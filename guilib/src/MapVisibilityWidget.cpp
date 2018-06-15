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

#include "rtabmap/gui/MapVisibilityWidget.h"

#include <QCheckBox>
#include <QVBoxLayout>
#include <QScrollArea>

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

MapVisibilityWidget::MapVisibilityWidget(QWidget * parent) : QWidget(parent) {

	QVBoxLayout * verticalLayout1 = new QVBoxLayout(this);
	QScrollArea * scrollArea = new QScrollArea(this);
	scrollArea->setWidgetResizable(true);
	QWidget * scrollAreaWidgetContent = new QWidget();
	scrollAreaWidgetContent->setObjectName("area");
	QVBoxLayout * layout2 = new QVBoxLayout(scrollAreaWidgetContent);
	scrollAreaWidgetContent->setLayout(layout2);
	scrollArea->setWidget(scrollAreaWidgetContent);

	QCheckBox * selectAll = new QCheckBox("Select all", this);
	connect(selectAll, SIGNAL(toggled(bool)), this, SLOT(selectAll(bool)));
	verticalLayout1->addWidget(selectAll);
	verticalLayout1->addWidget(scrollArea);
}

MapVisibilityWidget::~MapVisibilityWidget() {

}

void MapVisibilityWidget::showEvent(QShowEvent * event)
{
	updateCheckBoxes();
}

void MapVisibilityWidget::clear()
{
	_poses.clear();
	_mask.clear();
	updateCheckBoxes();
}

void MapVisibilityWidget::updateCheckBoxes()
{
	QWidget * area = this->findChild<QWidget*>("area");
	QVBoxLayout * layout = (QVBoxLayout *)area->layout();
	QList<QCheckBox*> checkboxes = area->findChildren<QCheckBox*>();
	while(checkboxes.size() && checkboxes.size() > (int)_poses.size())
	{
		delete *checkboxes.begin();
		checkboxes.erase(checkboxes.begin());
	}
	int i=0;
	for(std::map<int, Transform>::iterator iter=_poses.begin(); iter!=_poses.end(); ++iter)
	{
		bool added = false;
		if(i >= checkboxes.size())
		{
			checkboxes.push_back(new QCheckBox(area));
			added = true;
		}
		checkboxes[i]->setText(QString("%1 (%2)").arg(iter->first).arg(iter->second.prettyPrint().c_str()));
		checkboxes[i]->setChecked(_mask.at(iter->first));
		if(added)
		{
			connect(checkboxes[i], SIGNAL(stateChanged(int)), this, SLOT(signalVisibility()));
			layout->addWidget(checkboxes[i]);
		}

		++i;
	}
}

void MapVisibilityWidget::setMap(const std::map<int, Transform> & poses, const std::map<int, bool> & mask)
{
	UASSERT(poses.size() == mask.size());
	_poses = poses;
	_mask = mask;
	if(this->isVisible())
	{
		updateCheckBoxes();
	}
}

std::map<int, Transform> MapVisibilityWidget::getVisiblePoses() const
{
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=_poses.begin(); iter!=_poses.end(); ++iter)
	{
		if(_mask.at(iter->first) && iter->first > 0)
		{
			poses.insert(*iter);
		}
	}
	return poses;
}

void MapVisibilityWidget::signalVisibility()
{
	QCheckBox * check = qobject_cast<QCheckBox*>(sender());
	_mask.at(check->text().split('(').first().toInt()) = check->isChecked();
	Q_EMIT visibilityChanged(check->text().split('(').first().toInt(), check->isChecked());
}

void MapVisibilityWidget::selectAll(bool checked)
{
	QWidget * area = this->findChild<QWidget*>("area");
	QList<QCheckBox*> checkboxes = area->findChildren<QCheckBox*>();
	for(int i = 0; i<checkboxes.size(); ++i)
	{
		checkboxes[i]->setChecked(checked);
	}
}

} /* namespace rtabmap */
