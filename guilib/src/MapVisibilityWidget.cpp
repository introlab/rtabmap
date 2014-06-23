/*
 * MapVisibilityWidget.cpp
 *
 *  Created on: 2014-01-20
 *      Author: mathieu
 */

#include "MapVisibilityWidget.h"

#include <QtGui/QCheckBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QScrollArea>

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
	verticalLayout1->addWidget(scrollArea);
}

MapVisibilityWidget::~MapVisibilityWidget() {

}

void MapVisibilityWidget::showEvent(QShowEvent * event)
{
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
		if(_mask.at(iter->first))
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
	emit visibilityChanged(check->text().split('(').first().toInt(), check->isChecked());
}

} /* namespace rtabmap */
