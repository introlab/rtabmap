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

#include "rtabmap/gui/StatsToolBox.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMenu>
#include <QLabel>
#include <QToolButton>
#include <QtCore/QChildEvent>
#include <QtCore/QDir>
#include <QtGui/QContextMenuEvent>
#include <QToolBox>
#include <QDialog>

#include "rtabmap/utilite/UPlot.h"
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

StatItem::StatItem(const QString & name, bool cacheOn, const std::vector<qreal> & x, const std::vector<qreal> & y, const QString & unit, const QMenu * menu, QGridLayout * grid, QWidget * parent) :
	QWidget(parent),
	_button(0),
	_name(0),
	_value(0),
	_unit(0),
	_menu(0),
	_cacheOn(cacheOn)
{
	this->setupUi(grid);
	_name->setText(name);
	if(y.size() == 1 || (y.size() > 1 && _cacheOn))
	{
		_value->setNum(y[y.size()-1]);
	}
	else if(y.size() > 1)
	{
		_value->setText("*");
	}
	if(cacheOn)
	{
		_x = x;
		_y = y;
	}
	_unit->setText(unit);
	this->updateMenu(menu);
}

StatItem::~StatItem()
{

}

void StatItem::clearCache()
{
	_x.clear();
	_y.clear();
	_value->clear();
}

void StatItem::addValue(qreal y)
{
	if(_cacheOn)
	{
		_y.push_back(y);
	}
	_value->setText(QString::number(y, 'g', 3));
	Q_EMIT valueAdded(y);
}

void StatItem::addValue(qreal x, qreal y)
{
	if(_cacheOn)
	{
		if (_x.size() && x <_x.back())
		{
			clearCache();
		}

		_y.push_back(y);
		_x.push_back(x);
	}

	_value->setText(QString::number(y, 'g', 3));
	Q_EMIT valueAdded(x,y);
}

void StatItem::setValues(const std::vector<qreal> & x, const std::vector<qreal> & y)
{
	if(_cacheOn)
	{
		_x = x;
		_y = y;
		if(y.size())
		{
			_value->setNum(y[y.size()-1]);
		}
	}
	else
	{
		_value->setText("*");
	}
	Q_EMIT valuesChanged(x,y);
}

QString StatItem::value() const
{
	return _value->text();
}

void StatItem::setupUi(QGridLayout * grid)
{
	_menu = new QMenu(this);
	_menu->addMenu("Add to figure...");
	_button = new QToolButton(this);
	_button->setIcon(QIcon(":/images/Plot16.png"));
	_button->setPopupMode(QToolButton::InstantPopup);
	_button->setMenu(_menu);
	_name = new QLabel(this);
	_name->setTextInteractionFlags(Qt::TextSelectableByMouse);
	_name->setWordWrap(true);
	_value = new QLabel(this);
	_value->setTextInteractionFlags(Qt::TextSelectableByMouse);
	_unit = new QLabel(this);

	if(grid)
	{
		int row = grid->rowCount();

		//This fixes an issue where the
		//button (used on col 0) of the first line in the
		//toolbox couldn't be clicked
		grid->addWidget(_button, row, 3);
		grid->addWidget(_name, row, 0);
		grid->addWidget(_value, row, 1);
		grid->addWidget(_unit, row, 2);
	}
	else
	{
		QHBoxLayout * layout = new QHBoxLayout(this);
		this->setLayout(layout);
		layout->addWidget(_button);
		layout->addWidget(_name);
		layout->addWidget(_value);
		layout->addWidget(_unit);
		layout->addStretch();
		layout->setContentsMargins(0,0,0,0);
	}
}

void StatItem::setCacheOn(bool on)
{
	_cacheOn = on;
	if(!on)
	{
		_x.clear();
		_y.clear();
	}
}

void StatItem::updateMenu(const QMenu * menu)
{
	_menu->clear();
	QAction * action;
	QList<QAction *> actions = menu->actions();
	QMenu * plotMenu = _menu->addMenu("Add to figure...");
	for(int i=0; i<actions.size(); ++i)
	{
		action = plotMenu->addAction(actions.at(i)->text());
		connect(action, SIGNAL(triggered()), this, SLOT(preparePlotRequest()));
	}
}

void StatItem::preparePlotRequest()
{
	QAction * action = qobject_cast<QAction*>(sender());
	if(action)
	{
		Q_EMIT plotRequested(this, action->text());
	}
}








StatsToolBox::StatsToolBox(QWidget * parent) :
	QWidget(parent)
{
	ULOGGER_DEBUG("");
	//Statistics in the GUI (for plotting)
	_statBox = new QToolBox(this);
	this->setLayout(new QVBoxLayout());
	this->layout()->setContentsMargins(0,0,0,0);
	this->layout()->addWidget(_statBox);
	_statBox->layout()->setSpacing(0);
	_plotMenu = new QMenu(this);
	_plotMenu->addAction(tr("<New figure>"));
	_workingDirectory = QDir::homePath();
	_newFigureMaxItems = 0;
}

StatsToolBox::~StatsToolBox()
{
	closeFigures();
}

void StatsToolBox::closeFigures()
{
	QMap<QString, QWidget*> figuresTmp = _figures;
	for(QMap<QString, QWidget*>::iterator iter = figuresTmp.begin(); iter!=figuresTmp.end(); ++iter)
	{
		iter.value()->close();
	}
}

void StatsToolBox::setCacheOn(bool on)
{
	QList<StatItem *> items = _statBox->findChildren<StatItem *>();
	for(int i=0; i<items.size(); ++i)
	{
		items[i]->setCacheOn(on);
	}
}

void StatsToolBox::updateStat(const QString & statFullName, bool cacheOn)
{
	std::vector<qreal> vx,vy;
	updateStat(statFullName, vx, vy, cacheOn);
}

void StatsToolBox::updateStat(const QString & statFullName, qreal y, bool cacheOn)
{
	std::vector<qreal> vx,vy(1);
	vy[0] = y;
	updateStat(statFullName, vx, vy, cacheOn);
}

void StatsToolBox::updateStat(const QString & statFullName, qreal x, qreal y, bool cacheOn)
{
	std::vector<qreal> vx(1),vy(1);
	vx[0] = x;
	vy[0] = y;
	updateStat(statFullName, vx, vy, cacheOn);
}

void StatsToolBox::updateStat(const QString & statFullName, const std::vector<qreal> & x, const std::vector<qreal> & y, bool cacheOn)
{
	// round qreal to max 2 numbers after the dot
	//x = (qreal(int(100*x)))/100;
	//y = (qreal(int(100*y)))/100;

	StatItem * item = _statBox->findChild<StatItem *>(statFullName);
	if(item)
	{
		item->setCacheOn(cacheOn);
		if(y.size() == 1 && x.size() == 1)
		{
			item->addValue(x[0], y[0]);
		}
		else if(y.size() == 1 && x.size() == 0)
		{
			item->addValue(y[0]);
		}
		else
		{
			item->setValues(x, y);
		}
	}
	else
	{
		// statFullName format : "Grp/Name/unit"
		QStringList list = statFullName.split('/');
		QString grp;
		QString name;
		QString unit;
		if(list.size() >= 3)
		{
			grp = list.at(0);
			name = list.at(1);
			unit = list.at(2);
			for(int i=3; i<list.size(); ++i)
			{
				unit += "/" + list.at(i);
			}
		}
		else if(list.size() == 2)
		{
			grp = list.at(0);
			name = list.at(1);
		}
		else if(list.size() == 1)
		{
			name = list.at(0);
		}
		else
		{
			ULOGGER_WARN("A statistic has no name");
			return;
		}

		if(grp.isEmpty())
		{
			grp = tr("Global");
		}

		int index = -1;
		for(int i=0; i<_statBox->count(); ++i)
		{
			if(_statBox->itemText(i).compare(grp) == 0)
			{
				index = i;
				break;
			}
		}

		if(index<0)
		{
			QWidget * newWidget = new QWidget(_statBox);
			index = _statBox->addItem(newWidget, grp);
			QVBoxLayout * layout = new QVBoxLayout(newWidget);
			newWidget->setLayout(layout);
			QGridLayout * grid = new QGridLayout();
			grid->setVerticalSpacing(2);
			grid->setColumnStretch(0, 1);
			layout->addLayout(grid);
			layout->addStretch();
		}

		QVBoxLayout * layout = qobject_cast<QVBoxLayout *>(_statBox->widget(index)->layout());
		if(!layout)
		{
			ULOGGER_ERROR("Layout is null ?!?");
			return;
		}
		QGridLayout * grid = qobject_cast<QGridLayout *>(layout->itemAt(0)->layout());
		if(!grid)
		{
			ULOGGER_ERROR("Layout is null ?!?");
			return;
		}

		item = new StatItem(name, cacheOn, x, y, unit, _plotMenu, grid, _statBox->widget(index));
		item->setObjectName(statFullName);

		//layout->insertWidget(layout->count()-1, item);
		connect(item, SIGNAL(plotRequested(const StatItem *, const QString &)), this, SLOT(plot(const StatItem *, const QString &)));
		connect(this, SIGNAL(menuChanged(const QMenu *)), item, SLOT(updateMenu(const QMenu *)));
	}
}

void StatsToolBox::plot(const StatItem * stat, const QString & plotName)
{
	QWidget * fig = _figures.value(plotName, (QWidget*)0);
	UPlot * plot = 0;
	if(fig)
	{
		plot = fig->findChild<UPlot *>(plotName);
	}
	if(plot)
	{
		// if not already in the plot
		if(!plot->contains(stat->objectName()))
		{
			UPlotCurve * curve = new UPlotCurve(stat->objectName(), plot);
			curve->setPen(plot->getRandomPenColored());
			connect(stat, SIGNAL(valueAdded(qreal)), curve, SLOT(addValue(qreal)));
			connect(stat, SIGNAL(valueAdded(qreal, qreal)), curve, SLOT(addValue(qreal, qreal)));
			connect(stat, SIGNAL(valuesChanged(const std::vector<qreal> &, const std::vector<qreal> &)), curve, SLOT(setData(const std::vector<qreal> &, const std::vector<qreal> &)));
			if(stat->value().compare("*") == 0)
			{
				plot->setMaxVisibleItems(0);
			}
			if(!stat->yValues().empty())
			{
				if(stat->xValues().size() == stat->yValues().size())
				{
					curve->setData(stat->xValues(),stat->yValues());
				}
				else
				{
					curve->setData(stat->yValues());
				}
			}
			if(!plot->addCurve(curve))
			{
				ULOGGER_WARN("Already added to the figure");
			}
			Q_EMIT figuresSetupChanged();
		}
		else
		{
			ULOGGER_WARN("Already added to the figure");
		}
		plot->activateWindow();
	}
	else
	{
		//Create a new plot
		QString id = tr("Figure 0");
		if(_plotMenu->actions().size())
		{
			id = _plotMenu->actions().last()->text();
		}
		id.replace(tr("Figure "), "");
		QString newPlotName = QString(tr("Figure %1")).arg(id.toInt()+1);
		//Dock
		QDialog * figure = new QDialog(0, Qt::Window);
		_figures.insert(newPlotName, figure);
		QHBoxLayout * hLayout = new QHBoxLayout(figure);
		hLayout->setContentsMargins(0,0,0,0);
		figure->setWindowTitle(newPlotName);
		figure->setAttribute(Qt::WA_DeleteOnClose, true);
		connect(figure, SIGNAL(destroyed(QObject*)), this, SLOT(figureDeleted(QObject*)));
		//Plot
		UPlot * newPlot = new UPlot(figure);
		newPlot->setWorkingDirectory(_workingDirectory);
		newPlot->setMaxVisibleItems(_newFigureMaxItems);
		newPlot->setObjectName(newPlotName);
		hLayout->addWidget(newPlot);
		_plotMenu->addAction(newPlotName);
		figure->setSizeGripEnabled(true);

		//Add a new curve linked to the statBox
		UPlotCurve * curve = new UPlotCurve(stat->objectName(), newPlot);
		curve->setPen(newPlot->getRandomPenColored());
		connect(stat, SIGNAL(valueAdded(qreal)), curve, SLOT(addValue(qreal)));
		connect(stat, SIGNAL(valueAdded(qreal, qreal)), curve, SLOT(addValue(qreal, qreal)));
		connect(stat, SIGNAL(valuesChanged(const std::vector<qreal> &, const std::vector<qreal> &)), curve, SLOT(setData(const std::vector<qreal> &, const std::vector<qreal> &)));
		if(stat->value().compare("*") == 0)
		{
			newPlot->setMaxVisibleItems(0);
		}

		if(!stat->yValues().empty())
		{
			if(stat->xValues().size() == stat->yValues().size())
			{
				curve->setData(stat->xValues(),stat->yValues());
			}
			else
			{
				curve->setData(stat->yValues());
			}
		}

		if(!newPlot->addCurve(curve))
		{
			ULOGGER_ERROR("Not supposed to be here !?!");
			delete curve;
		}
		figure->show();
		Q_EMIT figuresSetupChanged();

		Q_EMIT menuChanged(_plotMenu);
	}
}

void StatsToolBox::figureDeleted(QObject * obj)
{
	if(obj)
	{
		QWidget * plot = qobject_cast<QWidget*>(obj);
		if(plot)
		{
			_figures.remove(plot->windowTitle());
			QList<QAction*> actions = _plotMenu->actions();
			for(int i=0; i<actions.size(); ++i)
			{
				if(actions.at(i)->text().compare(plot->windowTitle()) == 0)
				{
					_plotMenu->removeAction(actions.at(i));
					delete actions[i];
					Q_EMIT menuChanged(_plotMenu);
					break;
				}
			}
			Q_EMIT figuresSetupChanged();
		}
		else
		{
			UERROR("");
		}
	}
	else
	{
		UERROR("");
	}
}

void StatsToolBox::clear()
{
	for (QMap<QString, QWidget*>::iterator i = _figures.begin(); i != _figures.end(); ++i)
	{
		QList<UPlot *> plots = i.value()->findChildren<UPlot *>();
		if (plots.size() == 1)
		{
			QStringList names = plots[0]->curveNames();
			plots[0]->clearData();
		}
		else
		{
			UERROR("");
		}
	}
	QList<StatItem*> items = _statBox->currentWidget()->findChildren<StatItem*>();
	for (int i = 0; i<items.size(); ++i)
	{
		items[i]->clearCache();
	}
}

void StatsToolBox::contextMenuEvent(QContextMenuEvent * event)
{
	QMenu topMenu(this);
	QMenu * menu = topMenu.addMenu(tr("Add all statistics from tab \"%1\" to...").arg(_statBox->itemText(_statBox->currentIndex())));
	QList<QAction* > actions = _plotMenu->actions();
	menu->addActions(actions);
	QAction * aClearFigures = topMenu.addAction(tr("Clear all figures"));
	QAction * action = topMenu.exec(event->globalPos());
	QString plotName;
	if(action)
	{
		if(action == aClearFigures)
		{
			this->clear();
		}
		else
		{
			for(int i=0; i<actions.size(); ++i)
			{
				if(actions.at(i) == action)
				{
					plotName = actions.at(i)->text();
					break;
				}
			}
		}
	}

	if(!plotName.isEmpty())
	{
		QList<StatItem*> items = _statBox->currentWidget()->findChildren<StatItem*>();
		for(int i=0; i<items.size(); ++i)
		{
			this->plot(items.at(i), plotName);
			if(plotName.compare(tr("<New figure>")) == 0)
			{
				plotName = _plotMenu->actions().last()->text();
			}
		}
	}
}

void StatsToolBox::getFiguresSetup(QList<int> & curvesPerFigure, QStringList & curveNames)
{
	curvesPerFigure.clear();
	curveNames.clear();
	for(QMap<QString, QWidget*>::iterator i=_figures.begin(); i!=_figures.end(); ++i)
	{
		QList<UPlot *> plots = i.value()->findChildren<UPlot *>();
		if(plots.size() == 1)
		{
			QStringList names = plots[0]->curveNames();
			curvesPerFigure.append(names.size());
			curveNames.append(names);
		}
		else
		{
			UERROR("");
		}
	}
}
void StatsToolBox::addCurve(const QString & name, bool newFigure, bool cacheOn)
{
	StatItem * item = _statBox->findChild<StatItem *>(name);
	if(!item)
	{
		this->updateStat(name, cacheOn);
		item = _statBox->findChild<StatItem *>(name);
	}

	if(item)
	{
		if(newFigure)
		{
			this->plot(item, "");
		}
		else
		{
			this->plot(item, _plotMenu->actions().last()->text());
		}
	}
	else
	{
		ULOGGER_ERROR("Not supposed to be here...");
	}
}

void StatsToolBox::setWorkingDirectory(const QString & workingDirectory)
{
	if(QDir(workingDirectory).exists())
	{
		_workingDirectory = workingDirectory;
		for(QMap<QString, QWidget*>::iterator i=_figures.begin(); i!=_figures.end(); ++i)
		{
			QList<UPlot *> plots = i.value()->findChildren<UPlot *>();
			if(plots.size() == 1)
			{
				plots[0]->setWorkingDirectory(_workingDirectory);
			}
			else
			{
				UERROR("");
			}
		}
	}
	else
	{
		UWARN("The directory \"%s\" doesn't exist, using \"%s\" instead...",
				workingDirectory.toStdString().c_str(),
				_workingDirectory.toStdString().c_str());
	}
}

}
