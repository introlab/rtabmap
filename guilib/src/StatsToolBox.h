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

#ifndef STATSTOOLBOX_H_
#define STATSTOOLBOX_H_

#include <QtGui/QWidget>
#include <QtCore/QMap>

class QToolButton;
class QLabel;
class QMenu;
class QGridLayout;
class QToolBox;

namespace rtabmap {

class StatItem : public QWidget
{
	Q_OBJECT;

public:
	StatItem(const QString & name, const std::vector<float> & x, const std::vector<float> & y, const QString & unit = QString(), const QMenu * menu = 0, QGridLayout * grid = 0, QWidget * parent = 0);
	virtual ~StatItem();
	void setValue(float x, float y);
	void setValues(const std::vector<float> & x, const std::vector<float> & y);
	QString value() const;

public slots:
	void updateMenu(const QMenu * menu);

signals:
	void valueChanged(float, float);
	void valuesChanged(const std::vector<float> &, const std::vector<float> &);
	void plotRequested(const StatItem *, const QString &);

private slots:
	void preparePlotRequest();

private:
	void setupUi(QGridLayout * grid);

private:
	QToolButton * _button;
	QLabel * _name;
	QLabel * _value;
	QLabel * _unit;
	QMenu * _menu;
};




class StatsToolBox : public QWidget
{
	Q_OBJECT;

public:
	StatsToolBox(QWidget * parent);
	virtual ~StatsToolBox();

	void getFiguresSetup(QList<int> & curvesPerFigure, QStringList & curveNames);
	void addCurve(const QString & name, bool newFigure = true);
	void setWorkingDirectory(const QString & workingDirectory);
	void closeFigures();

public slots:
	void updateStat(const QString & statFullName, float x, float y);
	void updateStat(const QString & statFullName, const std::vector<float> & x, const std::vector<float> & y);

signals:
	void menuChanged(const QMenu *);

private slots:
	void plot(const StatItem * stat, const QString & plotName = QString());
	void figureDeleted(QObject * obj);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	QMenu * _plotMenu;
	QToolBox * _statBox;
	QString _workingDirectory;
	QMap<QString, QWidget*> _figures;
};

}

#endif /* STATSTOOLBOX_H_ */
