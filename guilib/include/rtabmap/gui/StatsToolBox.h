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

#ifndef RTABMAP_STATSTOOLBOX_H_
#define RTABMAP_STATSTOOLBOX_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QWidget>
#include <QtCore/QMap>

class QToolButton;
class QLabel;
class QMenu;
class QGridLayout;
class QToolBox;

namespace rtabmap {

class RTABMAP_GUI_EXPORT StatItem : public QWidget
{
	Q_OBJECT;

public:
	StatItem(const QString & name, bool cacheOn, const std::vector<qreal> & x, const std::vector<qreal> & y, const QString & unit = QString(), const QMenu * menu = 0, QGridLayout * grid = 0, QWidget * parent = 0);
	virtual ~StatItem();
	void addValue(qreal y);
	void addValue(qreal x, qreal y);
	void setValues(const std::vector<qreal> & x, const std::vector<qreal> & y);
	QString value() const;
	std::vector<qreal> xValues() const {return _x;}
	std::vector<qreal> yValues() const {return _y;}
	void setCacheOn(bool on);
	void clearCache();

public Q_SLOTS:
	void updateMenu(const QMenu * menu);

Q_SIGNALS:
	void valueAdded(qreal);
	void valueAdded(qreal, qreal);
	void valuesChanged(const std::vector<qreal> &, const std::vector<qreal> &);
	void plotRequested(const StatItem *, const QString &);

private Q_SLOTS:
	void preparePlotRequest();

private:
	void setupUi(QGridLayout * grid);

private:
	QToolButton * _button;
	QLabel * _name;
	QLabel * _value;
	QLabel * _unit;
	QMenu * _menu;

	bool _cacheOn;
	std::vector<qreal> _x;
	std::vector<qreal> _y;
};




class RTABMAP_GUI_EXPORT StatsToolBox : public QWidget
{
	Q_OBJECT;

public:
	StatsToolBox(QWidget * parent);
	virtual ~StatsToolBox();

	void getFiguresSetup(QList<int> & curvesPerFigure, QStringList & curveNames);
	void addCurve(const QString & name, bool newFigure = true, bool cacheOn = false);
	void setWorkingDirectory(const QString & workingDirectory);
	void setNewFigureMaxItems(int value) {_newFigureMaxItems = value;}
	void closeFigures();
	void setCacheOn(bool on);

public Q_SLOTS:
	void updateStat(const QString & statFullName, bool cacheOn);
	void updateStat(const QString & statFullName, qreal y, bool cacheOn);
	void updateStat(const QString & statFullName, qreal x, qreal y, bool cacheOn);
	void updateStat(const QString & statFullName, const std::vector<qreal> & x, const std::vector<qreal> & y, bool cacheOn);
	void clear();

Q_SIGNALS:
	void menuChanged(const QMenu *);
	void figuresSetupChanged();

private Q_SLOTS:
	void plot(const StatItem * stat, const QString & plotName = QString());
	void figureDeleted(QObject * obj);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	QMenu * _plotMenu;
	QToolBox * _statBox;
	QString _workingDirectory;
	int _newFigureMaxItems;
	QMap<QString, QWidget*> _figures;
};

}

#endif /* STATSTOOLBOX_H_ */
