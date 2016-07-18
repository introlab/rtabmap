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

#ifndef STATSTOOLBOX_H_
#define STATSTOOLBOX_H_

#include "rtabmap/gui/RtabmapGuiExp.h" // DLL export/import defines

#include <QWidget>
#include <QtCore/QMap>

class QToolButton;
class QLabel;
class QMenu;
class QGridLayout;
class QToolBox;

namespace rtabmap {

class RTABMAPGUI_EXP StatItem : public QWidget
{
	Q_OBJECT;

public:
	StatItem(const QString & name, const std::vector<float> & x, const std::vector<float> & y, const QString & unit = QString(), const QMenu * menu = 0, QGridLayout * grid = 0, QWidget * parent = 0);
	virtual ~StatItem();
	void addValue(float y);
	void addValue(float x, float y);
	void setValues(const std::vector<float> & x, const std::vector<float> & y);
	QString value() const;

public slots:
	void updateMenu(const QMenu * menu);

signals:
	void valueAdded(float);
	void valueAdded(float, float);
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




class RTABMAPGUI_EXP StatsToolBox : public QWidget
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
	void updateStat(const QString & statFullName, float y);
	void updateStat(const QString & statFullName, float x, float y);
	void updateStat(const QString & statFullName, const std::vector<float> & x, const std::vector<float> & y);

signals:
	void menuChanged(const QMenu *);
	void figuresSetupChanged();

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
