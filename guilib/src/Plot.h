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

#ifndef PLOT_H_
#define PLOT_H_

// Uncomment this to disable all references to RTAB-Map and UtiLite dependencies
//#define PLOT_WIDGET_OUT_OF_LIB

#include <QtGui/QFrame>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsEllipseItem>
#include <QtCore/QMutex>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtCore/QTime>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QFormLayout;

#ifndef PLOT_WIDGET_OUT_OF_LIB
namespace rtabmap {
#endif

class PlotItem : public QGraphicsEllipseItem
{
public:
	PlotItem(qreal dataX, qreal dataY, qreal width=2);
	PlotItem(const QPointF & data, qreal width=2);
	virtual ~PlotItem();

public:
	void setNextItem(PlotItem * nextItem);
	void setPreviousItem(PlotItem * previousItem);
	void setData(const QPointF & data);

	PlotItem * nextItem() const {return _nextItem;}
	PlotItem * previousItem() const {return _previousItem;};
	const QPointF & data() const {return _data;}

protected:
	virtual void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
	virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);
	virtual void focusInEvent(QFocusEvent * event);
	virtual void focusOutEvent(QFocusEvent * event);
	virtual void keyReleaseEvent(QKeyEvent * keyEvent);

	virtual void showDescription(bool shown);

private:
	QPointF _data;
	QGraphicsTextItem * _text;
	PlotItem * _previousItem;
	PlotItem * _nextItem;
};

class Plot;

class PlotCurve : public QObject
{
	Q_OBJECT

public:
	PlotCurve(const QString & name, QObject * parent = 0);
	PlotCurve(const QString & name, const QVector<PlotItem *> data, QObject * parent = 0);
	PlotCurve(const QString & name, const QVector<float> & x, const QVector<float> & y, QObject * parent = 0);
	virtual ~PlotCurve();

	const QPen & pen() const {return _pen;}
	const QBrush & brush() const {return _brush;}
	void setPen(const QPen & pen);
	void setBrush(const QBrush & brush);

	void setDefaultStepX(float stepX) {_defaultStepX = stepX;}
	QString name() const {return _name;}
	int itemsSize();
	QPointF getItemData(int index);
	void setStartX(float startX) {_startX = startX;}
	bool isVisible() const {return _visible;}
	void setData(QVector<PlotItem*> & data); // take the ownership
	void setData(const QVector<float> & x, const QVector<float> & y);
	void getData(QVector<float> & x, QVector<float> & y) const; // only call in Qt MainThread

public slots:
	virtual void clear();
    void setVisible(bool visible);
	void addValue(PlotItem * data); // take the ownership
	void addValue(float y);
	void addValue(float x, float y);
	void addValue(const QString & y);
	void addValues(QVector<PlotItem *> & data); // take the ownership
	void addValues(const QVector<float> & xs, const QVector<float> & ys);
	void addValues(const QVector<float> & ys);

signals:
	void dataChanged(const PlotCurve *);

protected:
	friend class Plot;
	void attach(Plot * plot);
	void detach(Plot * plot);
	void updateMinMax();
	const QVector<float> & getMinMax() const {return _minMax;}
	int removeItem(int index);
	void _addValue(PlotItem * data);;
	virtual bool isMinMaxValid() const {return _minMax.size();}
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, int xDir, int yDir, bool allDataKept);
	QList<QGraphicsItem *> _items;
	Plot * _plot;

private:
	void removeItem(PlotItem * item);

private:
	QString _name;
	QPen _pen;
	QBrush _brush;
	float _defaultStepX;
	float _startX;
	bool _visible;
	bool _valuesShown;
	QVector<float> _minMax; // minX, maxX, minY, maxY
};


class ThresholdCurve : public PlotCurve
{
	Q_OBJECT

public:
	ThresholdCurve(const QString & name, float thesholdValue, Qt::Orientation orientation = Qt::Horizontal, QObject * parent = 0);
	~ThresholdCurve();

public slots:
	void setThreshold(float threshold);
	void setOrientation(Qt::Orientation orientation);

protected:
	friend class Plot;
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, int xDir, int yDir, bool allDataKept);
	virtual bool isMinMaxValid() const {return false;}

private:
	Qt::Orientation _orientation;
};


class PlotAxis : public QWidget
{
public:
	PlotAxis(Qt::Orientation orientation = Qt::Horizontal, float min=0, float max=1, QWidget * parent = 0);
	virtual ~PlotAxis();

public:
	void setAxis(float & min, float & max);
	int border() const {return _border;}
	int step() const {return _step;}
	int count() const {return _count;}
	void setReversed(bool reversed); // Vertical :bottom->up, horizontal :right->left

protected:
	virtual void paintEvent(QPaintEvent * event);

private:
	Qt::Orientation _orientation;
	float _min;
	float _max;
	int _count;
	int _step;
	bool _reversed;
	int _gradMaxDigits;
	int _border;
};


class PlotLegendItem : public QPushButton
{
	Q_OBJECT

public:
	PlotLegendItem(const PlotCurve * curve, QWidget * parent = 0);
	virtual ~PlotLegendItem();
	const PlotCurve * curve() const {return _curve;}

signals:
	void legendItemRemoved(const PlotCurve *);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	const PlotCurve * _curve;
	QMenu * _menu;
	QAction * _aChangeText;
	QAction * _aResetText;
	QAction * _aRemoveCurve;
	QAction * _aCopyToClipboard;
};


class PlotLegend : public QWidget
{
	Q_OBJECT

public:
	PlotLegend(QWidget * parent = 0);
	virtual ~PlotLegend();

	void setFlat(bool on);
	bool isFlat() const {return _flat;}
	void addItem(const PlotCurve * curve);
	QPixmap createSymbol(const QPen & pen, const QBrush & brush);

public slots:
	void removeLegendItem(const PlotCurve * curve);

signals:
	void legendItemRemoved(const PlotCurve * curve);
	void legendItemToggled(const PlotCurve * curve, bool toggled);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private slots:
	void redirectToggled(bool);

private:
	bool _flat;
	QMenu * _menu;
	QAction * _aUseFlatButtons;
};


class OrientableLabel : public QLabel
{
	Q_OBJECT

public:
	OrientableLabel(const QString & text, Qt::Orientation orientation = Qt::Horizontal, QWidget * parent = 0);
	virtual ~OrientableLabel();
	Qt::Orientation orientation() const {return _orientation;}
	void setOrientation(Qt::Orientation orientation);
	QSize sizeHint() const;
	QSize minimumSizeHint() const;
protected:
    virtual void paintEvent(QPaintEvent* event);
private:
    Qt::Orientation _orientation;
};

/*
 * TODO It could be cool to right-click on a dot in the
 * plot to create a new plot to monitor the dot value changes.
 */
class Plot : public QWidget
{
	Q_OBJECT

public:
	Plot(QWidget * parent = 0);
	virtual ~Plot();

	PlotCurve * addCurve(const QString & curveName);
	bool addCurve(PlotCurve * curve);
	QStringList curveNames();
	bool contains(const QString & curveName);
	void removeCurves();
	ThresholdCurve * addThreshold(const QString & name, float value, Qt::Orientation orientation = Qt::Horizontal);
	QString title() const {return this->objectName();}
	QPen getRandomPenColored();
	void showLegend(bool shown);
	void showGrid(bool shown);
	void showRefreshRate(bool shown);
	void keepAllData(bool kept);
	void showXAxis(bool shown) {_horizontalAxis->setVisible(shown);}
	void showYAxis(bool shown) {_verticalAxis->setVisible(shown);}
	void setVariableXAxis() {_fixedAxis[0] = false;}
	void setVariableYAxis() {_fixedAxis[1] = false;}
	void setFixedXAxis(float x1, float x2);
	void setFixedYAxis(float y1, float y2);
	void setMaxVisibleItems(int maxVisibleItems);
	void setTitle(const QString & text);
	void setXLabel(const QString & text);
	void setYLabel(const QString & text, Qt::Orientation orientation = Qt::Vertical);
	QGraphicsScene * scene() const;
	void setWorkingDirectory(const QString & workingDirectory);

public slots:
	void removeCurve(const PlotCurve * curve);
	void showCurve(const PlotCurve * curve, bool shown);

private slots:
	void captureScreen();
	void updateAxis(const PlotCurve * curve);
	void updateAxis(); //reset axis and recompute it with all curves minMax

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void paintEvent(QPaintEvent * event);

private:
	void replot();
	bool updateAxis(float x, float y);
	bool updateAxis(float x1, float x2, float y1, float y2);
	void setupUi();
	void createActions();
	void createMenus();
	void selectScreenCaptureFormat();

private:
	PlotLegend * _legend;
	QGraphicsView * _view;
	float _axisMaximums[4]; // {x1->x2, y1->y2}
	bool _axisMaximumsSet[4]; // {x1->x2, y1->y2}
	bool _fixedAxis[2];
	PlotAxis * _verticalAxis;
	PlotAxis * _horizontalAxis;
	int _penStyleCount;
	int _maxVisibleItems;
	QList<QGraphicsLineItem *> hGridLines;
	QList<QGraphicsLineItem *> vGridLines;
	QMap<const PlotCurve*, PlotCurve*> _curves;
	QLabel * _title;
	QLabel * _xLabel;
	OrientableLabel * _yLabel;
	QLabel * _refreshRate;
	QString _workingDirectory;
	QTime _refreshIntervalTime;
	int _lowestRefreshRate;
	QTime _refreshStartTime;
	QString _autoScreenCaptureFormat;

	QMenu * _menu;
	QAction * _aShowLegend;
	QAction * _aShowGrid;
	QAction * _aKeepAllData;
	QAction * _aLimit0;
	QAction * _aLimit10;
	QAction * _aLimit50;
	QAction * _aLimit100;
	QAction * _aLimit500;
	QAction * _aLimit1000;
	QAction * _aLimitCustom;
	QAction * _aAddVerticalLine;
	QAction * _aAddHorizontalLine;
	QAction * _aChangeTitle;
	QAction * _aChangeXLabel;
	QAction * _aChangeYLabel;
	QAction * _aYLabelVertical;
	QAction * _aShowRefreshRate;
	QAction * _aSaveFigure;
	QAction * _aAutoScreenCapture;
	QAction * _aClearData;
};

#ifndef PLOT_WIDGET_OUT_OF_LIB
}
#endif

#endif /* PLOT_H_ */
