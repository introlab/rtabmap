/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UPLOT_H_
#define UPLOT_H_

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QFrame>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QGraphicsEllipseItem>
#include <QtCore/QMutex>
#include <QLabel>
#include <QPushButton>
#include <QtCore/QTime>
#include <QtCore/QElapsedTimer>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QFormLayout;
class QScrollArea;

/**
 * UPlotItem is a QGraphicsEllipseItem and can be inherited to do custom behaviors
 * on an hoverEnterEvent() for example.
 */
class RTABMAP_GUI_EXPORT UPlotItem : public QGraphicsEllipseItem
{
public:
	/**
	 * Constructor 1.
	 */
	UPlotItem(qreal dataX, qreal dataY, qreal width=2);
	/**
	 * Constructor 2.
	 */
	UPlotItem(const QPointF & data, qreal width=2);
	virtual ~UPlotItem();

public:
	void setNextItem(UPlotItem * nextItem);
	void setPreviousItem(UPlotItem * previousItem);
	void setData(const QPointF & data);

	UPlotItem * nextItem() const {return _nextItem;}
	UPlotItem * previousItem() const {return _previousItem;};
	const QPointF & data() const {return _data;}

protected:
	virtual void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
	virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);
	virtual void focusInEvent(QFocusEvent * event);
	virtual void focusOutEvent(QFocusEvent * event);
	virtual void keyReleaseEvent(QKeyEvent * keyEvent);

	virtual void showDescription(bool shown);
private:
	void init(qreal dataX, qreal dataY);

private:
	QPointF _data;
	UPlotItem * _previousItem;
	UPlotItem * _nextItem;
	QGraphicsTextItem * _text;
	QGraphicsRectItem * _textBackground;
};

class UPlot;

/**
 * UPlotCurve is a curve used to hold data shown in a UPlot.
 */
class RTABMAP_GUI_EXPORT UPlotCurve : public QObject
{
	Q_OBJECT

public:
	/**
	 * Constructor 1
	 */
	UPlotCurve(const QString & name, QObject * parent = 0);
	/**
	 * Constructor 2
	 */
	UPlotCurve(const QString & name, const QVector<UPlotItem *> data, QObject * parent = 0);
	/**
	 * Constructor 3
	 */
	UPlotCurve(const QString & name, const QVector<qreal> & x, const QVector<qreal> & y, QObject * parent = 0);
	virtual ~UPlotCurve();

	/**
	 * Get pen.
	 */
	const QPen & pen() const {return _pen;}
	/**
	 * Get brush.
	 */
	const QBrush & brush() const {return _brush;}

	/**
	 * Set pen.
	 */
	void setPen(const QPen & pen);
	/**
	 * Set brush.
	 */
	void setBrush(const QBrush & brush);

	void setItemsColor(const QColor & color);
	QColor itemsColor() const  {return _itemsColor;}

	/**
	 * Get name.
	 */
	QString name() const {return _name;}
	/**
	 * Get the number of items in the curve (dot + line items).
	 */
	int itemsSize() const;
	QPointF getItemData(int index);
	bool isVisible() const {return _visible;}
	void setData(QVector<UPlotItem*> & data); // take the ownership
	void getData(QVector<qreal> & x, QVector<qreal> & y) const; // only call in Qt MainThread
	void getData(QMap<qreal,qreal> & data) const; // only call in Qt MainThread
	void draw(QPainter * painter, const QRect & limits);

public Q_SLOTS:
	/**
	 *
	 * Clear curve's values.
	 */
	virtual void clear();
	/**
	 *
	 * Show or hide the curve.
	 */
    void setVisible(bool visible);
    /**
     *
	 * Set increment of the x values (when auto-increment is used).
	 */
    void setXIncrement(qreal increment);
    /**
     *
	 * Set starting x value (when auto-increment is used).
	 */
    void setXStart(qreal val);
    /**
     *
	 * Add a single value, using a custom UPlotItem.
	 */
	void addValue(UPlotItem * data); // take the ownership
	/**
	 *
	 * Add a single value y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValue(qreal y);
	/**
	 *
	 * Add a single value y at x.
	 */
	void addValue(qreal x, qreal y);
	/**
	 *
	 * For convenience...
	 * Add a single value y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValue(const QString & y);
	/**
	 *
	 * For convenience...
	 * Add multiple values, using custom UPlotItem.
	 */
	void addValues(QVector<UPlotItem *> & data); // take the ownership
	/**
	 *
	 * Add multiple values y at x. Vectors must have the same size.
	 */
	void addValues(const QVector<qreal> & xs, const QVector<qreal> & ys);
	/**
	 *
	 * Add multiple values y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValues(const QVector<qreal> & ys);
	void addValues(const QVector<int> & ys); // for convenience
	/**
	 *
	 * Add multiple values y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValues(const std::vector<qreal> & ys); // for convenience
	void addValues(const std::vector<int> & ys); // for convenience

	void setData(const QVector<qreal> & x, const QVector<qreal> & y);
	void setData(const std::vector<qreal> & x, const std::vector<qreal> & y);
	void setData(const QVector<qreal> & y);
	void setData(const std::vector<qreal> & y);

Q_SIGNALS:
	/**
	 *
	 *  emitted when data is changed.
	 */
	void dataChanged(const UPlotCurve *);

protected:
	friend class UPlot;
	void attach(UPlot * plot);
	void detach(UPlot * plot);
	void updateMinMax();
	const QVector<qreal> & getMinMax() const {return _minMax;}
	int removeItem(int index);
	void _addValue(UPlotItem * data);;
	virtual bool isMinMaxValid() const {return _minMax.size();}
	virtual void update(qreal scaleX, qreal scaleY, qreal offsetX, qreal offsetY, qreal xDir, qreal yDir, int maxItemsKept);
	QList<QGraphicsItem *> _items;
	UPlot * _plot;

private:
	void removeItem(UPlotItem * item);

private:
	QString _name;
	QPen _pen;
	QBrush _brush;
	qreal _xIncrement;
	qreal _xStart;
	bool _visible;
	bool _valuesShown;
	QVector<qreal> _minMax; // minX, maxX, minY, maxY
	QGraphicsRectItem * _rootItem;
	QColor _itemsColor;
};


/**
 * A special UPlotCurve that shows as a line at the specified value, spanning all the UPlot.
 */
class RTABMAP_GUI_EXPORT UPlotCurveThreshold : public UPlotCurve
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotCurveThreshold(const QString & name, qreal thesholdValue, Qt::Orientation orientation = Qt::Horizontal, QObject * parent = 0);
	virtual ~UPlotCurveThreshold();

public Q_SLOTS:
	/**
	 * Set threshold value.
	 */
	void setThreshold(qreal threshold);
	/**
	 * Set orientation (Qt::Horizontal or Qt::Vertical).
	 */
	void setOrientation(Qt::Orientation orientation);

protected:
	friend class UPlot;
	virtual void update(qreal scaleX, qreal scaleY, qreal offsetX, qreal offsetY, qreal xDir, qreal yDir, int maxItemsKept);
	virtual bool isMinMaxValid() const {return false;}

private:
	Qt::Orientation _orientation;
};

/**
 * The UPlot axis object.
 */
class RTABMAP_GUI_EXPORT UPlotAxis : public QWidget
{
public:
	/**
	 * Constructor.
	 */
	UPlotAxis(Qt::Orientation orientation = Qt::Horizontal, qreal min=0, qreal max=1, QWidget * parent = 0);
	virtual ~UPlotAxis();

public:
	/**
	 * Set axis minimum and maximum values, compute the resulting
	 * intervals depending on the size of the axis.
	 */
	void setAxis(qreal & min, qreal & max);
	/**
	 * Size of the border between the first line and the beginning of the widget.
	 */
	int border() const {return _border;}
	/**
	 * Interval step value.
	 */
	int step() const {return _step;}
	/**
	 * Number of intervals.
	 */
	int count() const {return _count;}
	/**
	 * Reverse the axis (for vertical :bottom->up, for horizontal :right->left)
	 */
	void setReversed(bool reversed); // Vertical :bottom->up, horizontal :right->left

protected:
	virtual void paintEvent(QPaintEvent * event);

private:
	Qt::Orientation _orientation;
	qreal _min;
	qreal _max;
	int _count;
	int _step;
	bool _reversed;
	int _gradMaxDigits;
	int _border;
};


/**
 * The UPlot legend item. Used internally by UPlot.
 */
class RTABMAP_GUI_EXPORT UPlotLegendItem : public QPushButton
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotLegendItem(UPlotCurve * curve, QWidget * parent = 0);
	virtual ~UPlotLegendItem();
	const UPlotCurve * curve() const {return _curve;}
	QPixmap createSymbol(const QPen & pen, const QBrush & brush);
	void showStdDevMeanMax(bool shown);

Q_SIGNALS:
	void legendItemRemoved(const UPlotCurve *);
	void moveUpRequest(UPlotLegendItem *);
	void moveDownRequest(UPlotLegendItem *);

private Q_SLOTS:
	void updateStdDevMeanMax();

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	UPlotCurve * _curve;
	QMenu * _menu;
	QAction * _aChangeText;
	QAction * _aResetText;
	QAction * _aChangeColor;
	QAction * _aCopyToClipboard;
	QAction * _aShowStdDevMeanMax;
	QAction * _aRemoveCurve;
	QAction * _aMoveUp;
	QAction * _aMoveDown;
};

/**
 * The UPlot legend. Used internally by UPlot.
 */
class RTABMAP_GUI_EXPORT UPlotLegend : public QWidget
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotLegend(QWidget * parent = 0);
	virtual ~UPlotLegend();

	void setFlat(bool on);
	bool isFlat() const {return _flat;}
	void addItem(UPlotCurve * curve);
	bool remove(const UPlotCurve * curve);
	QString getAllCurveDataAsText() const;

private Q_SLOTS:
	void removeLegendItem(const UPlotCurve * curve);
	void moveUp(UPlotLegendItem * item);
	void moveDown(UPlotLegendItem * item);

Q_SIGNALS:
	void legendItemRemoved(const UPlotCurve * curve);
	void legendItemToggled(const UPlotCurve * curve, bool toggled);
	void legendItemMoved(const UPlotCurve * curve, int);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private Q_SLOTS:
	void redirectToggled(bool);

private:
	bool _flat;
	QMenu * _menu;
	QAction * _aUseFlatButtons;
	QAction * _aCopyAllCurvesToClipboard;
	QAction * _aShowAllStdDevMeanMax;
	QLayout * _contentLayout;
	QScrollArea * _scrollArea;
};


/**
 * Orientable QLabel. Inherit QLabel and let you to specify the orientation.
 */
class RTABMAP_GUI_EXPORT UOrientableLabel : public QLabel
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UOrientableLabel(const QString & text, Qt::Orientation orientation = Qt::Horizontal, QWidget * parent = 0);
	virtual ~UOrientableLabel();
	/**
	 * Get orientation.
	 */
	Qt::Orientation orientation() const {return _orientation;}
	/**
	 * Set orientation (Qt::Vertical or Qt::Horizontal).
	 */
	void setOrientation(Qt::Orientation orientation);
	QSize sizeHint() const;
	QSize minimumSizeHint() const;
protected:
    virtual void paintEvent(QPaintEvent* event);
private:
    Qt::Orientation _orientation;
};

/**
 * UPlot is a QWidget to create a plot like MATLAB, and
 * incrementally add new values like a scope using Qt signals/slots.
 * Many customizations can be done at runtime with the right-click menu.
 * @image html UPlot.gif
 * @image html UPlotMenu.png
 *
 * Example:
 * @code
 * #include "utilite/UPlot.h"
 * #include <QApplication>
 *
 * int main(int argc, char * argv[])
 * {
 *	QApplication app(argc, argv);
 *	UPlot plot;
 *	UPlotCurve * curve = plot.addCurve("My curve");
 *	qreal y[10] = {0, 1, 2, 3, -3, -2, -1, 0, 1, 2};
 *	curve->addValues(std::vector<qreal>(y, y+10));
 *	plot.showGrid(true);
 *	plot.setGraphicsView(true);
 *	plot.show();
 *	app.exec();
 * 	return 0;
 * }
 * @endcode
 * @image html SimplePlot.tiff
 *
 *
 */
class RTABMAP_GUI_EXPORT UPlot : public QWidget
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlot(QWidget * parent = 0);
	virtual ~UPlot();

	/**
	 * Add a curve. The returned curve doesn't need to be deallocated (UPlot keeps the ownership).
	 */
	UPlotCurve * addCurve(const QString & curveName, const QColor & color = QColor());
	/**
	 * Add a curve. Ownership is transferred to UPlot if ownershipTransferred=true.
	 */
	bool addCurve(UPlotCurve * curve, bool ownershipTransferred = true);
	/**
	 * Get all curve names.
	 */
	QStringList curveNames();
	bool contains(const QString & curveName);
	void removeCurves();
	QString getAllCurveDataAsText() const;
	/**
	 * Add a threshold to the plot.
	 */
	UPlotCurveThreshold * addThreshold(const QString & name, qreal value, Qt::Orientation orientation = Qt::Horizontal);
	QString title() const {return this->objectName();}
	QPen getRandomPenColored();
	void showLegend(bool shown);
	void showGrid(bool shown);
	void showRefreshRate(bool shown);
	void trackMouse(bool tracking);
	void keepAllData(bool kept);
	void showXAxis(bool shown) {_horizontalAxis->setVisible(shown);}
	void showYAxis(bool shown) {_verticalAxis->setVisible(shown);}
	void setVariableXAxis() {_fixedAxis[0] = false;}
	void setVariableYAxis() {_fixedAxis[1] = false;}
	void setFixedXAxis(qreal x1, qreal x2);
	void setFixedYAxis(qreal y1, qreal y2);
	void setMaxVisibleItems(int maxVisibleItems);
	void setTitle(const QString & text);
	void setXLabel(const QString & text);
	void setYLabel(const QString & text, Qt::Orientation orientation = Qt::Vertical);
	void setWorkingDirectory(const QString & workingDirectory);
	void setGraphicsView(bool on);
	void setBackgroundColor(const QColor & color);
	QRectF sceneRect() const;

public Q_SLOTS:
	/**
	 *
	 * Remove a curve. If UPlot is the parent of the curve, the curve is deleted.
	 */
	void removeCurve(const UPlotCurve * curve);
	void showCurve(const UPlotCurve * curve, bool shown);
	void updateAxis(); //reset axis and recompute it with all curves minMax
	/**
	 *
	 * Clear all curves' data.
	 */
	void clearData();

	void frameData(bool xAxis = true, bool yAxis = false);

private Q_SLOTS:
	void captureScreen();
	void updateAxis(const UPlotCurve * curve);
	void moveCurve(const UPlotCurve *, int index);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void paintEvent(QPaintEvent * event);
	virtual void resizeEvent(QResizeEvent * event);
	virtual void mousePressEvent(QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent * event);
	virtual void mouseReleaseEvent(QMouseEvent * event);
	virtual void mouseDoubleClickEvent(QMouseEvent * event);

private:
	friend class UPlotCurve;
	void addItem(QGraphicsItem * item);

private:
	void replot(QPainter * painter);
	bool updateAxis(qreal x, qreal y);
	bool updateAxis(qreal x1, qreal x2, qreal y1, qreal y2);
	void setupUi();
	void createActions();
	void createMenus();
	void selectScreenCaptureFormat();
	bool mousePosToValue(const QPoint & pos, qreal & x, qreal & y);

private:
	UPlotLegend * _legend;
	QGraphicsView * _view;
	QGraphicsItem * _sceneRoot;
	QWidget * _graphicsViewHolder;
	qreal _axisMaximums[4]; // {x1->x2, y1->y2}
	bool _axisMaximumsSet[4]; // {x1->x2, y1->y2}
	bool _fixedAxis[2];
	UPlotAxis * _verticalAxis;
	UPlotAxis * _horizontalAxis;
	int _penStyleCount;
	int _maxVisibleItems;
	QList<QGraphicsLineItem *> hGridLines;
	QList<QGraphicsLineItem *> vGridLines;
	QList<UPlotCurve*> _curves;
	QLabel * _title;
	QLabel * _xLabel;
	UOrientableLabel * _yLabel;
	QLabel * _refreshRate;
	QString _workingDirectory;
	QElapsedTimer _refreshIntervalTime;
	int _lowestRefreshRate;
	QElapsedTimer _refreshStartTime;
	QString _autoScreenCaptureFormat;
	QPoint _mousePressedPos;
	QPoint _mouseCurrentPos;
	QColor _bgColor;

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
	QAction * _aChangeBackgroundColor;
	QAction * _aYLabelVertical;
	QAction * _aShowRefreshRate;
	QAction * _aMouseTracking;
	QAction * _aSaveFigure;
	QAction * _aAutoScreenCapture;
	QAction * _aClearData;
	QAction * _aGraphicsView;
};

#endif /* UPLOT_H_ */
