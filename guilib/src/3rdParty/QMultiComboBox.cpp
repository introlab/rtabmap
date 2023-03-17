/****************************************************************************
**
** Copyright (c) 2012 Richard Steffen and/or its subsidiary(-ies).
** All rights reserved.
** Contact: rsteffen@messbild.de, rsteffen@uni-bonn.de
**
** QMultiComboBox is free to use unter the terms of the LGPL 2.1 License in
** Free and Commercial Products.
****************************************************************************/

#include "QMultiComboBox.h"
#include <QApplication>
#include <QCoreApplication>
#include <QWindow>
#include <QScreen>

#include "rtabmap/utilite/ULogger.h"

QMultiComboBox::QMultiComboBox(QWidget *widget ) :
    QComboBox(widget),
    popheight_(0),
    screenbound_(50),
    popframe_(NULL, Qt::Popup)
{

    SetDisplayText("Not Set");

    // setup the popup list
    vlist_.setSelectionMode(QAbstractItemView::MultiSelection);
    vlist_.setSelectionBehavior(QAbstractItemView::SelectItems);
    vlist_.clearSelection();
    popframe_.setLayout(new QVBoxLayout());
    popframe_.layout()->addWidget(&vlist_);
    popframe_.layout()->setContentsMargins(0,0,0,0);

    connect(&vlist_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(scanItemSelect(QListWidgetItem*)));

}


QMultiComboBox::~QMultiComboBox()
{
    disconnect(&vlist_,0,0,0);
}


void QMultiComboBox::SetDisplayText(QString text)
{
    m_DisplayText_ = text;
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    const int textWidth = fontMetrics().horizontalAdvance(text);
#else
    const int textWidth = fontMetrics().width(text);
#endif
    setMinimumWidth(textWidth + 30);
    updateGeometry();
    repaint();
}


QString QMultiComboBox::GetDisplayText() const
{
    return m_DisplayText_;
}


void QMultiComboBox::setPopupHeight(int h)
{
    popheight_ = h;
}


void QMultiComboBox::paintEvent(QPaintEvent *e)
{
    QStylePainter painter(this);
    painter.setPen(palette().color(QPalette::Text));
    // draw the combobox frame, focusrect and selected etc.
    QStyleOptionComboBox opt;

    initStyleOption(&opt);
    opt.currentText = m_DisplayText_;
    painter.drawComplexControl(QStyle::CC_ComboBox, opt);
    // draw the icon and text
    painter.drawControl(QStyle::CE_ComboBoxLabel, opt);
}


void QMultiComboBox::showPopup()
{
    QRect rec = QRect(geometry());

    //QPoint p = this->mapToGlobal(QPoint(0,rec.height()));
    //QRect rec2(p , p + QPoint(rec.width(), rec.height()));

    // get the two possible list points and height
    QRect screen = this->window()->windowHandle()->screen()->availableGeometry();
    QPoint above = this->mapToGlobal(QPoint(0,0));
    int aboveHeight = above.y() - screen.y();
    QPoint below = this->mapToGlobal(QPoint(0,rec.height()));
    int belowHeight = screen.bottom() - below.y();

	// compute width
	int textWidth = vlist_.sizeHint().width();

    // first activate it with height 1px to get all the items initialized
    QRect rec2;
    rec2.setTopLeft(below);
    rec2.setWidth(textWidth>rec.width()? textWidth:rec.width());
    rec2.setHeight(rec.height());
    popframe_.setGeometry(rec2);
    popframe_.raise();
    popframe_.show();
    QCoreApplication::processEvents();

    // determine rect
    int contheight = vlist_.count()*vlist_.sizeHintForRow(0) + 4; // +4 - should be determined by margins?
    belowHeight = std::min(abs(belowHeight)-screenbound_, contheight);
    aboveHeight = std::min(abs(aboveHeight)-screenbound_, contheight);
    if (popheight_ > 0) // fixed
    {
        rec2.setHeight(popheight_);
    }
    else // dynamic
    {
        // do we use below or above
        if (belowHeight==contheight || belowHeight>aboveHeight)
        {
            rec2.setTopLeft(below);
            rec2.setHeight(belowHeight);
        }
        else
        {
            rec2.setTopLeft(above - QPoint(0,aboveHeight));
            rec2.setHeight(aboveHeight);
        }
    }
    popframe_.setGeometry(rec2);
    popframe_.raise();
    popframe_.show();
}


void QMultiComboBox::hidePopup()
{
    popframe_.hide();
}


void QMultiComboBox::addItem ( const QString & text, const QVariant & userData)
{
    QListWidgetItem* wi = new QListWidgetItem(text);
    wi->setFlags(wi->flags() | Qt::ItemIsUserCheckable);
    if (userData.toBool())
        wi->setCheckState(Qt::Checked);
    else
        wi->setCheckState(Qt::Unchecked);
    vlist_.addItem(wi);
	vlist_.setMinimumWidth(vlist_.sizeHintForColumn(0));
}


int QMultiComboBox::count()
{
    return vlist_.count();
}


void QMultiComboBox::setCurrentIndex(int index)
{
  //  cout << __FUNCTION__ << "DONT USE THIS ................" << endl;
}


QString QMultiComboBox::currentText()
{
    return vlist_.currentItem()->text();
}


QString QMultiComboBox::itemText(int row)
{
    return vlist_.item(row)->text();
}


QVariant QMultiComboBox::itemData(int row)
{
    QListWidgetItem* item = vlist_.item(row);
    if (item->checkState() == Qt::Checked) return QVariant(true);
    return QVariant(false);
}

void QMultiComboBox::setItemChecked(int row, bool checked)
{
	QListWidgetItem* item = vlist_.item(row);
	bool wasChecked = item->checkState() == Qt::Checked;
	if (wasChecked != checked)
	{
		item->setCheckState(checked?Qt::Checked:Qt::Unchecked);
		Q_EMIT itemChanged();
	}
}


void QMultiComboBox::scanItemSelect(QListWidgetItem* item)
{

    QList<QListWidgetItem*> list = vlist_.selectedItems();
    for (int i = 0; i < list.count(); i++)
    {
        if (item->checkState() == Qt::Checked)
        {
            list[i]->setCheckState(Qt::Checked);
        }
        else
        {
            list[i]->setCheckState(Qt::Unchecked);
        }
        list[i]->setSelected(false);
    }
    Q_EMIT itemChanged();
}

void QMultiComboBox::initStyleOption(QStyleOptionComboBox *option) const
{
    //Initializes the state, direction, rect, palette, and fontMetrics member variables based on the specified widget.
    //This is a convenience function; the member variables can also be initialized manually.
    option->initFrom(this);

}

void QMultiComboBox::clear()
{
	vlist_.clear();
	QComboBox::clear();
}

