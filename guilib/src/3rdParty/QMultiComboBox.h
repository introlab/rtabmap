/****************************************************************************
**
** Copyright (C) 2010 Richard Steffen and/or its subsidiary(-ies).
** All rights reserved.
** Contact: rsteffen@messbild.de, rsteffen@uni-bonn.de
**
** Observe the License Information
**
****************************************************************************/

#ifndef __MULTIBOXCOMBO_H__
#define __MULTIBOXCOMBO_H__

#include <iostream>

#include <QComboBox>
#include <QListWidget>
#include <QVBoxLayout>
#include <QStylePainter>

class QMultiComboBox: public QComboBox
{
    Q_OBJECT
public:

    /// Constructor
    QMultiComboBox(QWidget *widget = 0);

    virtual ~QMultiComboBox();

    /// the main display text
    void SetDisplayText(QString text);

    /// get the main display text
    QString GetDisplayText() const;

    /// add a item to the list
    void addItem(const QString& text, const QVariant& userData = QVariant());

    /// custom paint
    virtual void paintEvent(QPaintEvent *e);

    /// set the height of the popup
    void setPopupHeight(int h);

    /// replace standard QComboBox Popup
    void showPopup();
    void hidePopup();

    /// replace neccessary data access
    int count();
    void setCurrentIndex(int index);
    QString currentText();
    QString itemText(int row);
    QVariant itemData(int row);
    void setItemChecked(int row, bool checked);

Q_SIGNALS:
    /// item changed
    void itemChanged();

public Q_SLOTS:

    /// react on changes of the item checkbox
    void scanItemSelect(QListWidgetItem* item);

    /// the init style
    void initStyleOption(QStyleOptionComboBox *option) const;

    void clear();

protected:

    /// the height of the popup
    int popheight_;

    /// lower/upper screen bound
    int screenbound_;

    /// hold the main display text
    QString m_DisplayText_;

    /// popup frame
    QFrame popframe_;

    /// multi selection list in the popup frame
    QListWidget vlist_;

};

#endif
