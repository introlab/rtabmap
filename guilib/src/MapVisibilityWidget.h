/*
 * MapVisibilityWidget.h
 *
 *  Created on: 2014-01-20
 *      Author: mathieu
 */

#ifndef MAPVISIBILITYWIDGET_H_
#define MAPVISIBILITYWIDGET_H_

#include <QtGui/QWidget>
#include <rtabmap/core/Transform.h>
#include <map>

namespace rtabmap {

class MapVisibilityWidget : public QWidget {
	Q_OBJECT;
public:
	MapVisibilityWidget(QWidget * parent = 0);
	virtual ~MapVisibilityWidget();

	void setMap(const std::map<int, Transform> & poses);

protected:
	virtual void showEvent(QShowEvent * event);

private slots:
	void signalVisibility();

private:
	void updateCheckBoxes();

signals:
	void visibilityChanged(int id, bool visible);

private:
	std::map<int, Transform> _poses;
};

} /* namespace rtabmap */
#endif /* MAPVISIBILITYWIDGET_H_ */
