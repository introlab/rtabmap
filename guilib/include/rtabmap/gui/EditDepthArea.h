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

#ifndef RTABMAP_EDITDEPTHAREA_H
#define RTABMAP_EDITDEPTHAREA_H

#include "rtabmap/gui/rtabmap_gui_export.h" // DLL export/import defines

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QWidget>
#include <opencv2/opencv.hpp>
#include "rtabmap/utilite/UCv2Qt.h"

class QMenu;
class QAction;

namespace rtabmap {

class RTABMAP_GUI_EXPORT EditDepthArea : public QWidget
{
    Q_OBJECT

public:
    EditDepthArea(QWidget *parent = 0);

    void setImage(const cv::Mat & depth, const cv::Mat & rgb = cv::Mat());
    cv::Mat getModifiedImage() const;
    bool isModified() const {return modified_;}

    void setPenWidth(int newWidth);
    int penWidth() const { return myPenWidth_; }
    void setClusterError(double error) { clusterError_ = error; }
    double clusterError() const { return clusterError_; }
    void setColorMap(uCvQtDepthColorMap type);

public Q_SLOTS:
    void resetChanges();

protected:
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void paintEvent(QPaintEvent *event);
    virtual void resizeEvent(QResizeEvent *event);
    virtual void contextMenuEvent(QContextMenuEvent * e);

private:
    void drawLineTo(const QPoint &endPoint);
    void computeScaleOffsets(const QRect & targetRect, float & scale, float & offsetX, float & offsetY) const;

    bool modified_;
    bool scribbling_;
    int myPenWidth_;
    double clusterError_;
    QImage imageRGB_;
    QImage image_;
    cv::Mat originalImage_;
    QPoint lastPoint_;

    QMenu * menu_;
    QAction * showRGB_;
    QAction * removeCluster_;
    QAction * clusterErrorCluster_;
    QAction * resetChanges_;
    QAction * setPenWidth_;
    QAction * colorMapWhiteToBlack_;
	QAction * colorMapBlackToWhite_;
	QAction * colorMapRedToBlue_;
	QAction * colorMapBlueToRed_;
};

}

#endif
