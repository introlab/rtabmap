/*
Backward-compatible OpenCV 5 shim for rtabmap.

OpenCV 5 removed the legacy 1.x C API headers and the CV_ constant aliases they
provided. This header keeps rtabmap's existing CV_ usage working on OpenCV 5
without touching the OpenCV 2/3/4 code paths: on OpenCV < 5 it just pulls the
legacy C API headers, on OpenCV >= 5 it maps the CV_ aliases used by rtabmap to
their cv:: enum equivalents.
*/
#ifndef RTABMAP_CORE_OPENCV2_COMPAT_H_
#define RTABMAP_CORE_OPENCV2_COMPAT_H_

#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION < 5
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d_c.h>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/videoio/videoio_c.h> // videoio module only exists since OpenCV 3
#endif
#else
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

// Color conversions
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY
#define CV_RGB2GRAY cv::COLOR_RGB2GRAY
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_BGR2YCrCb cv::COLOR_BGR2YCrCb
#define CV_YCrCb2BGR cv::COLOR_YCrCb2BGR
#define CV_RGB2BGR cv::COLOR_RGB2BGR
#define CV_BGRA2BGR cv::COLOR_BGRA2BGR
#define CV_BGRA2GRAY cv::COLOR_BGRA2GRAY
#define CV_RGBA2BGR cv::COLOR_RGBA2BGR
#define CV_RGBA2GRAY cv::COLOR_RGBA2GRAY
#define CV_RGBA2RGB cv::COLOR_RGBA2RGB
#define CV_BayerBG2BGR cv::COLOR_BayerBG2BGR
#define CV_BayerRG2BGR cv::COLOR_BayerRG2BGR
#define CV_BayerRG2GRAY cv::COLOR_BayerRG2GRAY

// VideoCapture properties
#define CV_CAP_PROP_FRAME_WIDTH cv::CAP_PROP_FRAME_WIDTH
#define CV_CAP_PROP_FRAME_HEIGHT cv::CAP_PROP_FRAME_HEIGHT
#define CV_CAP_PROP_FPS cv::CAP_PROP_FPS
#define CV_CAP_PROP_FOURCC cv::CAP_PROP_FOURCC
#define CV_CAP_PROP_GUID cv::CAP_PROP_GUID
#define CV_CAP_PROP_CONVERT_RGB cv::CAP_PROP_CONVERT_RGB

// Chessboard detection flags
#define CV_CALIB_CB_ADAPTIVE_THRESH cv::CALIB_CB_ADAPTIVE_THRESH
#define CV_CALIB_CB_NORMALIZE_IMAGE cv::CALIB_CB_NORMALIZE_IMAGE

// solvePnP methods
#define CV_ITERATIVE cv::SOLVEPNP_ITERATIVE
#define CV_EPNP cv::SOLVEPNP_EPNP
#define CV_P3P cv::SOLVEPNP_P3P

// Misc
#define CV_TERMCRIT_ITER cv::TermCriteria::MAX_ITER
#define CV_TERMCRIT_EPS cv::TermCriteria::EPS
#define CV_INTER_CUBIC cv::INTER_CUBIC
#define CV_WINDOW_AUTOSIZE cv::WINDOW_AUTOSIZE
#define CV_SORT_DESCENDING cv::SORT_DESCENDING
#define CV_PCA_DATA_AS_ROW cv::PCA::DATA_AS_ROW
#define CV_L2 cv::NORM_L2
#endif

#endif // RTABMAP_CORE_OPENCV2_COMPAT_H_
