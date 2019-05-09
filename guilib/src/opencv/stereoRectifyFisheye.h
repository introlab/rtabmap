/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

This code is the same has cv::stereoRectify() but accepting fisheye distortion model:
All cvUndistortPoints() have been replaced by cv::fisheye::undistortPoints()
See https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp

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

#ifndef GUILIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_
#define GUILIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_

#include <opencv2/calib3d/calib3d.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/calib3d/calib3d_c.h>
#endif

namespace rtabmap
{

void
icvGetRectanglesFisheye( const CvMat* cameraMatrix, const CvMat* distCoeffs,
                 const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
                 cv::Rect_<float>& inner, cv::Rect_<float>& outer )
{
    const int N = 9;
    int x, y, k;
    cv::Mat _pts(1, N*N, CV_32FC2);
    CvPoint2D32f* pts = (CvPoint2D32f*)(_pts.data);

    for( y = k = 0; y < N; y++ )
        for( x = 0; x < N; x++ )
            pts[k++] = cvPoint2D32f((float)x*imgSize.width/(N-1),
                                    (float)y*imgSize.height/(N-1));

    cv::Mat cameraMatrixM(cameraMatrix->rows, cameraMatrix->cols, cameraMatrix->type, cameraMatrix->data.ptr);
    cv::Mat distCoeffsM(distCoeffs->rows, distCoeffs->cols, distCoeffs->type, distCoeffs->data.ptr);
    cv::Mat RM(R->rows, R->cols, R->type, R->data.ptr);
    cv::Mat newCameraMatrixM(newCameraMatrix->rows, newCameraMatrix->cols, newCameraMatrix->type, newCameraMatrix->data.ptr);
    cv::fisheye::undistortPoints(_pts, _pts, cameraMatrixM, distCoeffsM, RM, newCameraMatrixM);
    float iX0=-FLT_MAX, iX1=FLT_MAX, iY0=-FLT_MAX, iY1=FLT_MAX;
    float oX0=FLT_MAX, oX1=-FLT_MAX, oY0=FLT_MAX, oY1=-FLT_MAX;
    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for( y = k = 0; y < N; y++ )
        for( x = 0; x < N; x++ )
        {
            CvPoint2D32f p = pts[k++];
            oX0 = MIN(oX0, p.x);
            oX1 = MAX(oX1, p.x);
            oY0 = MIN(oY0, p.y);
            oY1 = MAX(oY1, p.y);

            if( x == 0 )
                iX0 = MAX(iX0, p.x);
            if( x == N-1 )
                iX1 = MIN(iX1, p.x);
            if( y == 0 )
                iY0 = MAX(iY0, p.y);
            if( y == N-1 )
                iY1 = MIN(iY1, p.y);
        }
    inner = cv::Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = cv::Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
}

void cvStereoRectifyFisheye( const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
                      const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
                      CvSize imageSize, const CvMat* matR, const CvMat* matT,
                      CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
                      CvMat* matQ, int flags, double alpha, CvSize newImgSize )
{
    double _om[3], _t[3] = {0}, _uu[3]={0,0,0}, _r_r[3][3], _pp[3][4];
    double _ww[3], _wr[3][3], _z[3] = {0,0,0}, _ri[3][3], _w3[3];
    cv::Rect_<float> inner1, inner2, outer1, outer2;

    CvMat om  = cvMat(3, 1, CV_64F, _om);
    CvMat t   = cvMat(3, 1, CV_64F, _t);
    CvMat uu  = cvMat(3, 1, CV_64F, _uu);
    CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
    CvMat pp  = cvMat(3, 4, CV_64F, _pp);
    CvMat ww  = cvMat(3, 1, CV_64F, _ww); // temps
    CvMat w3  = cvMat(3, 1, CV_64F, _w3); // temps
    CvMat wR  = cvMat(3, 3, CV_64F, _wr);
    CvMat Z   = cvMat(3, 1, CV_64F, _z);
    CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
    double nx = imageSize.width, ny = imageSize.height;
    int i, k;
    double nt, nw;

    if( matR->rows == 3 && matR->cols == 3 )
        cvRodrigues2(matR, &om);          // get vector rotation
    else
        cvConvert(matR, &om); // it's already a rotation vector
    cvConvertScale(&om, &om, -0.5); // get average rotation
    cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging

    cvMatMul(&r_r, matT, &t);
    int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
    // if idx == 0
    //   e1 = T / ||T||
    //   e2 = e1 x [0,0,1]

    // if idx == 1
    //   e2 = T / ||T||
    //   e1 = e2 x [0,0,1]

    // e3 = e1 x e2
    _uu[2] = 1;
    cvCrossProduct(&uu, &t, &ww);
    nt = cvNorm(&t, 0, CV_L2);
    CV_Assert(fabs(nt) > 0);
    nw = cvNorm(&ww, 0, CV_L2);
    CV_Assert(fabs(nw) > 0);
    cvConvertScale(&ww, &ww, 1 / nw);
    cvCrossProduct(&t, &ww, &w3);
    nw = cvNorm(&w3, 0, CV_L2);
    CV_Assert(fabs(nw) > 0);
    cvConvertScale(&w3, &w3, 1 / nw);
    _uu[2] = 0;
    for (i = 0; i < 3; ++i)
    {
        _wr[idx][i] = -_t[i] / nt;
        _wr[idx ^ 1][i] = -_ww[i];
        _wr[2][i] = _w3[i] * (1 - 2 * idx); // if idx == 1 -> opposite direction
    }
    // apply to both views
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
    cvConvert( &Ri, _R1 );
    cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
    cvConvert( &Ri, _R2 );
    cvMatMul(&Ri, matT, &t);
    // calculate projection/camera matrices
    // these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
    double fc_new = DBL_MAX;
    CvPoint2D64f cc_new[2] = {};

    newImgSize = newImgSize.width * newImgSize.height != 0 ? newImgSize : imageSize;
    const double ratio_x = (double)newImgSize.width / imageSize.width / 2;
    const double ratio_y = (double)newImgSize.height / imageSize.height / 2;
    const double ratio = idx == 1 ? ratio_x : ratio_y;
    fc_new = (cvmGet(_cameraMatrix1, idx ^ 1, idx ^ 1) + cvmGet(_cameraMatrix2, idx ^ 1, idx ^ 1)) * ratio;
    for( k = 0; k < 2; k++ )
    {
        const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
        const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
        CvPoint2D32f _pts[4] = {};
        CvPoint3D32f _pts_3[4] = {};
        CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
        CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

        for( i = 0; i < 4; i++ )
        {
            int j = (i<2) ? 0 : 1;
            _pts[i].x = (float)((i % 2)*(nx));
            _pts[i].y = (float)(j*(ny));
        }
        cv::Mat ptsM(pts.rows, pts.cols, pts.type, pts.data.ptr);
        cv::Mat A_m(A->rows, A->cols, A->type, A->data.ptr);
        cv::Mat Dk_m(Dk->rows, Dk->cols, Dk->type, Dk->data.ptr);
        cv::fisheye::undistortPoints( ptsM, ptsM, A_m, Dk_m, cv::Mat(), cv::Mat() );
        cvConvertPointsHomogeneous( &pts, &pts_3 );

        //Change camera matrix to have cc=[0,0] and fc = fc_new
        double _a_tmp[3][3];
        CvMat A_tmp  = cvMat(3, 3, CV_64F, _a_tmp);
        _a_tmp[0][0]=fc_new;
        _a_tmp[1][1]=fc_new;
        _a_tmp[0][2]=0.0;
        _a_tmp[1][2]=0.0;

        cvProjectPoints2( &pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts );
        CvScalar avg = cvAvg(&pts);

        cc_new[k].x = (nx)/2 - avg.val[0];
        cc_new[k].y = (ny)/2 - avg.val[1];
    }

    // vertical focal length must be the same for both images to keep the epipolar constraint
    // (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
    // use fy for fx also, for simplicity

    // For simplicity, set the principal points for both cameras to be the average
    // of the two principal points (either one of or both x- and y- coordinates)
    if( flags & cv::CALIB_ZERO_DISPARITY )
    {
        cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    }
    else if( idx == 0 ) // horizontal stereo
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    else // vertical stereo
        cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

    cvZero( &pp );
    _pp[0][0] = _pp[1][1] = fc_new;
    _pp[0][2] = cc_new[0].x;
    _pp[1][2] = cc_new[0].y;
    _pp[2][2] = 1;
    cvConvert(&pp, _P1);

    _pp[0][2] = cc_new[1].x;
    _pp[1][2] = cc_new[1].y;
    _pp[idx][3] = _t[idx]*fc_new; // baseline * focal length
    cvConvert(&pp, _P2);

    alpha = MIN(alpha, 1.);

    icvGetRectanglesFisheye( _cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1 );
    icvGetRectanglesFisheye( _cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2 );

    {
    newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
    double cx1_0 = cc_new[0].x;
    double cy1_0 = cc_new[0].y;
    double cx2_0 = cc_new[1].x;
    double cy2_0 = cc_new[1].y;
    double cx1 = newImgSize.width*cx1_0/imageSize.width;
    double cy1 = newImgSize.height*cy1_0/imageSize.height;
    double cx2 = newImgSize.width*cx2_0/imageSize.width;
    double cy2 = newImgSize.height*cy2_0/imageSize.height;
    double s = 1.;

    if( alpha >= 0 )
    {
        double s0 = std::max(std::max(std::max((double)cx1/(cx1_0 - inner1.x), (double)cy1/(cy1_0 - inner1.y)),
                            (double)(newImgSize.width - cx1)/(inner1.x + inner1.width - cx1_0)),
                        (double)(newImgSize.height - cy1)/(inner1.y + inner1.height - cy1_0));
        s0 = std::max(std::max(std::max(std::max((double)cx2/(cx2_0 - inner2.x), (double)cy2/(cy2_0 - inner2.y)),
                         (double)(newImgSize.width - cx2)/(inner2.x + inner2.width - cx2_0)),
                     (double)(newImgSize.height - cy2)/(inner2.y + inner2.height - cy2_0)),
                 s0);

        double s1 = std::min(std::min(std::min((double)cx1/(cx1_0 - outer1.x), (double)cy1/(cy1_0 - outer1.y)),
                            (double)(newImgSize.width - cx1)/(outer1.x + outer1.width - cx1_0)),
                        (double)(newImgSize.height - cy1)/(outer1.y + outer1.height - cy1_0));
        s1 = std::min(std::min(std::min(std::min((double)cx2/(cx2_0 - outer2.x), (double)cy2/(cy2_0 - outer2.y)),
                         (double)(newImgSize.width - cx2)/(outer2.x + outer2.width - cx2_0)),
                     (double)(newImgSize.height - cy2)/(outer2.y + outer2.height - cy2_0)),
                 s1);

        s = s0*(1 - alpha) + s1*alpha;
    }

    fc_new *= s;
    cc_new[0] = cvPoint2D64f(cx1, cy1);
    cc_new[1] = cvPoint2D64f(cx2, cy2);

    cvmSet(_P1, 0, 0, fc_new);
    cvmSet(_P1, 1, 1, fc_new);
    cvmSet(_P1, 0, 2, cx1);
    cvmSet(_P1, 1, 2, cy1);

    cvmSet(_P2, 0, 0, fc_new);
    cvmSet(_P2, 1, 1, fc_new);
    cvmSet(_P2, 0, 2, cx2);
    cvmSet(_P2, 1, 2, cy2);
    cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));

    }

    if( matQ )
    {
        double q[] =
        {
            1, 0, 0, -cc_new[0].x,
            0, 1, 0, -cc_new[0].y,
            0, 0, 0, fc_new,
            0, 0, -1./_t[idx],
            (idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y)/_t[idx]
        };
        CvMat Q = cvMat(4, 4, CV_64F, q);
        cvConvert( &Q, matQ );
    }
}

void stereoRectifyFisheye( cv::InputArray _cameraMatrix1, cv::InputArray _distCoeffs1,
				cv::InputArray _cameraMatrix2, cv::InputArray _distCoeffs2,
				cv::Size imageSize, cv::InputArray _Rmat, cv::InputArray _Tmat,
				cv::OutputArray _Rmat1, cv::OutputArray _Rmat2,
				cv::OutputArray _Pmat1, cv::OutputArray _Pmat2,
				cv::OutputArray _Qmat, int flags,
				double alpha, cv::Size newImageSize)
{
    cv::Mat cameraMatrix1 = _cameraMatrix1.getMat(), cameraMatrix2 = _cameraMatrix2.getMat();
    cv::Mat distCoeffs1 = _distCoeffs1.getMat(), distCoeffs2 = _distCoeffs2.getMat();
    cv::Mat Rmat = _Rmat.getMat(), Tmat = _Tmat.getMat();
    CvMat c_cameraMatrix1 = CvMat(cameraMatrix1);
    CvMat c_cameraMatrix2 = CvMat(cameraMatrix2);
    CvMat c_distCoeffs1 = CvMat(distCoeffs1);
    CvMat c_distCoeffs2 = CvMat(distCoeffs2);
    CvMat c_R = CvMat(Rmat), c_T = CvMat(Tmat);

    int rtype = CV_64F;
    _Rmat1.create(3, 3, rtype);
    _Rmat2.create(3, 3, rtype);
    _Pmat1.create(3, 4, rtype);
    _Pmat2.create(3, 4, rtype);
    cv::Mat R1 = _Rmat1.getMat(), R2 = _Rmat2.getMat(), P1 = _Pmat1.getMat(), P2 = _Pmat2.getMat(), Q;
    CvMat c_R1 = CvMat(R1), c_R2 = CvMat(R2), c_P1 = CvMat(P1), c_P2 = CvMat(P2);
    CvMat c_Q, *p_Q = 0;

    if( _Qmat.needed() )
    {
        _Qmat.create(4, 4, rtype);
        p_Q = &(c_Q = CvMat(Q = _Qmat.getMat()));
    }

    CvMat *p_distCoeffs1 = distCoeffs1.empty() ? NULL : &c_distCoeffs1;
    CvMat *p_distCoeffs2 = distCoeffs2.empty() ? NULL : &c_distCoeffs2;
    cvStereoRectifyFisheye( &c_cameraMatrix1, &c_cameraMatrix2, p_distCoeffs1, p_distCoeffs2,
        CvSize(imageSize), &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
        CvSize(newImageSize));
}

}


#endif /* GUILIB_SRC_OPENCV_STEREORECTIFYFISHEYE_H_ */
