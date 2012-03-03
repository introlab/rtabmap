#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <fftw3.h>
#include "rtabmap/core/Camera.h"
#include "ColorTable.h"

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kDebug);

    IplImage* src = 0;
    if( argc == 2)
    {
    	src=cvLoadImage(argv[1],1);
    }
    else
    {
    	rtabmap::CameraVideo cam(0,0,false, 640, 480);
    	if(cam.init())
    	{
    		src = cam.takeImage();
    	}
    }
    if(src)
    {
    	UTimer timer;
    	timer.start();
    	// Log-polar transform
    	int radius = src->height < src->width ? src->height/2: src->width/2;
    	CvSize polarSize = cvSize(64, 128);
    	float M = polarSize.width/std::log(radius);
    	UDEBUG("src size=(%d,%d) radius=%d, M=%f", src->width, src->height, radius, M);
        IplImage* polar = cvCreateImage( polarSize, 8, 3 );
        IplImage* src2 = cvCreateImage( cvGetSize(src), 8, 3 );
        cvLogPolar( src, polar, cvPoint2D32f(src->width/2,src->height/2), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
        cvLogPolar( polar, src2, cvPoint2D32f(src->width/2,src->height/2), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );
        UDEBUG("logpolar time=%fs", timer.ticks());

        // HSV transform
        IplImage* hsv = cvCreateImage( cvGetSize(polar), IPL_DEPTH_8U, 3 );
        cvCvtColor( polar, hsv, CV_BGR2HSV );
        UDEBUG("bgr->hsv time=%fs", timer.ticks());

        // Fetch H channel
        cv::Mat hsvMat(hsv);
        cv::vector<cv::Mat> channels;
        split(hsvMat, channels);
        cv::Mat hMat;
        channels[0].convertTo(hMat, CV_32F);
        hMat /= 255.0f;
        UDEBUG("fetch h channel time=%fs", timer.ticks());

        // DFT transform
        cv::Mat dftMat;
        cv::dft(hMat, dftMat, cv::DFT_ROWS);
        UDEBUG("dft opencv time=%fs", timer.ticks());

        // FFT transform
        float in[hMat.cols];
        fftwf_complex * out;
		fftwf_plan p;
		cv::Mat fftMat(hMat.rows, hMat.cols/2+1, CV_32F);
		out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * fftMat.cols);
		p = fftwf_plan_dft_r2c_1d(hMat.cols, in, out, 0);
		UDEBUG("fft create plan time=%fs", timer.ticks());
		if(p==0)
		{
			UFATAL("cannot create a plan");
		}

		for(int i=0; i<hMat.rows; ++i)
		{
			cv::Mat hRow = hMat(cv::Range(i,i+1), cv::Range(0,hMat.cols));
			memcpy(in, hRow.data, sizeof(float)*hRow.cols);

			fftwf_execute(p); /* repeat as needed */
			for(int j=0; j<fftMat.cols; ++j)
			{
				cv::Mat fftRow = fftMat(cv::Range(i,i+1), cv::Range(0,fftMat.cols));
				float re = (float)out[j][0];
				float im = (float)out[j][1];
				fftRow.at<float>(0,j) = sqrt(re*re+im*im); // TODO keep only real of the complex instead of the module ?
			}
		}
		UDEBUG("fft time=%fs", timer.ticks());

		fftwf_destroy_plan(p);
		fftwf_free(out);
		UDEBUG("fft cleanup time=%fs", timer.ticks());

		//std::cout << dftMat << std::endl;
		//std::cout << fftMat << std::endl;


        //UDEBUG("hMat row=%d cols=%d", hMat.rows, hMat.cols);
        //UDEBUG("hMat row=%d cols=%d, channels=%d", dftMat.rows, dftMat.cols, dftMat.channels());


		IplImage * ind = cvCloneImage(src);
		unsigned char * imageData = (unsigned char *)ind->imageData;
		rtabmap::ColorTable colorTable(65536);
		UDEBUG("widthStep=%d", ind->widthStep);
		for(int i=0; i<ind->height; ++i)
		{
			for(int j=0; j<ind->width; ++j)
			{
				unsigned char & b = imageData[i*ind->widthStep+j*3+0];
				unsigned char & g = imageData[i*ind->widthStep+j*3+1];
				unsigned char & r = imageData[i*ind->widthStep+j*3+2];
				int index = (int)colorTable.getIndex(r, g, b);
				colorTable.getRgb(index, r, g , b);
			}
		}

        cvNamedWindow( "log-original", 1 );
		cvShowImage( "log-original", src );
		cvNamedWindow( "log-polar", 1 );
		cvShowImage( "log-polar", polar );
		cvNamedWindow( "inverse log-polar", 1 );
		cvShowImage( "inverse log-polar", src2 );
		cvNamedWindow( "hsv", 1 );
		cvShowImage( "hsv", hsv );
		cvNamedWindow( "ind", 1 );
		cvShowImage( "ind", ind );

		UDEBUG("show time=%fs", timer.ticks());

        cvWaitKey();

        cvReleaseImage(&src);
        cvReleaseImage(&polar);
        cvReleaseImage(&src2);
        cvReleaseImage(&hsv);
        cvReleaseImage(&ind);
    }
    return 0;
}
