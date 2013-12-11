#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/gui/UCv2Qt.h"

#include "rtabmap/gui/ImageView.h"
#include "rtabmap/gui/KeypointItem.h"
#include <QtGui/QApplication>
#include <QtGui/QGraphicsLineItem>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtCore/QTime>
#include <QtGui/QGraphicsEffect>


using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-epipolar_geometry image1.jpg image2.jpg\n");
	exit(1);
}

class MainWidget : public QWidget
{
public:
	MainWidget(const cv::Mat & image1,
			   const cv::Mat & image2,
			   const std::multimap<int, cv::KeyPoint> & words1,
			   const std::multimap<int, cv::KeyPoint> & words2,
			   const std::vector<uchar> & status)
	{
		view1_ = new ImageView(this);
		this->setLayout(new QHBoxLayout());
		this->layout()->setSpacing(0);
		this->layout()->setContentsMargins(0,0,0,0);
		this->layout()->addWidget(view1_);
		view1_->setSceneRect(0,0,(float)image1.cols, (float)image1.rows);
		view1_->setLinesShown(true);
		view1_->setFeaturesShown(false);

		QGraphicsPixmapItem * item1 = view1_->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(image1)));
		QGraphicsPixmapItem * item2 = view1_->scene()->addPixmap(QPixmap::fromImage(uCvMat2QImage(image2)));

		QGraphicsOpacityEffect * effect1 = new QGraphicsOpacityEffect();
		QGraphicsOpacityEffect * effect2 = new QGraphicsOpacityEffect();
		effect1->setOpacity(0.5);
		effect2->setOpacity(0.5);
		item1->setGraphicsEffect(effect1);
		item2->setGraphicsEffect(effect2);

		item1->setVisible(view1_->isImageShown());
		item2->setVisible(view1_->isImageShown());

		drawKeypoints(words1, words2, status);
	}
protected:
	virtual void showEvent(QShowEvent* event)
	{
		resizeEvent(0);
	}
	virtual void resizeEvent(QResizeEvent* event)
	{
		view1_->fitInView(view1_->sceneRect(), Qt::KeepAspectRatio);
		view1_->resetZoom();
	}
private:
	void drawKeypoints(const std::multimap<int, cv::KeyPoint> & refWords, const std::multimap<int, cv::KeyPoint> & loopWords, const std::vector<uchar> & status)
	{
		UTimer timer;

		timer.start();
		KeypointItem * item = 0;
		int alpha = 10*255/100;
		QList<QPair<cv::Point2f, cv::Point2f> > uniqueCorrespondences;
		QList<bool> inliers;
		int j=0;
		for(std::multimap<int, cv::KeyPoint>::const_iterator i = refWords.begin(); i != refWords.end(); ++i )
		{
			const cv::KeyPoint & r = (*i).second;
			int id = (*i).first;

			QString info = QString( "WordRef = %1\n"
									"Laplacian = %2\n"
									"Dir = %3\n"
									"Hessian = %4\n"
									"X = %5\n"
									"Y = %6\n"
									"Size = %7").arg(id).arg(1).arg(r.angle).arg(r.response).arg(r.pt.x).arg(r.pt.y).arg(r.size);
			float radius = r.size*1.2/9.*2;
			if(uContains(loopWords, id))
			{
				// PINK = FOUND IN LOOP SIGNATURE
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 0, 255, alpha));
				//To draw lines... get only unique correspondences
				if(uValues(refWords, id).size() == 1 && uValues(loopWords, id).size() == 1)
				{
					uniqueCorrespondences.push_back(QPair<cv::Point2f, cv::Point2f>(r.pt, uValues(loopWords, id).begin()->pt));
					inliers.push_back(status[j++]);
				}
			}
			else if(refWords.count(id) > 1)
			{
				// YELLOW = NEW and multiple times
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(255, 255, 0, alpha));
			}
			else
			{
				// GREEN = NEW
				item = new KeypointItem(r.pt.x-radius, r.pt.y-radius, radius*2, info, QColor(0, 255, 0, alpha));
			}
			item->setVisible(view1_->isFeaturesShown());
			view1_->scene()->addItem(item);
			item->setZValue(1);
		}
		ULOGGER_DEBUG("source time = %f s", timer.ticks());

		// Draw lines between corresponding features...
		UASSERT(uniqueCorrespondences.size() == inliers.size());
		QList<bool>::iterator jter = inliers.begin();
		for(QList<QPair<cv::Point2f, cv::Point2f> >::iterator iter = uniqueCorrespondences.begin();
			iter!=uniqueCorrespondences.end();
			++iter)
		{
			QGraphicsLineItem * item = view1_->scene()->addLine(
					iter->first.x,
					iter->first.y,
					iter->second.x,
					iter->second.y,
					*jter?QPen(Qt::cyan):QPen(Qt::red));
			item->setVisible(view1_->isLinesShown());
			item->setZValue(1);
			++jter;
		}
	}

private:
	ImageView * view1_;
};

std::multimap<int, cv::KeyPoint> aggregate(const std::list<int> & wordIds, const std::vector<cv::KeyPoint> & keypoints)
{
	std::multimap<int, cv::KeyPoint> words;
	std::vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
	for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
		++kpIter;
	}
	return words;
}



int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	cv::Mat image1;
    cv::Mat image2;
	if(argc == 3)
	{
		image1 = cv::imread(argv[1]);
		image2 = cv::imread(argv[2]);
	}
	else
	{
		showUsage();
	}

	QTime timer;
	timer.start();

	// Extract words
	timer.start();
   VWDictionary dictionary;
   ParametersMap param;
   param.insert(ParametersPair(Parameters::kSURFExtended(), "true"));
   param.insert(ParametersPair(Parameters::kSURFHessianThreshold(), "100"));
   SURFDetector keypointDetector(param);
   SURFDescriptor descriptorExtractor(param);
   std::vector<cv::KeyPoint> kpts1 = keypointDetector.generateKeypoints(image1);
   std::vector<cv::KeyPoint> kpts2 = keypointDetector.generateKeypoints(image2);
   cv::Mat descriptors1 = descriptorExtractor.generateDescriptors(image1, kpts1);
   cv::Mat descriptors2 = descriptorExtractor.generateDescriptors(image2, kpts2);
   UINFO("detect/extract features = %d ms", timer.elapsed());

   timer.start();
   std::list<int> wordIds1 = dictionary.addNewWords(descriptors1, 1);
   std::list<int> wordIds2 = dictionary.addNewWords(descriptors2, 2);
   UINFO("quantization to words = %d ms", timer.elapsed());

   std::multimap<int, cv::KeyPoint> words1 = aggregate(wordIds1, kpts1);
   std::multimap<int, cv::KeyPoint> words2 = aggregate(wordIds2, kpts2);

   // Find pairs
   timer.start();
   std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
   EpipolarGeometry::findPairsUnique(words1, words2, pairs);
   UINFO("find pairs = %d ms", timer.elapsed());

   // Find fundamental matrix
   timer.start();
   std::vector<uchar> status;
   cv::Mat fundamentalMatrix = EpipolarGeometry::findFFromWords(pairs, status);
   UINFO("inliers = %d/%d", uSum(status), pairs.size());
   UINFO("find F = %d ms", timer.elapsed());
   if(!fundamentalMatrix.empty())
   {
	   int i = 0;
	   int goodCount = 0;
	   for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
		{
			if(status[i])
			{
				// the output of the correspondences can be easily copied in MatLab
				if(goodCount==0)
				{
					printf("x=[%f %f %d]; xp=[%f %f %d];\n",
							iter->second.first.pt.x,
							iter->second.first.pt.y,
							iter->first,
							iter->second.second.pt.x,
							iter->second.second.pt.y,
							iter->first);
				}
				else
				{
					printf("x=[x;[%f %f %d]]; xp=[xp;[%f %f %d]];\n",
							iter->second.first.pt.x,
							iter->second.first.pt.y,
							iter->first,
							iter->second.second.pt.x,
							iter->second.second.pt.y,
							iter->first);
				}
				++goodCount;
			}
			++i;
		}

		// Show the fundamental matrix
		std::cout << "F=" << fundamentalMatrix << std::endl;

		// Intrinsic parameters K of the camera (guest... non-calibrated camera)
		cv::Mat k = cv::Mat::zeros(3,3,CV_64FC1);
		k.at<double>(0,0) = image1.cols; // focal x
		k.at<double>(1,1) = image1.rows; // focal y
		k.at<double>(2,2) = 1;
		k.at<double>(0,2) = image1.cols/2; // center x in pixels
		k.at<double>(1,2) = image1.rows/2; // center y in pixels

		// Use essential matrix E=K'*F*K

		cv::Mat e = k.t()*fundamentalMatrix*k;

		//remove K from points xe = inv(K)*x
		cv::Mat x1(2, goodCount, CV_64FC1);
		cv::Mat x2(2, goodCount, CV_64FC1);
		i=0;
		int j=0;
		cv::Mat invK = k.inv();
		for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
		{
			if(status[i])
			{
				cv::Mat tmp(3,1,CV_64FC1);
				tmp.at<double>(0,0) = iter->second.first.pt.x;
				tmp.at<double>(1,0) = iter->second.first.pt.y;
				tmp.at<double>(2,0) = 1;
				tmp = invK*tmp;
				x1.at<double>(0,j) = tmp.at<double>(0,0);
				x1.at<double>(1,j) = tmp.at<double>(1,0);
				tmp.at<double>(0,0) = iter->second.second.pt.x;
				tmp.at<double>(1,0) = iter->second.second.pt.y;
				tmp.at<double>(2,0) = 1;
				tmp = invK*tmp;
				x2.at<double>(0,j) = tmp.at<double>(0,0);
				x2.at<double>(1,j) = tmp.at<double>(1,0);
				UDEBUG("i=%d j=%d, x1=[%f,%f] x2=[%f,%f]", i, j, x1.at<double>(0,j), x1.at<double>(1,j), x2.at<double>(0,j), x2.at<double>(1,j));
				++j;
			}
			++i;
		}

		std::cout<<"K=" << k << std::endl;
		timer.start();
		//std::cout<<"e=" << e << std::endl;
		cv::Mat p = EpipolarGeometry::findPFromF(e, x1, x2);
		cv::Mat p0 = cv::Mat::zeros(3, 4, CV_64FC1);
		p0.at<double>(0,0) = 1;
		p0.at<double>(1,1) = 1;
		p0.at<double>(2,2) = 1;
		UINFO("find P from F = %d ms", timer.elapsed());

		std::cout<<"P=" << p << std::endl;

		//find 4D homogeneous points
		cv::Mat x4d;
		timer.start();
		cv::triangulatePoints(p0, p, x1, x2, x4d);
		UINFO("find X (triangulate) = %d ms", timer.elapsed());

		//Show 4D points
		for(int i=0; i<x4d.cols; ++i)
		{
			x4d.at<double>(0,i) = x4d.at<double>(0,i)/x4d.at<double>(3,i);
			x4d.at<double>(1,i) = x4d.at<double>(1,i)/x4d.at<double>(3,i);
			x4d.at<double>(2,i) = x4d.at<double>(2,i)/x4d.at<double>(3,i);
			x4d.at<double>(3,i) = x4d.at<double>(3,i)/x4d.at<double>(3,i);
			if(i==0)
			{
				printf("X=[%f;%f;%f;%f];\n",
					x4d.at<double>(0,i),
					x4d.at<double>(1,i),
					x4d.at<double>(2,i),
					x4d.at<double>(3,i));
			}
			else
			{
				printf("X=[X [%f;%f;%f;%f]];\n",
					x4d.at<double>(0,i),
					x4d.at<double>(1,i),
					x4d.at<double>(2,i),
					x4d.at<double>(3,i));
			}
		}

		//Show rotation/translation of the second camera
		cv::Mat r;
		cv::Mat t;
		EpipolarGeometry::findRTFromP(p, r, t);
		std::cout<< "R=" << r << std::endl;
		std::cout<< "t=" << t << std::endl;

		//GUI
		QApplication app(argc, argv);
		MainWidget mainWidget(image1, image2, words1, words2, status);
		mainWidget.show();
		app.exec();
	}
	else
	{
		UINFO("Fundamental matrix not found...");
	}

    return 0;
}
