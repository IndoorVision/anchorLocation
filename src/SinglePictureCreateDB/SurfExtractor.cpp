#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include "SurfExtractor.h"



CSurfExtractor::CSurfExtractor()
{
}


CSurfExtractor::~CSurfExtractor()
{
}

bool CSurfExtractor::run(char* filename, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, cv::Mat &mK)
{
	cv::Mat img = cv::imread(filename, 0);
	//增加mK的定义  ljf 20190828
	int img_pix_w = 0;
	int img_pix_h = 0;
	float f_pix = 3500;
	//f_pix = 3737;
	//f_pix = 3070; // iphone焦距
	double max_wh = std::max(img.rows, img.cols);
	if (max_wh>1920)
	{
		double t = std::max(max_wh / 1920, 1.0);;
		cv::resize(img, img, cv::Size(img.cols / t, img.rows / t));
		f_pix /= t;
	}
	img_pix_w = img.cols;
	img_pix_h = img.rows;
	mK = (cv::Mat_<double>(3, 3) << f_pix, 0, img_pix_w / 2.0
		, 0, f_pix, img_pix_h / 2.0
		, 0, 0, 1);


	return runImg(img, desc, pt2dArray);
}

bool CSurfExtractor::runImg(cv::Mat img, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray)
{
	cv::Ptr<cv::SURF> surf = new cv::SURF;
	
	//cv::Ptr<cv::Feature2D> surf = cv::Algorithm::create<cv::Feature2D>("Feature2D.SURF");
	if (surf.empty()) {
		printf("OpenCV was built without SURF support");
		return false;
	}
	
//	surf->set("hessianThreshold", 1000);
//	surf->set("nOctaves", 3);
//	surf->set("nOctaveLayers", 3);

	std::vector<cv::KeyPoint> keypoints;
	surf->detect(img, keypoints);
	surf->compute(img, keypoints, desc);


	cv::Point2d pt2d;
	pt2dArray.reserve(keypoints.size());
	for (int i = 0; i < keypoints.size(); i++)
	{
		pt2d.x = keypoints[i].pt.x;
		pt2d.y = keypoints[i].pt.y;
		pt2dArray.push_back(pt2d);
	}
	return true;
}
