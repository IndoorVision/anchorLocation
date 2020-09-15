//#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include "SurfExtractor.h"
#include "Runtime.h"


CSurfExtractor::CSurfExtractor()
{
}


CSurfExtractor::~CSurfExtractor()
{
}

bool CSurfExtractor::run(char* filename, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, cv::Mat &mK)
{
	cv::Mat img = cv::imread(filename);
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

bool CSurfExtractor::run_corridor(char* filename, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, cv::Mat &mK)
{
	CRuntime tt("run_corridor");
	cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
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

	//write resize Picture
	std::string strPath = filename;
	std::string tempFileName = strPath.substr(0, strPath.find_last_of("\\") + 1);
	std::string PictureName = strPath.substr(strPath.find_last_of("\\") + 1);
	PictureName = PictureName.substr(0, PictureName.rfind("."));
	//cv::imwrite(tempFileName + "\\" + "resize_" + PictureName + ".jpg", img);
	//write end

	return runImg(img, desc, pt2dArray);
}

bool CSurfExtractor::run_corridor(char* filename, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, double& resizeRatio)
{
	cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
	double max_wh = std::max(img.rows, img.cols);
	if (max_wh>1920)
	{
		resizeRatio = std::max(max_wh / 1920, 1.0);;
		cv::resize(img, img, cv::Size(img.cols / resizeRatio, img.rows / resizeRatio));
	}
	
	return runImg(img, desc, pt2dArray);
}

bool CSurfExtractor::runImg(cv::Mat img, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray)
{
	CRuntime tt("runImg");
	cv::Ptr<cv::SURF> surf = new cv::SURF;
	surf->hessianThreshold = 600;
	surf->nOctaveLayers = 2;
	surf->nOctaves = 2;
	
	if (surf.empty()) {
		printf("OpenCV was built without SURF support");
		return false;
	}
	
	std::vector<cv::KeyPoint> keypoints;
	{
		CRuntime tt("detect");
		surf->detect(img, keypoints);
	}
	
	if (keypoints.size() > 2000)  
		keypoints.resize(2000);

	{
		CRuntime tt("compute");
		surf->compute(img, keypoints, desc);
	}
	

	printf("keypoints.size() = %d\n", keypoints.size());
	printf("------------------------------------\n");

	cv::Point2d pt2d;
	pt2dArray.resize(keypoints.size());
	for (int i = 0; i < keypoints.size(); i++)
	{
		pt2d.x = keypoints[i].pt.x;
		pt2d.y = keypoints[i].pt.y;
		pt2dArray[i] = pt2d;
	}
	return true;
}
