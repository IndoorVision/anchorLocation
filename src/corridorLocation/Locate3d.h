#pragma once
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "opencv2/opencv_modules.hpp"
#include <set>
#include <algorithm>
#include <iterator> 
#include <fstream>
#include <string>
//定义了CLocate3d类，其中有两个函数，run和sloverPnP_cv,run中嵌套着使用了sloverPnP_cv
//run的作用：利用经过匹配的，定位影像上的二维坐标（matchPt2dArray）和库中的三维点坐标（matchPt3dArray），后方交会得到相机的位姿
//run的输入：二维坐标（matchPt2dArray）和库中的三维点坐标（matchPt3dArray）；
//run的输出：后方交会得到的R与C
//sloverPnP_cv的输入：二维坐标（matchPt2dArray）和库中的三维点坐标（matchPt3dArray）、内参矩阵
//sloverPnP_cv的输出：R、C、内点数量

class CLocate3d
{
public:
	CLocate3d();
	~CLocate3d();

	bool run(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
		cv::Mat&R, cv::Point3d&C, cv::Mat mK);

	bool sloverPnP_cv(const std::vector<cv::Point2d> &MarkArray_,
		const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
		const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers);

	bool run_corridor(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
		cv::Mat&R, cv::Point3d&C, cv::Mat mK, cv::Mat distort);

	bool sloverPnP_cv_corridor(const std::vector<cv::Point2d> &MarkArray_,
		const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
		const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers);

	
};

