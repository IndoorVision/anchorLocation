#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

class CMonteCarlo2d
{
public:
	CMonteCarlo2d();
	~CMonteCarlo2d();
	bool run(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation, double& X, double& Y, double threshold = 200, double dStep = 1.0);

	double project2d(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, double X, double Y, double threshold);
	double estimateOri(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint);
};

