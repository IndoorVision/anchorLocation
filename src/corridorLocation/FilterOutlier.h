#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

class CFilterOutlier
{
public:
	CFilterOutlier();
	~CFilterOutlier();

	bool run(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double& orientation, cv::Point3d& centerPoint);
	bool removeByX(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, cv::Point3d centerPoint);

	double  estimateOri(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, cv::Point3d& centerPoint);

};

