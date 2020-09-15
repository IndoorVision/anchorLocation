#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
class CMeanByMaxBin
{
public:
	CMeanByMaxBin();
	~CMeanByMaxBin();
	double  estimateOri(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint);
protected:
	void buildBin(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint,double interval);
	double selectBin(double interval);
	std::vector<int> m_binCountArray;

};

