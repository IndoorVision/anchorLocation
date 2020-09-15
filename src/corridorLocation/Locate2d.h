#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>


struct Result {
	cv::Point2d C;
	double res = std::numeric_limits<double>::max();
	double angle = 0;
};

class CLocate2d
{
public:
	CLocate2d();
	~CLocate2d();
	bool run(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation,double& X,double& Y);
	bool run_erroranalysis(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation, double& X, double& Y, double &mx, double &my, double &moritation);

protected:
	void removeBigResidual(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, float orientation, double Xs, double Ys, double threshold);
	cv::Mat loactionCollinear2D_iter(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation_0, double Xs_0, double Ys_0, double &vx);
	cv::Mat loactionCollinear2D_iter_erroranalysis(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation_0, double Xs_0, double Ys_0, double &vx, double &mx, double &my, double &moritation);

	
};

