#pragma once
//#define IMAGENUM 2 // how many images in this project
//定义结构体
#include<iostream>
#include<fstream>
#include<opencv2\opencv.hpp>
#include<iomanip>
#include<vector>
#include<string>

struct ImageEo
{
	std::string imageName;
	double Xs;
	double Ys;
	double Zs;
	double phi;
	double omega;
	double kappa;
	double pitch;
	double yaw;
	double roll;
	cv::Mat R;
};

struct ImageIo
{
	std::string imageName;
	double f;
	double x0;
	double y0;
};

struct Line2D
{
	std::string lineID;
	double angle;
	cv::Point3d pointLeft;
	cv::Point3d pointRight;
	cv::Mat     Line;
};

struct Line3D
{
	std::string lineID;
	cv::Point3d pointLeft;
	cv::Point3d pointRight;
};

class LMatchResult
{
public:
	void push_back(int imgIndex1, int imgIndex2, int lineIndex1, int linegIndex2, int line3DIndex1) {
		imageIndex.push_back(imgIndex1);
		imageIndex.push_back(imgIndex2);
		line2DIndex.push_back(lineIndex1);
		line2DIndex.push_back(linegIndex2);
		line3DIndex.push_back(line3DIndex1);
	}
	void push_back(int imgIndex1, int imgIndex2, int lineIndex1, int linegIndex2) {
		imageIndex.push_back(imgIndex1);
		imageIndex.push_back(imgIndex2);
		line2DIndex.push_back(lineIndex1);
		line2DIndex.push_back(linegIndex2);	
	}
	std::vector<int> imageIndex;
	std::vector<int> line2DIndex;
	std::vector<int> line3DIndex;
};

typedef std::vector<cv::Mat>					  VecMat;
typedef std::vector<std::vector<Line2D>>		  VecVecLine2D;
typedef std::vector<std::vector<Line3D>>          VecVecLine3D;
typedef std::vector<std::vector<cv::Mat>>         VecVecMat;
typedef std::vector<std::vector<cv::Point2d>>	  VecVecPoint2D;
typedef std::vector<LMatchResult>                 VecLMatchArray;


