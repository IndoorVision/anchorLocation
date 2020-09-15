#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

#define deltaA 1.85 //二维物方坐标系中0度方向对应到手机方向传感器中的角度 
#define pi 3.14159265359
#define grid 1//房间划分网格的大小
#define room_length 14.55//定位房间的长
#define room_width 6.99//定位房间的宽
#define reduce_factor 2.06667// 缩小系数

class Cvote_locate2
{
public:
	Cvote_locate2();
	~Cvote_locate2();

	double *culAngle(int width, int f, double* angle, const std::vector<cv::Point2d> MarkArray, double oriention);
	void Locate2d(int num, std::vector<double> angle, std::vector<cv::Point2d> MarkArray, double &X, double &Y);
	void Vote(int num, std::vector<std::vector<std::vector<int>>> &vote, int** &mesh_vote, double* angle, std::vector<cv::Point3d> MapArray_);
	bool run(cv::Mat mK, std::vector<cv::Point2d>& MarkArray_, std::vector<cv::Point3d>& MapArray_,
		double &X, double &Y, double Angle_or = pi*310.66998/180);
};

