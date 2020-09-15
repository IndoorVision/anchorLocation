#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

#define deltaA 1.85 //��ά�﷽����ϵ��0�ȷ����Ӧ���ֻ����򴫸����еĽǶ� 
#define pi 3.14159265359
#define grid 1//���仮������Ĵ�С
#define room_length 14.55//��λ����ĳ�
#define room_width 6.99//��λ����Ŀ�
#define reduce_factor 2.06667// ��Сϵ��

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

