#pragma once
#include"DataStruction.h"
#define  PI  3.14159265358979323846264338327950288
class MatrixTools
{
public:
	MatrixTools();
	~MatrixTools();
	enum angleStyle{ Radians, Angle};
	//POK转旋转矩阵
	static void POK2RotateMatrix(double phi, double omega, double kappa, cv::Mat &R, angleStyle style = Radians);
	//OPK转旋转矩阵
	static void OPK2RotateMatrix(double omega, double phi, double kappa, cv::Mat &R, angleStyle style = Radians);
	//yaw pitch roll
	static void YPR2RotateMatrix(double yaw, double pitch, double roll, cv::Mat &R, angleStyle style = Radians);
	//RQ分解
	static void RQSplit(cv::Mat A1, cv::Mat &R, cv::Mat &Q);
};

