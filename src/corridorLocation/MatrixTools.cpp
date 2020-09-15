
#include "MatrixTools.h"


MatrixTools::MatrixTools()
{
}


MatrixTools::~MatrixTools()
{
}

//POK转旋转矩阵
void MatrixTools::POK2RotateMatrix(double phi, double omega, double kappa, cv::Mat &R, angleStyle style)
{
	if (style == Angle)
	{
		phi = phi*PI / 180.0;
		omega = omega*PI / 180.0;
		kappa = kappa*PI / 180.0;
	}
	cv::Mat Rx, Ry, Rz;
	Ry = (cv::Mat_<double>(3, 3) << cos(phi), 0, -sin(phi), 0, 1, 0, sin(phi), 0, cos(phi));
	Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(omega), -sin(omega), 0, sin(omega), cos(omega));
	Rz = (cv::Mat_<double>(3, 3) << cos(kappa), -sin(kappa), 0, sin(kappa), cos(kappa), 0, 0, 0, 1);

	R = Ry*Rx*Rz;
}
//OPK转旋转矩阵
void MatrixTools::OPK2RotateMatrix(double omega, double phi, double kappa, cv::Mat &R, angleStyle style)
{
	if (style == Angle)
	{
		phi = phi*PI / 180.0;
		omega = omega*PI / 180.0;
		kappa = kappa*PI / 180.0;
	}

	cv::Mat Rx, Ry, Rz;
	Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(omega), -sin(omega), 0, sin(omega), cos(omega));
	Ry = (cv::Mat_<double>(3, 3) << cos(phi), 0, sin(phi), 0, 1, 0, -sin(phi), 0, cos(phi));
	Rz = (cv::Mat_<double>(3, 3) << cos(kappa), -sin(kappa), 0, sin(kappa), cos(kappa), 0, 0, 0, 1);

	R = Rx*Ry*Rz;
}
//yaw pitch roll
void MatrixTools::YPR2RotateMatrix(double yaw, double pitch, double roll, cv::Mat &R, angleStyle style)
{
	if (style == Angle)
	{
		yaw = yaw*PI / 180.0;
		pitch = pitch*PI / 180.0;
		roll = roll*PI / 180.0;
	}

	cv::Mat Rx, Ry, Rz;
	Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll));
	Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 0, cos(pitch));
	Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);

	R = Rz*Ry*Rx;
}

//RQ分解
void MatrixTools::RQSplit(cv::Mat A1, cv::Mat &R, cv::Mat &Q)
{
	double c, s;
	double a, b;
	cv::Mat A;
	A = A1.clone();
	cv::Mat_<double> Qx(3, 3), Qy(3, 3), Qz(3, 3);

	a = A.at<double>(2, 1);
	b = A.at<double>(2, 2);
	c = -b / (sqrt(a*a + b*b));
	s = a / (sqrt(a*a + b*b));
	Qx << 1, 0, 0, 0, c, -s, 0, s, c;
	A = A*Qx;

	a = A.at<double>(2, 0);
	b = A.at<double>(2, 2);
	c = -b / (sqrt(a*a + b*b));
	s = a / (sqrt(a*a + b*b));
	Qy << c, 0, -s, 0, 1, 0, s, 0, c;
	A = A*Qy;

	a = A.at<double>(1, 0);
	b = A.at<double>(1, 1);
	c = -b / (sqrt(a*a + b*b));
	s = a / (sqrt(a*a + b*b));
	Qz << c, -s, 0, s, c, 0, 0, 0, 1;
	A = A*Qz;

	Q = Qz.t()*Qy.t()*Qx.t();
	R = A;
	return;
}