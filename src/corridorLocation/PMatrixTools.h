#pragma once
#include "DataStruction.h"
#include <opencv2/opencv.hpp>
class PMatrixTools
{
public:
	PMatrixTools();
	~PMatrixTools();
	//将DP旋转矩阵改为CV旋转矩阵
	static  void RMatrixFromDP2CV(ImageEo &imageEo);
	//p矩阵生成
	static  void createPMatrix(ImageEo imageEo, ImageIo imageIo, cv::Mat &P);
	//p矩阵分解
	static  void splitPmatrix(cv::Mat P, ImageIo& imageIo, ImageEo& imageEo);
	//p矩阵前方交会
	static void parameterCompute(cv::Mat P, double x, double y, cv::Mat& parameter);
	static void forwardIntersection(cv::Mat pMatrixLeft, cv::Mat pMatrixRight
		, double xLeft, double yLeft, double xRight, double yRight, cv::Point3d &point3D);
	static void ForwardIntersection_P_MultiImg(std::vector<cv::Mat> P,
											VecVecPoint2D &Point2DArray,
											std::vector<cv::Point3d> & point3DArray);

	static void ForwardIntersection_P_MultiImg_MaxDistanceIndex(std::vector<cv::Mat> P,VecVecPoint2D &Point2DArray,
                                               std::vector<cv::Point3d> & point3DArray, int MaxDistanceImgIndex);
	//p矩阵投影
	static  void projectFrom3D_T_2D_P(cv::Mat P,std::vector<cv::Point3d> & point3DArray ,
										std::vector<cv::Point2d> &imagePointArray);
	static  void projectFrom3D_T_2D_P(cv::Mat P, cv::Point3d & point3D, cv::Point2d &imagePoint);

};

